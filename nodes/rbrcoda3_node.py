#!/usr/bin/env python3
"""
ROS 2 node RBRcoda3 — lit le flux streamé du capteur et publie tout en JSON
sur 'rbrcoda3/data' :

  - mesures   : température (°C), pression absolue (dbar) si la voie existe
  - dérivées  : pression marine (dbar), profondeur (m) si voie pression
  - capteur   : modèle, n° série, firmware, mode et période d'échantillonnage
  - flux      : horodatage capteur, nombre d'échantillons reçus

Le coda3 émet ses mesures en continu (ex. "269500, 20.9781") ; le node lit
passivement le flux. Si plus rien n'arrive pendant 5 s, il tente un 'fetch'.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time
import json
import threading

from marble_sensors_hmi.drivers.rbrcoda3_driver import (
    wakeup, cmd, read_info, parse_stream,
    sea_pressure, depth, ATMOSPHERE_DBAR, SEAWATER_DENSITY,
)

_STREAM_TIMEOUT_S = 5.0   # silence avant de tenter un fetch

_FIELD_FMT = {
    'temperature_c':     ('.4f', '°C'),
    'pressure_dbar':     ('.3f', 'dbar'),
    'sea_pressure_dbar': ('.3f', 'dbar'),
    'depth_m':           ('.3f', 'm'),
}


class RBRcoda3Node(Node):

    def __init__(self):
        super().__init__('rbrcoda3_node')

        self.declare_parameter('port',            'COM11')
        self.declare_parameter('baud',            9600)
        self.declare_parameter('sample_interval', 0.0)   # 0 = publier chaque échantillon
        self.declare_parameter('atmosphere_dbar', ATMOSPHERE_DBAR)
        self.declare_parameter('density_kg_m3',   SEAWATER_DENSITY)

        self._port       = self.get_parameter('port').value
        self._baud       = self.get_parameter('baud').value
        self._interval   = float(self.get_parameter('sample_interval').value)
        self._atmosphere = float(self.get_parameter('atmosphere_dbar').value)
        self._density    = float(self.get_parameter('density_kg_m3').value)

        self._pub = self.create_publisher(String, 'rbrcoda3/data', 10)
        self.get_logger().info(
            f"RBRcoda3 node démarré — port={self._port}  baud={self._baud}")

        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    # ── Publication ───────────────────────────────────────────────────────────

    def _publish(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

    @staticmethod
    def _fmt_num(key: str, val) -> dict:
        fmt_str, unit = _FIELD_FMT[key]
        if val is None:
            return {'value': None, 'unit': unit, 'display': 'N/A'}
        return {'value': val, 'unit': unit, 'display': f"{val:{fmt_str}}"}

    @staticmethod
    def _fmt_txt(val, unit: str = '') -> dict:
        return {'value': val, 'unit': unit, 'display': str(val) if val not in (None, '') else '---'}

    # ── Construction du payload ───────────────────────────────────────────────

    def _build_fields(self, ts: str, values: list) -> dict:
        """Associe les valeurs streamées aux voies du capteur + dérivées."""
        fields = {}

        # Association valeur → voie (par nom de voie, sinon par position)
        names = [name.lower() for name, _ in self._channels]
        for i, val in enumerate(values):
            name = names[i] if i < len(names) else ''
            if 'temp' in name or (not name and i == 0):
                fields['temperature_c'] = self._fmt_num('temperature_c', round(val, 4))
            elif 'pres' in name or (not name and i == 1):
                fields['pressure_dbar'] = self._fmt_num('pressure_dbar', round(val, 3))
            else:
                # voie inconnue (ex. O2) : publiée telle quelle
                unit = self._channels[i][1] if i < len(self._channels) else ''
                fields[name or f'channel_{i + 1}'] = self._fmt_txt(round(val, 4), unit)

        # Dérivées si une voie pression est présente
        if 'pressure_dbar' in fields:
            p_sea = sea_pressure(fields['pressure_dbar']['value'], self._atmosphere)
            fields['sea_pressure_dbar'] = self._fmt_num('sea_pressure_dbar', round(p_sea, 3))
            fields['depth_m'] = self._fmt_num('depth_m', round(depth(p_sea, self._density), 3))

        # Métadonnées capteur
        fields['model']     = self._fmt_txt(self._info.get('model'))
        fields['serial']    = self._fmt_txt(self._info.get('serial'))
        fields['firmware']  = self._fmt_txt(self._info.get('firmware'))
        fields['mode']      = self._fmt_txt(self._info.get('mode'))
        fields['period_ms'] = self._fmt_txt(self._info.get('period_ms'), 'ms')

        # Horodatage capteur : compteur ms ("269500") ou datetime complet
        if ts.isdigit():
            fields['sample_time'] = self._fmt_txt(f"{int(ts) / 1000:.1f}", 's')
        else:
            fields['sample_time'] = self._fmt_txt(ts)
        fields['sample_count'] = self._fmt_txt(self._count)

        return fields

    # ── Boucle principale ─────────────────────────────────────────────────────

    def _loop(self) -> None:
        while self._running:
            try:
                conn = serial.Serial(
                    self._port, self._baud,
                    bytesize=8, parity='N', stopbits=1, timeout=1.0,
                )
                self.get_logger().info("Port série ouvert")

                # Identification (les commandes fonctionnent pendant le streaming)
                wakeup(conn)
                self._info = read_info(conn)
                self._channels = self._info.get('channels', [])
                self._count = 0
                self.get_logger().info(
                    f"Capteur : model={self._info.get('model') or '?'}  "
                    f"serial={self._info.get('serial') or '?'}  "
                    f"fw={self._info.get('firmware') or '?'}  "
                    f"mode={self._info.get('mode') or '?'}  "
                    f"période={self._info.get('period_ms') or '?'} ms  "
                    f"voies={self._channels or '?'}")

                last_pub  = 0.0
                last_data = time.time()

                while self._running:
                    line = conn.readline().decode('ascii', errors='ignore').strip()
                    sample = parse_stream(line) if line else None

                    if sample is None:
                        # Silence prolongé → le capteur ne streame pas : fetch
                        if time.time() - last_data > _STREAM_TIMEOUT_S:
                            self.get_logger().warn("Pas de flux — tentative fetch")
                            wakeup(conn)
                            sample = parse_stream(cmd(conn, 'fetch', wait=3.0)
                                                  .replace('fetch', '').strip())
                            if sample is None:
                                self._publish({'status': 'no_data'})
                                last_data = time.time()
                                continue
                        else:
                            continue

                    ts, values = sample
                    self._count += 1
                    last_data = time.time()

                    # Throttle optionnel de publication
                    if self._interval > 0 and time.time() - last_pub < self._interval:
                        continue
                    last_pub = time.time()

                    self._publish({
                        'status':    'ok',
                        'timestamp': time.strftime('%H:%M:%S'),
                        'fields':    self._build_fields(ts, values),
                    })
                    self.get_logger().info(f"échantillon #{self._count} — {ts}, {values}")

                conn.close()

            except serial.SerialException as e:
                self.get_logger().error(f"Erreur série : {e} — reconnexion dans 10 s")
                self._publish({'status': 'error', 'error': str(e)})
                time.sleep(10.0)
            except Exception as e:
                self.get_logger().error(f"Erreur inattendue : {e}")
                time.sleep(5.0)

    def destroy_node(self) -> None:
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RBRcoda3Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
