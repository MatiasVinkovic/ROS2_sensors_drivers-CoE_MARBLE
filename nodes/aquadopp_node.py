#!/usr/bin/env python3
"""
Aquadopp S4VP — NODE ROS2
=========================
Boucle d'acquisition validée (cf. test_aquadopp_sans_ros2.py) adaptée en node :
  - ouverture du port série
  - BREAK + SETDEFAULT,CONFIG -> mode commande
  - configuration du plan de mesure -> START
  - lecture des trames binaires -> parsing -> publication JSON sur 'aquadopp/data'

Changement de port à chaud depuis l'IHM via le topic 'aquadopp/set_port'.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time
import json
import threading

# Import du driver : fonctionne en local (dossier plat) comme dans le package ROS.
try:
    from aquadopp_driver import (
        flush, enter_command_mode, configure, start_measurement,
        capture_burst, read_packet, parse_packet,
    )
except ImportError:
    from marble_sensors_hmi.drivers.aquadopp_driver import (
        flush, enter_command_mode, configure, start_measurement,
        capture_burst, read_packet, parse_packet,
    )

_FIELD_FMT = {
    'speed_of_sound_ms': ('.2f', 'm/s'),
    'temperature_c':     ('.3f', '°C'),
    'pressure_dbar':     ('.4f', 'dbar'),
    'heading_deg':       ('.2f', '°'),
    'pitch_deg':         ('.3f', '°'),
    'roll_deg':          ('.3f', '°'),
}


class AquadoppNode(Node):

    def __init__(self):
        super().__init__('aquadopp_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        self._port = self.get_parameter('port').value
        self._baud = self.get_parameter('baud').value

        self._pub = self.create_publisher(String, 'aquadopp/data', 10)
        self.get_logger().info(f"Aquadopp node démarré — port={self._port}  baud={self._baud}")

        # Changement de port à chaud depuis l'IHM
        self._reconnect = False
        self.create_subscription(String, 'aquadopp/set_port', self._cb_set_port, 10)

        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _cb_set_port(self, msg: String) -> None:
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"set_port JSON invalide : {e}")
            return
        self._port = cfg.get('port', self._port)
        try:
            self._baud = int(cfg.get('baud', self._baud))
        except (TypeError, ValueError):
            pass
        self._reconnect = True
        self.get_logger().info(f"Changement de port demandé → {self._port} @ {self._baud}")

    def _sleep(self, seconds: float) -> None:
        """Attente interruptible par arrêt du node ou changement de port."""
        for _ in range(int(seconds / 0.1)):
            if not self._running or self._reconnect:
                return
            time.sleep(0.1)

    def _publish(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

    def _loop(self) -> None:
        while self._running:
            try:
                self._reconnect = False
                conn = serial.Serial(
                    self._port, self._baud,
                    bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE, timeout=2.0,
                )
                self.get_logger().info("Port série ouvert")

                if not enter_command_mode(conn, self.get_logger()):
                    self._publish({'status': 'error', 'error': 'impossible d\'entrer en mode commande'})
                    conn.close()
                    self._sleep(10.0)
                    continue

                if not configure(conn, self.get_logger()):
                    self.get_logger().warn("Configuration partielle — on continue")

                if not start_measurement(conn):
                    self._publish({'status': 'error', 'error': 'START refusé'})
                    conn.close()
                    self._sleep(5.0)
                    continue

                capture_burst(conn, idle_s=2.0, max_wait_s=20.0)
                self.get_logger().info("Mesure démarrée — attente des paquets binaires (~30 s)")

                while self._running and not self._reconnect:
                    raw = read_packet(conn, self.get_logger(), timeout_s=300)
                    if raw is None:
                        self._publish({'status': 'error', 'error': 'timeout paquet'})
                        break

                    data = parse_packet(raw)
                    if data:
                        fields = {}
                        for key, value in data.items():
                            fmt_str, unit = _FIELD_FMT.get(key, ('.3f', ''))
                            fields[key] = {'value': value, 'unit': unit, 'display': f"{value:{fmt_str}}"}
                        self._publish({'status': 'ok', 'timestamp': time.strftime('%H:%M:%S'), 'fields': fields})
                        self.get_logger().info(f"Paquet OK — T={data['temperature_c']:.3f} °C  P={data['pressure_dbar']:.4f} dbar")
                    else:
                        self.get_logger().warn("Parsing du paquet échoué")

                conn.close()

            except serial.SerialException as e:
                self.get_logger().error(f"Erreur série : {e} — reconnexion dans 10 s")
                self._publish({'status': 'error', 'error': str(e)})
                self._sleep(10.0)
            except Exception as e:
                self.get_logger().error(f"Erreur inattendue : {e}")
                self._sleep(5.0)

    def destroy_node(self) -> None:
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AquadoppNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
