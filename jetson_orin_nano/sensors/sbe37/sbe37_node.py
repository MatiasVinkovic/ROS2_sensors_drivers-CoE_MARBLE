#!/usr/bin/env python3
import json
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

from sbe37.sbe37_driver import (
    wakeup, cmd, parse_ts, salinity, sound_velocity
)

_MIN_CONDUCTIVITY_SM = 0.001

_FIELD_FMT = {
    'temperature_c': ('.4f', '°C'),
    'conductivity_sm': ('.6f', 'S/m'),
    'pressure_dbar': ('.4f', 'dbar'),
    'salinity_psu': ('.4f', 'PSU'),
    'sound_velocity_ms': ('.2f', 'm/s'),
    'depth_m': ('.3f', 'm'),
}


class SBE37Node(Node):
    def __init__(self):
        super().__init__('sbe37_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('sample_interval', 10.0)

        self._port = self.get_parameter('port').value
        self._baud = int(self.get_parameter('baud').value)
        self._interval = float(self.get_parameter('sample_interval').value)

        self._pub = self.create_publisher(String, 'sbe37/data', 10)
        self.get_logger().info(
            f"SBE37 node démarré — port={self._port} baud={self._baud} interval={self._interval} s"
        )

        self._reconnect = False
        self.create_subscription(String, 'sbe37/set_port', self._cb_set_port, 10)

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
        for _ in range(max(1, int(seconds / 0.1))):
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
                    self._port,
                    self._baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=3.0,
                )
                self.get_logger().info("Port série ouvert")
                wakeup(conn)
                self.get_logger().info(f"DS → {cmd(conn, 'DS', wait=2.0)[:80]}")

                while self._running and not self._reconnect:
                    raw = cmd(conn, 'TS', wait=3.0)
                    data = parse_ts(raw)

                    if data is None:
                        self.get_logger().warn(f"Parsing échoué : {repr(raw[:60])}")
                        self._publish({'status': 'error', 'error': f'parsing échoué : {raw[:40]}'})
                        wakeup(conn)
                    else:
                        T, C, P = data['T'], data['C'], data['P']
                        in_water = C >= _MIN_CONDUCTIVITY_SM
                        sal = salinity(max(0.0, C), T, P) if in_water else None
                        sos = sound_velocity(T, sal, P) if in_water else None

                        if not in_water:
                            self.get_logger().warn(f"Conductivité trop faible ({C:.6f} S/m) — hors eau")

                        def _fmt(key, val):
                            fmt_str, unit = _FIELD_FMT[key]
                            if val is None:
                                return {'value': None, 'unit': unit, 'display': 'N/A'}
                            return {'value': val, 'unit': unit, 'display': f"{val:{fmt_str}}"}

                        self._publish({
                            'status': 'ok',
                            'timestamp': time.strftime('%H:%M:%S'),
                            'fields': {
                                'temperature_c': _fmt('temperature_c', round(T, 4)),
                                'conductivity_sm': _fmt('conductivity_sm', round(C, 6)),
                                'pressure_dbar': _fmt('pressure_dbar', round(P, 4)),
                                'salinity_psu': _fmt('salinity_psu', round(sal, 4) if sal is not None else None),
                                'sound_velocity_ms': _fmt('sound_velocity_ms', round(sos, 2) if sos is not None else None),
                                'depth_m': _fmt('depth_m', round(P * 1.019716, 3)),
                            },
                        })
                        self.get_logger().info(f"TS OK — T={T:.4f} °C C={C:.6f} S/m P={P:.4f} dbar")

                    self._sleep(self._interval)

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
    node = SBE37Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()