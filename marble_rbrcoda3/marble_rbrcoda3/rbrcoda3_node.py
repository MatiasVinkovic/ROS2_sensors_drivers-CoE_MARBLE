#!/usr/bin/env python3

import json
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

# from marble_sensors_hmi.drivers.rbrcoda3_driver import (
#     ATMOSPHERE_DBAR,
#     SEAWATER_DENSITY,
#     cmd,
#     depth,
#     parse_stream,
#     read_info,
#     sea_pressure,
#     wakeup,
# )

from rbrcoda3.rbrcoda3_driver import (
    ATMOSPHERE_DBAR,
    SEAWATER_DENSITY,
    cmd,
    depth,
    parse_stream,
    read_info,
    sea_pressure,
    wakeup,
)


_STREAM_TIMEOUT_S = 5.0

_FIELD_FMT = {
    'temperature': ('.4f', ''),
    'pressure_dbar': ('.3f', 'dbar'),
    'sea_pressure_dbar': ('.3f', 'dbar'),
    'depth_m': ('.3f', 'm'),
}


class RBRcoda3Node(Node):
    def __init__(self):
        super().__init__('rbrcoda3_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('sample_interval', 0.0)
        self.declare_parameter('atmosphere_dbar', ATMOSPHERE_DBAR)
        self.declare_parameter('density_kg_m3', SEAWATER_DENSITY)
        self.declare_parameter('temperature_unit', 'C')

        self._port = self.get_parameter('port').value
        self._baud = int(self.get_parameter('baudrate').value)
        self._interval = float(self.get_parameter('sample_interval').value)
        self._atmosphere = float(self.get_parameter('atmosphere_dbar').value)
        self._density = float(self.get_parameter('density_kg_m3').value)
        self._temperature_unit = str(self.get_parameter('temperature_unit').value).upper()

        if self._temperature_unit not in ('C', 'F'):
            self.get_logger().warn("temperature_unit invalide, utilisation de 'C'")
            self._temperature_unit = 'C'

        self._pub = self.create_publisher(String, 'rbrcoda3/data', 10)

        self.get_logger().info(
            f"RBRcoda3 node démarré — port={self._port} baudrate={self._baud} temp_unit={self._temperature_unit}"
        )

        self._reconnect = False
        self._running = True
        self._info = {}
        self._channels = []
        self._count = 0

        threading.Thread(target=self._loop, daemon=True).start()

    def _sleep(self, seconds: float) -> None:
        steps = max(1, int(seconds / 0.1))
        for _ in range(steps):
            if not self._running or self._reconnect:
                return
            time.sleep(0.1)

    def _publish(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

    @staticmethod
    def _fmt_num(key: str, val, unit_override=None) -> dict:
        fmt_str, default_unit = _FIELD_FMT[key]
        unit = unit_override if unit_override is not None else default_unit
        if val is None:
            return {'value': None, 'unit': unit, 'display': 'N/A'}
        return {'value': val, 'unit': unit, 'display': f"{val:{fmt_str}}"}

    @staticmethod
    def _fmt_txt(val, unit: str = '') -> dict:
        return {
            'value': val,
            'unit': unit,
            'display': str(val) if val not in (None, '') else '---'
        }

    def _convert_temperature(self, temp_c: float):
        if self._temperature_unit == 'F':
            return round((temp_c * 9.0 / 5.0) + 32.0, 4), '°F'
        return round(temp_c, 4), '°C'

    def _build_fields(self, ts: str, values: list) -> dict:
        fields = {}
        names = [name.lower() for name, _ in self._channels]

        for i, val in enumerate(values):
            name = names[i] if i < len(names) else ''

            if 'temp' in name or (not name and i == 0):
                temp_value, temp_unit = self._convert_temperature(val)
                fields['temperature'] = self._fmt_num('temperature', temp_value, temp_unit)

            elif 'pres' in name or (not name and i == 1):
                fields['pressure_dbar'] = self._fmt_num('pressure_dbar', round(val, 3))

            else:
                unit = self._channels[i][1] if i < len(self._channels) else ''
                fields[name or f'channel_{i + 1}'] = self._fmt_txt(round(val, 4), unit)

        if 'pressure_dbar' in fields:
            p_sea = sea_pressure(fields['pressure_dbar']['value'], self._atmosphere)
            fields['sea_pressure_dbar'] = self._fmt_num('sea_pressure_dbar', round(p_sea, 3))
            fields['depth_m'] = self._fmt_num('depth_m', round(depth(p_sea, self._density), 3))

        fields['model'] = self._fmt_txt(self._info.get('model'))
        fields['serial'] = self._fmt_txt(self._info.get('serial'))
        fields['firmware'] = self._fmt_txt(self._info.get('firmware'))
        fields['mode'] = self._fmt_txt(self._info.get('mode'))
        fields['period_ms'] = self._fmt_txt(self._info.get('period_ms'), 'ms')

        if ts.isdigit():
            fields['sample_time'] = self._fmt_txt(f"{int(ts) / 1000:.1f}", 's')
        else:
            fields['sample_time'] = self._fmt_txt(ts)

        fields['sample_count'] = self._fmt_txt(self._count)

        return fields

    def _loop(self) -> None:
        while self._running:
            try:
                self._reconnect = False

                conn = serial.Serial(
                    self._port,
                    self._baud,
                    bytesize=8,
                    parity='N',
                    stopbits=1,
                    timeout=1.0,
                )

                self.get_logger().info("Port série ouvert")

                wakeup(conn)
                self._info = read_info(conn)
                self._channels = self._info.get('channels', [])
                self._count = 0

                self.get_logger().info(
                    f"Capteur : model={self._info.get('model') or '?'} "
                    f"serial={self._info.get('serial') or '?'} "
                    f"fw={self._info.get('firmware') or '?'} "
                    f"mode={self._info.get('mode') or '?'} "
                    f"période={self._info.get('period_ms') or '?'} ms "
                    f"voies={self._channels or '?'}"
                )

                last_pub = 0.0
                last_data = time.time()

                while self._running and not self._reconnect:
                    line = conn.readline().decode('ascii', errors='ignore').strip()
                    sample = parse_stream(line) if line else None

                    if sample is None:
                        if time.time() - last_data > _STREAM_TIMEOUT_S:
                            self.get_logger().warn("Pas de flux — tentative fetch")
                            wakeup(conn)
                            response = cmd(conn, 'fetch', wait=3.0)
                            sample = parse_stream(response.replace('fetch', '').strip())

                            if sample is None:
                                self._publish({'status': 'no_data'})
                                last_data = time.time()
                                continue
                        else:
                            continue

                    ts, values = sample
                    self._count += 1
                    last_data = time.time()

                    if self._interval > 0 and time.time() - last_pub < self._interval:
                        continue

                    last_pub = time.time()

                    self._publish({
                        'status': 'ok',
                        'timestamp': time.strftime('%H:%M:%S'),
                        'fields': self._build_fields(ts, values),
                    })

                conn.close()

            except serial.SerialException as e:
                self.get_logger().error(f"Erreur série : {e} — reconnexion dans 10 s")
                self._publish({'status': 'error', 'error': str(e)})
                self._sleep(10.0)

            except Exception as e:
                self.get_logger().error(f"Erreur inattendue : {e}")
                self._publish({'status': 'error', 'error': str(e)})
                self._sleep(5.0)

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
