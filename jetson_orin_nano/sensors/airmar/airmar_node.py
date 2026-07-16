#!/usr/bin/env python3
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from airmar.airmar_driver import (
    AirmarDriver,
    WindData,
    HeadingData,
    WeatherData,
    OrientationData,
    RainData,
    GPSData,
)


class AirmarNode(Node):
    def __init__(self):
        super().__init__('airmar_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 4800)
        self.declare_parameter('sample_interval', 0.0)

        self._port = self.get_parameter('port').value
        self._baud = int(self.get_parameter('baud').value)
        self._interval = float(self.get_parameter('sample_interval').value)

        self._pub = self.create_publisher(String, 'airmar/data', 10)
        self.get_logger().info(
            f"Airmar node démarré — port={self._port} baud={self._baud} interval={self._interval} s"
        )

        self._reconnect = False
        self.create_subscription(String, 'airmar/set_port', self._cb_set_port, 10)

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
        steps = max(1, int(seconds / 0.1))
        for _ in range(steps):
            if not self._running or self._reconnect:
                return
            time.sleep(0.1)

    def _publish(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

    def _fmt_num(self, val, unit: str, fmt: str) -> dict:
        if val is None:
            return {'value': None, 'unit': unit, 'display': 'N/A'}
        return {'value': val, 'unit': unit, 'display': format(val, fmt)}

    def _build_fields(self, sample) -> dict:
        fields = {}

        if isinstance(sample, WindData):
            fields['wind_direction_deg'] = self._fmt_num(round(sample.direction_deg, 1), '°', '.1f')
            fields['wind_speed_kn'] = self._fmt_num(round(sample.speed_kn, 1), 'kn', '.1f')
            fields['wind_speed_ms'] = self._fmt_num(round(sample.speed_ms, 2), 'm/s', '.2f')

        elif isinstance(sample, HeadingData):
            fields['heading_deg'] = self._fmt_num(round(sample.heading_deg, 1), '°', '.1f')

        elif isinstance(sample, WeatherData):
            if sample.pressure_pa is not None:
                fields['pressure_hpa'] = self._fmt_num(round(sample.pressure_pa / 100.0, 1), 'hPa', '.1f')
            if sample.air_temp_c is not None:
                fields['temperature_c'] = self._fmt_num(round(sample.air_temp_c, 1), '°C', '.1f')
            if sample.humidity_pct is not None:
                fields['humidity_pct'] = self._fmt_num(round(sample.humidity_pct, 1), '%', '.1f')

        elif isinstance(sample, OrientationData):
            fields['pitch_deg'] = self._fmt_num(round(sample.pitch_deg, 2), '°', '.2f')
            fields['roll_deg'] = self._fmt_num(round(sample.roll_deg, 2), '°', '.2f')

        elif isinstance(sample, RainData):
            fields['rain_amount_mm'] = self._fmt_num(round(sample.amount_mm, 2), 'mm', '.2f')
            fields['rain_duration_s'] = self._fmt_num(round(sample.duration_s, 0), 's', '.0f')
            fields['rain_rate_mmh'] = self._fmt_num(round(sample.rate_mm_h, 1), 'mm/h', '.1f')

        elif isinstance(sample, GPSData):
            fields['gps_latitude'] = self._fmt_num(round(sample.latitude, 6), '°', '.6f')
            fields['gps_longitude'] = self._fmt_num(round(sample.longitude, 6), '°', '.6f')
            fields['gps_altitude'] = self._fmt_num(round(sample.altitude_m, 1), 'm', '.1f')

        return fields

    def _loop(self) -> None:
        while self._running:
            try:
                self._reconnect = False
                driver = AirmarDriver(self._port, self._baud)
                driver.open()
                self.get_logger().info(f"Port série ouvert — {self._port} @ {self._baud}")

                last_pub = 0.0
                for sample in driver.stream():
                    if not self._running or self._reconnect:
                        break

                    now = time.time()
                    if self._interval > 0 and now - last_pub < self._interval:
                        continue
                    last_pub = now

                    fields = self._build_fields(sample)
                    if not fields:
                        continue

                    self._publish({
                        'status': 'ok',
                        'timestamp': time.strftime('%H:%M:%S'),
                        'fields': fields,
                    })
                    self.get_logger().info(f"Airmar OK — {list(fields.keys())}")

                driver.close()

            except Exception as e:
                self.get_logger().error(f"Erreur Airmar : {e}")
                self._publish({'status': 'error', 'error': str(e)})
                self._sleep(5.0)

    def destroy_node(self) -> None:
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AirmarNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()