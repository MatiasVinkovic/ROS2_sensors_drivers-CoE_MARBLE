#!/usr/bin/env python3
"""
ROS 2 node for the Oculus imaging sonar.

Publishes:
    oculus/image      (sensor_msgs/Image, mono8)
    oculus/ping_info   (my_sensor_interfaces/SonarPing)

Subscribes:
    oculus/set_ip      (std_msgs/String, JSON {"ip": "..."})
    oculus/set_config  (std_msgs/String, JSON {"range_m": .., "gain_percent": ..})
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from my_sensor_interfaces.msg import SonarPing

from oculus_sonar.oculus_driver import discover, connect, extract_ping, MSG_PING, MSG_PING2


class OculusNode(Node):
    def __init__(self):
        super().__init__('oculus_node')

        self.declare_parameter('ip', '')
        self.declare_parameter('auto_discover', True)
        self.declare_parameter('tcp_port', 52100)
        self.declare_parameter('udp_port', 52102)
        self.declare_parameter('discover_timeout', 10.0)
        self.declare_parameter('connect_timeout', 15.0)
        self.declare_parameter('connect_retries', 3)
        self.declare_parameter('connect_retry_delay', 5.0)
        self.declare_parameter('read_error_retry_delay', 3.0)
        self.declare_parameter('no_ip_retry_delay', 10.0)
        self.declare_parameter('range_m', 15.0)
        self.declare_parameter('gain_percent', 30.0)
        self.declare_parameter('frame_id', 'oculus')
        self.declare_parameter('enhance_mode', 'auto')
        self.declare_parameter('publish_rate_hz', 0.0)

        self._load_params()

        self._pub_ping_info = self.create_publisher(SonarPing, 'oculus/ping_info', 10)
        self._pub_image = self.create_publisher(Image, 'oculus/image', 10)

        self.create_subscription(String, 'oculus/set_ip', self._cb_set_ip, 10)
        self.create_subscription(String, 'oculus/set_config', self._cb_set_config, 10)

        self._reconnect = False
        self._running = True
        self._last_pub = 0.0

        self.get_logger().info(
            f"Oculus node started -- ip={self._ip or '(auto-discover)'} "
            f"range={self._range_m}m gain={self._gain_percent}% "
            f"enhance_mode={self._enhance_mode}"
        )
        threading.Thread(target=self._loop, daemon=True).start()

    def _load_params(self):
        self._ip = self.get_parameter('ip').value
        self._auto_discover = self.get_parameter('auto_discover').value
        self._tcp_port = int(self.get_parameter('tcp_port').value)
        self._udp_port = int(self.get_parameter('udp_port').value)
        self._discover_timeout = float(self.get_parameter('discover_timeout').value)
        self._connect_timeout = float(self.get_parameter('connect_timeout').value)
        self._connect_retries = int(self.get_parameter('connect_retries').value)
        self._connect_retry_delay = float(self.get_parameter('connect_retry_delay').value)
        self._read_error_retry_delay = float(self.get_parameter('read_error_retry_delay').value)
        self._no_ip_retry_delay = float(self.get_parameter('no_ip_retry_delay').value)
        self._range_m = float(self.get_parameter('range_m').value)
        self._gain_percent = float(self.get_parameter('gain_percent').value)
        self._frame_id = self.get_parameter('frame_id').value
        self._enhance_mode = self.get_parameter('enhance_mode').value
        self._publish_rate = float(self.get_parameter('publish_rate_hz').value)

    def _cb_set_ip(self, msg: String):
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("set_ip: invalid JSON")
            return
        self._ip = cfg.get('ip', self._ip)
        self._reconnect = True
        self.get_logger().info(f"Reconnect requested -> ip={self._ip}")

    def _cb_set_config(self, msg: String):
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("set_config: invalid JSON")
            return
        self._range_m = float(cfg.get('range_m', self._range_m))
        self._gain_percent = float(cfg.get('gain_percent', self._gain_percent))
        if 'enhance_mode' in cfg:
            self._enhance_mode = cfg['enhance_mode']
        self._reconnect = True
        self.get_logger().info(
            f"Reconfig requested -> range={self._range_m}m gain={self._gain_percent}% "
            f"enhance_mode={self._enhance_mode}"
        )

    def _publish(self, info: dict):
        ping_msg = SonarPing()
        ping_msg.frequency_hz = float(info.get('frequency_hz', 0.0))
        ping_msg.temperature_c = float(info.get('temperature_c', 0.0))
        ping_msg.range_m = float(info.get('range_m', 0.0))
        ping_msg.gain_percent = float(info.get('gain_percent', 0.0))
        ping_msg.n_ranges = int(info.get('n_ranges', 0))
        ping_msg.n_beams = int(info.get('n_beams', 0))
        self._pub_ping_info.publish(ping_msg)

        img = info.get('image')
        if img is None:
            return

        from oculus_sonar.oculus_driver import enhance_image
        img_enhanced = enhance_image(img, mode=self._enhance_mode)

        ros_img = Image()
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = self._frame_id
        ros_img.height = img_enhanced.shape[0]
        ros_img.width = img_enhanced.shape[1]
        ros_img.encoding = 'mono8'
        ros_img.is_bigendian = False
        ros_img.step = img_enhanced.shape[1]
        ros_img.data = img_enhanced.tobytes()
        self._pub_image.publish(ros_img)

    def _loop(self):
        while self._running:
            try:
                self._reconnect = False

                ip = self._ip
                if not ip and self._auto_discover:
                    self.get_logger().info("Searching for Oculus sonar (UDP)...")
                    ip = discover(timeout=self._discover_timeout, udp_port=self._udp_port,
                                  warn_callback=self.get_logger().warn)
                    if not ip:
                        self.get_logger().warn(
                            f"Oculus sonar not found -- retrying in "
                            f"{self._connect_retry_delay}s"
                        )
                        self._sleep(self._connect_retry_delay)
                        continue
                    self.get_logger().info(f"Oculus sonar found: {ip}")
                    self._ip = ip

                if not ip:
                    self.get_logger().error("No IP configured and auto-discover disabled")
                    self._sleep(self._no_ip_retry_delay)
                    continue

                sock, reader = connect(
                    ip,
                    tcp_port=self._tcp_port,
                    timeout=self._connect_timeout,
                    retries=self._connect_retries,
                    range_m=self._range_m,
                    gain_percent=self._gain_percent,
                    warn_callback=self.get_logger().warn,
                )
                self.get_logger().info(
                    f"Connected to {ip} -- range={self._range_m}m gain={self._gain_percent}%"
                )

                while self._running and not self._reconnect:
                    try:
                        msg_id, payload = reader.read_message()
                    except Exception as e:
                        self.get_logger().warn(f"Read error: {e}")
                        break

                    if msg_id in (MSG_PING, MSG_PING2):
                        info = extract_ping(payload, warn_callback=self.get_logger().warn)
                        if info:
                            if self._publish_rate > 0:
                                now = time.time()
                                if now - self._last_pub < 1.0 / self._publish_rate:
                                    continue
                                self._last_pub = now
                            self._publish(info)

                sock.close()

            except Exception as e:
                self.get_logger().error(f"Oculus error: {e}")
                self._sleep(self._read_error_retry_delay)

    def _sleep(self, seconds: float):
        for _ in range(int(seconds / 0.1)):
            if not self._running or self._reconnect:
                return
            time.sleep(0.1)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OculusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()