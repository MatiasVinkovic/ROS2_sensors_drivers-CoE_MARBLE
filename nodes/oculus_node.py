#!/usr/bin/env python3
"""
ROS 2 node — Oculus M750d imaging sonar.

Publie sur :
  /oculus/data   (std_msgs/String, JSON avec metadata + image base64)
  /oculus/image  (sensor_msgs/Image, mono8)

Souscrit a :
  /oculus/set_ip (std_msgs/String, JSON {"ip": "..."}) pour reconnexion
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

import socket
import struct
import time
import json
import threading
import base64
import numpy as np

from marble_sensors_hmi.drivers.oculus_driver import (
    OculusReader, discover, connect, extract_ping,
    MSG_PING, MSG_PING2, HEADER_FMT, OCULUS_ID
)


class OculusNode(Node):

    def __init__(self):
        super().__init__('oculus_node')

        self.declare_parameter('ip', '')
        self.declare_parameter('auto_discover', True)

        self._ip = self.get_parameter('ip').value
        self._auto_discover = self.get_parameter('auto_discover').value

        self._pub_data = self.create_publisher(String, 'oculus/data', 10)
        self._pub_image = self.create_publisher(Image, 'oculus/image', 10)

        self.create_subscription(String, 'oculus/set_ip', self._cb_set_ip, 10)

        self._reconnect = False
        self._running = True
        self.get_logger().info(f"Oculus node demarre — ip={self._ip or '(auto-discover)'}")
        threading.Thread(target=self._loop, daemon=True).start()

    def _cb_set_ip(self, msg: String):
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._ip = cfg.get('ip', self._ip)
        self._reconnect = True
        self.get_logger().info(f"Reconnexion demandee → {self._ip}")

    def _publish(self, info: dict):
        # Publier les metadata en JSON
        meta = {
            'status': 'ok',
            'timestamp': time.strftime('%H:%M:%S'),
            'fields': {}
        }
        field_map = {
            'frequency_hz': ('Frequence', 'Hz'),
            'temperature_c': ('Temperature', 'C'),
            'range_m': ('Portee', 'm'),
            'pressure_bar': ('Pression', 'bar'),
            'gain_percent': ('Gain', '%'),
            'n_ranges': ('Ranges', ''),
            'n_beams': ('Beams', ''),
        }
        for key, (label, unit) in field_map.items():
            val = info.get(key)
            if val is not None:
                if isinstance(val, float):
                    display = f"{val:.0f}" if abs(val) > 1000 else f"{val:.2f}"
                else:
                    display = str(val)
                meta['fields'][key] = {'display': display, 'unit': unit}

        img = info.get('image')
        if img is not None:
            meta['image_b64'] = base64.b64encode(img.tobytes()).decode('ascii')
            meta['image_shape'] = list(img.shape)

        msg = String()
        msg.data = json.dumps(meta)
        self._pub_data.publish(msg)

        # Publier l'image ROS
        if img is not None:
            ros_img = Image()
            ros_img.header.stamp = self.get_clock().now().to_msg()
            ros_img.header.frame_id = 'oculus'
            ros_img.height = img.shape[0]
            ros_img.width = img.shape[1]
            ros_img.encoding = 'mono8'
            ros_img.is_bigendian = False
            ros_img.step = img.shape[1]
            ros_img.data = img.tobytes()
            self._pub_image.publish(ros_img)

    def _loop(self):
        while self._running:
            try:
                self._reconnect = False

                # Decouverte IP si pas configuree
                ip = self._ip
                if not ip and self._auto_discover:
                    self.get_logger().info("Recherche du sonar Oculus (UDP)...")
                    ip = discover(timeout=10.0)
                    if not ip:
                        self.get_logger().warn("Sonar Oculus non trouve — nouvel essai dans 5s")
                        self._sleep(5)
                        continue
                    self.get_logger().info(f"Sonar Oculus trouve: {ip}")
                    self._ip = ip

                if not ip:
                    self.get_logger().error("Pas d'IP configuree et auto-discover desactive")
                    self._sleep(10)
                    continue

                sock, reader = connect(ip)
                self.get_logger().info(f"Connecte a {ip} — reception des pings...")

                while self._running and not self._reconnect:
                    try:
                        msg_id, payload = reader.read_message()
                    except socket.timeout:
                        continue
                    except Exception as e:
                        self.get_logger().warn(f"Erreur lecture: {e}")
                        break

                    if msg_id in (MSG_PING, MSG_PING2):
                        info = extract_ping(payload)
                        if info:
                            self._publish(info)

                sock.close()

            except Exception as e:
                self.get_logger().error(f"Erreur Oculus: {e}")
                self._sleep(3)

    def _sleep(self, seconds):
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
