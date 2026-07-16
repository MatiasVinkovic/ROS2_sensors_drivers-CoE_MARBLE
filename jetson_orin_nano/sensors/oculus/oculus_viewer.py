#!/usr/bin/env python3
"""
Mini IHM Oculus — affiche /oculus/image en rendu radar (eventail + colormap).
A lancer sur le PC, connecte au meme reseau/domaine ROS2 que le Jetson.
"""

import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal, QObject

from marble_sensors_hmi.drivers.oculus_driver import (
    build_fan_lut, make_sonar_colormap, enhance_image
)

FOV_DEG = 130.0
OUT_W = 500
OUT_H = 550
ENHANCE_MODE = 'auto'  # auto, log, sqrt, histeq, raw


class ImageBridge(QObject):
    new_frame = pyqtSignal(np.ndarray, int, int)


class OculusSubNode(Node):
    def __init__(self, bridge: ImageBridge):
        super().__init__('oculus_mini_viewer')
        self.bridge = bridge
        self.create_subscription(Image, '/oculus/image', self._cb, 10)

    def _cb(self, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        self.bridge.new_frame.emit(img.copy(), msg.height, msg.width)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Oculus — Mini viewer")
        self.resize(OUT_W, OUT_H + 40)

        self.label = QLabel("En attente d'images...")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("background-color: black; color: white;")

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.addWidget(self.label)
        self.setCentralWidget(container)

        self.colormap = make_sonar_colormap()
        self._lut_cache_key = None
        self._range_idx = None
        self._beam_idx = None
        self._valid = None

    def update_frame(self, img: np.ndarray, n_ranges: int, n_beams: int):
        img = enhance_image(img, mode=ENHANCE_MODE)

        key = (n_ranges, n_beams)
        if key != self._lut_cache_key:
            self._range_idx, self._beam_idx, self._valid = build_fan_lut(
                n_ranges, n_beams, FOV_DEG, OUT_W, OUT_H
            )
            self._lut_cache_key = key

        fan = np.zeros((OUT_H, OUT_W), dtype=np.uint8)
        fan[self._valid] = img[self._range_idx[self._valid], self._beam_idx[self._valid]]

        rgb = self.colormap[fan]
        rgb = np.ascontiguousarray(rgb)

        qimg = QImage(rgb.data, OUT_W, OUT_H, OUT_W * 3, QImage.Format_RGB888)
        self.label.setPixmap(QPixmap.fromImage(qimg))


def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()

    app = QApplication(sys.argv)
    window = MainWindow()

    bridge = ImageBridge()
    bridge.new_frame.connect(window.update_frame)

    node = OculusSubNode(bridge)
    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    window.show()
    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()