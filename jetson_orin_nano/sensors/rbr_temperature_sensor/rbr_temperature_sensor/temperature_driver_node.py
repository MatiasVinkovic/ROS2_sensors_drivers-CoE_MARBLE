#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from example_interfaces.srv import SetBool
from rbr_temperature_sensor.rbr_temperature_driver import RBRTemperatureDriver


class TemperatureDriverNode(Node):
    def __init__(self):
        super().__init__("temperature_driver")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("frame_id", "rbr_coda3_sensor")

        serial_port_name = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value
        self.frame_id_ = self.get_parameter("frame_id").value

        self.driver_ = RBRTemperatureDriver(
            port=serial_port_name, baud_rate=baud_rate, warn_callback=self.get_logger().warn)

        self.temperature_publisher_ = self.create_publisher(
            Temperature, "temperature", 10)
        self.enable_streaming_service_ = self.create_service(
            SetBool, "enable_streaming", self.callback_enable_streaming)
        self.read_timer_ = self.create_timer(0.1, self.read_serial_line)

        self.get_logger().info(
            "Temperature driver has been started on " + serial_port_name +
            " at " + str(baud_rate) + " baud.")

    def read_serial_line(self):
        temperature_value = self.driver_.read_available()
        if temperature_value is not None:
            self.publish_temperature(temperature_value)

    def publish_temperature(self, temperature_value: float):
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id_
        msg.temperature = temperature_value
        msg.variance = 0.0
        self.temperature_publisher_.publish(msg)

    def callback_enable_streaming(self, request: SetBool.Request, response: SetBool.Response):
        self.driver_.streaming_enabled = request.data
        response.success = True
        response.message = "Streaming " + ("enabled" if request.data else "paused")
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        self.driver_.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()