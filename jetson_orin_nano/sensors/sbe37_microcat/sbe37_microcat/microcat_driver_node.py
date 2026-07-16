#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_sensor_interfaces.msg import CtdReading as CtdReadingMsg
from example_interfaces.srv import SetBool
from std_srvs.srv import Trigger
from sbe37_microcat.sbe37_microcat_driver import SBE37MicrocatDriver


class MicrocatDriverNode(Node):
    def __init__(self):
        super().__init__("microcat_driver")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("sample_period", 4.0)

        serial_port_name = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value
        sample_period = self.get_parameter("sample_period").value

        self.driver_ = SBE37MicrocatDriver(
            port=serial_port_name,
            baud_rate=baud_rate,
            sample_period=sample_period,
            warn_callback=self.get_logger().warn
        )

        self.ctd_publisher_ = self.create_publisher(CtdReadingMsg, "ctd", 10)

        self.enable_sampling_service_ = self.create_service(
            SetBool, "enable_sampling", self.callback_enable_sampling)
        self.take_sample_service_ = self.create_service(
            Trigger, "take_sample_now", self.callback_take_sample_now)

        self.sample_timer_ = self.create_timer(sample_period, self.request_sample)

        self.get_logger().info(
            "MicroCAT driver has been started on " + serial_port_name +
            " at " + str(baud_rate) + " baud, sampling every " +
            str(sample_period) + "s.")

    def request_sample(self):
        if not self.driver_.sampling_enabled:
            return
        reading = self.driver_.take_sample()
        if reading is not None:
            self.publish_reading(reading)

    def publish_reading(self, reading):
        msg = CtdReadingMsg()
        msg.temperature_c = reading.temperature_c
        msg.conductivity_spm = reading.conductivity_spm
        msg.pressure_dbar = reading.pressure_dbar
        msg.timestamp = reading.timestamp
        msg.salinity_psu = reading.salinity_psu
        msg.sound_velocity_mps = reading.sound_velocity_mps
        msg.density_kgm3 = reading.density_kgm3
        self.ctd_publisher_.publish(msg)

    def callback_enable_sampling(self, request: SetBool.Request, response: SetBool.Response):
        self.driver_.enable_sampling(request.data)
        response.success = True
        response.message = "Periodic sampling " + ("enabled" if request.data else "paused")
        self.get_logger().info(response.message)
        return response

    def callback_take_sample_now(self, request: Trigger.Request, response: Trigger.Response):
        reading = self.driver_.take_sample()
        if reading is not None:
            self.publish_reading(reading)
            response.success = True
            response.message = "Sample taken"
        else:
            response.success = False
            response.message = "No response from sensor"
        return response

    def destroy_node(self):
        self.driver_.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MicrocatDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()