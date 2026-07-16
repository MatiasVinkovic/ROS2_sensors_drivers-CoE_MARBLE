#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from my_sensor_interfaces.msg import WaveStats, SystemStatus
from motus_wave_sensor.motus_wave_driver import MotusWaveDriver


class MotusDriverNode(Node):
    def __init__(self):
        super().__init__("motus_driver")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("frame_id", "motus_wave_sensor")
        self.declare_parameter("silence_gap_sec", 1.0)

        serial_port_name = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value
        self.frame_id_ = self.get_parameter("frame_id").value
        silence_gap_sec = self.get_parameter("silence_gap_sec").value

        self.driver_ = MotusWaveDriver(
            port=serial_port_name,
            baud_rate=baud_rate,
            silence_gap_sec=silence_gap_sec,
            warn_callback=self.get_logger().warn,
            info_callback=self.get_logger().info,
        )
        self.driver_.on_orientation(self.publish_imu)
        self.driver_.on_wave_stats(self.publish_wave_stats)
        self.driver_.on_system_status(self.publish_system_status)

        self.imu_publisher_ = self.create_publisher(Imu, "imu", 10)
        self.wave_stats_publisher_ = self.create_publisher(WaveStats, "wave_stats", 10)
        self.system_status_publisher_ = self.create_publisher(SystemStatus, "system_status", 10)

        self.check_timer_ = self.create_timer(1.0, self.driver_.poll)

        self.get_logger().info(
            "MOTUS wave driver has been started on " + serial_port_name +
            " at " + str(baud_rate) + " baud.")

    def publish_imu(self, reading):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id_

        qx, qy, qz, qw = reading.quaternion
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.orientation_covariance[0] = math.radians(reading.roll_stdev_deg) ** 2
        msg.orientation_covariance[4] = math.radians(reading.pitch_stdev_deg) ** 2
        msg.orientation_covariance[8] = math.radians(reading.yaw_stdev_deg) ** 2

        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.imu_publisher_.publish(msg)

    def publish_wave_stats(self, reading):
        msg = WaveStats()
        msg.significant_wave_height_hm0 = reading.significant_wave_height_hm0
        msg.wave_height_wind_hm0 = reading.wave_height_wind_hm0
        msg.wave_height_swell_hm0 = reading.wave_height_swell_hm0
        msg.significant_wave_height_h13 = reading.significant_wave_height_h13
        msg.wave_height_hmax = reading.wave_height_hmax
        msg.wave_height_crest = reading.wave_height_crest
        msg.wave_height_trough = reading.wave_height_trough
        msg.wave_period_tz = reading.wave_period_tz
        msg.wave_period_tmax = reading.wave_period_tmax
        msg.wave_mean_period_tm02 = reading.wave_mean_period_tm02
        msg.wave_peak_period = reading.wave_peak_period
        msg.wave_peak_period_wind = reading.wave_peak_period_wind
        msg.wave_peak_period_swell = reading.wave_peak_period_swell
        msg.wave_peak_direction = reading.wave_peak_direction
        msg.wave_peak_direction_wind = reading.wave_peak_direction_wind
        msg.wave_peak_direction_swell = reading.wave_peak_direction_swell
        msg.wave_mean_direction = reading.wave_mean_direction
        msg.mean_spreading_angle = reading.mean_spreading_angle
        msg.first_order_spread = reading.first_order_spread
        msg.long_crestedness_parameter = reading.long_crestedness_parameter
        self.wave_stats_publisher_.publish(msg)

    def publish_system_status(self, reading):
        msg = SystemStatus()
        msg.input_voltage = reading.input_voltage
        msg.input_current_ma = reading.input_current_ma
        msg.memory_used_bytes = reading.memory_used_bytes
        self.system_status_publisher_.publish(msg)

    def destroy_node(self):
        self.driver_.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotusDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()