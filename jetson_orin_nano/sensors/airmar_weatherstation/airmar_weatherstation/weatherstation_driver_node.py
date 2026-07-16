#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_sensor_interfaces.msg import Wind, Rain
from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure, Imu, NavSatFix, NavSatStatus
from airmar_weatherstation.airmar_weatherstation_driver import AirmarWeatherstationDriver


class WeatherstationDriverNode(Node):
    def __init__(self):
        super().__init__("weatherstation_driver")

        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("baud_rate", 4800)
        self.declare_parameter("frame_id", "weatherstation")

        serial_port_name = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value
        self.frame_id_ = self.get_parameter("frame_id").value

        self.driver_ = AirmarWeatherstationDriver(
            port=serial_port_name, baud_rate=baud_rate, warn_callback=self.get_logger().warn)
        self.driver_.on_wind(self.publish_wind)
        self.driver_.on_rain(self.publish_rain)
        self.driver_.on_environmental(self.publish_environmental)
        self.driver_.on_orientation(self.publish_imu)
        self.driver_.on_gps(self.publish_gps)

        self.wind_publisher_ = self.create_publisher(Wind, "wind", 10)
        self.rain_publisher_ = self.create_publisher(Rain, "rain", 10)
        self.temperature_publisher_ = self.create_publisher(Temperature, "temperature", 10)
        self.humidity_publisher_ = self.create_publisher(RelativeHumidity, "humidity", 10)
        self.pressure_publisher_ = self.create_publisher(FluidPressure, "pressure", 10)
        self.imu_publisher_ = self.create_publisher(Imu, "imu", 10)
        self.gps_publisher_ = self.create_publisher(NavSatFix, "gps/fix", 10)

        self.read_timer_ = self.create_timer(0.05, self.driver_.poll)

        self.get_logger().info(
            "Weatherstation driver has been started on " + serial_port_name +
            " at " + str(baud_rate) + " baud.")

    def publish_wind(self, reading):
        msg = Wind()
        msg.direction_deg = reading.direction_deg
        msg.reference = reading.reference
        msg.speed_mps = reading.speed_mps
        msg.valid = reading.valid
        self.wind_publisher_.publish(msg)

    def publish_rain(self, reading):
        msg = Rain()
        msg.amount_mm = reading.amount_mm
        msg.duration_sec = reading.duration_sec
        msg.rate_mmph = reading.rate_mmph
        msg.peak_rate_mmph = reading.peak_rate_mmph
        self.rain_publisher_.publish(msg)

    def publish_environmental(self, reading):
        now = self.get_clock().now().to_msg()

        if reading.pressure_pa is not None:
            msg = FluidPressure()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id_
            msg.fluid_pressure = reading.pressure_pa
            msg.variance = 0.0
            self.pressure_publisher_.publish(msg)

        if reading.temperature_c is not None:
            msg = Temperature()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id_
            msg.temperature = reading.temperature_c
            msg.variance = 0.0
            self.temperature_publisher_.publish(msg)

        if reading.humidity_ratio is not None:
            msg = RelativeHumidity()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id_
            msg.relative_humidity = reading.humidity_ratio
            msg.variance = 0.0
            self.humidity_publisher_.publish(msg)

    def publish_imu(self, reading):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id_

        qx, qy, qz, qw = reading.quaternion
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.imu_publisher_.publish(msg)

    def publish_gps(self, reading):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id_

        msg.status.status = NavSatStatus.STATUS_FIX if reading.has_fix else NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = reading.latitude
        msg.longitude = reading.longitude
        msg.altitude = reading.altitude

        self.gps_publisher_.publish(msg)

    def destroy_node(self):
        self.driver_.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WeatherstationDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()