#!/usr/bin/env python3
"""
ROS2 Jazzy driver node for Airmar 150WXRS WeatherStation
Serial: RS232, 4800 baud, 8N1

Topics published:
  /airmar/wind          [geometry_msgs/Vector3]  direction (deg), speed (knots), speed (m/s)
  /airmar/weather       [sensor_msgs/FluidPressure + std_msgs/Float32MultiArray]
  /airmar/heading       [std_msgs/Float32]        magnetic heading (deg)
  /airmar/orientation   [geometry_msgs/Vector3]   pitch, roll (deg)
  /airmar/rain          [std_msgs/Float32MultiArray] amount(mm), duration(s), rate(mm/h)
  /airmar/gps           [sensor_msgs/NavSatFix]   GPS fix (when available)
  /airmar/pressure      [sensor_msgs/FluidPressure]
  /airmar/temperature   [sensor_msgs/Temperature]
  /airmar/humidity      [std_msgs/Float32]
"""

import serial
import pynmea2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix, FluidPressure, Temperature


class Airmar150WXRSNode(Node):

    def __init__(self):
        super().__init__('airmar_150wxrs')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 4800)

        port     = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Publishers
        self.pub_wind        = self.create_publisher(Vector3,           '/airmar/wind',        10)
        self.pub_heading     = self.create_publisher(Float32,           '/airmar/heading',     10)
        self.pub_orientation = self.create_publisher(Vector3,           '/airmar/orientation', 10)
        self.pub_rain        = self.create_publisher(Float32MultiArray, '/airmar/rain',        10)
        self.pub_pressure    = self.create_publisher(FluidPressure,     '/airmar/pressure',    10)
        self.pub_temperature = self.create_publisher(Temperature,       '/airmar/temperature', 10)
        self.pub_humidity    = self.create_publisher(Float32,           '/airmar/humidity',    10)
        self.pub_gps         = self.create_publisher(NavSatFix,         '/airmar/gps',         10)

        # Serial connection
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'Connected to {port} @ {baudrate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # Read loop timer (10 Hz)
        self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        try:
            raw = self.ser.readline().decode('ascii', errors='replace').strip()
            if not raw.startswith('$'):
                return
            self.parse_nmea(raw)
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')

    def parse_nmea(self, raw: str):
        try:
            msg = pynmea2.parse(raw)
        except pynmea2.ParseError:
            return

        sentence = msg.sentence_type

        # --- Wind ($WIMWV) ---
        if sentence == 'MWV':
            try:
                direction = float(msg.wind_angle)
                speed_kn  = float(msg.wind_speed)
                speed_ms  = speed_kn * 0.514444
                wind_msg = Vector3()
                wind_msg.x = direction   # degrees
                wind_msg.y = speed_kn    # knots
                wind_msg.z = speed_ms    # m/s
                self.pub_wind.publish(wind_msg)
            except (ValueError, AttributeError):
                pass

        # --- Heading ($HCHDG) ---
        elif sentence == 'HDG':
            try:
                heading = float(msg.heading)
                h_msg = Float32()
                h_msg.data = heading
                self.pub_heading.publish(h_msg)
            except (ValueError, AttributeError):
                pass

        # --- Meteo ($WIMDA): pressure, temp, humidity ---
        elif sentence == 'MDA':
            try:
                pressure_bar = float(msg.b_pressure_bar)
                p_msg = FluidPressure()
                p_msg.header.stamp = self.get_clock().now().to_msg()
                p_msg.fluid_pressure = pressure_bar * 1e5  # Pa
                self.pub_pressure.publish(p_msg)
            except (ValueError, AttributeError):
                pass
            try:
                temp_c = float(msg.air_temp)
                t_msg = Temperature()
                t_msg.header.stamp = self.get_clock().now().to_msg()
                t_msg.temperature = temp_c
                self.pub_temperature.publish(t_msg)
            except (ValueError, AttributeError):
                pass
            try:
                humidity = float(msg.rel_humidity)
                hu_msg = Float32()
                hu_msg.data = humidity
                self.pub_humidity.publish(hu_msg)
            except (ValueError, AttributeError):
                pass

        # --- Pitch & Roll ($YXXDR) ---
        elif sentence == 'XDR':
            try:
                data = msg.data
                pitch, roll = None, None
                i = 0
                while i < len(data) - 3:
                    if data[i+3] == 'PTCH':
                        pitch = float(data[i+1])
                    elif data[i+3] == 'ROLL':
                        roll = float(data[i+1])
                    i += 4
                if pitch is not None and roll is not None:
                    o_msg = Vector3()
                    o_msg.x = pitch
                    o_msg.y = roll
                    self.pub_orientation.publish(o_msg)
            except (ValueError, AttributeError, IndexError):
                pass

        # --- Rain ($WIXDR) ---
        elif sentence == 'IXDR' or (sentence == 'XDR' and 'RAIN' in raw):
            try:
                data = msg.data
                amount, duration, rate = 0.0, 0.0, 0.0
                i = 0
                while i < len(data) - 3:
                    if data[i+3] == 'RAIN':
                        amount = float(data[i+1])
                    elif data[i+3] == 'DURA':
                        duration = float(data[i+1])
                    elif data[i+3] == 'RATE':
                        rate = float(data[i+1])
                    i += 4
                r_msg = Float32MultiArray()
                r_msg.data = [amount, duration, rate]
                self.pub_rain.publish(r_msg)
            except (ValueError, AttributeError, IndexError):
                pass

        # --- GPS ($GPGGA) ---
        elif sentence == 'GGA':
            try:
                if msg.gps_qual > 0:
                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = self.get_clock().now().to_msg()
                    gps_msg.latitude  = msg.latitude
                    gps_msg.longitude = msg.longitude
                    gps_msg.altitude  = float(msg.altitude)
                    self.pub_gps.publish(gps_msg)
            except (ValueError, AttributeError):
                pass

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Airmar150WXRSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
