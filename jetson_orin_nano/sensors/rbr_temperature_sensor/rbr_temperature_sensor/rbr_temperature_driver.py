#!/usr/bin/env python3
import time
import serial


class RBRTemperatureDriver:
    """
    Standalone driver for the RBR coda3 temperature sensor.
    No ROS 2 dependency -- just pyserial. Usable in any Python script.
    """

    def __init__(self, port="/dev/ttyUSB0", baud_rate=9600, timeout=1.0, warn_callback=None):
        self.port_name = port
        self.baud_rate = baud_rate
        self.streaming_enabled = True
        self._warn = warn_callback or (lambda msg: None)
        self.serial_port = None

        try:
            self.serial_port = serial.Serial(port=port, baudrate=baud_rate, timeout=timeout)
        except serial.SerialException as e:
            self._warn(f"Could not open serial port '{port}': {e}")

    def read_available(self):
        if self.serial_port is None:
            return None

        if self.serial_port.in_waiting == 0:
            return None

        raw_line = self.serial_port.readline()
        decoded_line = raw_line.decode("utf-8", errors="ignore").strip()

        if not decoded_line:
            return None

        temperature_value = self.parse_line(decoded_line)

        if temperature_value is None or not self.streaming_enabled:
            return None

        return temperature_value

    def run_forever(self, callback, poll_interval=0.05):
        """
        Blocking loop for standalone scripts: repeatedly checks for new
        data and calls callback(temperature_value) whenever a valid
        reading arrives. Runs until interrupted (e.g. Ctrl+C).
        """
        try:
            while True:
                value = self.read_available()
                if value is not None:
                    callback(value)
                time.sleep(poll_interval)
        except KeyboardInterrupt:
            pass

    @staticmethod
    def parse_line(line: str):
        """Parses a raw 'elapsed_ms, temperature' line into a float, or None if malformed."""
        parts = line.split(",")
        if len(parts) != 2:
            return None
        try:
            return float(parts[1].strip())
        except ValueError:
            return None

    def close(self):
        if self.serial_port:
            self.serial_port.close()