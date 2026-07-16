#!/usr/bin/env python3
import math
import time
import serial
from dataclasses import dataclass


@dataclass
class WindReading:
    direction_deg: float
    reference: str
    speed_mps: float
    valid: bool


@dataclass
class RainReading:
    amount_mm: float
    duration_sec: int
    rate_mmph: float
    peak_rate_mmph: float


@dataclass
class EnvironmentalReading:
    pressure_pa: float = None
    temperature_c: float = None
    humidity_ratio: float = None


@dataclass
class OrientationReading:
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    quaternion: tuple  # (x, y, z, w)


@dataclass
class GpsReading:
    has_fix: bool
    latitude: float
    longitude: float
    altitude: float


class AirmarWeatherstationDriver:
    """
    Standalone driver for the AIRMAR 150WXRS weatherstation.
    No ROS 2 dependency -- just pyserial. Usable in any Python script.

    Register callbacks for whichever measurements you care about, then
    call poll() repeatedly (non-blocking) or run_forever() (blocking).
    """

    def __init__(self, port="/dev/ttyUSB1", baud_rate=4800, timeout=1.0, warn_callback=None):
        self._warn = warn_callback or (lambda msg: None)
        self.serial_port = None

        try:
            self.serial_port = serial.Serial(port=port, baudrate=baud_rate, timeout=timeout)
        except serial.SerialException as e:
            self._warn(f"Could not open serial port '{port}': {e}")

        self.latest_yaw_deg = 0.0
        self.latest_pitch_deg = 0.0
        self.latest_roll_deg = 0.0

        self._on_wind = None
        self._on_rain = None
        self._on_environmental = None
        self._on_orientation = None
        self._on_gps = None

    # --- callback registration -------------------------------------------

    def on_wind(self, callback):
        self._on_wind = callback

    def on_rain(self, callback):
        self._on_rain = callback

    def on_environmental(self, callback):
        self._on_environmental = callback

    def on_orientation(self, callback):
        self._on_orientation = callback

    def on_gps(self, callback):
        self._on_gps = callback

    # --- main loop ----------------------------------------------------------

    def poll(self):
        """Non-blocking: reads and dispatches one line if available."""
        if self.serial_port is None:
            return

        if self.serial_port.in_waiting > 0:
            raw_line = self.serial_port.readline()
            decoded_line = raw_line.decode("ascii", errors="ignore").strip()
            if decoded_line:
                self._route_sentence(decoded_line)

    def run_forever(self, poll_interval=0.05):
        """Blocking loop for standalone scripts. Runs until interrupted."""
        try:
            while True:
                self.poll()
                time.sleep(poll_interval)
        except KeyboardInterrupt:
            pass

    def close(self):
        if self.serial_port:
            self.serial_port.close()

    # --- parsing --------------------------------------------------------

    def _route_sentence(self, sentence: str):
        if not self._is_checksum_valid(sentence):
            return

        if sentence.startswith("$WIMWV"):
            self._parse_wind(sentence)
        elif sentence.startswith("$WIMDA"):
            self._parse_mda(sentence)
        elif sentence.startswith("$WIXDR"):
            self._parse_rain(sentence)
        elif sentence.startswith("$HCHDG"):
            self._parse_heading(sentence)
        elif sentence.startswith("$YXXDR"):
            self._parse_pitch_roll(sentence)
        elif sentence.startswith("$GPGGA"):
            self._parse_gps(sentence)

    def _is_checksum_valid(self, sentence: str) -> bool:
        if "*" not in sentence or not sentence.startswith("$"):
            return False

        data_part, _, checksum_part = sentence[1:].partition("*")
        if len(checksum_part) < 2:
            return False

        calculated = 0
        for char in data_part:
            calculated ^= ord(char)

        try:
            received = int(checksum_part[:2], 16)
        except ValueError:
            return False

        return calculated == received

    def _parse_wind(self, sentence: str):
        body = sentence.split("*")[0]
        fields = body.split(",")
        if len(fields) < 6:
            return

        try:
            direction_deg = float(fields[1])
            reference = fields[2]
            speed_raw = float(fields[3])
            units = fields[4]
            status = fields[5]
        except ValueError:
            return

        speed_mps = self._convert_speed_to_mps(speed_raw, units)
        reading = WindReading(direction_deg, reference, speed_mps, status == "A")

        if self._on_wind:
            self._on_wind(reading)

    def _parse_mda(self, sentence: str):
        body = sentence.split("*")[0]
        fields = body.split(",")
        if len(fields) < 13:
            return

        pressure_pa = None
        temperature_c = None
        humidity_ratio = None

        if fields[3] != "":
            try:
                pressure_pa = float(fields[3]) * 100000.0
            except ValueError:
                pass

        if fields[5] != "":
            try:
                temperature_c = float(fields[5])
            except ValueError:
                pass

        if fields[9] != "":
            try:
                humidity_ratio = float(fields[9]) / 100.0
            except ValueError:
                pass

        reading = EnvironmentalReading(pressure_pa, temperature_c, humidity_ratio)

        if self._on_environmental:
            self._on_environmental(reading)

    def _parse_rain(self, sentence: str):
        body = sentence.split("*")[0]
        fields = body.split(",")
        if len(fields) < 17:
            return

        try:
            amount_m = float(fields[2]) if fields[2] != "" else 0.0
            duration_sec = int(float(fields[6])) if fields[6] != "" else 0
            rate_mmph = float(fields[10]) if fields[10] != "" else 0.0
            peak_rate_mmph = float(fields[14]) if fields[14] != "" else 0.0
        except ValueError:
            return

        reading = RainReading(amount_m * 1000.0, duration_sec, rate_mmph, peak_rate_mmph)

        if self._on_rain:
            self._on_rain(reading)

    def _parse_heading(self, sentence: str):
        body = sentence.split("*")[0]
        fields = body.split(",")
        if len(fields) < 2 or fields[1] == "":
            return

        try:
            self.latest_yaw_deg = float(fields[1])
        except ValueError:
            return

        self._emit_orientation()

    def _parse_pitch_roll(self, sentence: str):
        if "PTCH" not in sentence or "ROLL" not in sentence:
            return

        body = sentence.split("*")[0]
        fields = body.split(",")

        try:
            for i in range(1, len(fields) - 2, 4):
                name = fields[i + 3]
                value = fields[i + 1]
                if name == "PTCH" and value != "":
                    self.latest_pitch_deg = float(value)
                elif name == "ROLL" and value != "":
                    self.latest_roll_deg = float(value)
        except (ValueError, IndexError):
            return

        self._emit_orientation()

    def _emit_orientation(self):
        yaw_rad = math.radians(self.latest_yaw_deg)
        pitch_rad = math.radians(self.latest_pitch_deg)
        roll_rad = math.radians(self.latest_roll_deg)

        quaternion = self._euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
        reading = OrientationReading(
            self.latest_yaw_deg, self.latest_pitch_deg, self.latest_roll_deg, quaternion)

        if self._on_orientation:
            self._on_orientation(reading)

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return (qx, qy, qz, qw)

    def _parse_gps(self, sentence: str):
        body = sentence.split("*")[0]
        fields = body.split(",")
        if len(fields) < 10:
            return

        try:
            fix_quality = int(fields[6]) if fields[6] != "" else 0
        except ValueError:
            fix_quality = 0

        if fix_quality == 0 or fields[2] == "" or fields[4] == "":
            reading = GpsReading(False, 0.0, 0.0, 0.0)
            if self._on_gps:
                self._on_gps(reading)
            return

        try:
            latitude = self._nmea_to_decimal_degrees(fields[2], fields[3])
            longitude = self._nmea_to_decimal_degrees(fields[4], fields[5])
            altitude = float(fields[9]) if fields[9] != "" else 0.0
        except ValueError:
            return

        reading = GpsReading(True, latitude, longitude, altitude)
        if self._on_gps:
            self._on_gps(reading)

    @staticmethod
    def _nmea_to_decimal_degrees(raw_value: str, direction: str) -> float:
        dot_index = raw_value.index(".")
        degrees = float(raw_value[:dot_index - 2])
        minutes = float(raw_value[dot_index - 2:])
        decimal_degrees = degrees + minutes / 60.0
        if direction in ("S", "W"):
            decimal_degrees *= -1
        return decimal_degrees

    @staticmethod
    def _convert_speed_to_mps(value: float, units: str) -> float:
        if units == "N":
            return value * 0.514444
        elif units == "M":
            return value
        elif units == "K":
            return value / 3.6
        else:
            return value