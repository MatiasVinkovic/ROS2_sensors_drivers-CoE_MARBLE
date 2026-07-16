#!/usr/bin/env python3
import math
import time
import serial
from dataclasses import dataclass

KNOWN_FIELDS = {
    "Heading[Deg.M]", "Pitch[Deg]", "Roll[Deg]",
    "StDev Heading[Deg.M]", "StDev Pitch[Deg]", "StDev Roll[Deg]",
    "Input Voltage[V]", "Input Current[mA]", "Memory Used[Bytes]",
    "Significant Wave Height Hm0[m]", "Wave Peak Direction[Deg.M]",
    "Wave Peak Period[s]", "Wave Height Wind Hm0[m]", "Wave Height Swell Hm0[m]",
    "Wave Peak Period Wind[s]", "Wave Peak Period Swell[s]",
    "Wave Peak Direction Wind[Deg.M]", "Wave Peak Direction Swell[Deg.M]",
    "Wave Mean Direction[Deg.M]", "Wave Mean Period Tm02[s]",
    "Wave Height Hmax[m]", "Wave Height Crest[m]", "Wave Height Trough[m]",
    "Wave Period Tmax[s]", "Wave Period Tz[s]", "Significant Wave Height H1/3[m]",
    "Mean Spreading Angle[Deg.M]", "First Order Spread[Deg.M]",
    "Long Crestedness Parameters",
}


@dataclass
class OrientationReading:
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    yaw_stdev_deg: float
    pitch_stdev_deg: float
    roll_stdev_deg: float
    quaternion: tuple  # (x, y, z, w)


@dataclass
class WaveStatsReading:
    significant_wave_height_hm0: float = 0.0
    wave_height_wind_hm0: float = 0.0
    wave_height_swell_hm0: float = 0.0
    significant_wave_height_h13: float = 0.0
    wave_height_hmax: float = 0.0
    wave_height_crest: float = 0.0
    wave_height_trough: float = 0.0
    wave_period_tz: float = 0.0
    wave_period_tmax: float = 0.0
    wave_mean_period_tm02: float = 0.0
    wave_peak_period: float = 0.0
    wave_peak_period_wind: float = 0.0
    wave_peak_period_swell: float = 0.0
    wave_peak_direction: float = 0.0
    wave_peak_direction_wind: float = 0.0
    wave_peak_direction_swell: float = 0.0
    wave_mean_direction: float = 0.0
    mean_spreading_angle: float = 0.0
    first_order_spread: float = 0.0
    long_crestedness_parameter: float = 0.0


@dataclass
class SystemStatusReading:
    input_voltage: float = 0.0
    input_current_ma: float = 0.0
    memory_used_bytes: int = 0


class MotusWaveDriver:
    """
    Standalone driver for the Aanderaa MOTUS Wave Sensor 5729.
    No ROS 2 dependency -- just pyserial. Usable in any Python script.

    The sensor streams autonomously every ~4-5 minutes; no commands needed.
    Register callbacks, then call poll() repeatedly or run_forever().

    warn_callback / info_callback: optional function(str), default to no-op.
    """

    def __init__(self, port="/dev/ttyUSB0", baud_rate=115200, silence_gap_sec=1.0,
                 warn_callback=None, info_callback=None):
        self.silence_gap_sec = silence_gap_sec
        self._warn = warn_callback or (lambda msg: None)
        self._info = info_callback or (lambda msg: None)
        self.serial_port = None

        try:
            self.serial_port = serial.Serial(port=port, baudrate=baud_rate, timeout=1.0)
        except serial.SerialException as e:
            self._warn(f"Could not open serial port '{port}': {e}")

        self._buffer = b''
        self._last_rx_time = None

        self._on_orientation = None
        self._on_wave_stats = None
        self._on_system_status = None

    # --- callback registration -------------------------------------------

    def on_orientation(self, callback):
        self._on_orientation = callback

    def on_wave_stats(self, callback):
        self._on_wave_stats = callback

    def on_system_status(self, callback):
        self._on_system_status = callback

    # --- main loop --------------------------------------------------------

    def poll(self):
        """Non-blocking: accumulates bytes, and processes a block once a
        silence gap indicates the current block has finished arriving."""
        if self.serial_port is None:
            return

        n = self.serial_port.in_waiting
        if n > 0:
            self._buffer += self.serial_port.read(n)
            self._last_rx_time = time.time()
            return

        if self._buffer and self._last_rx_time is not None:
            if time.time() - self._last_rx_time > self.silence_gap_sec:
                self._process_block(self._buffer)
                self._buffer = b''
                self._last_rx_time = None

    def run_forever(self, poll_interval=1.0):
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

    # --- parsing ----------------------------------------------------------

    def _process_block(self, raw_bytes: bytes):
        text = raw_bytes.decode("ascii", errors="ignore")

        if "MEASUREMENT" not in text:
            self._warn("Block did not contain a MEASUREMENT line -- unexpected format.")
            return

        fields = self._extract_known_fields(text)
        self._info("Parsed " + str(len(fields)) + " known fields from block.")

        self._emit_orientation(fields)
        self._emit_wave_stats(fields)
        self._emit_system_status(fields)

    def _extract_known_fields(self, text: str) -> dict:
        tokens = text.split("\t")
        result = {}

        i = 0
        while i < len(tokens) - 1:
            token = tokens[i].strip()
            candidate = token[1:] if token.startswith("*") else token

            if candidate in KNOWN_FIELDS and candidate not in result:
                value_str = tokens[i + 1].strip()
                try:
                    result[candidate] = float(value_str)
                except ValueError:
                    pass
                i += 2
            else:
                i += 1

        return result

    def _emit_orientation(self, fields: dict):
        required = ["Heading[Deg.M]", "Pitch[Deg]", "Roll[Deg]",
                    "StDev Heading[Deg.M]", "StDev Pitch[Deg]", "StDev Roll[Deg]"]
        if not all(key in fields for key in required):
            self._warn("Missing orientation fields, skipping.")
            return

        yaw_deg = fields["Heading[Deg.M]"]
        pitch_deg = fields["Pitch[Deg]"]
        roll_deg = fields["Roll[Deg]"]

        quaternion = self._euler_to_quaternion(
            math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))

        reading = OrientationReading(
            yaw_deg=yaw_deg,
            pitch_deg=pitch_deg,
            roll_deg=roll_deg,
            yaw_stdev_deg=fields["StDev Heading[Deg.M]"],
            pitch_stdev_deg=fields["StDev Pitch[Deg]"],
            roll_stdev_deg=fields["StDev Roll[Deg]"],
            quaternion=quaternion,
        )

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

    def _emit_wave_stats(self, fields: dict):
        reading = WaveStatsReading(
            significant_wave_height_hm0=fields.get("Significant Wave Height Hm0[m]", 0.0),
            wave_height_wind_hm0=fields.get("Wave Height Wind Hm0[m]", 0.0),
            wave_height_swell_hm0=fields.get("Wave Height Swell Hm0[m]", 0.0),
            significant_wave_height_h13=fields.get("Significant Wave Height H1/3[m]", 0.0),
            wave_height_hmax=fields.get("Wave Height Hmax[m]", 0.0),
            wave_height_crest=fields.get("Wave Height Crest[m]", 0.0),
            wave_height_trough=fields.get("Wave Height Trough[m]", 0.0),
            wave_period_tz=fields.get("Wave Period Tz[s]", 0.0),
            wave_period_tmax=fields.get("Wave Period Tmax[s]", 0.0),
            wave_mean_period_tm02=fields.get("Wave Mean Period Tm02[s]", 0.0),
            wave_peak_period=fields.get("Wave Peak Period[s]", 0.0),
            wave_peak_period_wind=fields.get("Wave Peak Period Wind[s]", 0.0),
            wave_peak_period_swell=fields.get("Wave Peak Period Swell[s]", 0.0),
            wave_peak_direction=fields.get("Wave Peak Direction[Deg.M]", 0.0),
            wave_peak_direction_wind=fields.get("Wave Peak Direction Wind[Deg.M]", 0.0),
            wave_peak_direction_swell=fields.get("Wave Peak Direction Swell[Deg.M]", 0.0),
            wave_mean_direction=fields.get("Wave Mean Direction[Deg.M]", 0.0),
            mean_spreading_angle=fields.get("Mean Spreading Angle[Deg.M]", 0.0),
            first_order_spread=fields.get("First Order Spread[Deg.M]", 0.0),
            long_crestedness_parameter=fields.get("Long Crestedness Parameters", 0.0),
        )

        if self._on_wave_stats:
            self._on_wave_stats(reading)

    def _emit_system_status(self, fields: dict):
        reading = SystemStatusReading(
            input_voltage=fields.get("Input Voltage[V]", 0.0),
            input_current_ma=fields.get("Input Current[mA]", 0.0),
            memory_used_bytes=int(fields.get("Memory Used[Bytes]", 0.0)),
        )

        if self._on_system_status:
            self._on_system_status(reading)