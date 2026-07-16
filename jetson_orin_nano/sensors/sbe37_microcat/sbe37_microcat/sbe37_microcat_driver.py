#!/usr/bin/env python3
import math
import time
import serial
import gsw
from dataclasses import dataclass


@dataclass
class CtdReading:
    temperature_c: float
    conductivity_spm: float
    pressure_dbar: float
    timestamp: str
    salinity_psu: float
    sound_velocity_mps: float
    density_kgm3: float


class SBE37MicrocatDriver:
    """
    Standalone driver for the Sea-Bird SBE 37-SIP MicroCAT CTD.
    No ROS 2 dependency -- just pyserial and gsw. Usable in any Python script.

    warn_callback: optional function(str) called on parse/compute warnings.
    Defaults to doing nothing; pass `print` for a quick standalone script,
    or a ROS logger's .warn method when wrapped in a ROS 2 node.
    """

    def __init__(self, port="/dev/ttyUSB0", baud_rate=9600, sample_period=4.0,
                 warn_callback=None):
        self.sample_period = sample_period
        self.sampling_enabled = True
        self._warn = warn_callback or (lambda msg: None)
        self.serial_port = None

        try:
            self.serial_port = serial.Serial(port=port, baudrate=baud_rate, timeout=sample_period)
        except serial.SerialException as e:
            self._warn(f"Could not open serial port '{port}': {e}")

    def enable_sampling(self, enabled: bool):
        """Pause or resume periodic sampling in run_forever()."""
        self.sampling_enabled = enabled

    def take_sample(self):
        """
        Blocking: sends 'ts' and waits (up to sample_period seconds) for a
        valid data line. Returns a CtdReading, or None on timeout/parse
        failure/disconnected port. Always performs the request regardless
        of enable_sampling's state -- that flag only affects run_forever().
        """
        if self.serial_port is None:
            self._warn("Serial port not connected -- cannot take sample.")
            return None

        self.serial_port.reset_input_buffer()
        self.serial_port.write(b"ts\r\n")

        deadline = time.monotonic() + self.sample_period

        while time.monotonic() < deadline:
            raw_line = self.serial_port.readline()
            decoded_line = raw_line.decode("ascii", errors="ignore").strip()

            if not decoded_line:
                continue

            candidate = decoded_line
            if candidate.startswith("ts"):
                candidate = candidate[2:].strip()

            if "," in candidate:
                return self._parse_line(candidate)

        self._warn("No valid data line received before timeout.")
        return None

    def run_forever(self, callback):
        """
        Blocking loop for standalone scripts: calls take_sample() every
        sample_period seconds (skipping if sampling is disabled) and
        invokes callback(reading) whenever a valid sample is obtained.
        Runs until interrupted (e.g. Ctrl+C).
        """
        try:
            while True:
                if self.sampling_enabled:
                    reading = self.take_sample()
                    if reading is not None:
                        callback(reading)
                else:
                    time.sleep(self.sample_period)
        except KeyboardInterrupt:
            pass

    def _parse_line(self, line: str):
        fields = [f.strip() for f in line.split(",")]

        if len(fields) < 5:
            self._warn("Unexpected response format: " + line)
            return None

        try:
            temperature_c = float(fields[0])
            conductivity_spm = float(fields[1])
            pressure_dbar = float(fields[2])
            timestamp = fields[3] + ", " + fields[4]
        except ValueError:
            self._warn("Could not parse sample: " + line)
            return None

        salinity_psu, sound_velocity_mps, density_kgm3 = self._compute_derived_values(
            conductivity_spm, temperature_c, pressure_dbar)

        return CtdReading(
            temperature_c=temperature_c,
            conductivity_spm=conductivity_spm,
            pressure_dbar=pressure_dbar,
            timestamp=timestamp,
            salinity_psu=salinity_psu,
            sound_velocity_mps=sound_velocity_mps,
            density_kgm3=density_kgm3,
        )

    def _compute_derived_values(self, conductivity_spm, temperature_c, pressure_dbar):
        conductivity_mspcm = conductivity_spm * 10.0

        try:
            salinity_psu = gsw.SP_from_C(conductivity_mspcm, temperature_c, pressure_dbar)

            longitude, latitude = 0.0, 0.0
            absolute_salinity = gsw.SA_from_SP(salinity_psu, pressure_dbar, longitude, latitude)
            conservative_temp = gsw.CT_from_t(absolute_salinity, temperature_c, pressure_dbar)

            sound_velocity_mps = gsw.sound_speed(absolute_salinity, conservative_temp, pressure_dbar)
            density_kgm3 = gsw.rho(absolute_salinity, conservative_temp, pressure_dbar)

            salinity_psu = float(salinity_psu)
            sound_velocity_mps = float(sound_velocity_mps)
            density_kgm3 = float(density_kgm3)

            if math.isnan(salinity_psu) or math.isnan(sound_velocity_mps) or math.isnan(density_kgm3):
                self._warn(
                    "Derived values are NaN -- likely because conductivity is near zero "
                    "(sensor not submerged in real seawater).")

            return salinity_psu, sound_velocity_mps, density_kgm3

        except Exception as e:
            self._warn("Could not compute derived values: " + str(e))
            return 0.0, 0.0, 0.0

    def close(self):
        if self.serial_port:
            self.serial_port.close()