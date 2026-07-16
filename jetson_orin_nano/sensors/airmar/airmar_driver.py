#!/usr/bin/env python3
"""
Driver Airmar 150WXRS — agnostique ROS2.

Lit le flux NMEA série du capteur météo/GPS Airmar 150WXRS et parse
les sentences en structures de données Python pures.

Sentences supportées :
$WIMWV — vent (direction, vitesse)
$HCHDG — cap magnétique
$WIMDA — météo (pression, température, humidité)
$YXXDR — pitch / roll (et pluie via PTCH/ROLL/RAIN/DURA/RATE)
$GPGGA — position GPS

Utilisation minimale :
driver = AirmarDriver('/dev/ttyUSB0', baudrate=4800)
driver.open()
for data in driver.stream():
    print(data)
driver.close()
"""

import serial
import pynmea2
from dataclasses import dataclass
from typing import Optional, Iterator


@dataclass
class WindData:
    direction_deg: float
    speed_kn: float
    speed_ms: float
    reference: str = ''


@dataclass
class HeadingData:
    heading_deg: float


@dataclass
class WeatherData:
    pressure_pa: Optional[float] = None
    air_temp_c: Optional[float] = None
    humidity_pct: Optional[float] = None


@dataclass
class OrientationData:
    pitch_deg: float
    roll_deg: float


@dataclass
class RainData:
    amount_mm: float
    duration_s: float
    rate_mm_h: float


@dataclass
class GPSData:
    latitude: float
    longitude: float
    altitude_m: float
    fix_quality: int


AirmarSample = WindData | HeadingData | WeatherData | OrientationData | RainData | GPSData


def parse_mwv(msg) -> Optional[WindData]:
    try:
        direction = float(msg.wind_angle)
        speed_kn = float(msg.wind_speed)
        speed_ms = speed_kn * 0.514444
        reference = getattr(msg, 'reference', '')
        return WindData(
            direction_deg=direction,
            speed_kn=speed_kn,
            speed_ms=round(speed_ms, 3),
            reference=reference
        )
    except (ValueError, AttributeError):
        return None


def parse_hdg(msg) -> Optional[HeadingData]:
    try:
        return HeadingData(heading_deg=float(msg.heading))
    except (ValueError, AttributeError):
        return None


def parse_mda(msg) -> Optional[WeatherData]:
    data = WeatherData()
    try:
        data.pressure_pa = float(msg.b_pressure_bar) * 1e5
    except (ValueError, AttributeError):
        pass
    try:
        data.air_temp_c = float(msg.air_temp)
    except (ValueError, AttributeError):
        pass
    try:
        data.humidity_pct = float(msg.rel_humidity)
    except (ValueError, AttributeError):
        pass

    if any(v is not None for v in [data.pressure_pa, data.air_temp_c, data.humidity_pct]):
        return data
    return None


def parse_xdr(msg, raw: str):
    try:
        data = msg.data
    except AttributeError:
        return None

    pitch, roll = None, None
    amount, duration, rate = 0.0, 0.0, 0.0
    has_rain = False

    i = 0
    while i <= len(data) - 4:
        label = data[i + 3]
        try:
            value = float(data[i + 1])
        except (ValueError, TypeError):
            i += 4
            continue

        if label == 'PTCH':
            pitch = value
        elif label == 'ROLL':
            roll = value
        elif label == 'RAIN':
            amount = value
            has_rain = True
        elif label == 'DURA':
            duration = value
            has_rain = True
        elif label == 'RATE':
            rate = value
            has_rain = True
        i += 4

    if has_rain or 'RAIN' in raw:
        return RainData(amount_mm=amount, duration_s=duration, rate_mm_h=rate)

    if pitch is not None and roll is not None:
        return OrientationData(pitch_deg=pitch, roll_deg=roll)

    return None


def parse_gga(msg) -> Optional[GPSData]:
    try:
        quality = int(msg.gps_qual)
        if quality == 0:
            return None
        return GPSData(
            latitude=float(msg.latitude),
            longitude=float(msg.longitude),
            altitude_m=float(msg.altitude),
            fix_quality=quality,
        )
    except (ValueError, AttributeError):
        return None


def parse_nmea_line(raw: str) -> Optional[AirmarSample]:
    if not raw.startswith('$'):
        return None

    try:
        msg = pynmea2.parse(raw)
    except pynmea2.ParseError:
        return None

    sentence = msg.sentence_type

    if sentence == 'MWV':
        return parse_mwv(msg)
    elif sentence == 'HDG':
        return parse_hdg(msg)
    elif sentence == 'MDA':
        return parse_mda(msg)
    elif sentence in ('XDR', 'IXDR'):
        return parse_xdr(msg, raw)
    elif sentence == 'GGA':
        return parse_gga(msg)

    return None


class AirmarDriver:
    DEFAULT_BAUDRATE = 4800

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None

    def open(self) -> None:
        self._serial = serial.Serial(
            self.port,
            self.baudrate,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=self.timeout,
        )

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()

    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def read_line(self) -> Optional[str]:
        if not self.is_open():
            raise RuntimeError("Port série non ouvert — appelez open() d'abord.")
        raw = self._serial.readline()
        if not raw:
            return None
        return raw.decode('ascii', errors='replace').strip()

    def stream(self) -> Iterator[AirmarSample]:
        while True:
            line = self.read_line()
            if line is None:
                continue
            sample = parse_nmea_line(line)
            if sample is not None:
                yield sample

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()