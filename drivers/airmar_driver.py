#!/usr/bin/env python3
"""
Driver Airmar 150WXRS — agnostique ROS2.

Lit le flux NMEA série du capteur météo/GPS Airmar 150WXRS et parse
les sentences en structures de données Python pures.

Sentences supportées :
  $WIMWV  — vent (direction, vitesse)
  $HCHDG  — cap magnétique
  $WIMDA  — météo (pression, température, humidité)
  $YXXDR  — pitch / roll (et pluie via PTCH/ROLL/RAIN/DURA/RATE)
  $GPGGA  — position GPS

Utilisation minimale :
    driver = AirmarDriver('/dev/ttyUSB0', baudrate=4800)
    driver.open()
    for data in driver.stream():
        print(data)
    driver.close()
"""

import serial
import pynmea2
from dataclasses import dataclass, field
from typing import Optional, Iterator


# ─── Structures de données ────────────────────────────────────────────────────

@dataclass
class WindData:
    direction_deg: float        # degrés (0–360)
    speed_kn: float             # nœuds
    speed_ms: float             # m/s
    reference: str = ''         # 'R' relatif / 'T' true


@dataclass
class HeadingData:
    heading_deg: float          # cap magnétique (degrés)


@dataclass
class WeatherData:
    pressure_pa: Optional[float] = None    # Pascals
    air_temp_c: Optional[float] = None     # °C
    humidity_pct: Optional[float] = None   # %


@dataclass
class OrientationData:
    pitch_deg: float            # degrés
    roll_deg: float             # degrés


@dataclass
class RainData:
    amount_mm: float            # mm
    duration_s: float           # secondes
    rate_mm_h: float            # mm/h


@dataclass
class GPSData:
    latitude: float             # degrés décimaux
    longitude: float            # degrés décimaux
    altitude_m: float           # mètres
    fix_quality: int            # 0=no fix, 1=GPS, 2=DGPS…


# Type union retourné par stream()
AirmarSample = WindData | HeadingData | WeatherData | OrientationData | RainData | GPSData


# ─── Parsing NMEA ─────────────────────────────────────────────────────────────

def parse_mwv(msg) -> Optional[WindData]:
    """$WIMWV — Wind Speed and Angle."""
    try:
        direction = float(msg.wind_angle)
        speed_kn  = float(msg.wind_speed)
        speed_ms  = speed_kn * 0.514444
        reference = getattr(msg, 'reference', '')
        return WindData(direction_deg=direction,
                        speed_kn=speed_kn,
                        speed_ms=round(speed_ms, 3),
                        reference=reference)
    except (ValueError, AttributeError):
        return None


def parse_hdg(msg) -> Optional[HeadingData]:
    """$HCHDG — Heading, Deviation & Variation."""
    try:
        return HeadingData(heading_deg=float(msg.heading))
    except (ValueError, AttributeError):
        return None


def parse_mda(msg) -> Optional[WeatherData]:
    """$WIMDA — Meteorological Composite (pression, temp, humidité)."""
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
    """
    $YXXDR / $WIXDR — Transducer Measurements.
    Retourne OrientationData, RainData, ou None selon le contenu.
    """
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
            amount = value;  has_rain = True
        elif label == 'DURA':
            duration = value; has_rain = True
        elif label == 'RATE':
            rate = value;    has_rain = True
        i += 4

    # Priorité à la pluie si RAIN présent dans la trame brute
    if has_rain or 'RAIN' in raw:
        return RainData(amount_mm=amount, duration_s=duration, rate_mm_h=rate)

    if pitch is not None and roll is not None:
        return OrientationData(pitch_deg=pitch, roll_deg=roll)

    return None


def parse_gga(msg) -> Optional[GPSData]:
    """$GPGGA — Global Positioning System Fix Data."""
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
    """
    Parse une ligne NMEA brute et retourne la structure de données
    correspondante, ou None si la ligne est ignorée / invalide.
    """
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


# ─── Classe driver ────────────────────────────────────────────────────────────

class AirmarDriver:
    """
    Driver série pour l'Airmar 150WXRS.

    Gère l'ouverture/fermeture du port et expose un générateur
    ``stream()`` qui yield des objets AirmarSample.

    Paramètres
    ----------
    port : str
        Port série (ex. '/dev/ttyUSB0', 'COM3').
    baudrate : int
        Vitesse (défaut 4800 selon spec Airmar).
    timeout : float
        Timeout readline en secondes.
    """

    DEFAULT_BAUDRATE = 4800

    def __init__(self, port: str,
                 baudrate: int = DEFAULT_BAUDRATE,
                 timeout: float = 1.0):
        self.port     = port
        self.baudrate = baudrate
        self.timeout  = timeout
        self._serial: Optional[serial.Serial] = None

    # ── Connexion ─────────────────────────────────────────────────────────────

    def open(self) -> None:
        """Ouvre le port série."""
        self._serial = serial.Serial(
            self.port, self.baudrate,
            bytesize=8, parity='N', stopbits=1,
            timeout=self.timeout,
        )

    def close(self) -> None:
        """Ferme le port série proprement."""
        if self._serial and self._serial.is_open:
            self._serial.close()

    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    # ── Lecture ───────────────────────────────────────────────────────────────

    def read_line(self) -> Optional[str]:
        """
        Lit une ligne brute sur le port série.
        Retourne None en cas de timeout ou d'erreur de décodage.
        """
        if not self.is_open():
            raise RuntimeError("Port série non ouvert — appelez open() d'abord.")
        raw = self._serial.readline()
        if not raw:
            return None
        return raw.decode('ascii', errors='replace').strip()

    def stream(self) -> Iterator[AirmarSample]:
        """
        Générateur qui lit en continu le flux série et yield
        uniquement les AirmarSample valides (None filtrés).

        Exemple :
            driver.open()
            for sample in driver.stream():
                if isinstance(sample, WindData):
                    print(sample.speed_ms, 'm/s')
        """
        while True:
            line = self.read_line()
            if line is None:
                continue
            sample = parse_nmea_line(line)
            if sample is not None:
                yield sample

    # ── Context manager ───────────────────────────────────────────────────────

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()
