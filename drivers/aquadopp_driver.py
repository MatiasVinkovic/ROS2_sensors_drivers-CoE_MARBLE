"""
Aquadopp S4VP — driver (fonctions pures, sans ROS2)
====================================================
Logique extraite et validée en réel depuis test_aquadopp_sans_ros2.py
(2026-06-23, COM3 @ 115200 8N1).

Protocole binaire Nortek :
  - réveil en mode commande par BREAK puis SETDEFAULT,CONFIG
  - configuration du plan de mesure (intervalle MEAS_INTERVAL_S secondes)
  - START -> trames binaires (sync 0xA5, id 0x0A) capturées par burst
  - parsing des champs float little-endian aux offsets validés
"""

import serial
import struct
import time

_SYNC = 0xA5
MEAS_INTERVAL_S = 30

# Séquence de configuration — identique au test validé (intervalle 30 s).
_CONFIG_COMMANDS = [
    (f'SETPLAN,MIAVG={MEAS_INTERVAL_S},AVG=1,SA=35,BURST=0,MIBURST=3600,SV=0,FN="Data",SO=1', 2.0),
    ('SETEXTSENSOR,EN=0,TYPE="",PWROUT="OFF"', 2.0),
    (f'SETAVG,NC=40,CS=2.5,BD=1,DF=7,CY="ENU",PL=0,AI={MEAS_INTERVAL_S},VR=2.5,NPING={MEAS_INTERVAL_S},NB=3,ZCELL=0', 2.0),
    ('SETTMAVG,EN=0,AVG=60,CY="ENU",FO=1,SO=0,DF=100,CD=1,PD=1,TV=1,TA=1,TC=1,DISTILT=0,TPG=0,MAPBINS=0,CORRTH=50', 2.0),
]


def flush(conn: serial.Serial, duration: float) -> bytes:
    """Lit tout ce qui arrive pendant `duration` secondes."""
    buf, deadline = b'', time.time() + duration
    while time.time() < deadline:
        n = conn.in_waiting
        if n:
            buf += conn.read(n)
        else:
            time.sleep(0.05)
    return buf


def cmd(conn: serial.Serial, command: str, wait: float = 1.5) -> str:
    """Envoie une commande ASCII (CR+LF) et renvoie la réponse texte."""
    conn.reset_input_buffer()
    conn.write((command + '\r\n').encode('ascii'))
    return flush(conn, wait).decode('ascii', errors='replace').strip()


def enter_command_mode(conn: serial.Serial, logger, max_attempts: int = 3) -> bool:
    """Force le passage en mode commande par BREAK + SETDEFAULT,CONFIG.

    SETDEFAULT,CONFIG n'est accepté qu'en mode commande ; il réinitialise la
    config (configure() la réapplique ensuite). Logique validée en réel.
    """
    for attempt in range(1, max_attempts + 1):
        logger.info(f"Break #{attempt} (1.5 s)...")
        conn.reset_input_buffer()
        conn.send_break(duration=1.5)
        time.sleep(3.0)
        conn.reset_input_buffer()
        r = cmd(conn, 'SETDEFAULT,CONFIG', wait=3.0)
        if 'OK' in r and 'ERROR' not in r:
            logger.info("Mode commande confirmé")
            return True
        logger.warn("Pas de réponse — nouvel essai")
        time.sleep(1.0)
    return False


def configure(conn: serial.Serial, logger) -> bool:
    """Applique le plan de mesure. Renvoie False si une commande échoue."""
    logger.info("Envoi de la configuration...")
    all_ok = True
    for command, wait in _CONFIG_COMMANDS:
        r = cmd(conn, command, wait=wait)
        ok = 'OK' in r and 'ERROR' not in r
        logger.info(f"  {'OK' if ok else 'ECHEC'} — {command[:55]}")
        if not ok:
            all_ok = False
    return all_ok


def start_measurement(conn: serial.Serial) -> bool:
    """Envoie START et renvoie True si l'instrument répond OK."""
    conn.reset_input_buffer()
    conn.write(b'START\r\n')
    time.sleep(0.5)
    resp = flush(conn, 0.5).decode('ascii', errors='replace')
    return 'OK' in resp


def capture_burst(conn: serial.Serial, idle_s: float = 2.0, max_wait_s: float = 90.0) -> bytes:
    """Attend l'arrivée de données puis les agrège jusqu'à `idle_s` de silence."""
    buf, deadline = b'', time.time() + max_wait_s
    while time.time() < deadline:
        if conn.in_waiting:
            buf += conn.read(conn.in_waiting)
            break
        time.sleep(0.05)
    else:
        return b''
    while True:
        time.sleep(idle_s)
        n = conn.in_waiting
        if n:
            buf += conn.read(n)
        else:
            break
    return buf


def read_packet(conn: serial.Serial, logger, timeout_s: int = 300) -> bytes | None:
    """Capture des bursts jusqu'à trouver une trame Nortek complète (sync 0xA5, id 0x0A)."""
    deadline, burst_num = time.time() + timeout_s, 0
    while time.time() < deadline:
        remaining = int(deadline - time.time())
        logger.debug(f"Attente burst #{burst_num + 1} ({remaining} s restantes)")
        burst = capture_burst(conn, idle_s=2.0, max_wait_s=min(90.0, remaining))
        if not burst:
            return None
        burst_num += 1
        for i in range(len(burst) - 9):
            if burst[i] != _SYNC or burst[i + 1] != 10:
                continue
            data_sz = struct.unpack_from('<H', burst, i + 4)[0]
            total_sz = 10 + data_sz
            if 100 <= data_sz <= 4096 and i + total_sz <= len(burst):
                return burst[i: i + total_sz]
    return None


def parse_packet(raw: bytes) -> dict | None:
    """Extrait les champs scalaires aux offsets validés (roll@62, pitch@66)."""
    if len(raw) < 70:
        return None
    try:
        sos   = struct.unpack_from('<f', raw, 42)[0]
        temp  = struct.unpack_from('<f', raw, 46)[0]
        press = struct.unpack_from('<f', raw, 50)[0]
        head  = struct.unpack_from('<f', raw, 54)[0]
        roll  = struct.unpack_from('<f', raw, 62)[0]
        pitch = struct.unpack_from('<f', raw, 66)[0]
        if not (1400.0 <= sos <= 1600.0) or not (-5.0 <= temp <= 40.0):
            return None
        return {
            'speed_of_sound_ms': round(sos,   2),
            'temperature_c':     round(temp,  3),
            'pressure_dbar':     round(press, 4),
            'heading_deg':       round(head,  2),
            'pitch_deg':         round(pitch, 3),
            'roll_deg':          round(roll,  3),
        }
    except struct.error:
        return None