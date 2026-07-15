"""
Driver RBRcoda3 — protocole Ruskin ASCII (génération 3).
"""

import re
import time
import serial

ATMOSPHERE_DBAR = 10.1325
SEAWATER_DENSITY = 1026.0
_GRAVITY = 9.80665


def wakeup(conn: serial.Serial) -> None:
    conn.write(b'\r')
    time.sleep(0.01)
    conn.reset_input_buffer()


def cmd(conn: serial.Serial, command: str, wait: float = 1.5) -> str:
    keyword = command.split()[0].lower()
    conn.write((command + '\r\n').encode('ascii'))
    deadline = time.time() + wait

    while time.time() < deadline:
        line = conn.readline().decode('ascii', errors='ignore').strip()
        if not line:
            continue

        if line.startswith('Ready:'):
            line = line[len('Ready:'):].strip()

        if line.lower().startswith(keyword) and line.lower() != command.lower():
            return line

    return ''


def get_field(response: str, key: str) -> str:
    m = re.search(rf'\b{re.escape(key)}\s*=\s*([^,]+)', response)
    return m.group(1).strip() if m else ''


def parse_channelslist(response: str) -> list:
    if '=' not in response:
        return []

    out = []
    for item in response.split('=', 1)[1].split('|'):
        item = item.strip()
        m = re.match(r'([^()]+)\((.*)\)', item)
        if m:
            out.append((m.group(1).strip(), m.group(2).strip()))
        elif item:
            out.append((item, ''))
    return out


def read_info(conn: serial.Serial) -> dict:
    ident = cmd(conn, 'id')
    sampling = cmd(conn, 'sampling')
    chans = cmd(conn, 'outputformat channelslist')

    period_raw = get_field(sampling, 'period')

    return {
        'model': get_field(ident, 'model'),
        'serial': get_field(ident, 'serial'),
        'firmware': get_field(ident, 'fwversion'),
        'mode': get_field(sampling, 'mode'),
        'period_ms': int(period_raw) if period_raw.isdigit() else None,
        'channels': parse_channelslist(chans),
    }


def parse_stream(line: str):
    if not line or not line[0].isdigit() or 'Error' in line:
        return None

    parts = [p.strip() for p in line.split(',')]
    if len(parts) < 2:
        return None

    try:
        values = [float(p) for p in parts[1:]]
    except ValueError:
        return None

    return parts[0], values


def sea_pressure(pressure_dbar: float, atmosphere_dbar: float = ATMOSPHERE_DBAR) -> float:
    return pressure_dbar - atmosphere_dbar


def depth(sea_pressure_dbar: float, density: float = SEAWATER_DENSITY) -> float:
    return max(0.0, sea_pressure_dbar * 1e4 / (density * _GRAVITY))