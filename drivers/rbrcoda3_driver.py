"""
Driver RBRcoda3 — protocole Ruskin ASCII (génération 3).

Le coda3 streame ses mesures en continu sur le port série :
    "269500, 20.9781"                          → horodatage ms, valeur(s)
    "2024-05-10 14:32:01.000, 18.17, 12.70"    → datetime, valeur(s)

Les commandes ASCII (id, sampling, outputformat...) restent utilisables
pendant le streaming : leurs réponses sont interceptées parmi les lignes
de données grâce au mot-clé de la commande.
"""

import re
import time

import serial

# Pression atmosphérique standard (dbar) pour la pression marine dérivée
ATMOSPHERE_DBAR = 10.1325
# Densité d'eau de mer standard (kg/m³) pour la profondeur dérivée
SEAWATER_DENSITY = 1026.0
_GRAVITY = 9.80665


# ─── Bas niveau ───────────────────────────────────────────────────────────────

def wakeup(conn: serial.Serial) -> None:
    """Réveil RBR : envoyer \\r + pause 10 ms (obligatoire selon doc RBR)."""
    conn.write(b'\r')
    time.sleep(0.01)
    conn.reset_input_buffer()


def cmd(conn: serial.Serial, command: str, wait: float = 1.5) -> str:
    """
    Envoie une commande ASCII et retourne SA ligne de réponse
    (celle qui commence par le mot-clé de la commande), en ignorant
    les lignes de mesures streamées qui s'intercalent.
    """
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
    """Extrait 'valeur' d'une réponse Ruskin 'clé = valeur, clé = valeur, ...'."""
    m = re.search(rf'\b{re.escape(key)}\s*=\s*([^,]+)', response)
    return m.group(1).strip() if m else ''


# ─── Identification ───────────────────────────────────────────────────────────

def read_info(conn: serial.Serial) -> dict:
    """
    Interroge le capteur et retourne ses métadonnées :
    {'model', 'serial', 'firmware', 'mode', 'period_ms', 'channels'}
    channels : liste de (nom, unité) dans l'ordre du flux streamé.
    """
    ident    = cmd(conn, 'id')
    sampling = cmd(conn, 'sampling')
    chans    = cmd(conn, 'outputformat channelslist')

    period_raw = get_field(sampling, 'period')
    return {
        'model':     get_field(ident, 'model'),
        'serial':    get_field(ident, 'serial'),
        'firmware':  get_field(ident, 'fwversion'),
        'mode':      get_field(sampling, 'mode'),
        'period_ms': int(period_raw) if period_raw.isdigit() else None,
        'channels':  parse_channelslist(chans),
    }


def parse_channelslist(response: str) -> list:
    """
    Parse 'outputformat channelslist = temperature(C)|pressure(dbar)'.
    Retourne [('temperature', 'C'), ('pressure', 'dbar'), ...].
    """
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


# ─── Flux de mesures ──────────────────────────────────────────────────────────

def parse_stream(line: str):
    """
    Parse une ligne de mesure (streamée ou réponse à 'fetch').
    Formats : "269500, 20.9781"  ou  "2024-05-10 14:32:01.000, 18.17, 12.70"
    Retourne (horodatage_str, [valeurs float]) ou None.
    """
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


# ─── Valeurs dérivées ─────────────────────────────────────────────────────────

def sea_pressure(pressure_dbar: float,
                 atmosphere_dbar: float = ATMOSPHERE_DBAR) -> float:
    """Pression marine = pression absolue - pression atmosphérique."""
    return pressure_dbar - atmosphere_dbar


def depth(sea_pressure_dbar: float,
          density: float = SEAWATER_DENSITY) -> float:
    """Profondeur (m) à partir de la pression marine (1 dbar = 10⁴ Pa)."""
    return max(0.0, sea_pressure_dbar * 1e4 / (density * _GRAVITY))
