#!/usr/bin/env python3
"""
ROS 2 node — Nortek Aquadopp S4VP (profil de courant + CTD).

Publie : /aquadopp/data  (std_msgs/String, JSON)

Format JSON publié :
  { "status": "ok" | "error",
    "timestamp": "HH:MM:SS",   (si status == ok)
    "error": "...",             (si status == error)
    "fields": {                 (si status == ok)
      "speed_of_sound_ms": { "value": <float>, "unit": "m/s",  "display": "<str>" },
      "temperature_c":     { "value": <float>, "unit": "°C",   "display": "<str>" },
      "pressure_dbar":     { "value": <float>, "unit": "dbar", "display": "<str>" },
      "heading_deg":       { "value": <float>, "unit": "°",    "display": "<str>" },
      "pitch_deg":         { "value": <float>, "unit": "°",    "display": "<str>" },
      "roll_deg":          { "value": <float>, "unit": "°",    "display": "<str>" }
    }
  }

Paramètres ROS 2 :
  port   /dev/ttyUSB0
  baud   115200
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import struct
import time
import json
import threading

# ─── Configuration instrument (reprise de aquadopp_config.py) ────────────────

_CONFIG_COMMANDS = [
    ('SETPLAN,MIAVG=60,AVG=1,SA=35,BURST=0,MIBURST=3600,SV=0,FN="Data",SO=1', 2.0),
    ('SETEXTSENSOR,EN=0,TYPE="",PWROUT="OFF"',                                  2.0),
    ('SETAVG,NC=40,CS=2.5,BD=1,DF=7,CY="ENU",PL=0,AI=60,VR=2.5,NPING=60,NB=3,ZCELL=0', 2.0),
    ('SETTMAVG,EN=0,AVG=60,CY="ENU",FO=1,SO=0,DF=100,CD=1,PD=1,TV=1,TA=1,TC=1,DISTILT=0,TPG=0,MAPBINS=0,CORRTH=50', 2.0),
    ('SAVE,CONFIG', 4.0),
]

_SYNC_BYTE = 0xA5

# Format des valeurs publiées : (fmt_string, unité)
_FIELD_FMT = {
    'speed_of_sound_ms': ('.2f', 'm/s'),
    'temperature_c':     ('.3f', '°C'),
    'pressure_dbar':     ('.4f', 'dbar'),
    'heading_deg':       ('.2f', '°'),
    'pitch_deg':         ('.3f', '°'),
    'roll_deg':          ('.3f', '°'),
}

# ─── Fonctions driver (extraites de aquadopp_config.py / aquadopp_reader.py) ─

def _flush(conn, duration: float) -> bytes:
    buf = b''
    deadline = time.time() + duration
    while time.time() < deadline:
        n = conn.in_waiting
        if n:
            buf += conn.read(n)
        else:
            time.sleep(0.05)
    return buf


def _cmd(conn, command: str, wait: float = 1.5) -> str:
    conn.reset_input_buffer()
    conn.write((command + '\r\n').encode('ascii'))
    return _flush(conn, wait).decode('ascii', errors='replace').strip()


def _enter_command_mode(conn, logger, max_attempts: int = 3) -> bool:
    for attempt in range(1, max_attempts + 1):
        logger.info(f"Break #{attempt} (1.5 s)...")
        conn.reset_input_buffer()
        conn.send_break(duration=1.5)
        time.sleep(3.0)
        conn.reset_input_buffer()
        _cmd(conn, 'STOP', wait=1.5)
        conn.reset_input_buffer()
        r = _cmd(conn, 'GETPLAN', wait=3.0)
        if r.strip():
            logger.info("Mode commande confirmé")
            return True
        logger.warn("Pas de réponse ASCII — nouvel essai")
        time.sleep(1.0)
    return False


def _configure(conn, logger) -> bool:
    logger.info("Envoi de la configuration...")
    all_ok = True
    for command, wait in _CONFIG_COMMANDS:
        r = _cmd(conn, command, wait=wait)
        ok = 'OK' in r and 'ERROR' not in r
        logger.info(f"  {'OK' if ok else 'ECHEC'} — {command[:55]}")
        if not ok:
            all_ok = False
    return all_ok


def _capture_burst(conn, idle_s: float = 2.0, max_wait_s: float = 90.0) -> bytes:
    """Attend le premier octet, puis lit jusqu'à silence de idle_s secondes."""
    deadline = time.time() + max_wait_s
    buf = b''
    # Phase 1 : premier octet
    while time.time() < deadline:
        n = conn.in_waiting
        if n:
            buf += conn.read(n)
            break
        time.sleep(0.05)
    else:
        return b''
    # Phase 2 : drainer jusqu'au silence
    while True:
        time.sleep(idle_s)
        n = conn.in_waiting
        if n:
            buf += conn.read(n)
        else:
            break
    return buf


def _read_packet(conn, logger, timeout_s: int = 300):
    """Attend un paquet AD2CP valide ; retourne les octets bruts ou None."""
    deadline = time.time() + timeout_s
    burst_num = 0
    while time.time() < deadline:
        remaining = int(deadline - time.time())
        logger.debug(f"Attente burst #{burst_num + 1} ({remaining} s restantes)")
        burst = _capture_burst(conn, idle_s=2.0,
                               max_wait_s=min(90.0, remaining))
        if not burst:
            return None
        burst_num += 1
        for i in range(len(burst) - 9):
            if burst[i] != _SYNC_BYTE or burst[i + 1] != 10:
                continue
            data_sz = struct.unpack_from('<H', burst, i + 4)[0]
            total_sz = 10 + data_sz
            if 100 <= data_sz <= 4096 and i + total_sz <= len(burst):
                return burst[i: i + total_sz]
    return None


def _parse_packet(raw: bytes):
    """Décode un paquet AD2CP (offsets validés sur S4VP fw 2.10) ; retourne un dict ou None."""
    if len(raw) < 70:
        return None
    try:
        sos   = struct.unpack_from('<f', raw, 42)[0]
        temp  = struct.unpack_from('<f', raw, 46)[0]
        press = struct.unpack_from('<f', raw, 50)[0]
        head  = struct.unpack_from('<f', raw, 54)[0]
        pitch = struct.unpack_from('<f', raw, 58)[0]
        roll  = struct.unpack_from('<f', raw, 62)[0]
        # S4VP encode pitch/roll en [0°, 360°) — normaliser vers [-180°, +180°]
        if pitch > 180.0:
            pitch -= 360.0
        if roll > 180.0:
            roll -= 360.0
        # Validation plausibilité
        if not (1400.0 <= sos <= 1600.0):
            return None
        if not (-5.0 <= temp <= 40.0):
            return None
        if not (-90.0 <= pitch <= 90.0):
            return None
        if not (-180.0 <= roll <= 180.0):
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


# ─── Nœud ROS 2 ──────────────────────────────────────────────────────────────

class AquadoppNode(Node):

    def __init__(self):
        super().__init__('aquadopp_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        self._port = self.get_parameter('port').value
        self._baud = self.get_parameter('baud').value

        self._pub = self.create_publisher(String, 'aquadopp/data', 10)
        self.get_logger().info(
            f"Aquadopp node démarré — port={self._port}  baud={self._baud}")

        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    # ── Publication helper ────────────────────────────────────────────────────

    def _publish(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

    # ── Boucle série (thread séparé) ──────────────────────────────────────────

    def _loop(self) -> None:
        while self._running:
            try:
                conn = serial.Serial(
                    self._port, self._baud,
                    bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE, timeout=2.0,
                )
                self.get_logger().info("Port série ouvert")

                # 1. Mode commande
                if not _enter_command_mode(conn, self.get_logger()):
                    self._publish({'status': 'error',
                                   'error': 'impossible d\'entrer en mode commande'})
                    conn.close()
                    time.sleep(10.0)
                    continue

                # 2. Configuration
                if not _configure(conn, self.get_logger()):
                    self.get_logger().warn("Configuration partielle — on continue")

                # 3. Démarrage
                conn.reset_input_buffer()
                conn.write(b'START\r\n')
                time.sleep(0.5)
                start_resp = _flush(conn, 0.5).decode('ascii', errors='replace')
                if 'OK' not in start_resp:
                    self._publish({'status': 'error',
                                   'error': f'START refusé : {start_resp[:30]}'})
                    conn.close()
                    time.sleep(5.0)
                    continue

                # Vider le burst ASCII post-START (log de configuration)
                _capture_burst(conn, idle_s=2.0, max_wait_s=20.0)
                self.get_logger().info(
                    "Mesure démarrée — attente des paquets binaires (~60 s)")

                # 4. Boucle de lecture
                while self._running:
                    raw = _read_packet(conn, self.get_logger(), timeout_s=300)
                    if raw is None:
                        self._publish({'status': 'error', 'error': 'timeout paquet'})
                        break

                    data = _parse_packet(raw)
                    if data:
                        fields = {}
                        for key, value in data.items():
                            fmt_str, unit = _FIELD_FMT.get(key, ('.3f', ''))
                            fields[key] = {
                                'value':   value,
                                'unit':    unit,
                                'display': f"{value:{fmt_str}}",
                            }
                        self._publish({
                            'status':    'ok',
                            'timestamp': time.strftime('%H:%M:%S'),
                            'fields':    fields,
                        })
                        self.get_logger().info(
                            f"Paquet OK — T={data['temperature_c']:.3f} °C  "
                            f"P={data['pressure_dbar']:.4f} dbar")
                    else:
                        self.get_logger().warn("Parsing du paquet échoué")

                conn.close()

            except serial.SerialException as e:
                self.get_logger().error(f"Erreur série : {e} — reconnexion dans 10 s")
                self._publish({'status': 'error', 'error': str(e)})
                time.sleep(10.0)
            except Exception as e:
                self.get_logger().error(f"Erreur inattendue : {e}")
                time.sleep(5.0)

    def destroy_node(self) -> None:
        self._running = False
        super().destroy_node()


# ─── Entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = AquadoppNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
