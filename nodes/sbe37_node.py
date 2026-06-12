#!/usr/bin/env python3
"""
ROS 2 node — Sea-Bird SBE 37-SIP MicroCAT (CTD).

Publie : /sbe37/data  (std_msgs/String, JSON)

Format JSON publié :
  { "status": "ok" | "error",
    "timestamp": "HH:MM:SS",
    "error": "...",             (si status == error)
    "fields": {                 (si status == ok)
      "temperature_c":       { "value": <float>, "unit": "°C",   "display": "<str>" },
      "conductivity_sm":     { "value": <float>, "unit": "S/m",  "display": "<str>" },
      "pressure_dbar":       { "value": <float>, "unit": "dbar", "display": "<str>" },
      "salinity_psu":        { "value": <float>, "unit": "PSU",  "display": "<str>" },
      "sound_velocity_ms":   { "value": <float>, "unit": "m/s",  "display": "<str>" },
      "depth_m":             { "value": <float>, "unit": "m",    "display": "<str>" }
    }
  }

Paramètres ROS 2 :
  port              COM11  (ou /dev/ttyUSB0 sous Linux)
  baud              9600
  sample_interval   10.0   (secondes entre deux TS)
"""

import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from std_msgs.msg import String

import serial
import time
import json
import threading

# ─── Physique ─────────────────────────────────────────────────────────────────

def _salinity(C_sm: float, T_c: float, P_dbar: float) -> float:
    """Salinité pratique PSS-78 (simplifiée)."""
    C_std_mscm = 4.2914 * 10.0
    R = (C_sm * 10.0) / C_std_mscm
    t = T_c
    rt = (0.6766097 + 0.0200564 * t + 1.104259e-4 * t**2
          - 6.9698e-7 * t**3 + 1.0031e-9 * t**4)
    Rp = 1.0 + (P_dbar * (2.07e-5 + (-6.37e-10) * P_dbar
                           + 3.989e-15 * P_dbar**2)) / (
        1 + 0.1478 * t + (-2.02e-4 * t**2) + R * (0.1133 + (-1.41e-3 * t)))
    Rt = max(0.0, R / (Rp * rt))
    sr = Rt ** 0.5
    S = (0.008 - 0.1692 * sr + 25.3851 * Rt + 14.0941 * Rt**1.5
         - 7.0261 * Rt**2 + 2.7081 * Rt**2.5)
    dS = ((t - 15.0) / (1.0 + 0.0162 * (t - 15.0))) * (
        0.0005 - 0.0056 * sr - 0.0066 * Rt
        - 0.0375 * Rt**1.5 + 0.0636 * Rt**2 - 0.0144 * Rt**2.5)
    return max(0.0, S + dS)


def _sound_velocity(T_c: float, S_psu: float, P_dbar: float) -> float:
    """Vitesse du son — formule UNESCO/Chen-Millero-Li."""
    T, S, P = T_c, S_psu, P_dbar / 10.0
    Cw = (1402.388 + 5.03830 * T - 5.81090e-2 * T**2
          + 3.3432e-4 * T**3 - 1.47797e-6 * T**4 + 3.1419e-9 * T**5)
    A = (1.389 - 1.262e-2 * T + 7.166e-5 * T**2
         + 2.008e-6 * T**3 - 3.21e-9 * T**4)
    B = -1.922e-2 - 4.42e-5 * T
    D = 1.727e-3 - 7.9836e-6 * P
    return Cw + A * S + B * S**1.5 + D * S**2


# ─── Communication série ──────────────────────────────────────────────────────

def _wake_up(conn: serial.Serial) -> None:
    for _ in range(3):
        conn.write(b'\r\n')
        time.sleep(0.5)
    conn.reset_input_buffer()


def _send(conn: serial.Serial, cmd: str, wait: float = 2.0) -> str:
    conn.reset_input_buffer()
    conn.write((cmd + '\r\n').encode('ascii'))
    time.sleep(wait)
    return conn.read(conn.in_waiting or 1024).decode('ascii', errors='ignore').strip()


def _parse_ts(raw: str) -> dict | None:
    """Parse la réponse TS du firmware v6.x. Retourne {T, C, P} ou None."""
    for line in raw.splitlines():
        line = line.strip()
        if (not line
                or line.startswith('<')
                or 'Error' in line
                or 'Executed' in line
                or line == 'TS'):
            continue
        parts = [p.strip() for p in line.split(',')]
        if len(parts) >= 3:
            try:
                return {
                    'T': float(parts[0]),
                    'C': float(parts[1]),
                    'P': float(parts[2]),
                }
            except (ValueError, IndexError):
                continue
    return None


# ─── Constante seuil eau/air ──────────────────────────────────────────────────

_MIN_CONDUCTIVITY_SM = 0.001   # S/m — en dessous : capteur hors eau

# ─── Format des champs publiés ────────────────────────────────────────────────

_FIELD_FMT = {
    'temperature_c':     ('.4f', '°C'),
    'conductivity_sm':   ('.6f', 'S/m'),
    'pressure_dbar':     ('.4f', 'dbar'),
    'salinity_psu':      ('.4f', 'PSU'),
    'sound_velocity_ms': ('.2f', 'm/s'),
    'depth_m':           ('.3f', 'm'),
}


# ─── Nœud ROS 2 ──────────────────────────────────────────────────────────────

class SBE37Node(Node):

    def __init__(self):
        super().__init__('sbe37_node')

        self.declare_parameter('port',            'COM11')
        self.declare_parameter('baud',            9600)
        self.declare_parameter('sample_interval', 10.0)

        self._port     = self.get_parameter('port').value
        self._baud     = self.get_parameter('baud').value
        self._interval = float(self.get_parameter('sample_interval').value)

        self._pub = self.create_publisher(String, 'sbe37/data', 10)
        self.get_logger().info(
            f"SBE37 node démarré — port={self._port}  baud={self._baud}  "
            f"interval={self._interval} s")

        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _publish(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

    def _loop(self) -> None:
        while self._running:
            try:
                conn = serial.Serial(
                    self._port, self._baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=3.0,
                )
                self.get_logger().info("Port série ouvert")
                _wake_up(conn)

                status_raw = _send(conn, 'DS', wait=2.0)
                self.get_logger().info(f"DS → {status_raw[:80]}")

                while self._running:
                    raw = _send(conn, 'TS', wait=3.0)
                    data = _parse_ts(raw)

                    if data is None:
                        self.get_logger().warn(f"Parsing échoué : {repr(raw[:60])}")
                        self._publish({
                            'status': 'error',
                            'error':  f'parsing échoué : {raw[:40]}',
                        })
                        _wake_up(conn)
                    else:
                        T, C, P    = data['T'], data['C'], data['P']
                        in_water   = C >= _MIN_CONDUCTIVITY_SM

                        if in_water:
                            sal = _salinity(max(0.0, C), T, P)
                            sos = _sound_velocity(T, sal, P)
                        else:
                            sal = None
                            sos = None
                            self.get_logger().warn(
                                f"Conductivité trop faible ({C:.6f} S/m) — "
                                "capteur hors eau : salinité/SOS non publiées")

                        def _fmt(key, val):
                            if val is None:
                                return {'value': None, 'unit': _FIELD_FMT[key][1],
                                        'display': 'N/A'}
                            return {'value': val, 'unit': _FIELD_FMT[key][1],
                                    'display': f"{val:{_FIELD_FMT[key][0]}}"}

                        fields = {
                            'temperature_c':     _fmt('temperature_c',     round(T,   4)),
                            'conductivity_sm':   _fmt('conductivity_sm',   round(C,   6)),
                            'pressure_dbar':     _fmt('pressure_dbar',     round(P,   4)),
                            'salinity_psu':      _fmt('salinity_psu',      round(sal, 4) if sal is not None else None),
                            'sound_velocity_ms': _fmt('sound_velocity_ms', round(sos, 2) if sos is not None else None),
                            'depth_m':           _fmt('depth_m',           round(P * 1.019716, 3)),
                        }
                        self._publish({
                            'status':    'ok',
                            'timestamp': time.strftime('%H:%M:%S'),
                            'fields':    fields,
                        })
                        log_sal = f"{sal:.4f}" if sal is not None else "N/A"
                        log_sos = f"{sos:.2f}" if sos is not None else "N/A"
                        self.get_logger().info(
                            f"TS OK — T={T:.4f} °C  C={C:.6f} S/m  "
                            f"P={P:.4f} dbar  PSU={log_sal}  SOS={log_sos} m/s")

                    # Attente interruptible
                    for _ in range(int(self._interval / 0.1)):
                        if not self._running:
                            break
                        time.sleep(0.1)

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
    node = SBE37Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
