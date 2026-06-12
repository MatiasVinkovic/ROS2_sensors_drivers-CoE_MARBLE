#!/usr/bin/env python3
"""
ROS 2 node — AANDERAA Motus Wave Sensor 5729 (mode SST).

Publie : /aanderaa/data  (std_msgs/String, JSON)

Format JSON publié :
  { "status": "ok" | "error" | "no_data",
    "timestamp": "HH:MM:SS",            (si status == ok)
    "error": "...",                      (si status == error)
    "fields": {                          (si status == ok)
      "<field_name>": {
        "value":   <float>,
        "unit":    "<str>",
        "display": "<str>"
      }, ...
    }
  }

Paramètres ROS 2 (surchargeables via launch ou CLI) :
  port            /dev/ttyUSB1
  baud            115200
  passkey         1
  sample_interval 10   (secondes entre deux mesures)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time
import json
import threading

# ─── Champs scalaires à extraire ─────────────────────────────────────────────

SCALAR_FIELDS = {
    "Pitch", "Roll", "Heading",
    "StDev Pitch", "StDev Roll", "StDev Heading",
    "Significant Wave Height Hm0",
    "Wave Height Wind Hm0", "Wave Height Swell Hm0",
    "Wave Height H1/3", "Wave Height Hmax",
    "Wave Mean Period Tz", "Wave Mean Period Tm02",
    "Wave Peak Period Wind", "Wave Peak Period Swell",
    "Wave Peak Direction", "Wave Peak Direction Wind", "Wave Peak Direction Swell",
    "Wave Mean Direction", "Mean Spreading Angle",
    "Input Voltage", "Input Current", "Memory Used",
}

# ─── Fonctions driver (extraites de test_aanderaa.py) ────────────────────────

def _read_all(ser, seconds: float) -> bytes:
    buf = b""
    deadline = time.time() + seconds
    last_rx = time.time()
    while time.time() < deadline:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            last_rx = time.time()
        else:
            if buf and (time.time() - last_rx) > 0.5:
                break
            time.sleep(0.01)
    return buf


def _sst(ser, cmd: str, wait: float = 3.0) -> str:
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    raw = _read_all(ser, wait)
    return raw.decode("ascii", errors="replace") if raw else ""


def _parse_do_output(text: str) -> dict:
    data = {}
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith("MEASUREMENT"):
            continue
        parts = line.split("\t")
        i = 3
        current_name = None
        current_unit = ""
        while i < len(parts):
            p = parts[i].strip()
            if p.startswith("*"):
                raw = p[1:]
                if "[" in raw and "]" in raw:
                    bracket = raw.index("[")
                    current_name = raw[:bracket].strip()
                    current_unit = raw[bracket + 1:raw.index("]")]
                else:
                    current_name = raw.strip()
                    current_unit = ""
                i += 1
            elif current_name and p:
                if current_name not in data:
                    data[current_name] = (p, current_unit)
                i += 1
            else:
                i += 1
    return data


def _fmt(raw_val: str, unit: str) -> str:
    try:
        f = float(raw_val)
        if unit in ("Bytes", "") and f == int(f):
            return str(int(f))
        if "Deg" in unit or unit in ("deg", "°"):
            return f"{f:.1f}"
        if unit == "m":
            return f"{f:.3f}"
        if unit == "s":
            return f"{f:.2f}"
        if unit in ("V", "mA"):
            return f"{f:.2f}"
        if abs(f) >= 1000 or (abs(f) < 0.001 and f != 0):
            return f"{f:.3e}"
        return f"{f:.3f}"
    except ValueError:
        return raw_val


# ─── Nœud ROS 2 ──────────────────────────────────────────────────────────────

class AanderaaNode(Node):

    def __init__(self):
        super().__init__('aanderaa_node')

        self.declare_parameter('port',            '/dev/ttyUSB1')
        self.declare_parameter('baud',            115200)
        self.declare_parameter('passkey',         '1')
        self.declare_parameter('sample_interval', 10)

        self._port     = self.get_parameter('port').value
        self._baud     = self.get_parameter('baud').value
        self._passkey  = self.get_parameter('passkey').value
        self._interval = self.get_parameter('sample_interval').value

        self._pub = self.create_publisher(String, 'aanderaa/data', 10)
        self.get_logger().info(
            f"AANDERAA node démarré — port={self._port}  baud={self._baud}")

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
                ser = serial.Serial(
                    port=self._port, baudrate=self._baud,
                    bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    xonxoff=True, rtscts=False, dsrdtr=False,
                    timeout=0.05,
                )
                self.get_logger().info("Port série ouvert — wake-up...")

                # Wake-up SST
                ser.reset_input_buffer()
                ser.write(b"\r")
                resp = _read_all(ser, 2.0).decode("ascii", errors="replace")
                if "!" in resp:
                    self.get_logger().info("SST wake-up confirmé")
                time.sleep(0.2)

                # Passkey
                r = _sst(ser, f"Set Passkey({self._passkey})")
                if '#' in r:
                    self.get_logger().info("Passkey OK")
                else:
                    self.get_logger().warn(f"Passkey réponse inattendue : {r.strip()[:40]}")

                # Boucle de mesure
                while self._running:
                    _sst(ser, "Do Sample", wait=6.0)
                    time.sleep(0.3)
                    r_out = _sst(ser, "Do Output", wait=6.0)
                    raw_data = _parse_do_output(r_out)

                    if raw_data:
                        fields = {}
                        for field, (raw_val, unit) in raw_data.items():
                            if field not in SCALAR_FIELDS:
                                continue
                            try:
                                fval = float(raw_val)
                            except ValueError:
                                continue
                            fields[field] = {
                                'value':   fval,
                                'unit':    unit,
                                'display': _fmt(raw_val, unit),
                            }
                        self._publish({
                            'status':    'ok',
                            'timestamp': time.strftime('%H:%M:%S'),
                            'fields':    fields,
                        })
                        self.get_logger().info(f"Publié {len(fields)} champs")
                    else:
                        self._publish({'status': 'no_data'})
                        self.get_logger().warn("Aucune donnée parsée dans Do Output")

                    time.sleep(self._interval)

                ser.close()

            except serial.SerialException as e:
                self.get_logger().error(f"Erreur série : {e} — reconnexion dans 5 s")
                self._publish({'status': 'error', 'error': str(e)})
                time.sleep(5.0)
            except Exception as e:
                self.get_logger().error(f"Erreur inattendue : {e}")
                time.sleep(5.0)

    def destroy_node(self) -> None:
        self._running = False
        super().destroy_node()


# ─── Entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = AanderaaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
