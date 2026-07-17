#!/usr/bin/env python3
"""
Aquadopp S4VP — Continuous measurement GUI
===========================================
Prerequisite: pyserial  (py -m pip install pyserial)

Usage:
    py test_aquadopp_sans_ros2.py              # GUI, COM3 @ 115200
    py test_aquadopp_sans_ros2.py --port COM5
"""

import argparse
import struct
import sys
import time
import threading

import serial

# ── Instrument config ────────────────────────────────────────────────────────

SYNC_BYTE = 0xA5
MEAS_INTERVAL_S = 30

CONFIG_COMMANDS = [
    (f'SETPLAN,MIAVG={MEAS_INTERVAL_S},AVG=1,SA=35,BURST=0,MIBURST=3600,SV=0,FN="Data",SO=1', 2.0),
    ('SETEXTSENSOR,EN=0,TYPE="",PWROUT="OFF"', 2.0),
    (f'SETAVG,NC=40,CS=2.5,BD=1,DF=7,CY="ENU",PL=0,AI={MEAS_INTERVAL_S},VR=2.5,NPING={MEAS_INTERVAL_S},NB=3,ZCELL=0', 2.0),
    ('SETTMAVG,EN=0,AVG=60,CY="ENU",FO=1,SO=0,DF=100,CD=1,PD=1,TV=1,TA=1,TC=1,DISTILT=0,TPG=0,MAPBINS=0,CORRTH=50', 2.0),
]

_FIELD_FMT = {
    'speed_of_sound_ms': ('.2f', 'm/s'),
    'temperature_c':     ('.3f', '°C'),
    'pressure_dbar':     ('.4f', 'dbar'),
    'heading_deg':       ('.2f', '°'),
    'pitch_deg':         ('.3f', '°'),
    'roll_deg':          ('.3f', '°'),
}


# ═════════════════════════════════════════════════════════════════════════════
#  AQUADOPP FUNCTIONS
# ═════════════════════════════════════════════════════════════════════════════

def _flush_read(conn, duration):
    buf, deadline = b'', time.time() + duration
    while time.time() < deadline:
        n = conn.in_waiting
        if n:
            buf += conn.read(n)
        else:
            time.sleep(0.05)
    return buf


def _cmd(conn, command, wait=1.5):
    conn.reset_input_buffer()
    conn.write((command + '\r\n').encode('ascii'))
    return _flush_read(conn, wait).decode('ascii', errors='replace').strip()


def enter_command_mode(conn, max_attempts=3):
    for attempt in range(1, max_attempts + 1):
        conn.reset_input_buffer()
        conn.send_break(duration=1.5)
        time.sleep(3.0)
        conn.reset_input_buffer()
        r = _cmd(conn, 'SETDEFAULT,CONFIG', wait=3.0)
        if 'OK' in r and 'ERROR' not in r:
            return True
    return False


def configure(conn):
    for command, wait in CONFIG_COMMANDS:
        r = _cmd(conn, command, wait=wait)
        if 'OK' not in r or 'ERROR' in r:
            return False
    return True


def start_measurement(conn):
    conn.reset_input_buffer()
    conn.write(b'START\r\n')
    time.sleep(0.5)
    resp = b''
    while conn.in_waiting:
        resp += conn.read(conn.in_waiting)
        time.sleep(0.05)
    return 'OK' in resp.decode('ascii', errors='replace')


def capture_burst(conn, idle_s=2.0, max_wait_s=90.0):
    deadline = time.time() + max_wait_s
    buf = b''
    while time.time() < deadline:
        n = conn.in_waiting
        if n:
            buf += conn.read(n)
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


def read_packet(conn, timeout_s=120):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        burst = capture_burst(conn, idle_s=2.0,
                              max_wait_s=min(90.0, int(deadline - time.time())))
        if not burst:
            return None
        for i in range(len(burst) - 9):
            if burst[i] != SYNC_BYTE or burst[i + 1] != 10:
                continue
            data_sz = struct.unpack_from('<H', burst, i + 4)[0]
            if 100 <= data_sz <= 4096 and i + 10 + data_sz <= len(burst):
                return burst[i:i + 10 + data_sz]
    return None


def parse_packet(raw):
    if len(raw) < 70:
        return None
    try:
        sos   = struct.unpack_from('<f', raw, 42)[0]
        temp  = struct.unpack_from('<f', raw, 46)[0]
        press = struct.unpack_from('<f', raw, 50)[0]
        head  = struct.unpack_from('<f', raw, 54)[0]
        roll  = struct.unpack_from('<f', raw, 62)[0]
        pitch = struct.unpack_from('<f', raw, 66)[0]
        if not (1400 <= sos <= 1600) or not (-5 <= temp <= 40):
            return None
        return {
            'speed_of_sound_ms': round(sos, 2),
            'temperature_c':     round(temp, 3),
            'pressure_dbar':     round(press, 4),
            'heading_deg':       round(head, 2),
            'pitch_deg':         round(pitch, 3),
            'roll_deg':          round(roll, 3),
        }
    except struct.error:
        return None


def build_fields(data):
    fields = {}
    for key, value in data.items():
        fmt_str, unit = _FIELD_FMT[key]
        fields[key] = {'value': value, 'unit': unit, 'display': f"{value:{fmt_str}}"}
    return fields


# ═════════════════════════════════════════════════════════════════════════════
#  BACKGROUND READER
# ═════════════════════════════════════════════════════════════════════════════

class SensorReader:
    def __init__(self, port, baud, on_data, on_log):
        self.port = port
        self.baud = baud
        self._on_data = on_data
        self._on_log  = on_log
        self.running = True

    def stop(self):
        self.running = False

    def _log(self, msg):
        self._on_log(msg)

    def _sleep(self, seconds):
        for _ in range(int(seconds / 0.1)):
            if not self.running:
                return False
            time.sleep(0.1)
        return self.running

    def loop(self):
        while self.running:
            try:
                self._log(f"Opening {self.port} @ {self.baud} baud...")
                conn = serial.Serial(
                    self.port, self.baud,
                    bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE, timeout=2.0)
                self._log("Port opened. Sending break...")

                if not enter_command_mode(conn):
                    self._log("ERROR: Cannot enter command mode. Retry in 10s...")
                    conn.close()
                    if not self._sleep(10):
                        return
                    continue

                self._log("Command mode OK. Configuring instrument...")
                if not configure(conn):
                    self._log("WARNING: Configuration partially failed")

                self._log("Configuration done. Sending START...")
                if not start_measurement(conn):
                    self._log("ERROR: START rejected. Retry in 5s...")
                    conn.close()
                    if not self._sleep(5):
                        return
                    continue

                self._log("START OK. Discarding boot burst...")
                capture_burst(conn, idle_s=2.0, max_wait_s=20.0)
                self._log(f"Ready. First measurement in ~{MEAS_INTERVAL_S}s...")

                count = 0
                while self.running:
                    t0 = time.time()
                    raw = read_packet(conn, timeout_s=MEAS_INTERVAL_S + 60)
                    if raw is None:
                        self._log("Timeout — no packet. Reconnecting...")
                        break
                    data = parse_packet(raw)
                    if data:
                        count += 1
                        self._on_data({
                            'status': 'ok',
                            'timestamp': time.strftime('%H:%M:%S'),
                            'fields': build_fields(data),
                            'count': count,
                        })
                        self._log(f"Measurement #{count} received. "
                                  f"Next in ~{MEAS_INTERVAL_S}s...")
                    else:
                        self._log("Packet received but parsing failed. Waiting...")

                conn.close()
                self._log("Port closed.")

            except serial.SerialException as e:
                self._log(f"Serial error: {e}. Retry in 10s...")
                self._sleep(10)
            except Exception as e:
                self._log(f"Unexpected error: {e}. Retry in 5s...")
                self._sleep(5)


# ═════════════════════════════════════════════════════════════════════════════
#  GUI
# ═════════════════════════════════════════════════════════════════════════════

BG           = "#1e1e1e"
PANEL_BG     = "#252526"
HEADER_BG    = "#2d2d30"
BORDER       = "#3e3e42"
TEXT_DIM     = "#858585"
TEXT_VAL     = "#ffffff"
TEXT_SECTION = "#4ec9b0"
COL_GREEN    = "#4ec9b0"
COL_RED      = "#f44747"
COL_YELLOW   = "#dcdcaa"

FONT_TITLE   = ("Consolas", 12, "bold")
FONT_SECTION = ("Consolas", 9, "bold")
FONT_LABEL   = ("Consolas", 9)
FONT_VALUE   = ("Consolas", 9, "bold")
FONT_CLOCK   = ("Consolas", 9)
FONT_LOG     = ("Consolas", 8)

GROUPS = [
    ("Acoustics", [
        ('speed_of_sound_ms', 'Speed of Sound', 'm/s'),
    ]),
    ("Environment", [
        ('temperature_c', 'Temperature', '°C'),
        ('pressure_dbar', 'Pressure',    'dbar'),
    ]),
    ("Orientation", [
        ('heading_deg', 'Heading', '°'),
        ('pitch_deg',   'Pitch',   '°'),
        ('roll_deg',    'Roll',    '°'),
    ]),
]


def run_gui(args):
    import tkinter as tk
    from tkinter import ttk
    from serial.tools import list_ports

    lock = threading.Lock()
    latest_data = {}
    latest_log  = ""
    reader = None

    def on_data(payload):
        nonlocal latest_data
        with lock:
            latest_data = payload

    def on_log(msg):
        nonlocal latest_log
        with lock:
            latest_log = f"[{time.strftime('%H:%M:%S')}] {msg}"

    # ── Window ────────────────────────────────────────────────────────────────

    root = tk.Tk()
    root.title("Aquadopp S4VP — Monitor")
    root.configure(bg=BG)
    root.geometry("580x520")

    # ── Title bar ─────────────────────────────────────────────────────────────

    top = tk.Frame(root, bg=HEADER_BG)
    top.pack(fill=tk.X)
    tk.Label(top, text="Aquadopp S4VP  --  Monitor",
             bg=HEADER_BG, font=("Consolas", 14, "bold"),
             fg=TEXT_VAL, pady=10).pack(side=tk.LEFT, padx=14)

    # ── Connection bar ────────────────────────────────────────────────────────

    conn_bar = tk.Frame(root, bg=HEADER_BG)
    conn_bar.pack(fill=tk.X)

    tk.Label(conn_bar, text="Port:", bg=HEADER_BG,
             font=FONT_LABEL, fg=TEXT_DIM).pack(side=tk.LEFT, padx=(14, 4), pady=6)
    port_var = tk.StringVar(value=args.port)
    port_combo = ttk.Combobox(conn_bar, textvariable=port_var,
                              width=12, font=FONT_LABEL)
    port_combo.pack(side=tk.LEFT, pady=6)

    tk.Label(conn_bar, text="Baud:", bg=HEADER_BG,
             font=FONT_LABEL, fg=TEXT_DIM).pack(side=tk.LEFT, padx=(10, 4), pady=6)
    baud_var = tk.StringVar(value=str(args.baud))
    ttk.Combobox(conn_bar, textvariable=baud_var, width=7,
                 values=('9600', '19200', '38400', '57600', '115200'),
                 font=FONT_LABEL).pack(side=tk.LEFT, pady=6)

    def refresh_ports():
        ports = [p.device for p in list_ports.comports()]
        port_combo['values'] = ports
        if ports and port_var.get() not in ports:
            port_var.set(ports[0])

    def do_start():
        nonlocal reader
        if reader and reader.running:
            return
        port = port_var.get().strip()
        if not port:
            on_log("No port selected")
            return
        try:
            baud = int(baud_var.get())
        except ValueError:
            on_log(f"Invalid baudrate: {baud_var.get()}")
            return
        reader = SensorReader(port, baud, on_data, on_log)
        threading.Thread(target=reader.loop, daemon=True).start()
        start_btn.config(state=tk.DISABLED)
        stop_btn.config(state=tk.NORMAL)

    def do_stop():
        nonlocal reader
        if reader:
            reader.stop()
            on_log("Stop requested — closing after current operation...")
            reader = None
        start_btn.config(state=tk.NORMAL)
        stop_btn.config(state=tk.DISABLED)

    tk.Button(conn_bar, text="Refresh", command=refresh_ports,
              bg=PANEL_BG, fg=TEXT_VAL, font=FONT_LABEL,
              activebackground=BORDER, activeforeground=TEXT_VAL,
              relief=tk.FLAT, padx=6).pack(side=tk.LEFT, padx=(10, 0), pady=6)

    start_btn = tk.Button(conn_bar, text="  START  ", command=do_start,
                          bg=COL_GREEN, fg=BG, font=FONT_VALUE,
                          activebackground=TEXT_SECTION, activeforeground=BG,
                          relief=tk.FLAT, padx=10)
    start_btn.pack(side=tk.LEFT, padx=(8, 0), pady=6)

    stop_btn = tk.Button(conn_bar, text="  STOP  ", command=do_stop,
                         bg=COL_RED, fg=TEXT_VAL, font=FONT_VALUE,
                         activebackground="#d03030", activeforeground=TEXT_VAL,
                         relief=tk.FLAT, padx=10, state=tk.DISABLED)
    stop_btn.pack(side=tk.LEFT, padx=(6, 14), pady=6)

    refresh_ports()

    # ── Sensor panel ──────────────────────────────────────────────────────────

    body = tk.Frame(root, bg=BG)
    body.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

    panel_frame = tk.Frame(body, bg=PANEL_BG,
                           highlightbackground=BORDER, highlightthickness=1)
    panel_frame.pack(fill=tk.BOTH, expand=True)

    # Header
    hdr = tk.Frame(panel_frame, bg=HEADER_BG)
    hdr.pack(fill=tk.X)
    dot_lbl = tk.Label(hdr, text="●", bg=HEADER_BG,
                       font=("Consolas", 11), fg=COL_RED)
    dot_lbl.pack(side=tk.LEFT, padx=(10, 4), pady=7)
    tk.Label(hdr, text="Aquadopp S4VP", bg=HEADER_BG,
             font=FONT_TITLE, fg=TEXT_VAL).pack(side=tk.LEFT, pady=7)
    ts_var = tk.StringVar(value="--:--:--")
    tk.Label(hdr, textvariable=ts_var, bg=HEADER_BG,
             font=FONT_CLOCK, fg=TEXT_DIM).pack(side=tk.RIGHT, padx=10, pady=7)
    count_var = tk.StringVar(value="")
    tk.Label(hdr, textvariable=count_var, bg=HEADER_BG,
             font=FONT_CLOCK, fg=TEXT_DIM).pack(side=tk.RIGHT, padx=4, pady=7)

    # Fields
    panel_body = tk.Frame(panel_frame, bg=PANEL_BG)
    panel_body.pack(fill=tk.BOTH, expand=True)
    field_vars = {}
    for group_title, fields in GROUPS:
        tk.Label(panel_body, text=f"  -- {group_title}",
                 bg=PANEL_BG, font=FONT_SECTION,
                 fg=TEXT_SECTION, anchor=tk.W).pack(fill=tk.X, pady=(10, 3), padx=6)
        for key, label, unit in fields:
            row = tk.Frame(panel_body, bg=PANEL_BG)
            row.pack(fill=tk.X, padx=10, pady=1)
            tk.Label(row, text=f"{label}:", bg=PANEL_BG,
                     font=FONT_LABEL, fg=TEXT_DIM,
                     width=28, anchor=tk.W).pack(side=tk.LEFT)
            v = tk.StringVar(value="---")
            tk.Label(row, textvariable=v, bg=PANEL_BG,
                     font=FONT_VALUE, fg=TEXT_VAL,
                     width=20, anchor=tk.E).pack(side=tk.LEFT)
            field_vars[key] = v

    # ── Log bar (bottom) ──────────────────────────────────────────────────────

    log_frame = tk.Frame(root, bg="#1a1a2e", pady=4)
    log_frame.pack(fill=tk.X, side=tk.BOTTOM)
    log_var = tk.StringVar(value="Press START to begin.")
    tk.Label(log_frame, textvariable=log_var,
             bg="#1a1a2e", font=FONT_LOG, fg="#7f7faf",
             anchor=tk.W).pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
    clock_var = tk.StringVar()
    tk.Label(log_frame, textvariable=clock_var,
             bg="#1a1a2e", font=FONT_LOG, fg="#5f5f8f").pack(side=tk.RIGHT, padx=10)

    # ── Tick loop ─────────────────────────────────────────────────────────────

    def tick():
        clock_var.set(time.strftime("%Y-%m-%d  %H:%M:%S"))
        with lock:
            data = dict(latest_data) if latest_data else None
            log  = latest_log

        if log:
            log_var.set(log)

        if data and data.get('status') == 'ok':
            dot_lbl.config(fg=COL_GREEN)
            ts_var.set(data.get('timestamp', '--:--:--'))
            c = data.get('count', 0)
            count_var.set(f"#{c}" if c else "")
            for key, v in field_vars.items():
                entry = data.get('fields', {}).get(key)
                if entry:
                    v.set(f"{entry['display']} {entry['unit']}")
        elif reader and reader.running:
            dot_lbl.config(fg=COL_YELLOW)

        # Update start/stop button state
        if reader and reader.running:
            start_btn.config(state=tk.DISABLED)
            stop_btn.config(state=tk.NORMAL)
        else:
            start_btn.config(state=tk.NORMAL)
            stop_btn.config(state=tk.DISABLED)

        root.after(300, tick)

    def on_close():
        if reader:
            reader.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(300, tick)
    root.mainloop()


# ═════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="Aquadopp S4VP Monitor")
    parser.add_argument('--port', default='COM3', help="serial port (default: COM3)")
    parser.add_argument('--baud', type=int, default=115200, help="baudrate (default: 115200)")
    args = parser.parse_args()
    run_gui(args)


if __name__ == '__main__':
    main()
