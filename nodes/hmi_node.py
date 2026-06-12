#!/usr/bin/env python3
"""
ROS 2 node — IHM tkinter MARBLE Sensors Monitor (5 capteurs).

Souscrit à :
  aanderaa/data, aquadopp/data, sbe37/data, rbrcoda3/data  (std_msgs/String JSON)
  /airmar/wind, /airmar/pressure, /airmar/temperature, /airmar/humidity,
  /airmar/heading, /airmar/orientation, /airmar/rain, /airmar/gps  (topics natifs)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float32MultiArray
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix, FluidPressure, Temperature

import tkinter as tk
from tkinter import ttk
import threading, json, time

# ─── Palette ──────────────────────────────────────────────────────────────────
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


# ─── SensorPanel (JSON) ───────────────────────────────────────────────────────
class SensorPanel(tk.Frame):
    def __init__(self, parent, title, groups, **kw):
        super().__init__(parent, bg=PANEL_BG,
                         highlightbackground=BORDER, highlightthickness=1, **kw)
        self._rows = {}
        hdr = tk.Frame(self, bg=HEADER_BG)
        hdr.pack(fill=tk.X)
        self._dot = tk.Label(hdr, text="●", bg=HEADER_BG, font=("Consolas", 11), fg=COL_RED)
        self._dot.pack(side=tk.LEFT, padx=(10, 4), pady=7)
        tk.Label(hdr, text=title, bg=HEADER_BG, font=FONT_TITLE, fg=TEXT_VAL).pack(side=tk.LEFT, pady=7)
        self._ts_var = tk.StringVar(value="--:--:--")
        tk.Label(hdr, textvariable=self._ts_var, bg=HEADER_BG, font=FONT_CLOCK, fg=TEXT_DIM).pack(side=tk.RIGHT, padx=10, pady=7)

        outer = tk.Frame(self, bg=PANEL_BG)
        outer.pack(fill=tk.BOTH, expand=True)
        canvas = tk.Canvas(outer, bg=PANEL_BG, highlightthickness=0, bd=0)
        vsb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        inner = tk.Frame(canvas, bg=PANEL_BG)
        win_id = canvas.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfig(win_id, width=e.width))

        for group_title, fields in groups:
            tk.Label(inner, text=f"  ── {group_title}", bg=PANEL_BG, font=FONT_SECTION,
                     fg=TEXT_SECTION, anchor=tk.W).pack(fill=tk.X, pady=(10, 3), padx=6)
            for key, label, default_unit in fields:
                row = tk.Frame(inner, bg=PANEL_BG)
                row.pack(fill=tk.X, padx=10, pady=1)
                tk.Label(row, text=f"{label}:", bg=PANEL_BG, font=FONT_LABEL,
                         fg=TEXT_DIM, width=30, anchor=tk.W).pack(side=tk.LEFT)
                val_var = tk.StringVar(value="---")
                tk.Label(row, textvariable=val_var, bg=PANEL_BG, font=FONT_VALUE,
                         fg=TEXT_VAL, width=18, anchor=tk.E).pack(side=tk.LEFT)
                self._rows[key] = (val_var, default_unit)

        stat = tk.Frame(self, bg=HEADER_BG)
        stat.pack(fill=tk.X, side=tk.BOTTOM)
        self._status_var = tk.StringVar(value="En attente du capteur...")
        tk.Label(stat, textvariable=self._status_var, bg=HEADER_BG,
                 font=FONT_CLOCK, fg=TEXT_DIM).pack(side=tk.LEFT, padx=10, pady=3)

    def update_data(self, data):
        status = data.get('status', 'unknown')
        if status == 'ok':
            self._dot.config(fg=COL_GREEN)
            self._ts_var.set(data.get('timestamp', '--:--:--'))
            self._status_var.set("Connecté  ●  données reçues")
            for key, (val_var, default_unit) in self._rows.items():
                entry = data.get('fields', {}).get(key)
                if entry:
                    unit = entry.get('unit', default_unit)
                    val_var.set(f"{entry.get('display','---')} {unit}".strip() if unit else entry.get('display','---'))
        elif status == 'error':
            self._dot.config(fg=COL_RED)
            err = data.get('error', '?')
            self._status_var.set("⚠ Veuillez connecter le capteur" if 'could not open port' in err.lower() else f"Erreur : {err[:55]}")
        elif status == 'no_data':
            self._dot.config(fg=COL_YELLOW)
            self._status_var.set("Connecté — aucune donnée reçue")
        else:
            self._dot.config(fg=COL_YELLOW)
            self._status_var.set("En attente du capteur...")


# ─── AirmarPanel (topics natifs) ──────────────────────────────────────────────
class AirmarPanel(tk.Frame):
    FIELDS = [
        ("Vent",        [("wind_dir","Direction","°"), ("wind_spd_kn","Vitesse","kn"), ("wind_spd_ms","Vitesse","m/s")]),
        ("Météo",       [("pressure_pa","Pression","Pa"), ("temp_c","Température air","°C"), ("humidity","Humidité","%")]),
        ("Orientation", [("heading","Cap magnétique","°"), ("pitch","Pitch","°"), ("roll","Roll","°")]),
        ("Pluie",       [("rain_mm","Accumulation","mm"), ("rain_dur","Durée","s"), ("rain_rate","Intensité","mm/h")]),
        ("GPS",         [("gps_lat","Latitude","°"), ("gps_lon","Longitude","°"), ("gps_alt","Altitude","m")]),
    ]

    def __init__(self, parent, **kw):
        super().__init__(parent, bg=PANEL_BG, highlightbackground=BORDER, highlightthickness=1, **kw)
        self._vars = {}
        hdr = tk.Frame(self, bg=HEADER_BG)
        hdr.pack(fill=tk.X)
        self._dot = tk.Label(hdr, text="●", bg=HEADER_BG, font=("Consolas", 11), fg=COL_RED)
        self._dot.pack(side=tk.LEFT, padx=(10, 4), pady=7)
        tk.Label(hdr, text="Airmar 150WXRS", bg=HEADER_BG, font=FONT_TITLE, fg=TEXT_VAL).pack(side=tk.LEFT, pady=7)
        self._ts_var = tk.StringVar(value="--:--:--")
        tk.Label(hdr, textvariable=self._ts_var, bg=HEADER_BG, font=FONT_CLOCK, fg=TEXT_DIM).pack(side=tk.RIGHT, padx=10, pady=7)

        outer = tk.Frame(self, bg=PANEL_BG)
        outer.pack(fill=tk.BOTH, expand=True)
        canvas = tk.Canvas(outer, bg=PANEL_BG, highlightthickness=0, bd=0)
        vsb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        inner = tk.Frame(canvas, bg=PANEL_BG)
        win_id = canvas.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfig(win_id, width=e.width))

        for group_title, fields in self.FIELDS:
            tk.Label(inner, text=f"  ── {group_title}", bg=PANEL_BG, font=FONT_SECTION,
                     fg=TEXT_SECTION, anchor=tk.W).pack(fill=tk.X, pady=(10, 3), padx=6)
            for key, label, unit in fields:
                row = tk.Frame(inner, bg=PANEL_BG)
                row.pack(fill=tk.X, padx=10, pady=1)
                tk.Label(row, text=f"{label}:", bg=PANEL_BG, font=FONT_LABEL,
                         fg=TEXT_DIM, width=30, anchor=tk.W).pack(side=tk.LEFT)
                v = tk.StringVar(value="---")
                tk.Label(row, textvariable=v, bg=PANEL_BG, font=FONT_VALUE,
                         fg=TEXT_VAL, width=18, anchor=tk.E).pack(side=tk.LEFT)
                self._vars[key] = (v, unit)

        stat = tk.Frame(self, bg=HEADER_BG)
        stat.pack(fill=tk.X, side=tk.BOTTOM)
        self._status_var = tk.StringVar(value="En attente du capteur...")
        tk.Label(stat, textvariable=self._status_var, bg=HEADER_BG,
                 font=FONT_CLOCK, fg=TEXT_DIM).pack(side=tk.LEFT, padx=10, pady=3)

    def _set(self, key, value, decimals=2):
        if key in self._vars:
            v, unit = self._vars[key]
            v.set(f"{value:.{decimals}f} {unit}".strip())
            self._dot.config(fg=COL_GREEN)
            self._ts_var.set(time.strftime("%H:%M:%S"))
            self._status_var.set("Connecté  ●  données reçues")

    def on_wind(self, msg):
        self._set("wind_dir", msg.x, 1); self._set("wind_spd_kn", msg.y, 1); self._set("wind_spd_ms", msg.z, 2)
    def on_pressure(self, msg):    self._set("pressure_pa", msg.fluid_pressure, 1)
    def on_temperature(self, msg): self._set("temp_c", msg.temperature, 2)
    def on_humidity(self, msg):    self._set("humidity", msg.data, 1)
    def on_heading(self, msg):     self._set("heading", msg.data, 1)
    def on_orientation(self, msg): self._set("pitch", msg.x, 2); self._set("roll", msg.y, 2)
    def on_rain(self, msg):
        if len(msg.data) >= 3:
            self._set("rain_mm", msg.data[0], 2); self._set("rain_dur", msg.data[1], 0); self._set("rain_rate", msg.data[2], 1)
    def on_gps(self, msg):
        self._set("gps_lat", msg.latitude, 6); self._set("gps_lon", msg.longitude, 6); self._set("gps_alt", msg.altitude, 1)


# ─── HMIApp ───────────────────────────────────────────────────────────────────
class HMIApp:
    AANDERAA_GROUPS = [
        ("Attitude", [
            ("Pitch","Pitch","°"), ("Roll","Roll","°"), ("Heading","Heading (Cap)","°"),
            ("StDev Pitch","Pitch — écart-type","°"), ("StDev Roll","Roll — écart-type","°"), ("StDev Heading","Cap — écart-type","°"),
        ]),
        ("Hauteur des vagues", [
            ("Significant Wave Height Hm0","Hm0 (significatif)","m"), ("Wave Height Wind Hm0","Hm0 (vent)","m"),
            ("Wave Height Swell Hm0","Hm0 (houle)","m"), ("Wave Height H1/3","H1/3","m"), ("Wave Height Hmax","Hmax","m"),
        ]),
        ("Périodes et directions", [
            ("Wave Mean Period Tz","Période moyenne Tz","s"), ("Wave Mean Period Tm02","Période moyenne Tm02","s"),
            ("Wave Peak Period Wind","Période pic (vent)","s"), ("Wave Peak Period Swell","Période pic (houle)","s"),
            ("Wave Peak Direction","Direction pic","°"), ("Wave Peak Direction Wind","Direction pic (vent)","°"),
            ("Wave Peak Direction Swell","Direction pic (houle)","°"), ("Wave Mean Direction","Direction moyenne","°"),
            ("Mean Spreading Angle","Angle de spreading","°"),
        ]),
        ("Système", [("Input Voltage","Tension d'alimentation","V"), ("Input Current","Courant d'alimentation","mA"), ("Memory Used","Mémoire utilisée","Bytes")]),
    ]
    AQUADOPP_GROUPS = [
        ("Environnement", [("speed_of_sound_ms","Vitesse du son","m/s"), ("temperature_c","Température","°C"), ("pressure_dbar","Pression","dbar")]),
        ("Orientation",   [("heading_deg","Cap (Heading)","°"), ("pitch_deg","Pitch","°"), ("roll_deg","Roll","°")]),
    ]
    SBE37_GROUPS = [
        ("CTD",     [("temperature_c","Température","°C"), ("conductivity_sm","Conductivité","S/m"), ("pressure_dbar","Pression","dbar")]),
        ("Dérivées",[("salinity_psu","Salinité","PSU"), ("sound_velocity_ms","Vitesse du son","m/s"), ("depth_m","Profondeur","m")]),
    ]
    RBR_GROUPS = [
        ("Mesures",  [("temperature_c","Température","°C"), ("pressure_dbar","Pression absolue","dbar")]),
        ("Dérivées", [("sea_pressure_dbar","Pression marine","dbar"), ("depth_m","Profondeur","m")]),
        ("Capteur",  [("model","Modèle",""), ("serial","N° série",""), ("firmware","Firmware",""), ("mode","Mode",""), ("period_ms","Période","ms")]),
        ("Flux",     [("sample_time","Horodatage capteur",""), ("sample_count","Échantillons reçus","")]),
    ]

    def __init__(self, node):
        self._node = node
        self._root = tk.Tk()
        self._root.title("MARBLE — Sensors Monitor")
        self._root.configure(bg=BG)
        self._root.geometry("2600x850")
        self._root.resizable(True, True)

        top_bar = tk.Frame(self._root, bg=HEADER_BG)
        top_bar.pack(fill=tk.X)
        tk.Label(top_bar, text="MARBLE  ──  Sensors Monitor", bg=HEADER_BG,
                 font=("Consolas", 14, "bold"), fg=TEXT_VAL, pady=10).pack(side=tk.LEFT, padx=14)

        cols = tk.Frame(self._root, bg=BG)
        cols.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)
        for i, w in enumerate([3, 2, 2, 2, 2]):
            cols.columnconfigure(i, weight=w)
        cols.rowconfigure(0, weight=1)

        self._a_panel = SensorPanel(cols, "AANDERAA Motus Wave Sensor 5729", self.AANDERAA_GROUPS)
        self._a_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        self._q_panel = SensorPanel(cols, "Aquadopp S4VP", self.AQUADOPP_GROUPS)
        self._q_panel.grid(row=0, column=1, sticky="nsew", padx=(5, 5))
        self._s_panel = SensorPanel(cols, "SBE 37-SIP MicroCAT", self.SBE37_GROUPS)
        self._s_panel.grid(row=0, column=2, sticky="nsew", padx=(5, 5))
        self._airmar_panel = AirmarPanel(cols)
        self._airmar_panel.grid(row=0, column=3, sticky="nsew", padx=(5, 5))
        self._r_panel = SensorPanel(cols, "RBRcoda3", self.RBR_GROUPS)
        self._r_panel.grid(row=0, column=4, sticky="nsew", padx=(5, 0))

        node.airmar_panel = self._airmar_panel

        bot = tk.Frame(self._root, bg=HEADER_BG, pady=3)
        bot.pack(fill=tk.X, side=tk.BOTTOM)
        tk.Label(bot, text="ROS 2  |  marble_sensors_hmi", bg=HEADER_BG,
                 font=FONT_CLOCK, fg=TEXT_DIM).pack(side=tk.LEFT, padx=12)
        self._clock_var = tk.StringVar()
        tk.Label(bot, textvariable=self._clock_var, bg=HEADER_BG,
                 font=FONT_CLOCK, fg=TEXT_DIM).pack(side=tk.RIGHT, padx=12)

        self._root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._root.after(500, self._tick)

    def _tick(self):
        self._clock_var.set(time.strftime("%Y-%m-%d  %H:%M:%S"))
        a, q, s, r = self._node.get_data()
        if a: self._a_panel.update_data(a)
        if q: self._q_panel.update_data(q)
        if s: self._s_panel.update_data(s)
        if r: self._r_panel.update_data(r)
        self._root.after(500, self._tick)

    def _on_close(self):
        self._node.get_logger().info("Fermeture de l'IHM")
        self._root.destroy()

    def run(self): self._root.mainloop()


# ─── Nœud ROS 2 ───────────────────────────────────────────────────────────────
class SensorsHMINode(Node):
    def __init__(self):
        super().__init__('sensors_hmi')
        self._lock = threading.Lock()
        self._aanderaa = self._aquadopp = self._sbe37 = self._rbrcoda3 = None
        self.airmar_panel = None

        self.create_subscription(String, 'aanderaa/data', self._cb_aanderaa, 10)
        self.create_subscription(String, 'aquadopp/data', self._cb_aquadopp, 10)
        self.create_subscription(String, 'sbe37/data',    self._cb_sbe37,    10)
        self.create_subscription(String, 'rbrcoda3/data', self._cb_rbrcoda3, 10)

        self.create_subscription(Vector3,           '/airmar/wind',        lambda m: self.airmar_panel and self.airmar_panel.on_wind(m),        10)
        self.create_subscription(FluidPressure,     '/airmar/pressure',    lambda m: self.airmar_panel and self.airmar_panel.on_pressure(m),    10)
        self.create_subscription(Temperature,       '/airmar/temperature', lambda m: self.airmar_panel and self.airmar_panel.on_temperature(m), 10)
        self.create_subscription(Float32,           '/airmar/humidity',    lambda m: self.airmar_panel and self.airmar_panel.on_humidity(m),    10)
        self.create_subscription(Float32,           '/airmar/heading',     lambda m: self.airmar_panel and self.airmar_panel.on_heading(m),     10)
        self.create_subscription(Vector3,           '/airmar/orientation', lambda m: self.airmar_panel and self.airmar_panel.on_orientation(m), 10)
        self.create_subscription(Float32MultiArray, '/airmar/rain',        lambda m: self.airmar_panel and self.airmar_panel.on_rain(m),        10)
        self.create_subscription(NavSatFix,         '/airmar/gps',         lambda m: self.airmar_panel and self.airmar_panel.on_gps(m),         10)

        self.get_logger().info("HMI node démarré — en attente des données capteurs")

    def _cb(self, attr, msg):
        try:
            with self._lock: setattr(self, attr, json.loads(msg.data))
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON invalide ({attr}): {e}")

    def _cb_aanderaa(self, msg): self._cb('_aanderaa', msg)
    def _cb_aquadopp(self, msg): self._cb('_aquadopp', msg)
    def _cb_sbe37(self, msg):    self._cb('_sbe37', msg)
    def _cb_rbrcoda3(self, msg): self._cb('_rbrcoda3', msg)

    def get_data(self):
        with self._lock:
            return (
                dict(self._aanderaa) if self._aanderaa else {},
                dict(self._aquadopp) if self._aquadopp else {},
                dict(self._sbe37)    if self._sbe37    else {},
                dict(self._rbrcoda3) if self._rbrcoda3 else {},
            )


def main(args=None):
    rclpy.init(args=args)
    node = SensorsHMINode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    HMIApp(node).run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
