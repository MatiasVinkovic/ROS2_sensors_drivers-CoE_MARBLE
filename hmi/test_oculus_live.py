#!/usr/bin/env python3
"""
Oculus M750d — Image acoustique live en forme d'eventail (fan sector).
Fire message requis. Supporte PING2 (122k) et PING (33k).
"""
import socket, struct, time, sys, threading, traceback, math
import numpy as np
import tkinter as tk
sys.stdout.reconfigure(line_buffering=True)

try:
    from PIL import Image as PILImage, ImageTk, ImageDraw, ImageFont
except ImportError:
    print("pip install Pillow"); sys.exit(1)

OCULUS_ID    = 0x4F53
HEADER_FMT   = '<HHHHHIH'
HEADER_SIZE  = struct.calcsize(HEADER_FMT)
TCP_PORT     = 52100
SONAR_IP     = "169.254.106.24"
MSG_PING     = 0x0022
MSG_PING2    = 0x0023
SKIP_ROWS    = 4
FOV_DEG      = 130.0


def make_sonar_colormap():
    lut = np.zeros((256, 3), dtype=np.uint8)
    stops = [
        (0,   (0, 0, 0)),
        (25,  (0, 0, 60)),
        (50,  (0, 20, 130)),
        (80,  (0, 100, 180)),
        (110, (0, 180, 160)),
        (140, (30, 200, 80)),
        (170, (160, 220, 0)),
        (200, (240, 160, 0)),
        (230, (255, 60, 30)),
        (250, (255, 200, 200)),
        (255, (255, 255, 255)),
    ]
    for i in range(len(stops) - 1):
        i0, c0 = stops[i]
        i1, c1 = stops[i + 1]
        for j in range(i0, i1 + 1):
            t = (j - i0) / max(1, i1 - i0)
            lut[j] = [int(c0[k] + t * (c1[k] - c0[k])) for k in range(3)]
    return lut

COLORMAP = make_sonar_colormap()


def build_fan_lut(n_ranges, n_beams, fov_deg, out_size):
    """Precalcule la lookup table pour la conversion polaire -> cartesien."""
    fov_rad = math.radians(fov_deg)
    half_fov = fov_rad / 2.0
    ow, oh = out_size

    # Le sonar pointe vers le bas (convention), centre en haut
    cx = ow / 2.0
    cy = 10.0  # marge en haut
    max_r = oh - cy - 10

    # Pour chaque pixel de sortie, trouver (range_idx, beam_idx) correspondant
    y_coords, x_coords = np.mgrid[0:oh, 0:ow]
    dx = x_coords - cx
    dy = y_coords - cy

    r = np.sqrt(dx * dx + dy * dy)
    theta = np.arctan2(dx, dy)  # angle depuis l'axe vertical (vers le bas)

    range_idx = (r / max_r * n_ranges).astype(np.int32)
    beam_idx = ((theta + half_fov) / fov_rad * n_beams).astype(np.int32)

    valid = (range_idx >= 0) & (range_idx < n_ranges) & \
            (beam_idx >= 0) & (beam_idx < n_beams) & \
            (r > 5)  # trou central

    return range_idx, beam_idx, valid


def enhance_image(img_gray, mode='auto'):
    img = img_gray.astype(np.float32)
    if mode == 'auto':
        p_low = np.percentile(img, 2)
        p_high = np.percentile(img, 99.5)
        if p_high <= p_low:
            p_high = p_low + 1
        img = (img - p_low) / (p_high - p_low) * 255.0
    elif mode == 'log':
        img = np.log1p(img) / np.log1p(255) * 255.0
    elif mode == 'sqrt':
        img = np.sqrt(img / 255.0) * 255.0
    elif mode == 'histeq':
        hist, _ = np.histogram(img.flatten(), 256, [0, 256])
        cdf = hist.cumsum()
        cdf_min = cdf[cdf > 0].min()
        lut = ((cdf - cdf_min) / (img.size - cdf_min) * 255).astype(np.uint8)
        return lut[img_gray]
    return np.clip(img, 0, 255).astype(np.uint8)


class OculusReader:
    def __init__(self, sock):
        self.sock = sock
        self.buf = b''

    def _fill(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("Fermee")
            self.buf += chunk

    def _consume(self, n):
        self._fill(n)
        data, self.buf = self.buf[:n], self.buf[n:]
        return data

    def read_message(self):
        hdr = self._consume(HEADER_SIZE)
        oid = struct.unpack_from('<H', hdr, 0)[0]
        if oid != OCULUS_ID:
            self.buf = hdr[1:] + self.buf
            for _ in range(131072):
                self._fill(2)
                if struct.unpack_from('<H', self.buf, 0)[0] == OCULUS_ID:
                    break
                self.buf = self.buf[1:]
            else:
                raise ValueError("Resync failed")
            hdr = self._consume(HEADER_SIZE)
        fields = struct.unpack(HEADER_FMT, hdr)
        msg_id, payload_size = fields[3], fields[5]
        payload = self._consume(payload_size) if payload_size > 0 else b''
        return msg_id, payload


def extract_ping(payload):
    plen = len(payload)
    if plen < 200:
        return None

    info = {}

    if plen > 50000:
        # PING2 format (after fire message)
        try:
            info['range_m'] = struct.unpack_from('<d', payload, 5)[0]
            info['gain_percent'] = struct.unpack_from('<d', payload, 13)[0]
            info['frequency_hz'] = struct.unpack_from('<d', payload, 45)[0]
            info['temperature_c'] = struct.unpack_from('<d', payload, 53)[0]
        except struct.error:
            pass
        # Prefer higher beam counts: try 512, 256, 128 at the smallest valid offset
        for n_beams in [512, 256, 128]:
            remainder = plen % n_beams
            if remainder < 80:
                continue
            img_offset = remainder
            n_ranges = (plen - img_offset) // n_beams
            if 30 <= n_ranges <= 2000:
                raw = payload[img_offset:img_offset + n_ranges * n_beams]
                img = np.frombuffer(raw, dtype=np.uint8).reshape(n_ranges, n_beams).copy()
                if img.mean() > 0.5:
                    info['image'] = img[SKIP_ROWS:] if n_ranges > SKIP_ROWS else img
                    info['n_ranges'] = (n_ranges - SKIP_ROWS) if n_ranges > SKIP_ROWS else n_ranges
                    info['n_beams'] = n_beams
                    break
    else:
        # Old PING format (passive, no fire)
        try:
            info['pressure_bar'] = struct.unpack_from('<d', payload, 9)[0]
            info['frequency_hz'] = struct.unpack_from('<d', payload, 17)[0]
            info['gain_percent'] = struct.unpack_from('<d', payload, 59)[0]
            info['temperature_c'] = struct.unpack_from('<d', payload, 88)[0]
        except struct.error:
            pass
        img_offset = 240
        n_ranges, n_beams = 131, 256
        total = n_ranges * n_beams
        if plen >= img_offset + total:
            raw = payload[img_offset:img_offset + total]
            full = np.frombuffer(raw, dtype=np.uint8).reshape(n_ranges, n_beams).copy()
            info['image'] = full[SKIP_ROWS:]
            info['n_ranges'] = n_ranges - SKIP_ROWS
            info['n_beams'] = n_beams

    return info if 'image' in info else None


class App:
    MODES = ['auto', 'log', 'sqrt', 'histeq', 'raw']
    MODE_LABELS = {
        'auto': 'Auto (percentile)',
        'log': 'Logarithmique',
        'sqrt': 'Racine carree',
        'histeq': 'Egalisation histo',
        'raw': 'Brut (sans filtre)',
    }

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Oculus M750d — Live")
        self.root.geometry("900x750")
        self.root.configure(bg='#0a0a14')
        self.mode = 'auto'
        self.use_color = True
        self.fan_lut = None
        self.fan_size = (0, 0)

        top = tk.Frame(self.root, bg='#0a0a14')
        top.pack(fill='x', padx=10, pady=(8, 0))
        tk.Label(top, text="OCULUS M750d",
                 font=('Segoe UI', 16, 'bold'), fg='#00d4ff',
                 bg='#0a0a14').pack(side='left')
        tk.Label(top, text="  Imaging Sonar — Live View",
                 font=('Segoe UI', 11), fg='#4488aa',
                 bg='#0a0a14').pack(side='left', pady=(4, 0))

        ctrl = tk.Frame(self.root, bg='#0a0a14')
        ctrl.pack(fill='x', padx=10, pady=4)
        tk.Label(ctrl, text="Filtre:", font=('Consolas', 9),
                 fg='#888', bg='#0a0a14').pack(side='left', padx=(0, 5))
        for m in self.MODES:
            btn = tk.Button(ctrl, text=self.MODE_LABELS[m],
                            font=('Consolas', 8),
                            fg='#00d4ff' if m == self.mode else '#555',
                            bg='#151528', activebackground='#2a2a4e',
                            activeforeground='#fff', relief='flat',
                            padx=6, pady=2,
                            command=lambda mode=m: self._set_mode(mode))
            btn.pack(side='left', padx=2)
            btn._mode = m

        self.color_btn = tk.Button(ctrl, text="Couleur: ON",
                                   font=('Consolas', 8), fg='#0f0',
                                   bg='#151528', relief='flat', padx=6, pady=2,
                                   command=self._toggle_color)
        self.color_btn.pack(side='left', padx=(10, 0))

        self.status = tk.Label(self.root, text="Demarrage...",
                               font=('Consolas', 10), fg='#aaa', bg='#0a0a14')
        self.status.pack()
        self.canvas = tk.Canvas(self.root, bg='black', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True, padx=10, pady=5)

        bot = tk.Frame(self.root, bg='#0a0a14')
        bot.pack(fill='x', padx=10, pady=(0, 8))
        self.labels = {}
        fields = ['frequency_hz', 'temperature_c',
                  'range_m', 'gain_percent', 'n_ranges', 'n_beams']
        for i, n in enumerate(fields):
            c, r = (i % 3) * 2, i // 3
            tk.Label(bot, text=f"{n}:", font=('Consolas', 9),
                     fg='#555', bg='#0a0a14').grid(row=r, column=c,
                     sticky='e', padx=2)
            lbl = tk.Label(bot, text="—", font=('Consolas', 9, 'bold'),
                           fg='#0f0', bg='#0a0a14', width=14)
            lbl.grid(row=r, column=c + 1, sticky='w', padx=(2, 20))
            self.labels[n] = lbl

        self._photo = None
        self._count = 0
        self._last = 0

    def _set_mode(self, mode):
        self.mode = mode
        for w in self.root.winfo_children():
            if isinstance(w, tk.Frame):
                for btn in w.winfo_children():
                    if isinstance(btn, tk.Button) and hasattr(btn, '_mode'):
                        btn.configure(
                            fg='#00d4ff' if btn._mode == mode else '#555')

    def _toggle_color(self):
        self.use_color = not self.use_color
        self.color_btn.configure(
            text=f"Couleur: {'ON' if self.use_color else 'OFF'}",
            fg='#0f0' if self.use_color else '#555')

    def set_status(self, txt, color='#aaa'):
        self.status.configure(text=txt, fg=color)

    def _get_fan_lut(self, n_ranges, n_beams, cw, ch):
        key = (n_ranges, n_beams, cw, ch)
        if self.fan_lut is None or self.fan_size != key:
            self.fan_size = key
            self.fan_lut = build_fan_lut(n_ranges, n_beams, FOV_DEG, (cw, ch))
        return self.fan_lut

    def show_ping(self, info):
        now = time.time()
        if now - self._last < 0.08:
            return
        self._last = now
        self._count += 1

        img = info.get('image')
        if img is not None:
            cw = max(self.canvas.winfo_width(), 200)
            ch = max(self.canvas.winfo_height(), 200)
            nr, nb = img.shape

            if self.mode != 'raw':
                enhanced = enhance_image(img, self.mode)
            else:
                enhanced = img

            # Conversion polaire -> cartesien (fan sector)
            ri, bi, valid = self._get_fan_lut(nr, nb, cw, ch)

            if self.use_color:
                rgb_lut = COLORMAP[enhanced]  # (nr, nb, 3)
                out = np.zeros((ch, cw, 3), dtype=np.uint8)
                out[valid] = rgb_lut[ri[valid], bi[valid]]
                pil = PILImage.fromarray(out, mode='RGB')
            else:
                out = np.zeros((ch, cw), dtype=np.uint8)
                out[valid] = enhanced[ri[valid], bi[valid]]
                pil = PILImage.fromarray(out, mode='L')

            # Dessiner les arcs de distance
            draw = ImageDraw.Draw(pil)
            cx_f = cw / 2.0
            cy_f = 10.0
            max_r = ch - cy_f - 10
            half_fov = FOV_DEG / 2.0
            arc_color = (40, 60, 80) if self.use_color else 40
            for frac in [0.25, 0.5, 0.75, 1.0]:
                r = max_r * frac
                bbox = [cx_f - r, cy_f - r, cx_f + r, cy_f + r]
                start_angle = 90 - half_fov
                draw.arc(bbox, start=start_angle, end=start_angle + FOV_DEG,
                         fill=arc_color, width=1)

            # Lignes des bords du secteur
            for angle_deg in [-half_fov, half_fov]:
                angle_rad = math.radians(angle_deg)
                ex = cx_f + max_r * math.sin(angle_rad)
                ey = cy_f + max_r * math.cos(angle_rad)
                draw.line([(cx_f, cy_f), (ex, ey)], fill=arc_color, width=1)

            self._photo = ImageTk.PhotoImage(pil)
            self.canvas.delete('all')
            self.canvas.create_image(0, 0, anchor='nw', image=self._photo)

        for n, lbl in self.labels.items():
            v = info.get(n)
            if v is None:
                continue
            if isinstance(v, float):
                lbl.configure(text=f"{v:.0f}" if abs(v) > 10000 else f"{v:.2f}")
            else:
                lbl.configure(text=str(v))

        nr = info.get('n_ranges', '?')
        nb = info.get('n_beams', '?')
        self.set_status(
            f"{nr}x{nb}  |  {FOV_DEG:.0f}deg  |  "
            f"T={info.get('temperature_c', 0):.1f}C  |  "
            f"R={info.get('range_m', 0):.1f}m  |  "
            f"[{self.mode}]  frame #{self._count}", '#0f0')


def acquisition(app):
    def st(txt, col='#aaa'):
        app.root.after(0, app.set_status, txt, col)

    try:
        for attempt in range(5):
            try:
                st(f"Connexion {SONAR_IP}:{TCP_PORT}... (essai {attempt+1}/5)", '#ff0')
                print(f"Tentative {attempt+1}: connexion TCP {SONAR_IP}:{TCP_PORT}...")
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(15.0)
                sock.connect((SONAR_IP, TCP_PORT))
                print("Connecte!")
                break
            except Exception as e:
                print(f"Echec: {e}")
                sock.close()
                if attempt < 4:
                    time.sleep(2)
        else:
            st("Impossible de se connecter apres 5 essais", '#f00')
            print("Abandon apres 5 tentatives")
            return
        reader = OculusReader(sock)
        st("Connecte — envoi fire message...", '#ff0')

        # Envoyer un fire message pour declencher le stream de pings
        fire_body = struct.pack('<BBBBBdddd',
                                1,     # masterMode: 1=LF
                                0,     # pingRate: 0=normal
                                0xFF,  # networkSpeed
                                127,   # gammaCorrection
                                0x09,  # flags: 0x01=gain_assist + 0x08=simple_return
                                15.0,  # range (m)
                                30.0,  # gainPercent
                                0.0,   # speedOfSound (0=auto)
                                0.0)   # salinity
        fire_hdr = struct.pack(HEADER_FMT,
                               OCULUS_ID, 0, 0, 0x0015, 0,
                               len(fire_body), 0)
        sock.sendall(fire_hdr + fire_body)
        print(f"Fire message envoye ({len(fire_hdr)+len(fire_body)} bytes)")
        st("Connecte — attente pings...", '#ff0')

        count = 0
        while True:
            try:
                msg_id, payload = reader.read_message()
            except socket.timeout:
                st("Timeout recv...", '#ff0')
                continue
            except Exception as e:
                st(f"Erreur lecture: {e}", '#f00')
                break

            if msg_id in (MSG_PING, MSG_PING2):
                info = extract_ping(payload)
                if info and 'image' in info:
                    count += 1
                    info['frame'] = count
                    app.root.after(0, app.show_ping, info)
                    if count <= 2:
                        img = info['image']
                        print(f"Ping {count}: {img.shape[0]}x{img.shape[1]} "
                              f"freq={info.get('frequency_hz', 0):.0f}Hz "
                              f"T={info.get('temperature_c', 0):.1f}C "
                              f"min={img.min()} max={img.max()} "
                              f"mean={img.mean():.1f}")

    except Exception as e:
        st(f"Erreur: {e}", '#f00')
        traceback.print_exc()


def main():
    app = App()
    threading.Thread(target=acquisition, args=(app,), daemon=True).start()
    app.root.mainloop()

if __name__ == "__main__":
    main()
