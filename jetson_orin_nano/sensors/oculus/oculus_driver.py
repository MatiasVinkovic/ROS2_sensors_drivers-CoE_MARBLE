"""
Driver Oculus M750d — communication TCP directe.

Protocole reverse-engineered:
- Header: 16 bytes (oculusId, srcDeviceId, dstDeviceId, msgId, msgVersion, payloadSize, spare2)
- Fire message requis pour declencher le stream (0x0015), contient range + gain
- Reponse PING2 (0x0023): ~122096 bytes, metadata + image haute resolution
- Metadata PING2: range@5, gain@13, freq@45, temp@53
- Ancien format PING (0x0022): 33776 bytes (sans fire, obsolete)
"""

import socket
import struct
import math
import time
import numpy as np

OCULUS_ID = 0x4F53
HEADER_FMT = '<HHHHHIH'
HEADER_SIZE = struct.calcsize(HEADER_FMT)

TCP_PORT = 52100
UDP_PORT = 52102

MSG_PING = 0x0022
MSG_PING2 = 0x0023
SKIP_ROWS = 4


class OculusReader:
    """Lecteur bufferisé de messages Oculus via TCP."""

    def __init__(self, sock):
        self.sock = sock
        self.buf = b''

    def _fill(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("Connexion fermee par le sonar")
            self.buf += chunk

    def _consume(self, n):
        self._fill(n)
        data, self.buf = self.buf[:n], self.buf[n:]
        return data

    def read_message(self):
        """Lit un message complet (header + payload). Retourne (msg_id, payload)."""
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
                raise ValueError("Impossible de resynchroniser le flux Oculus")
            hdr = self._consume(HEADER_SIZE)

        fields = struct.unpack(HEADER_FMT, hdr)
        msg_id, payload_size = fields[3], fields[5]
        payload = self._consume(payload_size) if payload_size > 0 else b''
        return msg_id, payload


def discover(timeout=5.0, udp_port=UDP_PORT):
    """Decouverte UDP du sonar. Retourne l'IP ou None."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(timeout)
    sock.bind(('', udp_port))
    try:
        data, addr = sock.recvfrom(1024)
        if len(data) >= 2 and struct.unpack_from('<H', data, 0)[0] == OCULUS_ID:
            return addr[0]
    except socket.timeout:
        pass
    finally:
        sock.close()
    return None


def connect(ip, tcp_port=TCP_PORT, timeout=15.0, retries=3,
            range_m=15.0, gain_percent=30.0):
    """Connexion TCP au sonar + envoi du fire message (range, gain). Retourne (socket, OculusReader)."""
    last_exc = None
    for attempt in range(retries):
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            sock.connect((ip, tcp_port))

            fire_body = struct.pack(
                '<BBBBBdddd',
                1, 0, 0xFF, 127, 0x09,
                range_m, gain_percent, 0.0, 0.0
            )
            fire_hdr = struct.pack(
                HEADER_FMT, OCULUS_ID, 0, 0, 0x0015, 0, len(fire_body), 0
            )
            sock.sendall(fire_hdr + fire_body)
            return sock, OculusReader(sock)
        except Exception as e:
            last_exc = e
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
            if attempt < retries - 1:
                time.sleep(2)
    raise ConnectionError(f"Impossible de se connecter a {ip}:{tcp_port} ({last_exc})")


def extract_ping(payload):
    """Extrait les metadonnees et l'image d'un ping Oculus (PING ou PING2)."""
    plen = len(payload)
    if plen < 200:
        return None

    info = {}

    if plen > 50000:
        # Format PING2 (apres fire message)
        try:
            info['range_m'] = struct.unpack_from('<d', payload, 5)[0]
            info['gain_percent'] = struct.unpack_from('<d', payload, 13)[0]
            info['frequency_hz'] = struct.unpack_from('<d', payload, 45)[0]
            info['temperature_c'] = struct.unpack_from('<d', payload, 53)[0]
        except struct.error:
            pass

        for n_beams in (512, 256, 128):
            remainder = plen % n_beams
            if remainder < 80:
                continue
            img_offset = remainder
            n_ranges = (plen - img_offset) // n_beams
            if not (30 < n_ranges < 2000):
                continue
            raw = payload[img_offset:img_offset + n_ranges * n_beams]
            img = np.frombuffer(raw, dtype=np.uint8).reshape(n_ranges, n_beams).copy()
            if img.mean() > 0.5:
                info['image'] = img[SKIP_ROWS:] if n_ranges > SKIP_ROWS else img
                info['n_ranges'] = (n_ranges - SKIP_ROWS) if n_ranges > SKIP_ROWS else n_ranges
                info['n_beams'] = n_beams
                break
    else:
        # Format PING original (sans fire)
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


def build_fan_lut(n_ranges, n_beams, fov_deg, out_w, out_h):
    """Precalcule la LUT polaire -> cartesien pour l'affichage en eventail."""
    fov_rad = math.radians(fov_deg)
    half_fov = fov_rad / 2.0
    cx = out_w / 2.0
    cy = 10.0
    max_r = out_h - cy - 10

    y_coords, x_coords = np.mgrid[0:out_h, 0:out_w]
    dx = x_coords - cx
    dy = y_coords - cy
    r = np.sqrt(dx * dx + dy * dy)
    theta = np.arctan2(dx, dy)

    range_idx = (r / max_r * n_ranges).astype(np.int32)
    beam_idx = ((theta + half_fov) / fov_rad * n_beams).astype(np.int32)

    valid = ((range_idx >= 0) & (range_idx < n_ranges) &
             (beam_idx >= 0) & (beam_idx < n_beams) &
             (r > 5))

    return range_idx, beam_idx, valid


def make_sonar_colormap():
    """Colormap style sonar: noir -> bleu -> cyan -> vert -> jaune -> rouge -> blanc."""
    lut = np.zeros((256, 3), dtype=np.uint8)
    stops = [
        (0, (0, 0, 0)),
        (25, (0, 0, 60)),
        (50, (0, 20, 130)),
        (80, (0, 100, 180)),
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


def enhance_image(img_gray, mode='auto'):
    """Ameliore le contraste. Modes: auto, log, sqrt, histeq, raw."""
    if mode == 'raw':
        return img_gray
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
        return lut[img_gray.astype(np.uint8)]
    return np.clip(img, 0, 255).astype(np.uint8)