#!/usr/bin/env python3
"""Analyse struct — envoie UN fire puis capture les pings."""
import socket, struct, sys, time
import numpy as np
sys.stdout.reconfigure(line_buffering=True)

OCULUS_ID = 0x4F53
HEADER_FMT = '<HHHHHIH'
HEADER_SIZE = struct.calcsize(HEADER_FMT)
IP = "169.254.106.24"

print(f"Connexion TCP {IP}:52100...")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(30.0)
sock.connect((IP, 52100))
print("Connecte!")

# Un seul fire message
fire_body = struct.pack('<BBBBBdddd', 1, 0, 0xFF, 127, 0x09,
                        15.0, 30.0, 0.0, 0.0)
fire_hdr = struct.pack(HEADER_FMT, OCULUS_ID, 0, 0, 0x0015, 0,
                       len(fire_body), 0)
sock.sendall(fire_hdr + fire_body)
print(f"Fire envoye ({len(fire_hdr)+len(fire_body)} bytes), attente...")

buf = b''
def fill(n):
    global buf
    while len(buf) < n:
        buf += sock.recv(65536)
def consume(n):
    global buf
    fill(n); data, buf = buf[:n], buf[n:]; return data

pings = []
for i in range(50):
    hdr = consume(HEADER_SIZE)
    oid = struct.unpack_from('<H', hdr, 0)[0]
    if oid != OCULUS_ID:
        buf = hdr[1:] + buf
        for _ in range(65536):
            fill(2)
            if struct.unpack_from('<H', buf, 0)[0] == OCULUS_ID: break
            buf = buf[1:]
        hdr = consume(HEADER_SIZE)
    fields = struct.unpack(HEADER_FMT, hdr)
    msg_id, psize = fields[3], fields[5]
    payload = consume(psize) if psize > 0 else b''
    names = {0x0001:'STATUS', 0x0080:'BANNER', 0x0040:'UNK40',
             0x0022:'PING', 0x0023:'PING2', 0x00FF:'DUMMY'}
    if msg_id != 0x00FF:  # skip dummy spam
        print(f"  [{i}] msg_id=0x{msg_id:04X} ({names.get(msg_id,'?')}) payload={psize}")
    if msg_id in (0x0022, 0x0023):
        pings.append(payload)
        if len(pings) >= 2:
            break
sock.close()

if not pings:
    print("Pas de ping recu!"); sys.exit(1)

p = pings[0]
print(f"\n=== Payload: {len(p)} bytes ===")

if len(pings) >= 2:
    first_diff = len(p)
    for i in range(min(len(p), len(pings[1]))):
        if p[i] != pings[1][i]:
            first_diff = i; break
    print(f"Premiere diff: offset {first_diff}")
    print(f"Image estimee: {len(p) - first_diff} bytes")

print("\n--- Hex dump 0-300 ---")
for off in range(0, min(300, len(p)), 16):
    chunk = p[off:off+16]
    hex_str = ' '.join(f'{b:02x}' for b in chunk)
    print(f"  {off:4d}  {hex_str:<48s}")

print("\n--- Scan freq (500000-1500000) ---")
for off in range(0, min(300, len(p)-8)):
    val = struct.unpack_from('<d', p, off)[0]
    if 500000 <= val <= 1500000:
        print(f"  offset {off}: {val:.0f}")

print("\n--- Scan temperature (5-40) ---")
for off in range(0, min(300, len(p)-8)):
    val = struct.unpack_from('<d', p, off)[0]
    if 5.0 <= val <= 40.0:
        print(f"  offset {off}: {val:.4f}")

print("\n--- Scan pressure (0.5-2.0) ---")
for off in range(0, min(300, len(p)-8)):
    val = struct.unpack_from('<d', p, off)[0]
    if 0.5 <= val <= 2.0:
        print(f"  offset {off}: {val:.6f}")

print("\n--- Scan gain (10-100) ---")
for off in range(0, min(300, len(p)-8)):
    val = struct.unpack_from('<d', p, off)[0]
    if 10.0 <= val <= 100.0:
        print(f"  offset {off}: {val:.2f}")

print("\nDone.")
