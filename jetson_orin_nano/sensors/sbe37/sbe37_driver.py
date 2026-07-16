#!/usr/bin/env python3
import serial
import time


def wakeup(conn: serial.Serial) -> None:
    for _ in range(3):
        conn.write(b'\r\n')
    time.sleep(0.5)
    conn.reset_input_buffer()


def cmd(conn: serial.Serial, command: str, wait: float = 2.0) -> str:
    conn.reset_input_buffer()
    conn.write((command + '\r\n').encode('ascii'))
    time.sleep(wait)
    return conn.read(conn.in_waiting or 1024).decode('ascii', errors='ignore').strip()


def parse_ts(raw: str) -> dict | None:
    for line in raw.splitlines():
        line = line.strip()
        if not line or line.startswith('<') or 'Error' in line or 'Executed' in line or line == 'TS':
            continue
        parts = [p.strip() for p in line.split(',')]
        if len(parts) >= 3:
            try:
                return {'T': float(parts[0]), 'C': float(parts[1]), 'P': float(parts[2])}
            except (ValueError, IndexError):
                continue
    return None


def salinity(C_sm: float, T_c: float, P_dbar: float) -> float:
    C_std = 4.2914 * 10.0
    R = (C_sm * 10.0) / C_std
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


def sound_velocity(T_c: float, S_psu: float, P_dbar: float) -> float:
    T, S, P = T_c, S_psu, P_dbar / 10.0
    Cw = (1402.388 + 5.03830 * T - 5.81090e-2 * T**2
          + 3.3432e-4 * T**3 - 1.47797e-6 * T**4 + 3.1419e-9 * T**5)
    A = (1.389 - 1.262e-2 * T + 7.166e-5 * T**2
         + 2.008e-6 * T**3 - 3.21e-9 * T**4)
    B = -1.922e-2 - 4.42e-5 * T
    D = 1.727e-3 - 7.9836e-6 * P
    return Cw + A * S + B * S**1.5 + D * S**2