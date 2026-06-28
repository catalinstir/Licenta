#!/usr/bin/env python3
"""
UDP telemetry logger for NXP CUP car.

Receives broadcast from ESP8266 on UDP port 4210, parses all telemetry
lines, and saves a timestamped CSV for post-run analysis with plot_results.py.

Usage:
    python tools/data_logger.py --label pid
    python tools/data_logger.py --label lqr --output tools/data/
"""

import argparse
import csv
import re
import socket
import sys
import time
from datetime import datetime
from pathlib import Path

UDP_PORT = 4210

# 200 ms telemetry from STATE_WAIT
# "distance:50 steer:700 motor_speed:770 rpm:834.5 speed:22.3 vectors:2"
RE_TELEMETRY = re.compile(
    r'distance:(\d+)\s+steer:(\d+)\s+motor_speed:(\d+)\s+'
    r'rpm:(-?[0-9]+\.[0-9]+)\s+speed:(-?[0-9]+\.[0-9]+)\s+vectors:(\d+)'
)

# ~558 ms control print from STATE_CONTROL (LQR)
# "-STEER:700 SPEED:770 e_lat:0.050 theta_e:-0.034"
RE_CONTROL_LQR = re.compile(
    r'-STEER:(\d+)\s+SPEED:(\d+)\s+e_lat:(-?[0-9]+\.[0-9]+)\s+theta_e:(-?[0-9]+\.[0-9]+)'
)

# ~558 ms control print from STATE_CONTROL (PID)
# "-ERR:5.300 CONTROL:0.238 STEER:800 SPEED:770"
RE_CONTROL_PID = re.compile(
    r'-ERR:(-?[0-9]+\.[0-9]+)\s+CONTROL:(-?[0-9]+\.[0-9]+)\s+STEER:(\d+)\s+SPEED:(\d+)'
)

# Lines to silently discard (Motor_SetPwm noise, boot messages)
IGNORE_PREFIXES = ('SET speed:', 'SET steer:', 'Hello world', 'Hall Sensor',
                   'The battery', '-REQ_FRAME', 'Simulator_GetFrame',
                   '-RC TIMEOUT', '-DT:')

# Events worth recording as marker rows — (firmware_pattern, clean_label).
# More-specific patterns must come before shorter ones that are substrings.
EVENTS = [
    ('-INTERSECTION done',          'intersection_done'),
    ('-INTERSECTION (0 vectors)',   'intersection_no_vectors'),
    ('-INTERSECTION (bad vectors)', 'intersection_bad_vectors'),
    ('-INTERSECTION',               'intersection_real'),
    ('-STOP',                       'stop'),
    ('-RC MODE',                    'rc_mode'),
    ('-AUTO MODE',                  'auto_mode'),
]

CSV_FIELDS = [
    'time_s', 'record_type',
    'steer', 'motor_speed', 'rpm', 'speed_kmh', 'distance', 'vectors',
    'e_lat', 'theta_e',
    'pid_error', 'pid_control',
    'event',
]

EMPTY = {f: '' for f in CSV_FIELDS}


def parse_line(line: str, t: float) -> dict | None:
    if not line or any(line.startswith(p) for p in IGNORE_PREFIXES):
        return None

    m = RE_TELEMETRY.search(line)
    if m:
        return {**EMPTY,
                'time_s': f'{t:.3f}', 'record_type': 'telemetry',
                'distance': m.group(1), 'steer': m.group(2),
                'motor_speed': m.group(3), 'rpm': m.group(4),
                'speed_kmh': m.group(5), 'vectors': m.group(6)}

    m = RE_CONTROL_LQR.search(line)
    if m:
        return {**EMPTY,
                'time_s': f'{t:.3f}', 'record_type': 'control_lqr',
                'steer': m.group(1), 'motor_speed': m.group(2),
                'e_lat': m.group(3), 'theta_e': m.group(4)}

    m = RE_CONTROL_PID.search(line)
    if m:
        return {**EMPTY,
                'time_s': f'{t:.3f}', 'record_type': 'control_pid',
                'pid_error': m.group(1), 'pid_control': m.group(2),
                'steer': m.group(3), 'motor_speed': m.group(4)}

    for pattern, label in EVENTS:
        if pattern in line:
            return {**EMPTY,
                    'time_s': f'{t:.3f}', 'record_type': 'event',
                    'event': label}

    return None


def main():
    p = argparse.ArgumentParser(description='Log NXP CUP car telemetry from UDP.')
    p.add_argument('--label', required=True,
                   help='Run label written into the filename, e.g. pid / lqr / lqr_coupling')
    p.add_argument('--port', type=int, default=UDP_PORT,
                   help=f'UDP port to listen on (default: {UDP_PORT})')
    p.add_argument('--output', default='tools/data',
                   help='Directory for CSV files (default: tools/data/)')
    args = p.parse_args()

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = out_dir / f'{args.label}_{ts}.csv'

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.bind(('', args.port))
    sock.settimeout(1.0)

    print(f'Label  : {args.label}')
    print(f'Output : {csv_path}')
    print(f'Port   : UDP {args.port}')
    print('Waiting for packets — press Ctrl+C to finish.\n')

    start_time: float | None = None
    row_count = 0
    ctrl_count = 0

    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()

        try:
            while True:
                try:
                    data, _ = sock.recvfrom(512)
                except socket.timeout:
                    continue

                now = time.monotonic()
                if start_time is None:
                    start_time = now
                t = now - start_time

                line = data.decode('utf-8', errors='replace').strip().strip('\r')
                row = parse_line(line, t)
                if row is None:
                    continue

                writer.writerow(row)
                row_count += 1

                if row['record_type'] == 'control_lqr':
                    ctrl_count += 1
                    print(f't={t:7.1f}s  e_lat={float(row["e_lat"])*100:+6.1f}cm  '
                          f'theta_e={float(row["theta_e"]):+6.3f}rad  steer={row["steer"]}')
                elif row['record_type'] == 'control_pid':
                    ctrl_count += 1
                    print(f't={t:7.1f}s  err={row["pid_error"]}  '
                          f'ctrl={row["pid_control"]}  steer={row["steer"]}')
                elif row['record_type'] == 'event':
                    print(f't={t:7.1f}s  ** {row["event"]} **')

        except KeyboardInterrupt:
            pass

    sock.close()
    print(f'\nDone. {row_count} rows ({ctrl_count} control), saved to:\n  {csv_path}')


if __name__ == '__main__':
    main()
