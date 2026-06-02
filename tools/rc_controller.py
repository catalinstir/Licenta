#!/usr/bin/env python3
"""
rc_controller.py — Xbox One controller → ESP8266 UDP → FRDM-K66F bridge

Communication chain:
  Laptop ←─ UDP broadcast port 4210 ─→ ESP8266 ←─ UART 153600 ─→ K66F

Protocol sent to car (ESP8266 forwards verbatim to K66F UART):
  S\r           emergency stop  (any state)
  R\r           enter RC mode
  A\r           return to autonomous
  <steer> <speed>\r   RC control packet (steer 500-900, speed duty×100)

Discovery: waits for the first UDP broadcast from the ESP8266, then sends
           commands back to that IP.

Usage:
  python3 rc_controller.py [--port 4210]

Dependencies:
  pip install inputs
"""

import argparse
import socket
import threading
import time
import sys

try:
    import inputs
except ImportError:
    print("Missing 'inputs' library. Run: pip install inputs")
    sys.exit(1)

# ---------------------------------------------------------------------------
# PWM mapping constants — must match firmware config.h
# ---------------------------------------------------------------------------
STEER_NEUTRAL = 700
STEER_MIN     = 500
STEER_MAX     = 900

SPEED_STOP    = 770   # duty×100 when trigger is fully released
SPEED_MAX     = 790   # duty×100 at full trigger (conservative start)

SEND_INTERVAL = 0.05  # 20 Hz — keeps firmware watchdog (500 ms) fed


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def map_axis(raw: int, in_min: int, in_max: int, out_min: int, out_max: int) -> int:
    ratio = (raw - in_min) / (in_max - in_min)
    ratio = max(0.0, min(1.0, ratio))
    return round(out_min + ratio * (out_max - out_min))


def snap_steer(value: int) -> int:
    """Round to the nearest discrete PWM step the firmware accepts."""
    steps = [500, 600, 700, 800, 900]
    return min(steps, key=lambda s: abs(s - value))


# ---------------------------------------------------------------------------
# Shared state between controller-reader and serial-sender threads
# ---------------------------------------------------------------------------

class State:
    def __init__(self):
        self.steer = STEER_NEUTRAL
        self.speed = SPEED_STOP
        self.mode  = "auto"   # "auto" | "rc"
        self.stop  = False
        self.lock  = threading.Lock()


# ---------------------------------------------------------------------------
# Xbox One axis / button codes (XInput via 'inputs' library)
# ---------------------------------------------------------------------------
ABS_X     = "ABS_X"     # left stick horizontal  -32768..32767
ABS_RZ    = "ABS_RZ"    # right trigger           0..255
BTN_SOUTH = "BTN_SOUTH" # A — emergency stop
BTN_EAST  = "BTN_EAST"  # B — toggle RC / auto


def controller_reader(state: State, done: threading.Event):
    print("Waiting for gamepad...")
    while not done.is_set():
        try:
            events = inputs.get_gamepad()
        except inputs.UnpluggedError:
            print("Controller disconnected.")
            done.set()
            return
        except Exception:
            time.sleep(0.1)
            continue

        for ev in events:
            with state.lock:
                if ev.ev_type == "Absolute":
                    if ev.code == ABS_X:
                        state.steer = snap_steer(
                            map_axis(ev.state, -32768, 32767, STEER_MIN, STEER_MAX)
                        )
                    elif ev.code == ABS_RZ:
                        state.speed = map_axis(ev.state, 0, 255, SPEED_STOP, SPEED_MAX)

                elif ev.ev_type == "Key" and ev.state == 1:
                    if ev.code == BTN_SOUTH:
                        state.stop = True
                    elif ev.code == BTN_EAST:
                        state.mode = "rc" if state.mode == "auto" else "auto"


def udp_sender(state: State, sock: socket.socket, esp_addr_ref: list,
               done: threading.Event):
    """Sends commands to the ESP8266. esp_addr_ref[0] is set by the receiver thread."""
    in_rc = False

    while not done.is_set():
        time.sleep(SEND_INTERVAL)

        addr = esp_addr_ref[0]
        if addr is None:
            continue  # not discovered yet

        with state.lock:
            stop_req = state.stop
            mode     = state.mode
            steer    = state.steer
            speed    = state.speed
            if stop_req:
                state.stop = False

        if stop_req:
            sock.sendto(b"S\r", addr)
            print("[TX] S  (emergency stop)")
            in_rc = False
            continue

        if mode == "rc" and not in_rc:
            sock.sendto(b"R\r", addr)
            print("[TX] R  (enter RC mode)")
            in_rc = True
            continue

        if mode == "auto" and in_rc:
            sock.sendto(b"A\r", addr)
            print("[TX] A  (return to auto)")
            in_rc = False
            continue

        if in_rc:
            sock.sendto(f"{steer} {speed}\r".encode(), addr)
            # Uncomment for verbose logging:
            # print(f"[TX] steer={steer} speed={speed}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Xbox One → ESP8266 UDP → K66F RC bridge")
    parser.add_argument("--port", type=int, default=4210)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", args.port))
    sock.settimeout(1.0)

    print(f"Listening for ESP8266 on UDP port {args.port}...")
    print("Controls: left stick = steer | right trigger = throttle")
    print("          A button = EMERGENCY STOP | B button = toggle RC/auto")
    print("Press Ctrl+C to quit.\n")

    state        = State()
    done         = threading.Event()
    esp_addr_ref = [None]  # mutable container so threads can share the discovered IP

    reader = threading.Thread(target=controller_reader, args=(state, done), daemon=True)
    sender = threading.Thread(target=udp_sender, args=(state, sock, esp_addr_ref, done),
                               daemon=True)

    reader.start()
    sender.start()

    try:
        while not done.is_set():
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue

            if esp_addr_ref[0] is None:
                esp_addr_ref[0] = addr
                print(f"ESP8266 discovered at {addr[0]}")

            line = data.decode(errors="replace").strip()
            if line:
                print(f"[CAR] {line}")

    except KeyboardInterrupt:
        print("\nQuitting — sending stop.")
        if esp_addr_ref[0]:
            sock.sendto(b"S\r", esp_addr_ref[0])
        time.sleep(0.1)
    finally:
        done.set()
        sock.close()


if __name__ == "__main__":
    main()
