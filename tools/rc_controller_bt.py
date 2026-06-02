#!/usr/bin/env python3
"""
rc_controller_bt.py — Xbox One controller (Bluetooth) → ESP8266 UDP → FRDM-K66F bridge

Same controls and protocol as rc_controller.py, but uses pygame instead of
the 'inputs' library, which is required for Bluetooth-paired controllers.

Communication chain:
  Laptop ←─ UDP broadcast port 4210 ─→ ESP8266 ←─ UART 153600 ─→ K66F

Protocol sent to car:
  S\r                 emergency stop  (any state)
  R\r                 enter RC mode
  A\r                 return to autonomous
  <steer> <speed>\r   RC control packet (steer 500-900, speed duty×100)

Usage:
  python3 rc_controller_bt.py           # normal
  python3 rc_controller_bt.py --debug   # prints every axis/button event and TX packet

Dependencies:
  pip install pygame
"""

import argparse
import socket
import sys
import threading
import time

try:
    import pygame
except ImportError:
    print("Missing 'pygame' library.  Run: pip install pygame")
    sys.exit(1)

# ---------------------------------------------------------------------------
# PWM mapping constants — must match firmware config.h
# ---------------------------------------------------------------------------
STEER_NEUTRAL = 700
STEER_MIN     = 500
STEER_MAX     = 900

SPEED_STOPPED = 700   # parks the ESC (matches Motor_Stop final value)
SPEED_MIN     = 770   # slowest running speed
SPEED_MAX     = 790   # full trigger

STICK_DEADZONE   = 0.05   # ignore stick drift within 5 % of centre
TRIGGER_DEADZONE = 0.05   # ignore trigger within 5 % of rest position

SEND_INTERVAL = 0.05   # 20 Hz — keeps firmware 500 ms watchdog fed

# ---------------------------------------------------------------------------
# Controller axis / button indices for Xbox One on Linux via Bluetooth.
# Run with --debug and move each input to confirm these match your setup.
# ---------------------------------------------------------------------------
AXIS_STEER    = 0   # left stick X      (-1.0 = full left,  +1.0 = full right)
AXIS_THROTTLE = 4   # right trigger     (-1.0 = rest,       +1.0 = full press)
BTN_STOP      = 0   # A — emergency stop
BTN_RC_TOGGLE = 1   # B — toggle RC / auto

UDP_PORT = 4210


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def axis_to_pwm(value: float, deadzone: float,
                out_min: int, out_max: int, neutral: int) -> int:
    """Map a pygame stick axis (-1..1) to a PWM integer with a dead zone."""
    if abs(value) < deadzone:
        return neutral
    sign   = 1 if value > 0 else -1
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    raw    = neutral + sign * scaled * abs(out_max - out_min) / 2
    lo, hi = min(out_min, out_max), max(out_min, out_max)
    return round(max(lo, min(hi, raw)))


def trigger_to_pwm(raw: float, deadzone: float, out_min: int, out_max: int) -> int:
    """
    Map a trigger axis to a PWM integer.
    On Linux BT the trigger rests at -1.0 and reaches +1.0 at full press.
    Normalise to 0..1 before applying the dead zone.
    """
    normalized = (raw + 1.0) / 2.0      # -1..1  →  0..1
    if normalized < deadzone:
        return SPEED_STOPPED
    scaled = (normalized - deadzone) / (1.0 - deadzone)
    return round(out_min + scaled * (out_max - out_min))


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

class State:
    def __init__(self):
        self.steer = STEER_NEUTRAL
        self.speed = SPEED_STOPPED
        self.mode  = "auto"   # "auto" | "rc"
        self.stop  = False
        self.lock  = threading.Lock()


# ---------------------------------------------------------------------------
# Controller reader thread
# ---------------------------------------------------------------------------

def controller_reader(state: State, done: threading.Event, debug: bool):
    pygame.init()
    pygame.joystick.init()

    joystick = None

    while not done.is_set():
        if joystick is None:
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                if debug:
                    print("[CTL] no gamepad found, retrying...")
                time.sleep(1.0)
                continue
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Gamepad connected: {joystick.get_name()}")
            if debug:
                print(f"[CTL] axes={joystick.get_numaxes()}  "
                      f"buttons={joystick.get_numbuttons()}")

        for ev in pygame.event.get():
            if ev.type == pygame.JOYDEVICEREMOVED:
                print("Gamepad disconnected.")
                joystick = None
                break

            if ev.type == pygame.JOYAXISMOTION:
                if debug:
                    print(f"[CTL] axis={ev.axis} value={ev.value:.3f}")
                with state.lock:
                    if ev.axis == AXIS_STEER:
                        raw = axis_to_pwm(-ev.value, STICK_DEADZONE,
                                          STEER_MIN, STEER_MAX, STEER_NEUTRAL)
                        state.steer = round(raw / 50) * 50   # snap to 50-unit steps

                    elif ev.axis == AXIS_THROTTLE:
                        state.speed = trigger_to_pwm(
                            ev.value, TRIGGER_DEADZONE, SPEED_MIN, SPEED_MAX
                        )

            elif ev.type == pygame.JOYBUTTONDOWN:
                if debug:
                    print(f"[CTL] button={ev.button} down")
                with state.lock:
                    if ev.button == BTN_STOP:
                        state.stop = True
                    elif ev.button == BTN_RC_TOGGLE:
                        new_mode = "rc" if state.mode == "auto" else "auto"
                        print(f"[CTL] mode → {new_mode}")
                        state.mode = new_mode

        time.sleep(0.005)   # poll at ~200 Hz

    pygame.quit()


# ---------------------------------------------------------------------------
# UDP sender thread
# ---------------------------------------------------------------------------

def udp_sender(state: State, sock: socket.socket, esp_addr_ref: list,
               done: threading.Event, debug: bool):
    in_rc = False

    while not done.is_set():
        time.sleep(SEND_INTERVAL)

        addr = esp_addr_ref[0]
        if addr is None:
            continue

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
            pkt = f"{steer} {speed}\r"
            sock.sendto(pkt.encode(), addr)
            if debug:
                print(f"[TX] {pkt.strip()}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Xbox One (BT) → ESP8266 UDP → K66F RC bridge")
    parser.add_argument("--debug", action="store_true",
                        help="print every axis/button event and TX packet")
    args  = parser.parse_args()
    debug = args.debug

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", UDP_PORT))
    sock.settimeout(1.0)

    print(f"Listening for ESP8266 on UDP port {UDP_PORT}...")
    print("Controls: left stick = steer | right trigger = throttle")
    print("          A button = EMERGENCY STOP | B button = toggle RC/auto")
    print("Press Ctrl+C to quit.\n")

    state        = State()
    done         = threading.Event()
    esp_addr_ref = [None]

    reader = threading.Thread(target=controller_reader,
                               args=(state, done, debug), daemon=True)
    sender = threading.Thread(target=udp_sender,
                               args=(state, sock, esp_addr_ref, done, debug),
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
