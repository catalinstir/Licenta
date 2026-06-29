#!/usr/bin/env python3
"""
rc_controller_bt.py — Xbox One controller (Bluetooth) → ESP8266 UDP → K66F

Communication chain:
  Laptop ←─ UDP broadcast port 4210 ─→ ESP8266 ←─ UART 153600 ─→ K66F

Protocol:
  S\r                                     emergency stop
  R\r                                     enter RC mode
  A\r                                     return to autonomous
  <steer> <speed>\r                       RC control packet
  G:<ke×1000> <kt×1000> <kp×1000>\r      LQR gain update

Controls:
  Left stick X        steer
  Right trigger       throttle  (axis 4 on Bluetooth)
  A button            emergency stop
  B button            toggle RC / autonomous
  Y button            toggle editing STRAIGHT / TURN gain set
  D-pad Up/Down       k_e_lat   ±0.020  (of selected set)
  D-pad Right/Left    k_theta_e ±0.050  (of selected set)
  RB / LB             k_psi_dot ±0.020  (of selected set)

Usage:
  python3 rc_controller_bt.py [--debug]
"""

import argparse
import socket
import sys
import threading
import time

try:
    import pygame
except ImportError:
    print("Missing 'pygame'.  Run: pip install pygame")
    sys.exit(1)

# ---------------------------------------------------------------------------
# PWM constants — must match firmware config.h
# ---------------------------------------------------------------------------
STEER_NEUTRAL = 700
STEER_MIN     = 500
STEER_MAX     = 900

SPEED_STOPPED = 700
SPEED_MIN     = 770
SPEED_MAX     = 790

STICK_DEADZONE   = 0.05
TRIGGER_DEADZONE = 0.05

SEND_INTERVAL = 0.05   # 20 Hz

AXIS_STEER    = 0   # left stick X
AXIS_THROTTLE = 4   # right trigger on Bluetooth

BTN_STOP        = 0   # A
BTN_RC_TOGGLE   = 1   # B
BTN_GAIN_TOGGLE = 3   # Y — toggle editing STRAIGHT / TURN gain set
BTN_LB          = 4   # Left Bumper  → k_psi_dot -
BTN_RB          = 5   # Right Bumper → k_psi_dot +

UDP_PORT = 4210

# ---------------------------------------------------------------------------
# Gain tuning constants — [k_e_lat, k_theta_e, k_psi_dot]
# ---------------------------------------------------------------------------
K_STRAIGHT_INIT = [0.400, 0.500, 0.240]
K_TURN_INIT     = [0.430, 0.450, 0.250]

K_E_LAT_STEP   = 0.020
K_THETA_E_STEP = 0.050
K_PSI_DOT_STEP = 0.020

K_E_LAT_MIN,   K_E_LAT_MAX   = 0.020, 2.000
K_THETA_E_MIN, K_THETA_E_MAX = 0.020, 2.000
K_PSI_DOT_MIN, K_PSI_DOT_MAX = 0.000, 1.000


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def axis_to_pwm(value, deadzone, out_min, out_max, neutral):
    if abs(value) < deadzone:
        return neutral
    sign   = 1 if value > 0 else -1
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    raw    = neutral + sign * scaled * abs(out_max - out_min) / 2
    return round(max(min(out_min, out_max), min(max(out_min, out_max), raw)))


def trigger_to_pwm(raw, deadzone, out_min, out_max):
    normalized = (raw + 1.0) / 2.0
    if normalized < deadzone:
        return SPEED_STOPPED
    scaled = (normalized - deadzone) / (1.0 - deadzone)
    return round(out_min + scaled * (out_max - out_min))


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def fmt(v):
    return f"{v:.3f}"


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

class State:
    def __init__(self):
        self.steer          = STEER_NEUTRAL
        self.speed          = SPEED_STOPPED
        self.mode           = "auto"
        self.stop           = False
        self.lock           = threading.Lock()
        self.k_straight     = list(K_STRAIGHT_INIT)   # [k_e_lat, k_theta_e, k_psi_dot]
        self.k_turn         = list(K_TURN_INIT)
        self.editing_turn   = False   # False = STRAIGHT, True = TURN
        self.straight_dirty = False
        self.turn_dirty     = False


# ---------------------------------------------------------------------------
# Controller reader thread
# ---------------------------------------------------------------------------

def controller_reader(state, done, debug):
    pygame.init()
    pygame.joystick.init()
    joystick = None

    while not done.is_set():
        if joystick is None:
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                if debug:
                    print("[CTL] no gamepad, retrying...")
                time.sleep(1.0)
                continue
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Gamepad connected: {joystick.get_name()}")
            print(f"Throttle axis: {AXIS_THROTTLE} (Bluetooth hardcoded)")
            if debug:
                print(f"[CTL] axes={joystick.get_numaxes()}  "
                      f"buttons={joystick.get_numbuttons()}")
            with state.lock:
                tag = "TURN" if state.editing_turn else "STRAIGHT"
                k = state.k_turn if state.editing_turn else state.k_straight
                print(f"[TUNE] editing={tag}  "
                      f"k_e_lat={fmt(k[0])}  k_theta_e={fmt(k[1])}  k_psi_dot={fmt(k[2])}")

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
                        state.steer = round(raw / 50) * 50
                    elif ev.axis == AXIS_THROTTLE:
                        state.speed = trigger_to_pwm(
                            ev.value, TRIGGER_DEADZONE, SPEED_MIN, SPEED_MAX)

            elif ev.type == pygame.JOYHATMOTION:
                hx, hy = ev.value
                if debug:
                    print(f"[CTL] hat={ev.hat} value={ev.value}")
                with state.lock:
                    k   = state.k_turn if state.editing_turn else state.k_straight
                    tag = "TURN" if state.editing_turn else "STR"
                    if hy == 1:
                        k[0] = round(clamp(k[0] + K_E_LAT_STEP, K_E_LAT_MIN, K_E_LAT_MAX), 3)
                        if state.editing_turn: state.turn_dirty = True
                        else:                  state.straight_dirty = True
                        print(f"[TUNE/{tag}] k_e_lat → {fmt(k[0])}  k_theta_e={fmt(k[1])}  k_psi_dot={fmt(k[2])}")
                    elif hy == -1:
                        k[0] = round(clamp(k[0] - K_E_LAT_STEP, K_E_LAT_MIN, K_E_LAT_MAX), 3)
                        if state.editing_turn: state.turn_dirty = True
                        else:                  state.straight_dirty = True
                        print(f"[TUNE/{tag}] k_e_lat → {fmt(k[0])}  k_theta_e={fmt(k[1])}  k_psi_dot={fmt(k[2])}")
                    elif hx == 1:
                        k[1] = round(clamp(k[1] + K_THETA_E_STEP, K_THETA_E_MIN, K_THETA_E_MAX), 3)
                        if state.editing_turn: state.turn_dirty = True
                        else:                  state.straight_dirty = True
                        print(f"[TUNE/{tag}] k_theta_e → {fmt(k[1])}  k_e_lat={fmt(k[0])}  k_psi_dot={fmt(k[2])}")
                    elif hx == -1:
                        k[1] = round(clamp(k[1] - K_THETA_E_STEP, K_THETA_E_MIN, K_THETA_E_MAX), 3)
                        if state.editing_turn: state.turn_dirty = True
                        else:                  state.straight_dirty = True
                        print(f"[TUNE/{tag}] k_theta_e → {fmt(k[1])}  k_e_lat={fmt(k[0])}  k_psi_dot={fmt(k[2])}")

            elif ev.type == pygame.JOYBUTTONDOWN:
                if debug:
                    print(f"[CTL] button={ev.button} down")
                with state.lock:
                    if ev.button == BTN_STOP:
                        state.stop = True
                    elif ev.button == BTN_RC_TOGGLE:
                        state.mode = "rc" if state.mode == "auto" else "auto"
                        print(f"[CTL] mode → {state.mode}")
                    elif ev.button == BTN_GAIN_TOGGLE:
                        state.editing_turn = not state.editing_turn
                        tag = "TURN" if state.editing_turn else "STRAIGHT"
                        k   = state.k_turn if state.editing_turn else state.k_straight
                        print(f"[TUNE] now editing {tag}: "
                              f"k_e_lat={fmt(k[0])}  k_theta_e={fmt(k[1])}  k_psi_dot={fmt(k[2])}")
                    elif ev.button == BTN_RB:
                        k   = state.k_turn if state.editing_turn else state.k_straight
                        tag = "TURN" if state.editing_turn else "STR"
                        k[2] = round(clamp(k[2] + K_PSI_DOT_STEP, K_PSI_DOT_MIN, K_PSI_DOT_MAX), 3)
                        if state.editing_turn: state.turn_dirty = True
                        else:                  state.straight_dirty = True
                        print(f"[TUNE/{tag}] k_psi_dot → {fmt(k[2])}  k_e_lat={fmt(k[0])}  k_theta_e={fmt(k[1])}")
                    elif ev.button == BTN_LB:
                        k   = state.k_turn if state.editing_turn else state.k_straight
                        tag = "TURN" if state.editing_turn else "STR"
                        k[2] = round(clamp(k[2] - K_PSI_DOT_STEP, K_PSI_DOT_MIN, K_PSI_DOT_MAX), 3)
                        if state.editing_turn: state.turn_dirty = True
                        else:                  state.straight_dirty = True
                        print(f"[TUNE/{tag}] k_psi_dot → {fmt(k[2])}  k_e_lat={fmt(k[0])}  k_theta_e={fmt(k[1])}")

        time.sleep(0.005)

    pygame.quit()


# ---------------------------------------------------------------------------
# UDP sender thread
# ---------------------------------------------------------------------------

def udp_sender(state, sock, esp_addr_ref, done, debug):
    in_rc = False

    while not done.is_set():
        time.sleep(SEND_INTERVAL)

        addr = esp_addr_ref[0]
        if addr is None:
            continue

        with state.lock:
            stop_req       = state.stop
            mode           = state.mode
            steer          = state.steer
            speed          = state.speed
            str_dirty      = state.straight_dirty
            trn_dirty      = state.turn_dirty
            k_str          = list(state.k_straight)
            k_trn          = list(state.k_turn)
            if stop_req:
                state.stop = False
            state.straight_dirty = False
            state.turn_dirty     = False

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

        if str_dirty:
            pkt = f"GS:{round(k_str[0]*1000)} {round(k_str[1]*1000)} {round(k_str[2]*1000)}\r"
            sock.sendto(pkt.encode(), addr)
            print(f"[TX] {pkt.strip()}")
        if trn_dirty:
            pkt = f"GT:{round(k_trn[0]*1000)} {round(k_trn[1]*1000)} {round(k_trn[2]*1000)}\r"
            sock.sendto(pkt.encode(), addr)
            print(f"[TX] {pkt.strip()}")

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
        description="Xbox One (Bluetooth) → ESP8266 UDP → K66F")
    parser.add_argument("--debug", action="store_true",
                        help="print every axis/button event and TX packet")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.bind(("", UDP_PORT))
    sock.settimeout(1.0)

    print(f"Listening for ESP8266 on UDP port {UDP_PORT}...")
    print("Controls: left stick = steer | right trigger = throttle (axis 4)")
    print("          A = STOP | B = toggle RC/auto")
    print("Gain tuning: D-pad Up/Down = k_e_lat | D-pad Right/Left = k_theta_e")
    print("             RB = k_psi_dot+ | LB = k_psi_dot-")
    print("Press Ctrl+C to quit.\n")

    state        = State()
    done         = threading.Event()
    esp_addr_ref = [None]

    threading.Thread(target=controller_reader,
                     args=(state, done, args.debug),
                     daemon=True).start()
    threading.Thread(target=udp_sender,
                     args=(state, sock, esp_addr_ref, done, args.debug),
                     daemon=True).start()

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
