#!/usr/bin/env python3
"""
rc_controller_universal.py — Xbox One controller (USB or Bluetooth) → ESP8266 UDP → K66F

Uses pygame for input, which works for both USB and Bluetooth connections.
On connect, auto-detects the trigger axis by reading which axes rest at -1.0.
If auto-detection fails, pass --throttle-axis N to override.

Communication chain:
  Laptop ←─ UDP broadcast port 4210 ─→ ESP8266 ←─ UART 153600 ─→ K66F

Protocol:
  S\r                              emergency stop  (any state)
  R\r                              enter RC mode
  A\r                              return to autonomous
  <steer> <speed>\r                RC control packet
  GS:<ke×1000> <kt×1000> <kp×1000>\r  set LQR straight gains
  GT:<ke×1000> <kt×1000> <kp×1000>\r  set LQR turn    gains

Live gain scheduling (works in both RC and auto modes):
  Y button         toggle editing STRAIGHT / TURN gain set
  D-pad Up/Down    k_e_lat   ±0.020  on the active set
  D-pad Right/Left k_theta_e ±0.050  on the active set
  RB / LB          k_psi_dot ±0.020  on the active set

Usage:
  python3 rc_controller_universal.py
  python3 rc_controller_universal.py --debug
  python3 rc_controller_universal.py --throttle-axis 5

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
# PWM constants — must match firmware config.h
# ---------------------------------------------------------------------------
STEER_NEUTRAL = 700
STEER_MIN = 500
STEER_MAX = 900

SPEED_STOPPED = 700
SPEED_MIN = 770
SPEED_MAX = 790

STICK_DEADZONE = 0.05
TRIGGER_DEADZONE = 0.05

SEND_INTERVAL = 0.05  # 20 Hz

AXIS_STEER = 0  # left stick X on all Xbox One variants with pygame/SDL2

BTN_STOP      = 0  # A
BTN_RC_TOGGLE = 1  # B
BTN_TUNE_SWAP = 3  # Y — toggle straight/turn gain set
BTN_LB        = 4  # Left Bumper  → k_psi_dot -
BTN_RB        = 5  # Right Bumper → k_psi_dot +

UDP_PORT = 4210

# ---------------------------------------------------------------------------
# Gain scheduling constants — must match firmware initial values in
# control_algorithms.c (LQR_K_STRAIGHT / LQR_K_TURN)
# ---------------------------------------------------------------------------
K_STRAIGHT_INIT = [0.430, 0.280, 0.180]  # [k_e_lat, k_theta_e, k_psi_dot]
K_TURN_INIT     = [0.430, 0.700, 0.280]

K_E_LAT_STEP   = 0.020  # D-pad Up/Down
K_THETA_E_STEP = 0.050  # D-pad Right/Left
K_PSI_DOT_STEP = 0.020  # RB/LB

K_E_LAT_MIN = 0.020;  K_E_LAT_MAX = 2.000
K_THETA_E_MIN = 0.020; K_THETA_E_MAX = 2.000
K_PSI_DOT_MIN = 0.000; K_PSI_DOT_MAX = 1.000


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def detect_trigger_axis(joystick) -> int | None:
    pygame.event.pump()
    trigger_axes = [i for i in range(joystick.get_numaxes()) if joystick.get_axis(i) < -0.9]
    if not trigger_axes:
        return None
    return max(trigger_axes)


def axis_to_pwm(value: float, deadzone: float, out_min: int, out_max: int, neutral: int) -> int:
    if abs(value) < deadzone:
        return neutral
    sign = 1 if value > 0 else -1
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    raw = neutral + sign * scaled * abs(out_max - out_min) / 2
    lo, hi = min(out_min, out_max), max(out_min, out_max)
    return round(max(lo, min(hi, raw)))


def trigger_to_pwm(raw: float, deadzone: float, out_min: int, out_max: int) -> int:
    normalized = (raw + 1.0) / 2.0
    if normalized < deadzone:
        return SPEED_STOPPED
    scaled = (normalized - deadzone) / (1.0 - deadzone)
    return round(out_min + scaled * (out_max - out_min))


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def fmt_gain(v: float) -> str:
    return f"{v:.3f}"


def _fmt3(g: list) -> str:
    return f"[{fmt_gain(g[0])} {fmt_gain(g[1])} {fmt_gain(g[2])}]"


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

class State:
    def __init__(self):
        self.steer = STEER_NEUTRAL
        self.speed = SPEED_STOPPED
        self.mode  = "auto"
        self.stop  = False
        self.lock  = threading.Lock()

        # Gain-scheduled LQR — two sets tuned independently.
        # tune_straight=True  → D-pad / bumpers edit straight set
        # tune_straight=False → D-pad / bumpers edit turn set
        self.tune_straight = True
        self.k_straight    = list(K_STRAIGHT_INIT)  # [k_e_lat, k_theta_e, k_psi_dot]
        self.k_turn        = list(K_TURN_INIT)
        self.gains_dirty_s = False  # straight set changed, needs sending
        self.gains_dirty_t = False  # turn    set changed, needs sending


# ---------------------------------------------------------------------------
# Controller reader thread
# ---------------------------------------------------------------------------

def controller_reader(state: State, done: threading.Event,
                      debug: bool, throttle_axis_override: int | None):
    pygame.init()
    pygame.joystick.init()

    joystick      = None
    throttle_axis = None

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

            if throttle_axis_override is not None:
                throttle_axis = throttle_axis_override
                print(f"Throttle axis: {throttle_axis} (manual override)")
            else:
                throttle_axis = detect_trigger_axis(joystick)
                if throttle_axis is not None:
                    print(f"Throttle axis: {throttle_axis} (auto-detected)")
                else:
                    print("WARNING: could not auto-detect trigger axis.")
                    print(f"  Axes: {[round(joystick.get_axis(i), 2) for i in range(joystick.get_numaxes())]}")
                    print("  Re-run with --throttle-axis N to set it manually.")

            if debug:
                print(f"[CTL] total axes={joystick.get_numaxes()}  buttons={joystick.get_numbuttons()}")

            with state.lock:
                print(f"[TUNE] S={_fmt3(state.k_straight)}  T={_fmt3(state.k_turn)}"
                      f"  editing={'STRAIGHT' if state.tune_straight else 'TURN'}")

        for ev in pygame.event.get():
            if ev.type == pygame.JOYDEVICEREMOVED:
                print("Gamepad disconnected.")
                joystick      = None
                throttle_axis = None
                break

            if ev.type == pygame.JOYAXISMOTION:
                if debug:
                    print(f"[CTL] axis={ev.axis} value={ev.value:.3f}")
                with state.lock:
                    if ev.axis == AXIS_STEER:
                        raw = axis_to_pwm(-ev.value, STICK_DEADZONE,
                                          STEER_MIN, STEER_MAX, STEER_NEUTRAL)
                        state.steer = round(raw / 50) * 50
                    elif throttle_axis is not None and ev.axis == throttle_axis:
                        state.speed = trigger_to_pwm(ev.value, TRIGGER_DEADZONE,
                                                     SPEED_MIN, SPEED_MAX)

            elif ev.type == pygame.JOYHATMOTION:
                hx, hy = ev.value
                if debug:
                    print(f"[CTL] hat={ev.hat} value={ev.value}")
                with state.lock:
                    g   = state.k_straight if state.tune_straight else state.k_turn
                    tag = 'S' if state.tune_straight else 'T'
                    if hy == 1:       # D-pad Up   → k_e_lat +
                        g[0] = round(clamp(g[0] + K_E_LAT_STEP, K_E_LAT_MIN, K_E_LAT_MAX), 3)
                        print(f"[TUNE/{tag}] k_e_lat   → {fmt_gain(g[0])}  {_fmt3(g)}")
                    elif hy == -1:    # D-pad Down → k_e_lat -
                        g[0] = round(clamp(g[0] - K_E_LAT_STEP, K_E_LAT_MIN, K_E_LAT_MAX), 3)
                        print(f"[TUNE/{tag}] k_e_lat   → {fmt_gain(g[0])}  {_fmt3(g)}")
                    elif hx == 1:     # D-pad Right → k_theta_e +
                        g[1] = round(clamp(g[1] + K_THETA_E_STEP, K_THETA_E_MIN, K_THETA_E_MAX), 3)
                        print(f"[TUNE/{tag}] k_theta_e → {fmt_gain(g[1])}  {_fmt3(g)}")
                    elif hx == -1:    # D-pad Left → k_theta_e -
                        g[1] = round(clamp(g[1] - K_THETA_E_STEP, K_THETA_E_MIN, K_THETA_E_MAX), 3)
                        print(f"[TUNE/{tag}] k_theta_e → {fmt_gain(g[1])}  {_fmt3(g)}")
                    else:
                        continue
                    if state.tune_straight:
                        state.gains_dirty_s = True
                    else:
                        state.gains_dirty_t = True

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

                    elif ev.button == BTN_TUNE_SWAP:   # Y — toggle which set D-pad edits
                        state.tune_straight = not state.tune_straight
                        label = 'STRAIGHT' if state.tune_straight else 'TURN'
                        g = state.k_straight if state.tune_straight else state.k_turn
                        print(f"[TUNE] now editing {label} gains  {_fmt3(g)}")

                    elif ev.button == BTN_RB:   # k_psi_dot +
                        g   = state.k_straight if state.tune_straight else state.k_turn
                        tag = 'S' if state.tune_straight else 'T'
                        g[2] = round(clamp(g[2] + K_PSI_DOT_STEP, K_PSI_DOT_MIN, K_PSI_DOT_MAX), 3)
                        print(f"[TUNE/{tag}] k_psi_dot → {fmt_gain(g[2])}  {_fmt3(g)}")
                        if state.tune_straight:
                            state.gains_dirty_s = True
                        else:
                            state.gains_dirty_t = True

                    elif ev.button == BTN_LB:   # k_psi_dot -
                        g   = state.k_straight if state.tune_straight else state.k_turn
                        tag = 'S' if state.tune_straight else 'T'
                        g[2] = round(clamp(g[2] - K_PSI_DOT_STEP, K_PSI_DOT_MIN, K_PSI_DOT_MAX), 3)
                        print(f"[TUNE/{tag}] k_psi_dot → {fmt_gain(g[2])}  {_fmt3(g)}")
                        if state.tune_straight:
                            state.gains_dirty_s = True
                        else:
                            state.gains_dirty_t = True

        time.sleep(0.005)

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
            stop_req   = state.stop
            mode       = state.mode
            steer      = state.steer
            speed      = state.speed
            dirty_s    = state.gains_dirty_s
            dirty_t    = state.gains_dirty_t
            ks         = list(state.k_straight)
            kt         = list(state.k_turn)
            if stop_req:
                state.stop = False
            state.gains_dirty_s = False
            state.gains_dirty_t = False

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

        # Send whichever gain set changed — works in both RC and auto modes
        if dirty_s:
            pkt = f"GS:{round(ks[0]*1000)} {round(ks[1]*1000)} {round(ks[2]*1000)}\r"
            sock.sendto(pkt.encode(), addr)
            print(f"[TX] {pkt.strip()}")

        if dirty_t:
            pkt = f"GT:{round(kt[0]*1000)} {round(kt[1]*1000)} {round(kt[2]*1000)}\r"
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
        description="Xbox One (USB or BT) → ESP8266 UDP → K66F RC bridge")
    parser.add_argument("--debug", action="store_true",
                        help="print every axis/button event and TX packet")
    parser.add_argument("--throttle-axis", type=int, default=None, metavar="N",
                        help="override auto-detected right trigger axis index")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.bind(("", UDP_PORT))
    sock.settimeout(1.0)

    print(f"Listening for ESP8266 on UDP port {UDP_PORT}...")
    print("Controls: left stick = steer | right trigger = throttle")
    print("          A = EMERGENCY STOP | B = toggle RC/auto")
    print("Gain scheduling: Y = swap STRAIGHT/TURN set")
    print("  D-pad Up/Down = k_e_lat | D-pad Right/Left = k_theta_e | RB/LB = k_psi_dot")
    print("Press Ctrl+C to quit.\n")

    state        = State()
    done         = threading.Event()
    esp_addr_ref = [None]

    reader = threading.Thread(target=controller_reader,
                              args=(state, done, args.debug, args.throttle_axis),
                              daemon=True)
    sender = threading.Thread(target=udp_sender,
                              args=(state, sock, esp_addr_ref, done, args.debug),
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
