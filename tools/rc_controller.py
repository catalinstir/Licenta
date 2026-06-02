#!/usr/bin/env python3
"""
rc_controller.py — Xbox One controller → FRDM-K66F serial bridge

Protocol sent to car:
  S\r           emergency stop  (any state)
  R\r           enter RC mode
  A\r           return to autonomous
  <steer> <speed>\r   RC control packet (steer 500-900, speed duty×100)

Usage:
  python3 rc_controller.py [--port /dev/ttyACM0] [--baud 115200]

Dependencies:
  pip install pyserial inputs
"""

import argparse
import serial
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

SPEED_STOP    = 770   # duty×100 used when trigger is fully released
SPEED_MAX     = 790   # duty×100 at full trigger (keep conservative)

# Watchdog: send a neutral packet at this interval even when stick is centred,
# so the firmware watchdog (500 ms) never fires.
SEND_INTERVAL = 0.05  # 20 Hz


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def map_axis(raw: int, in_min: int, in_max: int, out_min: int, out_max: int) -> int:
    """Map a raw axis value to an integer output range."""
    ratio = (raw - in_min) / (in_max - in_min)
    ratio = max(0.0, min(1.0, ratio))
    return round(out_min + ratio * (out_max - out_min))


def snap_steer(value: int) -> int:
    """Snap steering to the discrete steps the firmware expects."""
    steps = [500, 600, 700, 800, 900]
    return min(steps, key=lambda s: abs(s - value))


# ---------------------------------------------------------------------------
# Controller state (written by reader thread, read by send thread)
# ---------------------------------------------------------------------------

class State:
    def __init__(self):
        self.steer = STEER_NEUTRAL
        self.speed = SPEED_STOP
        self.mode  = "auto"   # "auto" | "rc"
        self.stop  = False
        self.lock  = threading.Lock()


# ---------------------------------------------------------------------------
# Xbox One axis / button codes (via 'inputs' library)
# ---------------------------------------------------------------------------
# Axes
ABS_X          = "ABS_X"          # left stick horizontal, range -32768..32767
ABS_RZ         = "ABS_RZ"         # right trigger,         range 0..255 (XInput)

# Buttons
BTN_SOUTH      = "BTN_SOUTH"      # A — emergency stop
BTN_EAST       = "BTN_EAST"       # B — toggle RC / auto


def controller_reader(state: State, done: threading.Event):
    """Reads gamepad events and updates shared state."""
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
                        # Left stick X: -32768 (full left) … 32767 (full right)
                        steer_raw = map_axis(ev.state, -32768, 32767,
                                             STEER_MIN, STEER_MAX)
                        state.steer = snap_steer(steer_raw)

                    elif ev.code == ABS_RZ:
                        # Right trigger: 0 (released) … 255 (full press)
                        state.speed = map_axis(ev.state, 0, 255,
                                               SPEED_STOP, SPEED_MAX)

                elif ev.ev_type == "Key" and ev.state == 1:  # button down
                    if ev.code == BTN_SOUTH:
                        state.stop = True

                    elif ev.code == BTN_EAST:
                        state.mode = "rc" if state.mode == "auto" else "auto"


def serial_sender(state: State, port: serial.Serial, done: threading.Event):
    """Reads shared state and sends the appropriate serial packets."""
    in_rc = False

    while not done.is_set():
        time.sleep(SEND_INTERVAL)

        with state.lock:
            stop_req  = state.stop
            mode      = state.mode
            steer     = state.steer
            speed     = state.speed
            if stop_req:
                state.stop = False

        if stop_req:
            port.write(b"S\r")
            print("[TX] S  (emergency stop)")
            in_rc = False
            continue

        if mode == "rc" and not in_rc:
            port.write(b"R\r")
            print("[TX] R  (enter RC mode)")
            in_rc = True
            continue

        if mode == "auto" and in_rc:
            port.write(b"A\r")
            print("[TX] A  (return to auto)")
            in_rc = False
            continue

        if in_rc:
            pkt = f"{steer} {speed}\r".encode()
            port.write(pkt)
            # Uncomment for verbose logging:
            # print(f"[TX] steer={steer} speed={speed}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Xbox One → FRDM-K66F RC bridge")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0)
    except serial.SerialException as e:
        print(f"Cannot open serial port: {e}")
        sys.exit(1)

    print("Connected.")
    print("Controls: left stick = steer | right trigger = throttle")
    print("          A button = EMERGENCY STOP | B button = toggle RC/auto")
    print("Press Ctrl+C to quit.\n")

    state = State()
    done  = threading.Event()

    reader = threading.Thread(target=controller_reader, args=(state, done), daemon=True)
    sender = threading.Thread(target=serial_sender,     args=(state, ser, done), daemon=True)

    reader.start()
    sender.start()

    # Print incoming car debug output
    try:
        while not done.is_set():
            line = ser.readline()
            if line:
                print(f"[CAR] {line.decode(errors='replace').strip()}")
    except KeyboardInterrupt:
        print("\nQuitting — sending stop.")
        ser.write(b"S\r")
        time.sleep(0.1)
    finally:
        done.set()
        ser.close()


if __name__ == "__main__":
    main()
