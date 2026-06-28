# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Firmware for an autonomous racing robot built on the **RDDRONE-FMUK66E** board (NXP MK66FN2M0VLQ18, LQFP-144, ARM Cortex-M4F). This is an NXP CUP competition entry. The system reads lane vectors from a Pixy2 camera, computes LQR-based steering corrections, and drives brushed/servo motors via PWM. A simulator mode allows testing over UART without hardware. A remote-control (RC) mode allows a human operator to override the autonomous loop via an Xbox One controller bridged through an ESP8266.

## Build

The project uses an **Eclipse CDT Managed Build** (GNU Make). The `Debug/` directory contains the generated Makefile.

```bash
cd Debug
make clean && make all
```

Toolchain: `arm-none-eabi-gcc` (must be on PATH). Output: `Debug/LicentaV1.axf`.

To flash and debug, use MCUXpresso IDE or OpenOCD with the FRDM-K66F's OpenSDA interface.

Serial monitor (debug output + simulator input):
- Port: `/dev/ttyACM0` or `/dev/ttyUSB0`
- Baud: **153600**, 8N1 — despite the SDK and peripheral config tool reporting 115200, the RDDRONE-FMUK66E's UART4 actually operates at 153600 baud. Set your serial terminal to 153600 or output will be garbled. This is a known quirk of this platform.

## Architecture

### State Machine (`source/main.c`)

```
STATE_WAIT ──► STATE_READ_CAMERA ──► STATE_PROCESS_VECTOR ──► STATE_CONTROL
    ▲   │              │                      │                      │
    │   │              └──────────────────────┴──► STATE_INTERSECTION│
    │   │                    (0 vecs / bad vecs / pixy intersection)  │
    │   └─────────────────────────────────────────────────────────────┘
    │
    ├──► STATE_REMOTE_CONTROL ◄──► (RC packets via UART)
    │         │ (on 'A' packet → STATE_WAIT; on stop → STATE_STOP)
    │
    ├──► STATE_INTERSECTION ──► STATE_WAIT (after INTERSECTION_TIMEOUT_MS)
    │
    └──► STATE_STOP ──► STATE_WAIT
```

- **STATE_WAIT**: Checks `check_obstacle` flag (set by `PIT0_IRQHandler`); sets `stop=1` if obstacle < 30 cm. Emits telemetry every 200 ms: `distance:%u steer:%u motor_speed:%u rpm:%d.%d speed:%d.%d vectors:%u`. Transitions to `STATE_REMOTE_CONTROL` if `rc_enter_requested`, otherwise `STATE_READ_CAMERA` or `STATE_STOP`.
- **STATE_READ_CAMERA**: Reads vectors from Pixy2 (I2C) or UART (simulator). On I2C error or `g_pixy_intersection_detected`, enters `STATE_INTERSECTION`.
- **STATE_PROCESS_VECTOR**: Calls `PreprocessVectors()`; goes to `STATE_INTERSECTION` (not stop) if no valid vectors.
- **STATE_CONTROL**: Calls `LQRState_Update()` + `LQR_SteerControl()`, applies the resulting PWM, and stores `last_steer`/`last_speed` for telemetry.
- **STATE_REMOTE_CONTROL**: Applies `rc_steer_cmd`/`rc_speed_cmd` set by the UART ISR. Returns to `STATE_WAIT` on 'A' packet; falls back to neutral (700/770) if no packet arrives within 500 ms.
- **STATE_INTERSECTION**: Holds straight (steer=700, speed=`last_speed`) for `INTERSECTION_TIMEOUT_MS` (500 ms), then resets `lqr_state.initialized = false` and returns to `STATE_WAIT`. Also entered on I2C read failure or when Pixy reports a feature-type-2 intersection.
- **STATE_STOP**: Calls `Motor_Stop()`, transitions back to `STATE_WAIT`.

### Source Modules (`source/`)

| File | Responsibility |
|------|---------------|
| `main.c` | State machine, ISR definitions, peripheral init |
| `globals.c/h` | Shared global state (`stop`, `currentState`, `g_systemTime_ms`, `pid_steering`, RC globals) |
| `common_types.h` | Core types: `VectorType`, `PIDController`, `MotorCommand_t`, `SystemState`, `UARTPacketType` |
| `config.h` | All tunable constants: pin names, timing, PID gains, PWM limits, ADC settings |
| `utils.c/h` | `delay(cycles)` busy-wait loop; `print_float()` PRINTF wrapper for floats |
| `camera.c/h` | Pixy2 I2C protocol; reads up to 2 lane vectors per frame |
| `vector_processing.c/h` | Validates vectors (length > 1px), merges, normalises ordering |
| `control_algorithms.c/h` | PID + LQR steering, PI speed control, speed-steering coupling |
| `motor_control.c/h` | FTM0/FTM3 PWM setup and duty cycle updates |
| `hall_sensor.c/h` | RPM/speed (km/h) receiver — values fed externally by ESP8266 via `HallSensor_SetExternalRPM()`; `HallSensor_Update()` handles 500 ms timeout only |
| `ultrasonic_sensor.c/h` | GPIO trigger/echo timing → distance in cm; `USonic_GetLastDistance()` returns cached value |
| `adc.c/h` | ADC1_CH10 (PTB4) battery voltage with 10.15:1 divider |
| `uart_handler.c/h` | UART4 ISR; handles RC protocol (`S\r`, `R\r`, `A\r`, `<steer> <speed>\r`), hall RPM packets (`H:<rpm>\r`), and legacy simulator frame parsing |

### Key Hardware Mappings

| Peripheral | Function |
|-----------|----------|
| I2C0 | Pixy2 camera (addr 0x54, 100 kHz) |
| UART4 | Debug output + RC/simulator input (153600 baud — see note above) |
| FTM3_CH6 (PTE11) | Steering servo PWM |
| FTM0_CH5 (PTD5) | Brushed drive motor PWM |
| ESP8266 GPIO4 | Hall sensor signal input — interrupt-driven, sends `H:<rpm>\r` to K66 every 100 ms |
| PTA6 / PTC1 | Ultrasonic trigger / echo |
| PTB4 (ADC1_SE10) | Battery voltage ADC |

### Tuning Parameters (`source/config.h`)

All tunable constants live in `config.h`. `InitGlobals()` reads them via `PID_Init()`.

- PID steering: `PID_STEERING_KP=0.045`, `PID_STEERING_KI=0.0`, `PID_STEERING_KD=0.06`
- PWM steering limits: `PWM_MIN=5.0`, `PWM_MAX=9.0` (percentage, stored ×100)
- PWM speed limits: `PWM_MIN_SPEED=PWM_MAX_SPEED=7.7f` (currently flat — no speed variation)
- Simulator PID alternatives are in comments inside `config.h`

### PWM Representation

All PWM duty cycle values are stored as **percentage × 100** (e.g., `770` = 7.70% duty at 50 Hz ≈ 1.54 ms pulse). `MapControlToDutyCycle()` converts a normalised control value [-1, 1] to a **continuous** duty cycle via linear interpolation between `PWM_MIN` (500) and `PWM_MAX` (900).

`Motor_ApplyCommand()` sends a neutral pulse (8000) then delays before applying the target value — this is intentional smoothing, not a bug. `Motor_Stop()` does the same when stopping.

### Vector Processing Details

- Vectors are **normalised** so `m_y0 >= m_y1` (bottom of image first) before use.
- Camera image space is 128×128 pixels; `CENTER_X = 39.0` (calibrated to physical camera mount, not geometric centre).
- Lane classification uses slope sign: negative slope → left lane, positive slope → right lane.
- If only one lane is visible, the missing lane defaults to a hard-coded edge (left: x=0, right: x=78).
- If no valid vectors remain after preprocessing, `stop = 1` is set by `STATE_PROCESS_VECTOR` in `main.c`.

### Hall Sensor Integration

RPM is measured by the **ESP8266**, not the K66F directly. The RDDRONE-FMUK66E's peripheral configuration (complex drone FMU setup) interfered with all attempts to use a K66 pin for hall sensing. The ESP8266 bridge already present for RC control was extended to handle this.

**Hardware**: Hall sensor VCC → J4 +5V. Hall sensor signal → 10kΩ/20kΩ voltage divider (5V→3.3V) → ESP8266 GPIO4. All grounds tied together (J4 GND ↔ ESP8266 GND).

**ESP8266 side** (`ESP8266/setup.c`): `RISING` interrupt on GPIO4 records `micros()` on every edge. Every 100 ms, computes wheel RPM as `60 000 000 / (interval_µs × POLES_PER_REV)` where `POLES_PER_REV = 10` (disc has 10 pole pairs → 10 pulses per wheel revolution). Sends `H:<rpm>\r` over UART to K66. Sends `H:0\r` if no edge has arrived within 500 ms. Edges closer than 3 ms apart are rejected as noise.

**K66 side**: `uart_handler.c` ISR parses `H:<rpm>\r` packets and calls `HallSensor_SetExternalRPM(float rpm)`. `hall_sensor.c` applies a plausibility filter — rejects any non-zero update that deviates by more than 2× or less than 0.5× from the current RPM while the wheel is running (catches WiFi-blocked readings from the ESP8266 which arrive as ~10% of true RPM). Stores the value, derives `frequency` and `speed_kmh` (wheel circumference 0.220 m), and zeroes out after 500 ms with no update. No K66 GPIO or FTM is involved.

`HallSensor_Update(g_systemTime_ms)` is called every millisecond from `SysTick_Handler()` for the timeout check only. Speed is reported in **km/h** via `HallSensor_GetSpeed_kmh()`. Telemetry is printed every 200 ms in `STATE_WAIT`: `distance:%u steer:%u motor_speed:%u rpm:%d.%d speed:%d.%d vectors:%u`.

### Remote Control Mode

RC mode is triggered by the ESP8266 sending `R\r` over UART4. The firmware enters `STATE_REMOTE_CONTROL` and applies `rc_steer_cmd`/`rc_speed_cmd` set by the UART ISR on each `<steer> <speed>\r` packet.

UART protocol (`\r`-terminated):
- `S\r` — emergency stop (sets `stop=1` immediately in ISR, any state)
- `R\r` — enter RC mode (`rc_enter_requested = true`)
- `A\r` — return to autonomous (`PACKET_RC_AUTO`)
- `<steer> <speed>\r` — RC control (e.g. `700 770\r`)
- `H:<rpm>\r` — hall sensor RPM from ESP8266 (e.g. `H:834\r`); calls `HallSensor_SetExternalRPM()`

A 500 ms watchdog in `STATE_REMOTE_CONTROL` holds motors at neutral (700/770) if no packet arrives.

Physical chain: Xbox One controller → `tools/rc_controller_universal.py` (pygame) → UDP broadcast port 4210 → ESP8266 (`Serial.begin(153600)`) → UART4 → K66F.

**Why ESP8266 and not XBee/HC-05**: The 153600 baud quirk means standard wireless serial modules (XBee, HC-05) cannot bridge UART4 — they only support standard baud rates. The ESP8266 handles any baud rate and bridges via WiFi/UDP instead, sidestepping the constraint entirely.

### Active Control Algorithm

**LQR steering** is the active control path (wired into `STATE_CONTROL` in `main.c`). It uses a discrete kinematic bicycle model with 3 states:

- `e_lat` — lateral error (m): car offset from lane centre, positive = car left of centre
- `theta_e` — heading error (rad): lane angle from `atan2(dx, dy)` of visible vectors
- `psi_dot` — yaw rate (rad/s): finite-difference of `theta_e` over `LQR_DT`

Key parameters (all in `control_algorithms.h` / `control_algorithms.c`):
- `LQR_DT = 0.558f` — measured actual loop period
- `L = 0.25 m` — wheelbase (axle-to-axle)
- `LQR_PIX2M` — pixel-to-metre scale for `e_lat`
- `delta = -delta` applied after the gain multiplication (hardware sign correction)

Current gains (manually tuned, `v0 ≈ 3 m/s`):
```c
static const float LQR_K[LQR_NUM_STATES] = {
    0.04000000f,  /* k_e_lat   [rad/m]       */
    0.50000000f,  /* k_theta_e [rad/rad]     */
    0.24000000f   /* k_psi_dot [rad/(rad/s)] */
};
```

Run `tools/lqr_tuning.py --v 0.83 --dt 0.558` to recompute gains via DARE and paste the output here.
Note: car speed is ~3 km/h = 0.83 m/s (verified from hall sensor after POLES_PER_REV fix).

`ProcessVectorsPID()` still exists for PID baseline comparison runs (useful for thesis data collection).

### Unactivated Algorithms

- **PI speed control** (`PIController_t`, `PI_SpeedControl()`): tracks a target RPM using `HallSensor_GetRPM()` feedback. Not yet wired in.
- **Speed-steering coupling** (`SpeedSteeringCoupling()`): reduces target RPM proportionally when steering deflection is large; floor at 40% of target. Not yet wired in.

### Simulator Mode

Set `simulator = false → true` in `globals.c` (`InitGlobals()`). The system reads vectors from UART4 instead of the Pixy2. Input format (8 space-separated integers, terminated by `\r`):
```
x0 y0 x1 y1 x0 y0 x1 y1\r
```
`ProcessVectorsPID()` prints a per-frame log on every control cycle. In hardware mode: `-STEER:%d SPEED:%d RPM:<float> MPS:<float>`. In simulator mode: `-ERR:<float> CONTROL:<float> STEER:%d SPEED:%d RPM:<float> MPS:<float>`. `Motor_Stop()` prints `-STOP`.

### Tools (`tools/`)

- **`lqr_tuning.py`**: Offline LQR gain computation (3-state bicycle model). Run with `python tools/lqr_tuning.py [--v 3] [--dt 0.558] [--no-plot]`. Prints a C array literal ready to paste into `control_algorithms.c`. Requires `scipy` and `matplotlib` (installed in `.venv`).
- **`rc_controller_universal.py`**: Xbox One controller bridge (USB or Bluetooth via pygame) → UDP → ESP8266. Run with `python tools/rc_controller_universal.py [--debug] [--throttle-axis N]`. Requires `pip install pygame`.
- **`data_logger.py`**: Listens on UDP port 4210 (SO_REUSEPORT — safe to run alongside rc_controller), parses telemetry and LQR control lines, saves a timestamped CSV to `tools/data/`. Run with `python tools/data_logger.py [--label lqr]`. Parses `distance:…`, `-STEER:… e_lat:… theta_e:…`, and `-ERR:… CONTROL:…` lines.
- **`plot_results.py`**: Loads CSVs from `data_logger.py` and generates comparison figures (e_lat, theta_e, steer, speed vs time; phase portrait; histogram). Prints a stats table. Run with `python tools/plot_results.py tools/data/*.csv [--labels PID LQR] [--save tools/data/plots/]`. Requires `pandas` (installed in `.venv`).

### PythonRobotics (`PythonRobotics/`)

Reference algorithm implementations for offline simulation and tuning — not integrated into the firmware:

- `lqr_steer_control/` — LQR path tracking with PID speed (from PythonRobotics)
- `lqr_speed_steer_control/` — LQR combined steering + speed control
- `stanley_control/` — Stanley method path tracking

These require the PythonRobotics utils and PathPlanning packages from the upstream repo. Use the `.venv` in the project root (`source .venv/bin/activate`).

### Board Support Files (`board/`)

`pin_mux.c/h`, `clock_config.c/h`, and `peripherals.c/h` are **auto-generated** from `rangers_nxpcup.mex` (MCUXpresso Config Tools). Do not edit manually — regenerate via MCUXpresso IDE if hardware config changes. `peripherals.c/h` configures the PIT channel 0 (fires `PIT0_IRQHandler` → sets `check_obstacle`) and UART4 baud rate init.
