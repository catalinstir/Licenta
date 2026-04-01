# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Firmware for an autonomous racing robot built on the **FRDM-K66F** board (NXP MK66FN2M0xxx18, ARM Cortex-M4F). This is an NXP CUP competition entry. The system reads lane vectors from a Pixy camera, computes PID-based steering corrections, and drives brushed/servo motors via PWM. A simulator mode allows testing over UART without hardware.

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
- Baud: 115200, 8N1

## Architecture

### State Machine (`source/main.c`)

The main loop cycles through these states:

```
STATE_INIT → STATE_WAIT → STATE_CHECK_OBSTACLE → STATE_READ_CAMERA
                                                        ↓
              STATE_STOP ←──────────────── STATE_CONTROL ← STATE_PROCESS_VECTOR
```

- **STATE_CHECK_OBSTACLE**: Ultrasonic sensor; stops if obstacle < 30 cm
- **STATE_READ_CAMERA**: Reads vectors from Pixy (I2C) or UART (simulator)
- **STATE_PROCESS_VECTOR**: Validates, merges, normalizes lane vectors
- **STATE_CONTROL**: PID steering + speed adjustment via PWM

### Source Modules (`source/`)

| File | Responsibility |
|------|---------------|
| `main.c` | State machine, ISR declarations, peripheral init calls |
| `globals.c/h` | Shared global state, PID struct initialization |
| `common_types.h` | Core types: `VectorType`, `PIDController`, `SystemState` |
| `config.h` | All hardware constants: pin names, timing cycles, PID gains, PWM limits |
| `camera.c/h` | Pixy2 I2C protocol; reads up to 2 lane vectors per frame |
| `vector_processing.c/h` | Validates vectors (length > 1px), merges, normalizes ordering |
| `control_algorithms.c/h` | PID error calc (`error = 39.0 - center`), maps to discrete PWM steps |
| `motor_control.c/h` | FTM0/FTM3 PWM setup and duty cycle updates |
| `hall_sensor.c/h` | FTM0_CH2 input capture → RPM calculation |
| `ultrasonic_sensor.c/h` | GPIO trigger/echo timing → distance in cm |
| `adc.c/h` | ADC1_CH10 (PTB4) battery voltage with 10.15:1 divider |
| `uart_handler.c/h` | UART4 ISR; parses `x0 y0 x1 y1 x0 y0 x1 y1\r` for simulator |

### Key Hardware Mappings

| Peripheral | Function |
|-----------|----------|
| I2C0 | Pixy camera (addr 0x54, 100 kHz) |
| UART4 | Debug output + simulator input (115200) |
| FTM3_CH6 | Steering servo PWM |
| FTM0_CH5 | Brushed drive motor PWM |
| FTM0_CH2 (PTC3) | Hall sensor input capture |
| PTA6 / PTC1 | Ultrasonic trigger / echo |
| PTB4 (ADC1_SE10) | Battery voltage ADC |

### PID Tuning (`source/globals.c`)

PID gains are set in `InitGlobals()`, **not** in `config.h`. `config.h` only holds `CENTER_X = 39.0f`.

- Active: `Kp=0.045, Ki=0.0, Kd=0.06` (hardware-tuned)
- Commented-out alternatives exist for simulator tuning
- PWM limits (`PWM_MIN_SPEED`, `PWM_MAX_SPEED`) are also set in `InitGlobals()`, currently both `7.7f`

### PWM Representation

All PWM duty cycle values are stored as **percentage × 100** (e.g., `770` = 7.70% duty at 50 Hz ≈ 1.54 ms pulse). The discrete steering steps `{500, 600, 700, 800, 900}` and speed values follow this same convention. `MapControlToDutyCycle()` in `control_algorithms.c` converts the PID [-1, 1] output to these discrete steps.

Each motor command in `ProcessVectorsPID()` sends a neutral pulse (8000 = center) then delays before applying the target value — this is intentional smoothing, not a bug.

### Vector Processing Details

- Vectors are **normalized** so `m_y0 >= m_y1` (bottom of image first) before use.
- Camera image space is 128×128 pixels; `CENTER_X = 39.0` (calibrated to physical camera mount, not geometric center).
- Lane classification uses slope sign: negative slope → left lane, positive slope → right lane.
- If only one lane is visible, the missing lane defaults to a hard-coded edge position (left: x=0, right: x=78). If **no** valid vectors remain, `stop = 1` is set directly inside `vector_processing.c` — not by the state machine.
- `HallSensor_Init()` is currently commented out in `Init_Peripherals()`; hall RPM feedback is computed but not used for speed control.

### Simulator Mode

Set `simulator = true` in `globals.c` (`InitGlobals()`). The system then reads vectors from UART4 instead of the Pixy camera. Input format (8 space-separated integers, terminated by `\r`):
```
x0 y0 x1 y1 x0 y0 x1 y1\r
```
In simulator mode, `ProcessVectorsPID()` also prints error/control/PWM values over UART for feedback.

### Board Support Files (`board/`)

`pin_mux.c/h` and `clock_config.c/h` are **auto-generated** from `rangers_nxpcup.mex` (MCUXpresso Config Tools). Do not edit manually — regenerate via MCUXpresso IDE if hardware config changes.
