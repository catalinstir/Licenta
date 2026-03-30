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

Toolchain: `arm-none-eabi-gcc` (must be on PATH). Output: `Debug/Licenta_UART.axf`.

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

### PID Tuning Constants (`source/config.h`)

- Kp = 0.045, Ki = 0.0, Kd = 0.06
- Center reference: X = 39.0 pixels
- Steering PWM steps: {500, 600, 700, 800, 900}
- Speed: `PWM_MAX_SPEED` when straight, `PWM_MIN_SPEED` during steering

### Simulator Mode

Set `simulator = true` in `globals.c`. The system then reads vectors from UART4 instead of the Pixy camera. Input format (8 space-separated integers, terminated by `\r`):
```
x0 y0 x1 y1 x0 y0 x1 y1\r
```

### Board Support Files (`board/`)

`pin_mux.c/h` and `clock_config.c/h` are **auto-generated** from `rangers_nxpcup.mex` (MCUXpresso Config Tools). Do not edit manually — regenerate via MCUXpresso IDE if hardware config changes.
