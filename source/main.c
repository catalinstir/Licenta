/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "adc.h"
#include "board.h"
#include "camera.h"
#include "clock_config.h"
#include "common_types.h"
#include "config.h"
#include "control_algorithms.h"
#include "fsl_debug_console.h"
#include "fsl_ftm.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"
#include "fsl_uart.h"
#include "globals.h"
#include "hall_sensor.h"
#include "motor_control.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "stdio.h"
#include "uart_handler.h"
#include "ultrasonic_sensor.h"
#include "utils.h"
#include "vector_processing.h"

void Init_Peripherals(void);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 *
 */
int main(void)
{
    PRINTF("\r\nHello world from NXPCUP\r\n");
    InitGlobals();
    Init_Peripherals();

    while (1)
    {
        switch (currentState)
        {
            case STATE_WAIT:
                if (check_obstacle)
                {
                    if (!simulator && verificaObstacol())
                    {
                        stop = 1;
                    }
                    check_obstacle = false;
                }
                if (stop)
                {
                    currentState = STATE_STOP;
                }
                else if (rc_enter_requested)
                {
                    rc_enter_requested = false;
                    rc_last_packet_ms  = g_systemTime_ms;
                    PRINTF("-RC MODE\r\n");
                    currentState = STATE_REMOTE_CONTROL;
                }
                else
                {
                    currentState = STATE_READ_CAMERA;
                }
                break;

            case STATE_READ_CAMERA:
                if (simulator)
                {
                    if (Simulator_GetFrame(vectori, &count))
                    {
                        currentState = STATE_PROCESS_VECTOR;
                    }
                    else
                    {
                        currentState = STATE_WAIT;
                    }
                }
                else
                {
                    if (Pixy_GetVectors(vectori, &count) == E_OK)
                    {
                        currentState = STATE_PROCESS_VECTOR;
                    }
                    else
                    {
                        currentState = STATE_WAIT;
                    }
                }
                break;

            case STATE_PROCESS_VECTOR:
                if (!PreprocessVectors(vectori, count, &Vector1, &Vector2))
                {
                    stop = 1;
                }
                if (stop)
                {
                    currentState = STATE_STOP;
                }
                else
                {
                    currentState = STATE_CONTROL;
                }
                break;

            case STATE_CONTROL:
            {
                MotorCommand_t cmd = ProcessVectorsPID(Vector1, Vector2);
                Motor_ApplyCommand(cmd.steer_duty, cmd.speed_duty);

                if (g_systemTime_ms - lastRpmPrint >= 200)
                {
                    lastRpmPrint      = g_systemTime_ms;
                    float rpm         = HallSensor_GetRPM();
                    float speed_kmh   = HallSensor_GetSpeed_kmh();
                    int rpm_int       = (int)rpm;
                    int rpm_dec       = (int)((rpm - rpm_int) * 10);
                    int kmh_int       = (int)speed_kmh;
                    int kmh_dec       = (int)((speed_kmh - kmh_int) * 10);
                    PRINTF("-RPM:%d.%d SPD:%d.%d km/h PULSES:%u RUNNING:%d\r\n",
                           rpm_int, rpm_dec, kmh_int, kmh_dec,
                           HallSensor_GetPulseCount(), HallSensor_IsRunning());
                }

                currentState = STATE_WAIT;
                break;
            }

            case STATE_REMOTE_CONTROL:
            {
                if (stop)
                {
                    currentState = STATE_STOP;
                    break;
                }

                if (uart_packet_type == PACKET_RC_AUTO)
                {
                    uart_packet_type = PACKET_NONE;
                    PRINTF("-AUTO MODE\r\n");
                    currentState = STATE_WAIT;
                    break;
                }

                if (uart_packet_type == PACKET_RC_REVERSE)
                {
                    uart_packet_type  = PACKET_NONE;
                    rc_last_packet_ms = g_systemTime_ms;
                    /* 480A double-click: brake → neutral → brake again */
                    Motor_SetPwm(MOTOR_BRUSHED_FTM_BASEADDR, MOTOR_BRUSHED, 630);
                    delay(2000000);  /* ~150 ms brake hold */
                    Motor_SetPwm(MOTOR_BRUSHED_FTM_BASEADDR, MOTOR_BRUSHED, 700);
                    delay(1000000);  /* ~75 ms neutral pause */
                    Motor_SetPwm(MOTOR_BRUSHED_FTM_BASEADDR, MOTOR_BRUSHED, 630);
                    PRINTF("-REVERSE\r\n");
                }
                else if (rc_packet_ready)
                {
                    rc_packet_ready   = false;
                    uart_packet_type  = PACKET_NONE;
                    rc_last_packet_ms = g_systemTime_ms;
                    Motor_ApplyCommand(rc_steer_cmd, rc_speed_cmd);
                }
                else if (g_systemTime_ms - rc_last_packet_ms > 500U)
                {
                    /* Watchdog: no packet for 500 ms — hold motors at neutral */
                    Motor_ApplyCommand(700, 770);
                    rc_last_packet_ms = g_systemTime_ms;
                    PRINTF("-RC TIMEOUT\r\n");
                }
                break;
            }

            case STATE_STOP:
                Motor_Stop();
                currentState = STATE_WAIT;
                break;

            default:
                currentState = STATE_WAIT;
                break;
        }
    }
}

/*13300000 ~ 1 second*/

/*!
 * Init all used peripherals.
 */
void Init_Peripherals(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    UART_EnableInterrupts(UART4, kUART_RxDataRegFullInterruptEnable);
    EnableIRQ(UART4_RX_TX_IRQn);
    BOARD_InitPeripherals();

    Motor_Init(MOTOR_BRUSHED_FTM_BASEADDR, MOTOR_BRUSHED, 740);
    Motor_Init(MOTOR_STEERING_FTM_BASEADDR, MOTOR_STEERING, 700);

    Pixy_Init();
    ADC_Init();

    // HallSensor_Init();
    // HallSensor_SetWheelCircumference(0.201f);


    float batteryVoltage = ADC_ReadBatteryVoltage();

    if (batteryVoltage <= 9.9f)
    {
        PRINTF("\r\nThe battery is too low.\r\n");
    }

    SysTick_Config(SystemCoreClock / 1000U);

    delay(50000000);
}

void PIT0_IRQHandler(void)
{
    PIT_ClearStatusFlags(PIT, PIT_CHANNEL_0, kPIT_TimerFlag);
    check_obstacle = true;
}
void SysTick_Handler(void)
{
    g_systemTime_ms++;
    HallSensor_Update(g_systemTime_ms);
}
