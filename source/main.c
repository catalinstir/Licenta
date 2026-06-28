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

    static uint16_t  last_steer = 700;
    static uint16_t  last_speed = 770;
    static uint32_t  intersection_entry_ms = 0;
    static LQRState_t lqr_state = {0};

    while (1)
    {
        switch (currentState)
        {
            case STATE_WAIT:
                if (check_obstacle)
                {
                    check_obstacle = false;
                    if (verificaObstacol())
                        stop = 1;
                }

                if (g_systemTime_ms - lastRpmPrint >= 200)
                {
                    lastRpmPrint    = g_systemTime_ms;
                    float rpm       = HallSensor_GetRPM();
                    float speed_kmh = HallSensor_GetSpeed_kmh();
                    int rpm_int     = (int)rpm;
                    int rpm_dec     = (int)((rpm - rpm_int) * 10);
                    int kmh_int     = (int)speed_kmh;
                    int kmh_dec     = (int)((speed_kmh - kmh_int) * 10);
                    PRINTF("distance:%u steer:%u motor_speed:%u rpm:%d.%d speed:%d.%d vectors:%u\r\n",
                        USonic_GetLastDistance(), last_steer, last_speed, rpm_int, rpm_dec, kmh_int,
                        kmh_dec, (unsigned)count);
                }

                if (lqr_gains_updated)
                {
                    lqr_gains_updated = false;
                    float ke, kt, kp;
                    LQR_GetGains(&ke, &kt, &kp);
                    PRINTF("GAINS:");
                    print_float(ke);
                    PRINTF(" ");
                    print_float(kt);
                    PRINTF(" ");
                    print_float(kp);
                    PRINTF("\r\n");
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
                        if (g_pixy_intersection_detected)
                        {
                            PRINTF("-INTERSECTION\r\n");
                            intersection_entry_ms = g_systemTime_ms;
                            currentState = STATE_INTERSECTION;
                        }
                        else
                        {
                            currentState = STATE_PROCESS_VECTOR;
                        }
                    }
                    else
                    {
                        PRINTF("-INTERSECTION (0 vectors)\r\n");
                        intersection_entry_ms = g_systemTime_ms;
                        currentState = STATE_INTERSECTION;
                    }
                }
                break;

            case STATE_PROCESS_VECTOR:
                if (!PreprocessVectors(vectori, count, &Vector1, &Vector2))
                {
                    PRINTF("-INTERSECTION (bad vectors)\r\n");
                    intersection_entry_ms = g_systemTime_ms;
                    currentState = STATE_INTERSECTION;
                }
                else
                {
                    currentState = STATE_CONTROL;
                }
                break;

            case STATE_CONTROL:
            {
                LQRState_Update(&lqr_state, Vector1, Vector2);
                uint16_t steer_duty = LQR_SteerControl(&lqr_state);
                uint16_t speed_duty = CalculateSpeedFromDuty(steer_duty);
                PRINTF("-STEER:%d SPEED:%d e_lat:", steer_duty, speed_duty);
                print_float(lqr_state.e_lat);
                PRINTF(" theta_e:");
                print_float(lqr_state.theta_e);
                PRINTF("\r\n");
                Motor_ApplyCommand(steer_duty, speed_duty);
                last_steer   = steer_duty;
                last_speed   = speed_duty;
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

                if (rc_packet_ready)
                {
                    rc_packet_ready   = false;
                    uart_packet_type  = PACKET_NONE;
                    rc_last_packet_ms = g_systemTime_ms;
                    Motor_ApplyCommand(rc_steer_cmd, rc_speed_cmd);
                    last_steer = rc_steer_cmd;
                    last_speed = rc_speed_cmd;
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

            case STATE_INTERSECTION:
            {
                if (stop)
                {
                    currentState = STATE_STOP;
                    break;
                }
                if (g_systemTime_ms - intersection_entry_ms >= INTERSECTION_TIMEOUT_MS)
                {
                    PRINTF("-INTERSECTION done\r\n");
                    lqr_state.initialized = false;
                    currentState = STATE_WAIT;
                    break;
                }
                Motor_ApplyCommand(700, last_speed);
                break;
            }

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

    HallSensor_Init();
    HallSensor_SetWheelCircumference(0.220f);

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
