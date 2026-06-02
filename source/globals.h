#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "common_types.h"
#include <stdbool.h>
#include <stdint.h>

extern unsigned int stop;

extern VectorType Vector1;
extern VectorType Vector2;
extern VectorType vectori[8];
extern volatile uint8_t count;
extern volatile SystemState currentState;
extern volatile bool check_obstacle;
extern volatile bool simulator;
extern PIDController pid_steering;
extern PIDController pid_brushed;
extern volatile bool uart_data_ready;
extern volatile uint32_t lastRpmPrint;
extern volatile uint32_t g_systemTime_ms;

extern volatile UARTPacketType uart_packet_type;
extern volatile bool rc_enter_requested;
extern volatile bool rc_packet_ready;
extern volatile uint16_t rc_steer_cmd;
extern volatile uint16_t rc_speed_cmd;
extern volatile uint32_t rc_last_packet_ms;

void PID_Init(PIDController *pid, float kp, float ki, float kd, float output_max);
void InitGlobals(void);
#endif /* GLOBALS_H_ */
