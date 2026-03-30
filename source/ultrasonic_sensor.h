#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "common_types.h"
#include "config.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_uart.h"
#include "globals.h"
#include "motor_control.h"

unsigned int USonic_GetDistance(void);
unsigned int Switch_Status(void);
void delay(uint32_t delay);
void verificaObstacol(void);
void seteazaMotoareStop(void);

#endif // ULTRASONIC_SENSOR_H
