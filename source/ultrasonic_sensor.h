#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "common_types.h"
#include "config.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "globals.h"
#include "utils.h"
#include <stdbool.h>

unsigned int Switch_Status(void);
unsigned int USonic_GetDistance(void);
uint16_t USonic_GetLastDistance(void);

/* Returns true if an obstacle closer than 30 cm is detected. */
bool verificaObstacol(void);

#endif // ULTRASONIC_SENSOR_H
