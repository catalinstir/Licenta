#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>
#include <ultrasonic_sensor.h>
#include "common_types.h"
#include "config.h"
#include "fsl_i2c.h"

extern uint8_t g_master_rx_buff[I2C_DATA_LENGTH];
extern uint8_t g_master_tx_buff[I2C_DATA_LENGTH];

/* Set to true by Pixy_GetVectors() when the Pixy2 reports a feature-type 2
   (intersection) block in its response. Cleared at the start of each call. */
extern bool g_pixy_intersection_detected;

ReturnType Pixy_GetVectors(VectorType* pVectors, uint8_t* pVectorCount);
ReturnType Pixy_GetVector(VectorType *pVector1, VectorType *pVector2);
void Pixy_Init(void);

#endif // CAMERA_H
