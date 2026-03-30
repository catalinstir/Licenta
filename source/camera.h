#ifndef CAMERA_H
#define CAMERA_H

#include <ultrasonic_sensor.h>
#include "common_types.h"
#include "config.h"
#include "fsl_i2c.h"

extern uint8_t g_master_rx_buff[I2C_DATA_LENGTH];
extern uint8_t g_master_tx_buff[I2C_DATA_LENGTH];


ReturnType Pixy_GetVectors(VectorType* pVectors, uint8_t* pVectorCount);
ReturnType Pixy_GetVector(VectorType *pVector1, VectorType *pVector2);
void Pixy_Init(void);

#endif // CAMERA_H
