/*
 * hall_sensor.h
 *
 * Hall sensor driver — RPM received from ESP8266 over UART4.
 * The ESP8266 measures hall sensor pulses on a GPIO pin and sends
 * "H:<rpm>\r\n" packets. uart_handler.c calls HallSensor_SetExternalRPM()
 * when a packet arrives. No local GPIO or FTM involved.
 */

#ifndef HALL_SENSOR_H_
#define HALL_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

/* If no H: packet arrives within this window, RPM is zeroed */
#define HALL_TIMEOUT_MS 500U


typedef struct
{
    float    rpm;
    float    frequency;
    float    speed_kmh;
    bool     isRunning;
    uint32_t lastUpdateTime;
    float    wheelCircumference_m;
    float    last_valid_rpm;
} HallSensor_t;

void  HallSensor_Init(void);
void  HallSensor_SetWheelCircumference(float circumference_m);
void  HallSensor_SetExternalRPM(float rpm);
void  HallSensor_Update(uint32_t currentTime_ms);
float HallSensor_GetRPM(void);
float HallSensor_GetFrequency(void);
float HallSensor_GetSpeed_kmh(void);
bool  HallSensor_IsRunning(void);
void  HallSensor_Reset(void);
const HallSensor_t *HallSensor_GetData(void);

#endif /* HALL_SENSOR_H_ */
