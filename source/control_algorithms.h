#ifndef CONTROL_ALGORITHMS_H_
#define CONTROL_ALGORITHMS_H_

#include "common_types.h"
#include "config.h"
#include <stdint.h>
#include "fsl_debug_console.h"
#include "motor_control.h"
#include "math.h"
#include "globals.h"


void print_float(float num);

uint16_t MapControlToDutyCycle(float control);
float CalculateError(float x_center);
float CalculateCombinedError(float x_center1, float x_center2);
void CalculateLaneCenter2(VectorType v1, VectorType v2, float *x_1, float *x_2, float* y_1, float *y_2);
void CalculateLaneCenter(VectorType v1, VectorType v2, float *x_center, float *y_center);


float PID_Control(PIDController *pid, float error);
void ProcessVectorsPID(VectorType v1, VectorType v2);
uint16_t CalculateSpeedFromDuty(uint16_t duty_cycle);
uint16_t CalculateSpeedFromControl(float control);




#endif /* CONTROL_ALGORITHMS_H_ */
