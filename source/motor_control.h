#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ultrasonic_sensor.h>
#include "common_types.h"
#include "config.h"
#include "fsl_debug_console.h"
#include "fsl_ftm.h"
#include "globals.h"

ReturnType Motor_Init(FTM_Type *ftmBaseAddress, ftm_chnl_t ftmChnl, uint16_t ftmDefaultDuty);
ReturnType Motor_SetPwm(FTM_Type *ftmBaseAddress, ftm_chnl_t ftmCh, uint16_t dutyCycle);

#endif // MOTOR_CONTROL_H
