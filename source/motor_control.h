#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "common_types.h"
#include "config.h"
#include "fsl_debug_console.h"
#include "fsl_ftm.h"
#include "globals.h"
#include "utils.h"

ReturnType Motor_Init(FTM_Type *ftmBaseAddress, ftm_chnl_t ftmChnl, uint16_t ftmDefaultDuty);
ReturnType Motor_SetPwm(FTM_Type *ftmBaseAddress, ftm_chnl_t ftmCh, uint16_t dutyCycle);

/* Set both motors to neutral and clear the stop flag. */
void Motor_Stop(void);

/* Apply a steering + speed command using the smoothed reset-pulse pattern. */
void Motor_ApplyCommand(uint16_t steer_duty, uint16_t speed_duty);

#endif // MOTOR_CONTROL_H
