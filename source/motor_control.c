#include "motor_control.h"

ReturnType Motor_Init(FTM_Type *ftmBaseAddress, ftm_chnl_t ftmChnl, uint16_t ftmDefaultDuty) {
    ReturnType retValue = E_OK;
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam[1];

    // Fill in the FTM config struct with the default settings
    FTM_GetDefaultConfig(&ftmInfo);
    // Calculate the clock division based on the PWM frequency to be obtained
    ftmInfo.prescale =
        FTM_CalculateCounterClkDiv(ftmBaseAddress, DEMO_PWM_FREQUENCY, FTM_SOURCE_CLOCK);
    // Initialize FTM module
    FTM_Init(ftmBaseAddress, &ftmInfo);

    // Configure ftm params with frequency 50HZ
    ftmParam[0].chnlNumber = ftmChnl; // Brushed motor
    ftmParam[0].level = FTM_PWM_ON_LEVEL;
    ftmParam[0].dutyCyclePercent = ftmDefaultDuty;
    ftmParam[0].firstEdgeDelayPercent = 0U;
    ftmParam[0].enableComplementary = false;
    ftmParam[0].enableDeadtime = false;

    if (kStatus_Success != FTM_SetupPwm(ftmBaseAddress, ftmParam, 1U, kFTM_CenterAlignedPwm,
                                        DEMO_PWM_FREQUENCY, FTM_SOURCE_CLOCK)) {
        retValue = E_NOT_OK;
    } else {
        FTM_StartTimer(ftmBaseAddress, kFTM_SystemClock);
    }

    return retValue;
}

ReturnType Motor_SetPwm(FTM_Type *ftmBaseAddress, ftm_chnl_t ftmCh, uint16_t dutyCycle) {

    if (ftmCh == MOTOR_BRUSHED) {
        PRINTF("SET speed:%d;\r\n", dutyCycle);
    } else {
        PRINTF("SET steer:%d;\r\n", dutyCycle);
    }

    ReturnType retValue = E_OK;

    // Disable channel output before updating the duty cycle
    FTM_UpdateChnlEdgeLevelSelect(ftmBaseAddress, ftmCh, 0U);

    // Update PWM duty cycle
    if (kStatus_Success !=
        FTM_UpdatePwmDutycycle(ftmBaseAddress, ftmCh, kFTM_CenterAlignedPwm, dutyCycle)) {
        // PRINTF("Update duty cycle fail, the target duty cycle may out of range!\r\n");
        retValue = E_NOT_OK;
    }

    // Software trigger to update registers
    FTM_SetSoftwareTrigger(ftmBaseAddress, true);

    // Start channel output with updated dutycycle
    FTM_UpdateChnlEdgeLevelSelect(ftmBaseAddress, ftmCh, FTM_PWM_ON_LEVEL);

    return retValue;
}

void Motor_Stop(void) {
    stop = 0;
    Motor_SetPwm(MOTOR_STEERING_FTM_BASEADDR, MOTOR_STEERING, 8000);
    delay(DELAY_STEERING_RESET);
    Motor_SetPwm(MOTOR_STEERING_FTM_BASEADDR, MOTOR_STEERING, 700);
    delay(DELAY_STEERING_CMD);

    Motor_SetPwm(MOTOR_BRUSHED_FTM_BASEADDR, MOTOR_BRUSHED, 8000);
    delay(DELAY_BRUSHED_RESET);
    Motor_SetPwm(MOTOR_BRUSHED_FTM_BASEADDR, MOTOR_BRUSHED, 700);
    delay(DELAY_BRUSHED_CMD);
}

void Motor_ApplyCommand(uint16_t steer_duty, uint16_t speed_duty) {
    Motor_SetPwm(MOTOR_STEERING_FTM_BASEADDR, MOTOR_STEERING, 8000);
    delay(DELAY_STEERING_RESET);
    Motor_SetPwm(MOTOR_STEERING_FTM_BASEADDR, MOTOR_STEERING, steer_duty);
    delay(DELAY_STEERING_CMD);

    Motor_SetPwm(MOTOR_BRUSHED_FTM_BASEADDR, MOTOR_BRUSHED, 8000);
    delay(DELAY_BRUSHED_RESET);
    Motor_SetPwm(MOTOR_BRUSHED_FTM_BASEADDR, MOTOR_BRUSHED, speed_duty);
    delay(DELAY_BRUSHED_CMD);
}
