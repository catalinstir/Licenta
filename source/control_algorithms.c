#include "control_algorithms.h"

void print_float(float num) {

    float intpart;
    float frac = modff(num, &intpart);
    int integer_part = (int)intpart;
    int decimal_part = (int)(fabsf(frac) * 1000.0f);

    if (num < 0.0f && integer_part == 0) {
        PRINTF("-0.%03d", decimal_part);
    } else if (num < 0.0f) {
        PRINTF("-%d.%03d", abs(integer_part), decimal_part);
    } else {
        PRINTF("%d.%03d", integer_part, decimal_part);
    }
}

// PID
uint16_t MapControlToDutyCycle(float control) {

    float pwm_value = PWM_MIN + (control + 1.0f) * (PWM_MAX - PWM_MIN) / 2.0f;

    if (pwm_value < PWM_MIN)
        pwm_value = PWM_MIN;
    if (pwm_value > PWM_MAX)
        pwm_value = PWM_MAX;

    const uint16_t discrete_pwm[] = {500, 600, 700, 800, 900};
    const int num_values = sizeof(discrete_pwm) / sizeof(discrete_pwm[0]);

    uint16_t closest = discrete_pwm[0];
    float min_diff = fabsf(pwm_value * 100.0f - (float)discrete_pwm[0]);

    for (int i = 1; i < num_values; i++) {
        float diff = fabsf(pwm_value * 100.0f - (float)discrete_pwm[i]);
        if (diff < min_diff) {
            min_diff = diff;
            closest = discrete_pwm[i];
        }
    }

    return closest;
}
float CalculateError(float x_center) {

    return CENTER_X - x_center;
}
float CalculateCombinedError(float x_center1, float x_center2) {
    float error1 = CalculateError(x_center1);
    float error2 = CalculateError(x_center2);

    // Ponderi: 70% aproape, 30% anticipare
    return 0.7f * error1 + 0.3f * error2;
}

void CalculateLaneCenter2(VectorType v1, VectorType v2, float *x_1, float *x_2, float *y_1,
                          float *y_2) {
    int v1_jos_x = (v1.m_y0 > v1.m_y1) ? v1.m_x0 : v1.m_x1;
    int v1_jos_y = (v1.m_y0 > v1.m_y1) ? v1.m_y0 : v1.m_y1;
    int v2_jos_x = (v2.m_y0 > v2.m_y1) ? v2.m_x0 : v2.m_x1;
    int v2_jos_y = (v2.m_y0 > v2.m_y1) ? v2.m_y0 : v2.m_y1;

    int v1_sus_x = (v1.m_y0 <= v1.m_y1) ? v1.m_x0 : v1.m_x1;
    int v1_sus_y = (v1.m_y0 <= v1.m_y1) ? v1.m_y0 : v1.m_y1;
    int v2_sus_x = (v2.m_y0 <= v2.m_y1) ? v2.m_x0 : v2.m_x1;
    int v2_sus_y = (v2.m_y0 <= v2.m_y1) ? v2.m_y0 : v2.m_y1;

    *x_1 = ((float)v1_jos_x + (float)v2_jos_x) / 2.0f;
    *y_1 = ((float)v1_jos_y + (float)v2_jos_y) / 2.0f;
    *x_2 = ((float)v1_sus_x + (float)v2_sus_x) / 2.0f;
    *y_2 = ((float)v1_sus_y + (float)v2_sus_y) / 2.0f;
}
void CalculateLaneCenter(VectorType v1, VectorType v2, float *x_center, float *y_center) {
    // Media coordonatelor x și y dintre v1 și v2
    *x_center = ((float)v1.m_x0 + (float)v1.m_x1 + (float)v2.m_x0 + (float)v2.m_x1) / 4.0f;
    *y_center = ((float)v1.m_y0 + (float)v1.m_y1 + (float)v2.m_y0 + (float)v2.m_y1) / 4.0f;
}

// PID
float PID_Control(PIDController *pid, float error) {
    pid->integral += error;

    if (pid->integral > pid->max_integral)
        pid->integral = pid->max_integral;
    if (pid->integral < -pid->max_integral)
        pid->integral = -pid->max_integral;

    float derivative = error - pid->previous_error;
    pid->previous_error = error;

    float control = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    if (control < -1.0f)
        control = -1.0f;
    if (control > 1.0f)
        control = 1.0f;
    return control;
}

void ProcessVectorsPID(VectorType v1, VectorType v2) {

    //     float x_1, y_1,x_2, y_2;
    //     CalculateLaneCenter2(v1, v2, &x_1,&x_2,&y_1,&y_2);
    //     float error = CalculateCombinedError(x_1,x_2);

    float x_center, y_center;
    CalculateLaneCenter(v1, v2, &x_center, &y_center);
    float error = CalculateError(x_center);

    float control_steer = PID_Control(&pid_steering, error);
    uint16_t duty_cycle = MapControlToDutyCycle(control_steer);

    uint16_t speed_pwm = CalculateSpeedFromDuty(duty_cycle);
    // uint16_t speed_pwm = CalculateSpeedFromControl(control_steer);

    Motor_SetPwm(MOTOR_STEERING_FTM_BASEADDR, MOTOR_STEERING, 8000);
    delay(DELAY_STEERING_RESET);
    Motor_SetPwm(MOTOR_STEERING_FTM_BASEADDR, MOTOR_STEERING, duty_cycle);
    delay(DELAY_STEERING_CMD);

    Motor_SetPwm(MOTOR_BRUSHED_FTM_BASEADDR, MOTOR_BRUSHED, 8000);
    delay(DELAY_BRUSHED_RESET);
    Motor_SetPwm(MOTOR_BRUSHED_FTM_BASEADDR, MOTOR_BRUSHED, speed_pwm);
    delay(DELAY_BRUSHED_CMD);

    if (simulator) {
        PRINTF("-ERR:");
        print_float(error);
        PRINTF(" CONTROL:");
        print_float(control_steer);
        PRINTF(" STEER:%d SPEED:%d\r\n", duty_cycle, speed_pwm);
    }
}

uint16_t CalculateSpeedFromDuty(uint16_t duty_cycle) {
    //      int deviation = abs((int)duty_cycle - 700);
    //      float scale = 1.0f - ((float)deviation / 300.0f);
    //
    //      float pwm_speed = PWM_MIN_SPEED + scale * (PWM_MAX_SPEED - PWM_MIN_SPEED);

    float pwm_speed;
    if (duty_cycle == 700) {
        pwm_speed = PWM_MAX_SPEED;

    } else
        pwm_speed = PWM_MIN_SPEED;

    return (uint16_t)(pwm_speed * 100);
}

uint16_t CalculateSpeedFromControl(float control) {
    if (control < -1.0f)
        control = -1.0f;
    if (control > 1.0f)
        control = 1.0f;

    float deviation = fabsf(control);
    float scale = 1.0f - deviation;

    float pwm_speed = PWM_MIN_SPEED + scale * (PWM_MAX_SPEED - PWM_MIN_SPEED);

    return (uint16_t)(pwm_speed * 100);
}
