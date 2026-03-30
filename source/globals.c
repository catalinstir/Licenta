#include "globals.h"

unsigned int stop;
VectorType Vector1;
VectorType Vector2;
VectorType vectori[8];
volatile uint8_t count;
volatile SystemState currentState;
volatile bool check_obstacle;
volatile bool simulator;
PIDController pid_steering;
volatile bool uart_data_ready;
volatile uint32_t lastRpmPrint;
volatile uint32_t g_systemTime_ms;

float PWM_MIN, PWM_MAX;
float PWM_MIN_SPEED, PWM_MAX_SPEED;

void PID_Init(PIDController *pid, float kp, float ki, float kd, float output_max)
{
    pid->Kp             = kp;
    pid->Ki             = ki;
    pid->Kd             = kd;
    pid->integral       = 0.0f;
    pid->previous_error = 0.0f;
    pid->output_max     = output_max;

    if (ki != 0.0f)
        pid->max_integral = (output_max / ki) * 0.8f;
    else
        pid->max_integral = 0.0f;
}

void InitGlobals(void)
{
    stop            = 0;
    currentState    = STATE_INIT;
    check_obstacle  = false;
    simulator       = false;
    uart_data_ready = false;
    count           = 0;
    lastRpmPrint    = 0;
    g_systemTime_ms = 0;

    PID_Init(&pid_steering, 0.045f, 0.0f, 0.06f, 1.0f); // testat real bun

    // PID_Init(&pid_steering, 0.035f, 0.0f, 0.025f, 1.0f); // simulator bun1
    // PID_Init(&pid_steering, 0.04f, 0.0f, 0.03f, 1.0f); // simulator bun2

    // PWM limits
    PWM_MIN       = 5.0f;
    PWM_MAX       = 9.0f;
    PWM_MIN_SPEED = 7.7f;
    PWM_MAX_SPEED = 7.7f;
}
