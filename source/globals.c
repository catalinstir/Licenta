#include "globals.h"
#include "config.h"

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

volatile UARTPacketType uart_packet_type;
volatile bool rc_enter_requested;
volatile bool rc_packet_ready;
volatile uint16_t rc_steer_cmd;
volatile uint16_t rc_speed_cmd;
volatile uint32_t rc_last_packet_ms;

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
    stop               = 0;
    currentState       = STATE_WAIT;
    check_obstacle     = false;
    simulator          = false;
    uart_data_ready    = false;
    count              = 0;
    lastRpmPrint       = 0;
    g_systemTime_ms    = 0;
    uart_packet_type   = PACKET_NONE;
    rc_enter_requested = false;
    rc_packet_ready    = false;
    rc_steer_cmd       = 700;
    rc_speed_cmd       = 770;
    rc_last_packet_ms  = 0;

    PID_Init(&pid_steering, PID_STEERING_KP, PID_STEERING_KI, PID_STEERING_KD, 1.0f);
}
