#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>

typedef enum {
    E_OK,
    E_NOT_OK
} ReturnType;

typedef struct {
    uint8_t m_x0;
    uint8_t m_y0;
    uint8_t m_x1;
    uint8_t m_y1;
    uint8_t m_index;
    uint8_t m_flags;
} VectorType;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float max_integral;
    float previous_error;
    float output_max;
} PIDController;

typedef struct {
    uint16_t steer_duty;
    uint16_t speed_duty;
} MotorCommand_t;

typedef enum {
    STATE_WAIT,
    STATE_READ_CAMERA,
    STATE_PROCESS_VECTOR,
    STATE_CONTROL,
    STATE_STOP,
    STATE_REMOTE_CONTROL
} SystemState;

typedef enum {
    PACKET_NONE,
    PACKET_STOP,
    PACKET_RC_ENTER,
    PACKET_RC_AUTO,
    PACKET_RC_CONTROL
} UARTPacketType;

#endif // COMMON_TYPES_H
