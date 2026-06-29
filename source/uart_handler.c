#include "uart_handler.h"
#include "control_algorithms.h"
#include "hall_sensor.h"

#define RX_BUFFER_SIZE 32
static volatile char rxBuffer[RX_BUFFER_SIZE];
static volatile uint8_t rxIndex = 0;

volatile bool lqr_gains_updated = false;

void UART4_UserIRQHandler(void)
{
    if (!(kUART_RxDataRegFullFlag & UART_GetStatusFlags(UART4)))
        return;

    char c = UART_ReadByte(UART4);

    if (c == '\r')
    {
        rxBuffer[rxIndex] = '\0';

        if (rxIndex == 1 && rxBuffer[0] == 'S')
        {
            /* Emergency stop: set flag immediately, no main-loop polling needed */
            stop             = 1;
            uart_packet_type = PACKET_STOP;
        }
        else if (rxIndex == 1 && rxBuffer[0] == 'R')
        {
            stop               = 0;
            rc_enter_requested = true;
            uart_packet_type   = PACKET_RC_ENTER;
        }
        else if (rxIndex == 1 && rxBuffer[0] == 'A')
        {
            uart_packet_type = PACKET_RC_AUTO;
        }
        else if (rxIndex >= 3 && rxBuffer[0] == 'H' && rxBuffer[1] == ':')
        {
            /* Hall RPM packet from ESP8266: "H:<rpm>" */
            int rpm_int;
            if (sscanf((const char *)&rxBuffer[2], "%d", &rpm_int) == 1)
                HallSensor_SetExternalRPM((float)rpm_int);
        }
        else if (rxIndex >= 6 && rxBuffer[0] == 'G' &&
                 (rxBuffer[1] == 'S' || rxBuffer[1] == 'T') && rxBuffer[2] == ':')
        {
            /* Gain-schedule packets: "GS:<ke> <kt> <kp>" or "GT:<ke> <kt> <kp>"
             * Values are integers × 1000 to avoid float sscanf in ISR. */
            int ke_i, kt_i, kp_i;
            if (sscanf((const char *)&rxBuffer[3], "%d %d %d", &ke_i, &kt_i, &kp_i) == 3)
            {
                float ke = ke_i / 1000.0f, kt = kt_i / 1000.0f, kp = kp_i / 1000.0f;
                if (rxBuffer[1] == 'S')
                    LQR_SetStraightGains(ke, kt, kp);
                else
                    LQR_SetTurnGains(ke, kt, kp);
                lqr_gains_updated = true;
            }
        }
        else
        {
            /* Try to parse as "<steer> <speed>" RC control packet */
            int steer, speed;
            if (sscanf((const char *)rxBuffer, "%d %d", &steer, &speed) == 2)
            {
                rc_steer_cmd     = (uint16_t)steer;
                rc_speed_cmd     = (uint16_t)speed;
                rc_packet_ready  = true;
                uart_packet_type = PACKET_RC_CONTROL;
            }
            /* Anything else is silently ignored */
        }

        rxIndex = 0;
    }
    else if (c == '\n')
    {
        /* ignore — ESP8266 may append LF after CR */
    }
    else if (rxIndex < (RX_BUFFER_SIZE - 1))
    {
        rxBuffer[rxIndex++] = c;
    }
    else
    {
        /* Buffer overrun — discard line */
        rxIndex = 0;
    }
}

/* Legacy simulator frame reader — kept for reference, not used in RC mode */
bool Simulator_GetFrame(VectorType *vectori, uint8_t *count)
{
    PRINTF("-REQ_FRAME-\r\n");

    int timeout = 1000000;
    while (!uart_data_ready && timeout > 0)
    {
        __NOP();
        timeout--;
    }

    if (timeout == 0)
    {
        PRINTF("Simulator_GetFrame: timeout!\r\n");
        return false;
    }
    uart_data_ready = false;
    ParseUARTMessage((char *)rxBuffer, &vectori[0], &vectori[1]);
    *count = 2;
    return true;
}

void ParseUARTMessage(char *message, VectorType *v1, VectorType *v2)
{
    int values[8];
    int parsed = sscanf(message, "%d %d %d %d %d %d %d %d",
                        &values[0], &values[1], &values[2], &values[3],
                        &values[4], &values[5], &values[6], &values[7]);
    if (parsed == 8)
    {
        v1->m_x0 = (uint8_t)values[0];
        v1->m_y0 = (uint8_t)values[1];
        v1->m_x1 = (uint8_t)values[2];
        v1->m_y1 = (uint8_t)values[3];

        v2->m_x0 = (uint8_t)values[4];
        v2->m_y0 = (uint8_t)values[5];
        v2->m_x1 = (uint8_t)values[6];
        v2->m_y1 = (uint8_t)values[7];
    }
    else
    {
        PRINTF("\r\nEroare la parsarea mesajului!\r\n");
    }
}
