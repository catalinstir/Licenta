#include "uart_handler.h"

#define RX_BUFFER_SIZE 100
volatile char rxBuffer[RX_BUFFER_SIZE];
volatile uint8_t rxIndex = 0;

bool Simulator_GetFrame(VectorType *vectori, uint8_t *count) {
    PRINTF("-REQ_FRAME-\r\n");

    int timeout = 1000000;
    while (!uart_data_ready && timeout > 0) {
        __NOP();
        timeout--;
    }

    if (timeout == 0) {
        PRINTF("Simulator_GetFrame: timeout!\r\n");
        return false;
    }
    uart_data_ready = false;
    ParseUARTMessage(rxBuffer, &vectori[0], &vectori[1]);
    *count = 2;

    return true;
}
void UART4_UserIRQHandler(void) {
    if (kUART_RxDataRegFullFlag & UART_GetStatusFlags(UART4)) {
        char receivedChar = UART_ReadByte(UART4);

        if (rxIndex < (RX_BUFFER_SIZE - 1)) {
            rxBuffer[rxIndex++] = receivedChar;
        }
        if (receivedChar == '\r') {
            rxBuffer[rxIndex] = '\0';
            uart_data_ready = true;
            rxIndex = 0;
        }
    }
}
void ParseUARTMessage(char *message, VectorType *v1, VectorType *v2) {
    int values[8];
    int parsed = sscanf(message, "%d %d %d %d %d %d %d %d", &values[0], &values[1], &values[2],
                        &values[3], &values[4], &values[5], &values[6], &values[7]);
    if (parsed == 8) {
        v1->m_x0 = (uint8_t)values[0];
        v1->m_y0 = (uint8_t)values[1];
        v1->m_x1 = (uint8_t)values[2];
        v1->m_y1 = (uint8_t)values[3];

        v2->m_x0 = (uint8_t)values[4];
        v2->m_y0 = (uint8_t)values[5];
        v2->m_x1 = (uint8_t)values[6];
        v2->m_y1 = (uint8_t)values[7];
    } else {
        PRINTF("\r\nEroare la parsarea mesajului!\r\n");
        return;
    }
    PRINTF("\r\nVectori extrași: V1(%d,%d -> %d,%d) | V2(%d,%d -> %d,%d)\r\n", v1->m_x0, v1->m_y0,
           v1->m_x1, v1->m_y1, v2->m_x0, v2->m_y0, v2->m_x1, v2->m_y1);
}
