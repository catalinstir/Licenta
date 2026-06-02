#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include "common_types.h"
#include "fsl_debug_console.h"
#include "fsl_uart.h"
#include "globals.h"
#include <stdio.h>

void UART4_UserIRQHandler(void);
bool Simulator_GetFrame(VectorType *vectori, uint8_t *count);
void ParseUARTMessage(char *message, VectorType *v1, VectorType *v2);

#endif // UART_HANDLER_H
