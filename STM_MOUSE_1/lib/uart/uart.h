#ifndef UART_H
#define UART_H

#include "stm32f1xx_hal.h"

void UART_Init(void);
void UART_Print(const char *str);

#endif
