#ifndef __MILLIS_DELAY_H
#define __MILLIS_DELAY_H

#include "stm32f1xx_hal.h"

volatile uint32_t msTicks; // Variable to hold the millisecond counter

void SysTick_Handler(void);
uint32_t millis(void);
void delay(uint32_t delayMs);

#endif /* __MILLIS_DELAY_H */