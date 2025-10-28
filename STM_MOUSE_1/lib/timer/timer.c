#include "stm32f1xx_hal.h"

volatile uint32_t msTicks; // Variable to hold the millisecond counter

void SysTick_Handler(void)
{
    msTicks++;
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

uint32_t millis(void)
{
    return msTicks;
}

void delay(uint32_t delayMs)
{
    uint32_t start = millis();
    while (millis() - start < delayMs)
        ;
}
