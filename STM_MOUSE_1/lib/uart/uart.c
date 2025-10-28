// #include "uart.h"
// #include "stm32f1xx_hal.h"
// #include <string.h>

// UART_HandleTypeDef huart2; // UART handler for USART2

// void UART_Init(void)
// {
//     // Enable UART2 Clock
//     __HAL_RCC_USART2_CLK_ENABLE();

//     // Initialize the UART peripheral (USART2)
//     huart2.Instance = USART2;
//     huart2.Init.BaudRate = 9600;
//     huart2.Init.WordLength = UART_WORDLENGTH_8B;
//     huart2.Init.StopBits = UART_STOPBITS_1;
//     huart2.Init.Parity = UART_PARITY_NONE;
//     huart2.Init.Mode = UART_MODE_TX_RX;
//     huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//     huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//     if (HAL_UART_Init(&huart2) != HAL_OK)
//     {
//         // Initialization Error
//         // Error_Handler();
//     }
// }

// void UART_Print(const char *str)
// {
//     HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
// }
