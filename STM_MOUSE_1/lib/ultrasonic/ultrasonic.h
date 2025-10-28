#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim2;

// Define Ultrasonic pins
#define ltrigger_Pin GPIO_PIN_11
#define ltrigger_GPIO_Port GPIOA
#define lecho_Pin GPIO_PIN_12
#define lecho_GPIO_Port GPIOA

#define ftrigger_Pin GPIO_PIN_9
#define ftrigger_GPIO_Port GPIOA
#define fecho_Pin GPIO_PIN_10
#define fecho_GPIO_Port GPIOA

#define rtrigger_Pin GPIO_PIN_0
#define rtrigger_GPIO_Port GPIOA
#define recho_Pin GPIO_PIN_1
#define recho_GPIO_Port GPIOA

// Function prototypes
uint16_t GetUltrasonicDistance(GPIO_TypeDef *trig_port, uint16_t trig_pin, GPIO_TypeDef *echo_port, uint16_t echo_pin);
uint8_t wall_present(GPIO_TypeDef *trig_port, uint16_t trig_pin, GPIO_TypeDef *echo_port, uint16_t echo_pin);
// void Ultrasonics_Test();

#endif // ULTRASONIC_H
