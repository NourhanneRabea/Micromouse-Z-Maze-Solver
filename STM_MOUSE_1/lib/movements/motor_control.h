#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"
#include "../utils.h"

// Define motor control pins (example pins, adjust as necessary)
#define Motor_A_6_Pin GPIO_PIN_6
#define Motor_A_6_GPIO_Port GPIOB
#define Motor_A_5_Pin GPIO_PIN_7
#define Motor_A_5_GPIO_Port GPIOB
#define Motor_B_4_Pin GPIO_PIN_8
#define Motor_B_4_GPIO_Port GPIOB
#define Motor_B_3_Pin GPIO_PIN_9
#define Motor_B_3_GPIO_Port GPIOB

// Define movement states
#define FORWARD 0
#define HALF_TURN 1
#define RIGHT 2
#define LEFT 3
#define STOP 4

// Function prototypes
void MotorControl_Init(void);
void Movements(uint8_t MOVING, uint8_t speed_A, uint8_t speed_B);
void Oop(void);
void rotate_left(void);
void rotate_right(void);
void forward(void);
// void Motors_Test();

#endif // MOTOR_CONTROL_H
