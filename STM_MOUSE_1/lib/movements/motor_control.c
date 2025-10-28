#include "motor_control.h"
#include "mpuFunctions.h"
_Bool MovedCell = 0;

uint8_t x_cor;
uint8_t y_cor;
uint8_t Navegating_times;
uint8_t isNavigating = 1; // navigating should be button to check wheather you are navigating or not (TODO) SOLVING MAZE --> it's done but not tested properly

/**
 * @brief Initialize GPIO pins for motor control.
 */
void MotorControl_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Initialize GPIO pins for Motor Control
    GPIO_InitStruct.Pin = Motor_A_6_Pin | Motor_A_5_Pin | Motor_B_4_Pin | Motor_B_3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // PWM output mode
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize PWM
    // Note: You will need to configure the PWM timers and channels separately
}

/**
 * @brief Control the motors based on the specified movement and speeds.
 * @param MOVING The movement state (e.g., FORWARD, HALF_TURN).
 * @param speed_A Speed for motor A.
 * @param speed_B Speed for motor B.
 */
void Movements(uint8_t MOVING, uint8_t speed_A, uint8_t speed_B)
{
    switch (MOVING)
    {
    case FORWARD:
        // Set PWM duty cycles to control motor speed
        // Update with appropriate HAL_TIM_PWM_SetCompare function calls
        HAL_GPIO_WritePin(Motor_A_6_GPIO_Port, Motor_A_6_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_A_5_GPIO_Port, Motor_A_5_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motor_B_4_GPIO_Port, Motor_B_4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_B_3_GPIO_Port, Motor_B_3_Pin, GPIO_PIN_SET);
        HAL_Delay(10); // Adjust delay as needed
        break;

    case HALF_TURN:
        // Update with appropriate PWM control
        HAL_GPIO_WritePin(Motor_A_6_GPIO_Port, Motor_A_6_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motor_A_5_GPIO_Port, Motor_A_5_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_B_4_GPIO_Port, Motor_B_4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_B_3_GPIO_Port, Motor_B_3_Pin, GPIO_PIN_SET);
        HAL_Delay(10); // Adjust delay as needed
        break;

    case RIGHT:
        // Update with appropriate PWM control
        HAL_GPIO_WritePin(Motor_A_6_GPIO_Port, Motor_A_6_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motor_A_5_GPIO_Port, Motor_A_5_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_B_4_GPIO_Port, Motor_B_4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_B_3_GPIO_Port, Motor_B_3_Pin, GPIO_PIN_SET);
        HAL_Delay(10); // Adjust delay as needed
        break;

    case LEFT:
        // Update with appropriate PWM control
        HAL_GPIO_WritePin(Motor_A_6_GPIO_Port, Motor_A_6_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_A_5_GPIO_Port, Motor_A_5_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motor_B_4_GPIO_Port, Motor_B_4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motor_B_3_GPIO_Port, Motor_B_3_Pin, GPIO_PIN_RESET);
        HAL_Delay(10); // Adjust delay as needed
        break;

    case STOP:
        // Stop all motors
        HAL_GPIO_WritePin(Motor_A_6_GPIO_Port, Motor_A_6_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_A_5_GPIO_Port, Motor_A_5_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_B_4_GPIO_Port, Motor_B_4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_B_3_GPIO_Port, Motor_B_3_Pin, GPIO_PIN_RESET);
        HAL_Delay(10); // Adjust delay as needed
        break;

    default:
        // Stop all motors
        HAL_GPIO_WritePin(Motor_A_6_GPIO_Port, Motor_A_6_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_A_5_GPIO_Port, Motor_A_5_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_B_4_GPIO_Port, Motor_B_4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_B_3_GPIO_Port, Motor_B_3_Pin, GPIO_PIN_RESET);
        HAL_Delay(10); // Adjust delay as needed
        break;
    }
}

/**
 * @brief Stop the motors.
 */
void Oop(void)
{
    // Stop all movements
    Movements(STOP, 0, 0);
    HAL_Delay(100); // Adjust delay as needed
}

/**
 * @brief Rotate 90 degrees counterclockwise.
 */
void rotate_left(void)
{
    // Implement rotation logic
    if (CurrentDirection() == NORTH)
    {
        while (CurrentDirection() == WEST)
            Movements(RIGHT, 100, 100);
    }
    else if (CurrentDirection() == EAST)
    {
        while (CurrentDirection() == NORTH)
            Movements(RIGHT, 100, 100);
    }
    else if (CurrentDirection() == SOUTH)
    {
        while (CurrentDirection() == EAST)
            Movements(RIGHT, 100, 100);
    }
    else if (CurrentDirection() == WEST)
    {
        while (CurrentDirection() == SOUTH)
            Movements(RIGHT, 100, 100);
    }
}

/**
 * @brief Rotate 90 degrees clockwise.
 */
void rotate_right(void)
{
    // Implement rotation logic
    if (CurrentDirection() == NORTH)
    {
        while (CurrentDirection() == EAST)
            Movements(LEFT, 100, 100);
    }
    else if (CurrentDirection() == EAST)
    {
        while (CurrentDirection() == SOUTH)
            Movements(LEFT, 100, 100);
    }
    else if (CurrentDirection() == SOUTH)
    {
        while (CurrentDirection() == WEST)
            Movements(LEFT, 100, 100);
    }
    else if (CurrentDirection() == WEST)
    {
        while (CurrentDirection() == NORTH)
            Movements(LEFT, 100, 100);
    }
}

/**
 * @brief Move forward one cell.
 */
void forward(void)
{
    // Increment the trail marker for the current cell
    if (isNavigating)
    {
        if (Navegating_times == DONE_Navigating)
        {
            trail[x_cor][y_cor] = 0;
            // EEPROM.write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
        }
        else
        {
            if (x_cor != 0 && y_cor != 0)
            {
                trail[x_cor][y_cor]++;
                // EEPROM.write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
            }
            else
            {
                trail[x_cor][y_cor] = 255;
                // EEPROM.write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
            }
        }
    }

    while (!MovedCell)
        Movements(FORWARD, 100, 100);
}

// void Motors_Test()
// {
// Oop();
// delay(1000);
// Movements(LEFT, 100, 100);
// Oop();
// delay(1000);
// Movements(RIGHT, 100, 100);
// Oop();
// delay(1000);
// Movements(HALF_TURN, 100, 100);
// Oop();
// delay(1000);
//   Movements(FORWARD, 255, 255);
// }