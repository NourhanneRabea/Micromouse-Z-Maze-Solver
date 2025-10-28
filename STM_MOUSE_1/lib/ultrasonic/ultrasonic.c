#include "ultrasonic.h"
#include "../utils.h"
#include <stm32f1xx_hal_tim.h>
#include "uart.h"

void HAL_DelayMicroseconds(uint32_t us)
{
    // Enable the DWT counter
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk))
    {
        // Enable DWT Cycle Counter
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t startTick = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000); // Calculate the number of ticks for the required microseconds

    while ((DWT->CYCCNT - startTick) < ticks); // Wait for the required number of ticks
}



/**
 * @brief Measure and return the distance using the ultrasonic sensor.
 * @param trig_port The GPIO port for the trigger pin.
 * @param trig_pin The GPIO pin for the trigger.
 * @param echo_port The GPIO port for the echo pin.
 * @param echo_pin The GPIO pin for the echo.
 * @return The measured distance in centimeters.
 */uint16_t GetUltrasonicDistance(GPIO_TypeDef *trig_port, uint16_t trig_pin, GPIO_TypeDef *echo_port, uint16_t echo_pin)
{
    // Trigger the ultrasonic sensor with a 10 µs pulse
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
    HAL_Delay(1); // Delay 1 ms to settle the sensor
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
    HAL_DelayMicroseconds(10); // Delay for 10 µs
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);

    // Measure the echo return time using DWT
    uint32_t startTick, endTick, duration;

    // Wait for the echo pin to go HIGH
    startTick = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_RESET)
    {
        if ((DWT->CYCCNT - startTick) > HAL_RCC_GetHCLKFreq() / 10) // Timeout after 100ms
        {
            return 0;
        }
    }

    // Start timing the high pulse
    startTick = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET)
    {
        if ((DWT->CYCCNT - startTick) > HAL_RCC_GetHCLKFreq()) // Timeout after 1 second (too long)
        {
            return 0;
        }
    }

    // Calculate duration in microseconds
    endTick = DWT->CYCCNT;
    duration = endTick - startTick;
    float duration_us = (duration / (HAL_RCC_GetHCLKFreq() / 1000000)); // Convert ticks to microseconds

    // Calculate distance in cm (duration in microseconds)
    float distance_cm = (duration_us * 0.0343) / 2.0;

    // Return the distance in millimeters
    return (uint16_t)(distance_cm * 10); // Convert to millimeters
}


/**
 * @brief Check if a wall is present based on ultrasonic distance measurements.
 * @param trig_port The GPIO port for the trigger pin.
 * @param trig_pin The GPIO pin for the trigger.
 * @param echo_port The GPIO port for the echo pin.
 * @param echo_pin The GPIO pin for the echo.
 * @return 0 if no wall is present, 1 if a wall is detected, 11 if error.
 */
uint8_t wall_present(GPIO_TypeDef *trig_port, uint16_t trig_pin, GPIO_TypeDef *echo_port, uint16_t echo_pin)
{
    // Measure the distance from the sensor to the wall
    uint32_t distance = GetUltrasonicDistance(trig_port, trig_pin, echo_port, echo_pin);

    // Check distance against thresholds
    if (distance > THRESHOLD_DISTANCE && distance <= LARGEST_DISTANCE)
    {
        return 0; // No wall
    }
    else if (distance <= THRESHOLD_DISTANCE || distance <= SMALLEST_DISTANCE || distance > LARGEST_DISTANCE)
    {
        return 1; // Wall detected
    }
    return 11; // Error
}



// void Ultrasonics_Test()
// {
//   // Check if there are walls in different directions
//   uint32_t way_left = GetUltrasonicDistance(ltrigger_GPIO_Port, ltrigger_Pin, lecho_GPIO_Port, lecho_Pin);
//   uint32_t way_front = GetUltrasonicDistance(ftrigger_GPIO_Port, ftrigger_Pin, fecho_GPIO_Port, fecho_Pin);
//   uint32_t way_right = GetUltrasonicDistance(rtrigger_GPIO_Port, rtrigger_Pin, recho_GPIO_Port, recho_Pin);

//   char buffer[50]; // Buffer to hold formatted strings

//   // Print left sensor value
//   UART_Print("Left Sensor: ");
//   sprintf(buffer, "%ld\n", way_left); // Format the integer as a string
//   UART_Print(buffer);                 // Print the formatted value

//   // Print front sensor value
//   UART_Print("Front Sensor: ");
//   sprintf(buffer, "%ld\n", way_front); // Format the integer as a string
//   UART_Print(buffer);

//   // Print right sensor value
//   UART_Print("Right Sensor: ");
//   sprintf(buffer, "%ld\n", way_right); // Format the integer as a string
//   UART_Print(buffer);
// }
