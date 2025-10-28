// #include "mpu6050.h"
// #include <math.h>
// #include <stdbool.h>
// #include "mpuFunctions.h"
// #include "timer.h"
// #include "uart.h"

// int sample_index = 0;
// float total_displacement = 0.0;
// double ax_samples[WINDOW_SIZE] = {0};
// float ax_offset = 0, ay_offset = 0, az_offset = 0;
// float gyroXcal = 0, gyroYcal = 0, gyroZcal = 0;
// float yaw = 0.0;
// float Accelration = 0, Velocity = 0;
// int Displacement = 0;

// /**
//  * @brief Calculates the moving average of the given samples.
//  *
//  * @param new_sample The new sample to add to the moving average.
//  * @param samples The array holding the past samples.
//  * @param size The size of the samples array.
//  * @return double The updated moving average.
//  */
// double moving_average(double new_sample, double *samples, int size)
// {
//     samples[sample_index % size] = new_sample;
//     sample_index++;
//     double sum = 0;
//     for (int i = 0; i < size; i++)
//     {
//         sum += samples[i];
//     }
//     return sum / size;
// }

// /**
//  * @brief Normalizes an angle to the range [0, 360] degrees.
//  *
//  * @param angle The angle to normalize.
//  * @return float The normalized angle.
//  */
// float normalize_angle(float angle)
// {
//     // Use modulo to wrap the angle within 0 to 360
//     angle = fmod(angle, 360.0);

//     // Ensure positive angle
//     if (angle < 0)
//         angle += 360.0;

//     return angle;
// }



// void calibrate_sensor(void)
// {
//     UART_Print("Calibrating...\n");

//     const int samples = 1000;
//     for (int i = 0; i < samples; i++)
//     {
//         Struct_MPU6050 data;
//         mpu6050_readData(&data);

//         ax_offset += data.accel.x;
//         ay_offset += data.accel.y;
//         az_offset += data.accel.z;

//         gyroXcal += data.gyro.x;
//         gyroYcal += data.gyro.y;
//         gyroZcal += data.gyro.z;

//         delay(2); // Small delay to ensure accurate readings
//     }

//     ax_offset /= samples;
//     ay_offset /= samples;
//     az_offset /= samples;

//     gyroXcal /= samples;
//     gyroYcal /= samples;
//     gyroZcal /= samples;

//     UART_Print("\nCalibration complete.\n");
//     char buffer[100];
//     sprintf(buffer, "Accel Offsets: %f, %f, %f\n", ax_offset, ay_offset, az_offset);
//     UART_Print(buffer);
//     sprintf(buffer, "Gyro Offsets: %f, %f, %f\n", gyroXcal, gyroYcal, gyroZcal);
//     UART_Print(buffer);
// }

// void initializeMPU(void)
// {
//     I2C_HandleTypeDef hi2c1;
//     hi2c1.Instance = I2C2;
//     hi2c1.Init.ClockSpeed = 400000;
//     hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//     hi2c1.Init.OwnAddress1 = 0;
//     hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//     hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//     hi2c1.Init.OwnAddress2 = 0;
//     hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//     hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//     if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//     {
//         // Initialization Error
//         // Error_Handler();
//     }

//     if (!mpu6050_init(&hi2c1))
//     {
//         UART_Print("ERRRRRRRRRRR!\n");
//     }
//     UART_Print("MPU6050 Initialized!\n");
//     calibrate_sensor();
//     delay(100);
// }

// void Calculate_Accelration_Velocity_Displacement_Yaw(void)
// {
//     static uint32_t previousTime = 0;
//     uint32_t currentTime = millis();
//     float elapsedTime = (currentTime - previousTime) / 1000.0f; // Convert to seconds

//     Struct_MPU6050 data;
//     mpu6050_readData(&data);
//     float ax = data.acc_x - ax_offset;

//     Accelration = moving_average(ax, ax_samples, WINDOW_SIZE);
//     Velocity = fabs(Accelration) * elapsedTime;
//     Displacement = Velocity * elapsedTime;

//     if (total_displacement == 15)
//     {
//         total_displacement = 0;
//         MovedCell = true;
//     }
//     else
//     {
//         MovedCell = false;
//         total_displacement += Displacement;
//     }

//     float gz = data.gyro_z - gyroZcal;
//     yaw += gz * elapsedTime;
//     yaw = normalize_angle(yaw);

//     char buffer[100];
//     sprintf(buffer, "A : %f V : %f D: %d\n", Accelration, Velocity, Displacement);
//     UART_Print(buffer);

//     previousTime = currentTime;
// }

// E_Direction_t CurrentDirection()
// {
//     if ((yaw >= 345 || yaw <= 15)) // 0
//         return NORTH;
//     else if ((yaw >= 165 && yaw <= 195)) // 180
//         return SOUTH;
//     else if ((yaw >= 75 && yaw <= 105)) // 90
//         return WEST;
//     else if ((yaw >= 255 && yaw <= 285)) // 270
//         return EAST;
//     return DONTCARE;
// }
