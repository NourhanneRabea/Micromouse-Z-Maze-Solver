#include "main.h"
#include "stm32f1xx_hal.h"
// #include "mpuFunctions.h"
#include "timer.h"
// #include "uart.h"
#include "ultrasonic.h"
#include "motor_control.h"
#include "Floodfill.h"
#include <string.h>

UART_HandleTypeDef huart2; // UART handler for USART2
TIM_HandleTypeDef htim2;
int sample_index = 0;
uint32_t previousTime = 0;
float total_displacement = 0.0;
double ax_samples[WINDOW_SIZE] = {0};
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gyroXcal = 0, gyroYcal = 0, gyroZcal = 0;
float yaw = 0.0, roll = 0.0, pitch = 0.0;
float Accelration = 0, Velocity = 0;
int Displacement = 0;

#define TAU 0.98 // Complementary filter constant

// Function prototypes
void SystemClock_Config(void);
void UART_Init(void);
void MX_GPIO_Init(void);
void UART_Print(const char *str);
void Error_Handler(void);
static void MX_TIM_Init(void);
static void MX_TIM2_Init(void);

void Motors_Test()
{
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
  Movements(FORWARD, 255, 255);
}

void Ultrasonics_Test()
{
  // Check if there are walls in different directions
  uint32_t way_left = GetUltrasonicDistance(ltrigger_GPIO_Port, ltrigger_Pin, lecho_GPIO_Port, lecho_Pin);
  uint32_t way_front = GetUltrasonicDistance(ftrigger_GPIO_Port, ftrigger_Pin, fecho_GPIO_Port, fecho_Pin);
  uint32_t way_right = GetUltrasonicDistance(rtrigger_GPIO_Port, rtrigger_Pin, recho_GPIO_Port, recho_Pin);

  char buffer[50]; // Buffer to hold formatted strings

  // Print left sensor value
  UART_Print("Left Sensor: ");
  sprintf(buffer, "%ld\n", way_left); // Format the integer as a string
  UART_Print(buffer);                 // Print the formatted value

  // Print front sensor value
  UART_Print("Front Sensor: ");
  sprintf(buffer, "%ld\n", way_front); // Format the integer as a string
  UART_Print(buffer);

  // Print right sensor value
  UART_Print("Right Sensor: ");
  sprintf(buffer, "%ld\n", way_right); // Format the integer as a string
  UART_Print(buffer);
}

////////////////////////
/////////// I2C ////////

I2C_HandleTypeDef hi2c2;

/* I2C2 init function */
void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}
void HAL_I2C_MspInit(I2C_HandleTypeDef *i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (i2cHandle->Instance == I2C2)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE(); // Ensure GPIOB clock is enabled
    __HAL_RCC_I2C2_CLK_ENABLE();  // Enable I2C2 clock

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *i2cHandle)
{

  if (i2cHandle->Instance == I2C1)
  {
    /* USER CODE BEGIN I2C1_MspDeInit 0 */

    /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    /* USER CODE BEGIN I2C1_MspDeInit 1 */

    /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#define MPU6050_ADDR 0xD0

#define MPU6050_SMPRT_DIV 0X19
#define MPU6050_WHO_AM_I 0X75
#define MPU6050_CONFIG 0X1A
#define MPU6050_GYRO_CONFIG 0X1B
#define MPU6050_ACCEL_CONFIG 0X1C
#define MPU6050_INT_PIN_CFG 0X37
#define MPU6050_INT_ENABLE 0X38
#define MPU6050_INT_STATUS 0X3A
#define MPU6050_ACCEL_XOUT_H 0X3B
#define MPU6050_ACCEL_XOUT_L 0X3C
#define MPU6050_PWR_MGMT_1 0X6B // most important

#define MPU6050_INT_PORT GPIOB
#define MPU6050_INT_PIN GPIO_PIN_5

typedef struct _MPU6050
{
  short acc_x_raw;
  short acc_y_raw;
  short acc_z_raw;
  short temperature_raw;
  short gyro_x_raw;
  short gyro_y_raw;
  short gyro_z_raw;

  float acc_x;
  float acc_y;
  float acc_z;
  float temperature;
  float gyro_x;
  float gyro_y;
  float gyro_z;
} Struct_MPU6050;

Struct_MPU6050 MPU6050;
// I2C_HandleTypeDef hi2c1;

static float LSB_Sensitivity_ACC;
static float LSB_Sensitivity_GYRO;

void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val)
{
  HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}

void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t *data)
{
  HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

void MPU6050_Readbyte(uint8_t reg_addr, uint8_t *data)
{
  HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 1);
}

void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t *data)
{
  HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1);
}

void MPU6050_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC)
{
  switch (FS_SCALE_GYRO)
  {
  case 0:
    LSB_Sensitivity_GYRO = 131.f;
    break;
  case 1:
    LSB_Sensitivity_GYRO = 65.5f;
    break;
  case 2:
    LSB_Sensitivity_GYRO = 32.8f;
    break;
  case 3:
    LSB_Sensitivity_GYRO = 16.4f;
    break;
  }
  switch (FS_SCALE_ACC)
  {
  case 0:
    LSB_Sensitivity_ACC = 16384.f;
    break;
  case 1:
    LSB_Sensitivity_ACC = 8192.f;
    break;
  case 2:
    LSB_Sensitivity_ACC = 4096.f;
    break;
  case 3:
    LSB_Sensitivity_ACC = 2048.f;
    break;
  }
}

// MPU6050 Initialization
void MPU6050_Initialization(void)
{
  HAL_Delay(100);
  uint8_t who_am_i;
  MPU6050_Readbytes(MPU6050_WHO_AM_I, 1, &who_am_i);
  if (who_am_i != 0x68)
  {
    UART_Print("Error: MPU6050 not found!\n");
    while (1)
      ; // Infinite loop on error
  }

  MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x00);  // Wake up MPU6050
  MPU6050_Writebyte(MPU6050_SMPRT_DIV, 39);     // Sample Rate = 200Hz
  MPU6050_Writebyte(MPU6050_INT_PIN_CFG, 0x00); // Set INT pin
  MPU6050_Writebyte(MPU6050_INT_ENABLE, 0x01);  // Enable data ready interrupt
}

// Get raw data from the MPU6050
void MPU6050_Get6AxisRawData(Struct_MPU6050 *mpu6050)
{
  uint8_t data[14];
  MPU6050_Readbytes(MPU6050_ACCEL_XOUT_H, 14, data);
  mpu6050->acc_x_raw = (data[0] << 8) | data[1];
  mpu6050->acc_y_raw = (data[2] << 8) | data[3];
  mpu6050->acc_z_raw = (data[4] << 8) | data[5];
  mpu6050->gyro_x_raw = (data[8] << 8) | data[9];
  mpu6050->gyro_y_raw = (data[10] << 8) | data[11];
  mpu6050->gyro_z_raw = (data[12] << 8) | data[13];
}
/**
 * @brief Calculates the moving average of the given samples.
 *
 * @param new_sample The new sample to add to the moving average.
 * @param samples The array holding the past samples.
 * @param size The size of the samples array.
 * @return double The updated moving average.
 */
double moving_average(double new_sample, double *samples, int size)
{
  samples[sample_index % size] = new_sample;
  sample_index++;
  double sum = 0;
  for (int i = 0; i < size; i++)
  {
    sum += samples[i];
  }
  return sum / size;
}

/**
 * @brief Normalizes an angle to the range [0, 360] degrees.
 *
 * @param angle The angle to normalize.
 * @return float The normalized angle.
 */
float normalize_angle(float angle)
{
  // Use modulo to wrap the angle within 0 to 360
  angle = fmod(angle, 360.0);

  // Ensure positive angle
  if (angle < 0)
    angle += 360.0;

  return angle;
}

// Convert raw data to meaningful values
void MPU6050_DataConvert(Struct_MPU6050 *mpu6050)
{
  mpu6050->acc_x = (mpu6050->acc_x_raw - ax_offset) / LSB_Sensitivity_ACC;
  mpu6050->acc_y = (mpu6050->acc_y_raw - ay_offset) / LSB_Sensitivity_ACC;
  mpu6050->acc_z = (mpu6050->acc_z_raw - az_offset) / LSB_Sensitivity_ACC;

  mpu6050->gyro_x = (mpu6050->gyro_x_raw - gyroXcal) / LSB_Sensitivity_GYRO;
  mpu6050->gyro_y = (mpu6050->gyro_y_raw - gyroYcal) / LSB_Sensitivity_GYRO;
  mpu6050->gyro_z = (mpu6050->gyro_z_raw - gyroZcal) / LSB_Sensitivity_GYRO;
}

int MPU6050_DataReady(void)
{
  // old school way
  /*
  static uint8_t INT_STATE_FLAG = 0;
  static uint8_t DATA_RDY_INT_FLAG = 0;
  static uint8_t INT_PIN = 0;
  INT_PIN = LL_GPIO_IsInputPinSet(MPU6050_INT_PORT, MPU6050_INT_PIN);
  if(INT_PIN == 1)
  {
    MPU6050_Readbyte(MPU6050_INT_STATUS, &INT_STATE_FLAG); //flag cleared automatically within the sensor
    DATA_RDY_INT_FLAG = INT_STATE_FLAG & 0x01;
    if(DATA_RDY_INT_FLAG == 1)
    {
      INT_STATE_FLAG = 0; //flag clearing
      DATA_RDY_INT_FLAG = 0;
      INT_PIN = 0;
      return 1;
    }
  }
  return 0;
   */
  return HAL_GPIO_ReadPin(MPU6050_INT_PORT, MPU6050_INT_PIN);
}

// Calibration function
void calibrate_sensor(Struct_MPU6050 *data)
{
  const int samples = 1000;
  for (int i = 0; i < samples; i++)
  {
    MPU6050_Get6AxisRawData(data);
    ax_offset += data->acc_x_raw;
    ay_offset += data->acc_y_raw;
    az_offset += data->acc_z_raw;
    gyroXcal += data->gyro_x_raw;
    gyroYcal += data->gyro_y_raw;
    gyroZcal += data->gyro_z_raw;
    HAL_Delay(2); // Delay to stabilize readings
  }
  ax_offset /= samples;
  ay_offset /= samples;
  az_offset /= samples;
  gyroXcal /= samples;
  gyroYcal /= samples;
  gyroZcal /= samples;
}

void MPU6050_ProcessData(Struct_MPU6050 *mpu6050)
{
  MPU6050_Get6AxisRawData(mpu6050);
  MPU6050_DataConvert(mpu6050);
}

void Calculate_Accelration_Velocity_Displacement_Yaw(Struct_MPU6050 *data)
{
  uint32_t currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0f; // Convert to seconds

  // Struct_MPU6050 data;
  // mpu6050_readData(&data);
  float ax = data->acc_x - ax_offset;

  Accelration = moving_average(ax, ax_samples, WINDOW_SIZE);
  Velocity = fabs(Accelration) * elapsedTime;
  Displacement = Velocity * elapsedTime;

  if (total_displacement == 15)
  {
    total_displacement = 0;
    MovedCell = 1;
  }
  else
  {
    MovedCell = 0;
    total_displacement += Displacement;
  }

  float gz = data->gyro_z - gyroZcal;
  yaw += gz * elapsedTime;
  yaw = normalize_angle(yaw);

  char buffer[100];
  sprintf(buffer, "A : %d V : %d D: %d\n", Accelration, Velocity, Displacement);
  UART_Print(buffer);

  previousTime = currentTime;
}
void I2C_Scan(void)
{
  printf("Scanning I2C bus...\n");
  for (uint8_t i = 1; i < 128; i++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i << 1), 1, 10) == HAL_OK)
    {
      printf("I2C device found at address 0x%X\n", i);
    }
  }
  printf("Scan complete.\n");
}
////////////////////////
/////////// I2C ////////
// Main program
int main(void)
{
  // Initialize the HAL Library
  HAL_Init();
  MX_TIM_Init();
  MX_TIM2_Init();

  // Configure the system clock
  SystemClock_Config();

  // Initialize GPIOs and UART
  MX_GPIO_Init();

  UART_Init();

  // MX_I2C2_Init();
  // HAL_I2C_MspInit(&hi2c2);
  // I2C_Scan();
  // MPU6050_Initialization();
  // calibrate_sensor(&MPU6050);
  // initializeMPU();

  // Main loop
  while (1)
  {
    {
      Movements(FORWARD, 255, 255);
    }
    // Motors_Test();
  }
}

// Initialize UART (USART2)
void UART_Init(void)
{
  // Enable UART2 Clock
  __HAL_RCC_USART2_CLK_ENABLE();

  // Initialize the UART peripheral (USART2)
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    // Initialization Error
    Error_Handler();
  }
}

// GPIO Initialization
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIO Clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Motor Control Pins on GPIOB
  GPIO_InitStruct.Pin = Motor_A_6_Pin | Motor_A_5_Pin | Motor_B_4_Pin | Motor_B_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // // Ultrasonic Trigger Pins on GPIOA
  GPIO_InitStruct.Pin = ltrigger_Pin | ftrigger_Pin | rtrigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Ultrasonic Echo Pins on GPIOA
  GPIO_InitStruct.Pin = lecho_Pin | fecho_Pin | recho_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // TX Pin (PA2) for USART2
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // RX Pin (PA3) for USART2
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // I2C2 SCL and SDA on PB10, PB11
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  __HAL_RCC_I2C2_CLK_ENABLE();
}

// Send string over UART
void UART_Print(const char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

// System Clock Configuration
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Initialize the CPU, AHB, and APB buses
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // Initialize CPU and bus clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

// Error handler
void Error_Handler(void)
{
  while (1)
  {
    // Stay here on error
  }
}

void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72 - 1; // Assuming 72 MHz system clock, this will give 1 MHz timer clock (1 us per count)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    // Initialization Error
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    // Initialization Error
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    // Initialization Error
    Error_Handler();
  }
}

// Initialize PWM for motors
static void MX_TIM_Init(void)
{
  TIM_HandleTypeDef htim1;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim1);

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

  // Start PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}