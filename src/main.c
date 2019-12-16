/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */

#define DEBUG
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <inttypes.h>
#include <math.h>

#ifdef DEBUG
#include <string.h>
#include <stdio.h>
#endif

/* Private defines -----------------------------------------------------------*/
#define MPU6050_ADDR            (0x68 << 1)
#define TMP_REG                 0x41
#define MPU6050_ACCEL_XOUT_H		0x3B

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

static int16_t acx, acy, acz, tmp, gyx, gyy, gyz;
static int16_t offset_gyx = 0, offset_gyy = 0, offset_gyz = 0;
static float degrees_pitch_acc, degrees_roll_acc;
static float acc_vector;
static float degrees_pitch = 0, degrees_roll = 0;
static uint32_t _millis = 0;

static uint8_t serialBuffer[80];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

void MPU6050_Init(void);
void calibirate_MPU6050(int num_iterations);
void read_MPU6050(void);

/* Main function -------------------------------------------------------------*/
int main(void)
{
  /* System init */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* Initialize MPU6050 */
  MPU6050_Init();
  calibirate_MPU6050(3000);

  _millis = HAL_GetTick();
  while (1) {
    read_MPU6050();
    gyx -= offset_gyx;
    gyy -= offset_gyy;
    gyz -= offset_gyz;

    degrees_pitch += gyy * 0.0000610687;
    degrees_roll  += gyx * 0.0000610687;

    degrees_pitch += degrees_roll * sin(gyz * 0.000001066);
    degrees_roll  -= degrees_pitch * sin(gyz * 0.000001066);
    
    acc_vector = sqrt((acx * acx) + (acy * acy) + (acz * acz));
    degrees_pitch_acc = asin((float) acy/acc_vector) * 57.2957795;
    degrees_roll_acc  = asin((float) acx/acc_vector) * -57.2957795;
    
    degrees_pitch = degrees_pitch * 0.97 + degrees_pitch_acc * 0.03;
    degrees_roll  = degrees_roll * 0.97 + degrees_roll_acc * 0.03;

    float temp = (tmp / 340.00 + 36.53) * 100;
    float formatted_pitch = degrees_pitch * 100;
    float formatted_roll = degrees_roll * 100;

    sprintf(
      (char *)serialBuffer, "Pitch: %d.%02u, Roll: %d.%02u, Temp: %d.%02u\r\n",
      (int)formatted_pitch / 100, 
      (unsigned int)formatted_pitch % 100,
      (int)formatted_roll / 100, 
      (unsigned int)formatted_roll % 100,
      (int)temp / 100, 
      (unsigned int)temp % 100
    );

    HAL_UART_Transmit(&huart1, serialBuffer, strlen((char *)serialBuffer), 10);
    while ((HAL_GetTick() - _millis) < 4);
  }
}

/* MPU6050 configuration -----------------------------------------------------*/
void MPU6050_Init(void) {
  uint8_t PWR_MGMT_1[2] = {0x6B, 0x00};
  while (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 2, 10) != HAL_OK);

  uint8_t GYR_CONFIG[2] = {0x1B, 0x08};
  while(HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, GYR_CONFIG, 2, 10) != HAL_OK);

  uint8_t ACC_CONFIG[2] = {0x1C, 0x10};
  while(HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, ACC_CONFIG, 2, 10) != HAL_OK);

  uint8_t LPF_CONFIG[2] = {0x1A, 0x03};
  while(HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, LPF_CONFIG, 2, 10) != HAL_OK);
}

/* MPU6050 read data function ------------------------------------------------*/
void read_MPU6050(void) {
  uint8_t data[14];
  uint8_t reg = MPU6050_ACCEL_XOUT_H;

  while(HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 1000) != HAL_OK);
  while(HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, data, 14, 1000) != HAL_OK);

  /* Formatted accelerometer data */
  acx = (int16_t)(data[0] << 8 | data[1]);
  acy = (int16_t)(data[2] << 8 | data[3]);
  acz = (int16_t)(data[4] << 8 | data[5]);

  /* Formatted temperature */
  tmp = (data[6] << 8 | data[7]);

  /* Formatted gyroscope data */
  gyx = (int16_t)(data[8] << 8 | data[9]);
  gyy = (int16_t)(data[10] << 8 | data[11]);
  gyz = (int16_t)(data[12] << 8 | data[13]);
}

/* MPU6050 calibrate function ------------------------------------------------*/
void calibirate_MPU6050(int num_iterations) {
  strcpy((char *)serialBuffer, "Calibrating MPU6050..\r\n");
  HAL_UART_Transmit(&huart1, serialBuffer, strlen((char *)serialBuffer), 10);
  for (int i = 0; i < num_iterations; i++) {
    read_MPU6050();
    offset_gyx += gyx;
    offset_gyy += gyy;
    offset_gyz += gyz;
  }

  offset_gyx /= num_iterations;
  offset_gyy /= num_iterations;
  offset_gyz /= num_iterations;

  strcpy((char *)serialBuffer, "Done calibrating!\r\n");
  HAL_UART_Transmit(&huart1, serialBuffer, strlen((char *)serialBuffer), 10);
}

/* System configuration ------------------------------------------------------*/
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
