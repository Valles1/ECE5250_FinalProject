/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "UltrasonicSensor.h"
#include "SendDistanceI2C.h"
#include "Motor_GPIO_Init.h"
#include "Motor_Control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADXL345_I2C_ADDR        (0x53 << 1)  // 7-bit address 0x53, shifted for HAL
#define ADXL345_REG_DEVID       0x00
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0      0x32   // X0, X1, Y0, Y1, Z0, Z1
#define ADXL345_DEVID_EXPECTED  0xE5
#define ADXL345_LSB_PER_G       256.0f // in full-res, ±2g mode
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C4_Init(void);
/* USER CODE BEGIN PFP */
void LED_Init(void);
HAL_StatusTypeDef ADXL345_Init(void);
HAL_StatusTypeDef ADXL345_ReadRaw(int16_t *x, int16_t *y, int16_t *z);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void SendDistances_I2C(uint8_t ID,uint16_t distance)
{
    uint8_t buf[3];
    buf[0] = ID;
    buf[1] = (distance >> 8) & 0xFF;
    buf[2] = distance & 0xFF;

    HAL_I2C_Master_Transmit(&hi2c2, 0x42 << 1, buf, 3, HAL_MAX_DELAY);
}

void MotorControl_Init(void) {
	// 1) Set direction: IN1=1, IN2=0
	 GPIOB -> BSRR |= GPIO_PIN_2;    // IN1
	 GPIOB -> BRR = GPIO_PIN_6;  // IN2
	 // 2) Start PWM on TIM1 CH1
	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	 // 3) Set duty cycle (e.g., 40%)


	 // Start PWM CH2
	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	 // Direction example: forward (IN3=1, IN4=0)
	 GPIOB -> BSRR |= GPIO_PIN_10;    // IN3 (right)
	 GPIOB -> BRR = GPIO_PIN_11;  // IN4 (right)

	 //Start PWN CH3
	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	 //Set direction: IN1=1, IN2=0
	 GPIOD -> BSRR |= GPIO_PIN_4;
	 GPIOD -> BRR = GPIO_PIN_5;

	 //Start PWN CH4
	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	 //Set direction: IN3=1, IN4=0
	 GPIOD -> BSRR |= GPIO_PIN_6;
	 GPIOD -> BRR = GPIO_PIN_3;

}

void Reverse_MotorControl(void) {
	//Set direction: IN1=1, IN2=0
	 GPIOB -> BRR = GPIO_PIN_2;    // IN1
	 GPIOB -> BSRR |= GPIO_PIN_6;  // IN2

	 GPIOB -> BSRR |= GPIO_PIN_10;    // IN3
	 GPIOB -> BRR = GPIO_PIN_11;  // IN4

	 //Set direction: IN1=1, IN2=0
	 GPIOD -> BSRR |= GPIO_PIN_4;    // IN1
	 GPIOD -> BRR = GPIO_PIN_5;  // IN2

	 GPIOD -> BRR = GPIO_PIN_6;    // IN3
	 GPIOD -> BSRR |= GPIO_PIN_3;  // IN4
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

	 uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
	 // 50% duty
	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ((arr + 1) * 50) / 100);
	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ((arr + 1) * 50) / 100);
	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ((arr + 1) * 95) / 100);
	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ((arr + 1) * 95) / 100);

}

void Forward_MotorControl(void) {
	//Set direction: IN1=0, IN2=1
	 GPIOB -> BSRR |= GPIO_PIN_2;    // IN1
	 GPIOB -> BRR = GPIO_PIN_6;  // IN2

	 GPIOB -> BRR = GPIO_PIN_10;    // IN3
	 GPIOB -> BSRR |= GPIO_PIN_11;  // IN4

	 //Set direction: IN1=0, IN2=1
	 GPIOD -> BRR = GPIO_PIN_4;    // IN1
	 GPIOD -> BSRR |= GPIO_PIN_5;  // IN2

	 GPIOD -> BSRR |= GPIO_PIN_6;    // IN3
	 GPIOD -> BRR = GPIO_PIN_3;  // IN4

	 uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
	 // 50% duty
	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ((arr + 1) * 50) / 100);

	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ((arr + 1) * 50) / 100);

	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ((arr + 1) * 95) / 100);

	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ((arr + 1) * 95) / 100);


}

void Right_MotorControl(void) {
	//Set direction: IN1=0, IN2=1
	 GPIOB -> BRR = GPIO_PIN_2;    // IN1
	 GPIOB -> BSRR |= GPIO_PIN_6;  // IN2

	 GPIOB -> BSRR |= GPIO_PIN_10;    // IN3
	 GPIOB -> BRR = GPIO_PIN_11;  // IN4

	 //Set direction: IN1=0, IN2=1
	 GPIOD -> BRR = GPIO_PIN_4;    // IN1
	 GPIOD -> BSRR |= GPIO_PIN_5;  // IN2

	 GPIOD -> BSRR |= GPIO_PIN_6;    // IN3
	 GPIOD -> BRR = GPIO_PIN_3;  // IN4

	 uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
	 // 50% duty
	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ((arr + 1) * 75) / 100);

	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ((arr + 1) * 75) / 100);

	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ((arr + 1) * 100) / 100);

	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ((arr + 1) * 100) / 100);


}

void Left_MotorControl(void) {
	//Set direction: IN1=0, IN2=1
	 GPIOB -> BSRR |= GPIO_PIN_2;    // IN1
	 GPIOB -> BRR = GPIO_PIN_6;  // IN2

	 GPIOB -> BRR = GPIO_PIN_10;    // IN3
	 GPIOB -> BSRR |= GPIO_PIN_11;  // IN4

	 //Set direction: IN1=0, IN2=1
	 GPIOD -> BRR = GPIO_PIN_4;    // IN1
	 GPIOD -> BRR = GPIO_PIN_5;  // IN2

	 GPIOD -> BRR = GPIO_PIN_6;    // IN3
	 GPIOD -> BRR = GPIO_PIN_3;  // IN4

	 uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
	 // 50% duty
	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ((arr + 1) * 75) / 100);

	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ((arr + 1) * 75) / 100);

	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ((arr + 1) * 80) / 100);

	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ((arr + 1) * 80) / 100);


}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Sensor_GPIO_Init();
  //LED_Init();
  //Motor_GPIO_Init();
  //Motor_GPIO_Init2();
  //Motor_GPIO_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SysTick_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_I2C4_Init();
  /* USER CODE BEGIN 2 */
  //Motor_GPIO_Init();
  //Motor_GPIO_Init2();
  MotorControl_Init();
  ADXL345_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	  //receive commands from the Featherboard Web Page
	  	  uint8_t cmd;
	  	  HAL_I2C_Master_Receive(&hi2c2, 0x42 << 1, &cmd, 1, HAL_MAX_DELAY);
	  	  delay_us( 1000 );

	  	  //send the rear and front distances to the Featherboard
	  	  float rear_distance = get_distance_cm(GPIO_PIN_3, GPIO_PIN_1);
	  	  uint16_t rear_dist_int = (uint16_t)rear_distance;
	  	  SendDistances_I2C(0x02, rear_dist_int);
	  	  delay_us( 1000 );

	  	  float front_distance = get_distance_cm(GPIO_PIN_2, GPIO_PIN_0);
	  	  uint16_t front_dist_int = (uint16_t)front_distance;
	  	  SendDistances_I2C(0x01, front_dist_int);

	  	  //code for the accelerometer, including I2C communication of these values
	  	  int16_t x_raw, y_raw, z_raw;

	  	  if (ADXL345_ReadRaw(&x_raw, &y_raw, &z_raw) == HAL_OK)
	  		  {
	  		        // Convert raw to g
	  		        float x_g = (float)x_raw / ADXL345_LSB_PER_G;
	  		        float y_g = (float)y_raw / ADXL345_LSB_PER_G;
	  		        float z_g = (float)z_raw / ADXL345_LSB_PER_G;
	  		        float mag = sqrtf(x_g * x_g + y_g * y_g + z_g * z_g - 0.9);

	  		      //mg
	  		      uint16_t x_g_int = (uint16_t)(x_g * 1000.0f);
	  		      SendDistances_I2C(0x03, x_g_int);
	  		      delay_us( 1000 );
	  		      uint16_t y_g_int = (uint16_t)(y_g * 1000.0f);
	  		      SendDistances_I2C(0x04, y_g_int);
	  		      delay_us( 1000 );
	  		      uint16_t z_g_int = (uint16_t)(z_g * 1000.0f);
	  		      SendDistances_I2C(0x05, z_g_int);
	  		      delay_us( 1000 );
	  		      uint16_t mag_int = (uint16_t)(mag * 1000.0f);
	  		      SendDistances_I2C(0x06, mag_int);

	  		  }

	  	  //switch case statement for the motor controls
	  	  switch (cmd) {
	  	    case 'F':
	  	        // Forward

	  	        Forward_MotorControl();
	  	        break;

	  	    case 'B':
	  	        // Reverse
	  	    	Reverse_MotorControl();

	  	        break;

	  	    case 'L':
	  	        // Left
	  	    	Left_MotorControl();

	  	        break;

	  	    case 'R':
	  	        // Right
	  	    	Right_MotorControl();

	  	        break;

	  	    default: // 'S'
	  	        // Stop
	  	        Stop_MotorControl();
	  	        break;
	  	}
	  	delay_us( 250000 );	//delay by 250 ms

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000508;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00000508;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1066;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|LD3_Pin
                          |GPIO_PIN_6|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 LD3_Pin
                           PB6 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|LD3_Pin
                          |GPIO_PIN_6|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static HAL_StatusTypeDef ADXL345_WriteReg(uint8_t reg, uint8_t value)
{
	return HAL_I2C_Mem_Write(&hi2c4,
                          ADXL345_I2C_ADDR,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          &value,
                          1,
                          HAL_MAX_DELAY);
}
static HAL_StatusTypeDef ADXL345_ReadMulti(uint8_t reg, uint8_t *pData, uint16_t len)
{
	return HAL_I2C_Mem_Read(&hi2c4,
                         ADXL345_I2C_ADDR,
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         pData,
                         len,
                         HAL_MAX_DELAY);
}
HAL_StatusTypeDef ADXL345_Init(void)
{
	uint8_t devid = 0;
	// Read device ID
	if (ADXL345_ReadMulti(ADXL345_REG_DEVID, &devid, 1) != HAL_OK)
		return HAL_ERROR;
	if (devid != ADXL345_DEVID_EXPECTED)
		return HAL_ERROR;
	// Power control: measurement mode (set MEASURE bit)
	if (ADXL345_WriteReg(ADXL345_REG_POWER_CTL, 0x08) != HAL_OK)
		return HAL_ERROR;
	// Data format: full resolution, ±2g
	if (ADXL345_WriteReg(ADXL345_REG_DATA_FORMAT, 0x08) != HAL_OK)
		return HAL_ERROR;
	return HAL_OK;
}

HAL_StatusTypeDef ADXL345_ReadRaw(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t buf[6];
	if (ADXL345_ReadMulti(ADXL345_REG_DATAX0, buf, 6) != HAL_OK)
		return HAL_ERROR;
	*x = (int16_t)((buf[1] << 8) | buf[0]);
	*y = (int16_t)((buf[3] << 8) | buf[2]);
	*z = (int16_t)((buf[5] << 8) | buf[4]);
	return HAL_OK;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
