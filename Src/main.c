/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "global.h"
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  initializePeripherals();
  initializeGlobalVariables();
  initializeTimers();
  HAL_UART_Receive_IT(&huart1, rx_data, 1); // IS

  while (1)
  {
    // printTicker();
    UART_MODULE();    // reads string from user and sets values into rc
    DIST_MODULE();    // reads parameters and sets desired distance values
    NAVCOMP_MODULE(); // reads mode and parameters and instructs motor to drive.
    MOTOR_MODULE();   // reads instructions from navcomp and drives.
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // EM Complete Callback
{
  HAL_UART_Receive_IT(&huart1, rx_data, 1);
  if (strlen(data_buffer) == 0) // IS
  {
    strncpy(data_buffer, (char *)rx_data, sizeof rx_data); // IS
  }
  else
  {    
    strcat(data_buffer, (char *)rx_data); // IS
  }
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief NVIC Configuration, for Interrupts
 * @retval None
 */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief I2C2 Initialization Function, Inter integrated circuit controller
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
  hi2c2.Init.Timing = 0x2000090E;
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
 * @brief SPI2 Initialization Function, serial peripheral interface, for communication
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = SystemCoreClock / FREQUENCY_GOAL - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin | EXT_RESET_Pin | IN1_R_Pin | IN2_R_Pin | IN3_R_Pin | IN4_R_Pin | GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2 | IN1_L_Pin | IN2_L_Pin | IN3_L_Pin | IN4_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin IN1_R_Pin IN2_R_Pin
                           IN3_R_Pin IN4_R_Pin PC10 */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin | EXT_RESET_Pin | IN1_R_Pin | IN2_R_Pin | IN3_R_Pin | IN4_R_Pin | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 IN1_L_Pin IN2_L_Pin IN3_L_Pin
                           IN4_L_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | IN1_L_Pin | IN2_L_Pin | IN3_L_Pin | IN4_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  DIST_MODULE_MEASURE(GPIO_Pin);
}
/* USER CODE END 4 */
void initializePeripherals()
{
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();
  SystemClock_Config();

  // Initializing all configured peripheral functions.
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  // MX_TSC_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();

  // Initialize interrupts
  MX_NVIC_Init();
}
void initializeGlobalVariables()
{

  // Transmission variables:
  // char uart_buf[50]; // buffer

  // commandos
  comm = 0;
  // output for sprintf function
  messDisR = 0;

  // time stamps for US sensor distance calculation
  t0_X = 0;
  t1_X = 0;
  t0_L = 0;
  t1_L = 0;
  t0_R = 0;
  t1_R = 0;
  type_GPIO_L = GPIOB;
  IN1_L = GPIO_PIN_8;
  IN2_L = GPIO_PIN_5;
  IN3_L = GPIO_PIN_4;
  IN4_L = GPIO_PIN_3;
  // Pins for the right motor
  type_GPIO_R = GPIOC;
  IN1_R = GPIO_PIN_6;
  IN2_R = GPIO_PIN_7;
  IN3_R = GPIO_PIN_8;
  IN4_R = GPIO_PIN_9;
  // Iteration Steps
  indexOfRightMotorStep = 0;
  indexOfLeftMotorStep = 0;
  indexOfLeftMotorStepTotal = indexOfLeftMotorStep;
  indexOfRightMotorStepTotal = indexOfRightMotorStep;

  totalDistanceInMillimeters = 0;
  requiredYaw = 0;
  ADJUST_SCALER = 0.9;

  Diff_L = 0;
  Diff_R = 0;
  dis_L = 0.0;
  Diff_X = 0; // intermediate variable for calculations
  dis_X = 0.0;
  dis_R = 0.0;

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      latestTwoDistanceMeasurements[i][j] = 0.0;
    }
  }

  Is_First_Captured_X = 0;
  Is_First_Captured_L = 0;
  Is_First_Captured_R = 0;
  for (uint8_t i = 0; i < 20; i++)
  {
    for (size_t j = 0; j < 3; j++)
    {
      distanceMatrix[i][j] = 0;
    }
  }
  currentYaw = 0.0;

  for (uint8_t i = 0; i < 15; i++)
  {
    data_buffer[i] = 0;
    data_buffer_temp[i] = 0;
  }

  commandCount = 0;
  tempCommandCount = 0;
  receivedFullCommand = 0;
  stepCompleted = 0;
  taskCompleted = 0;

  // labyrith binary matrix

  labyrinth[0][0] = 0b1001; // x = 0, y = 0, wall = north, TRUE; east, FALSE; south, FALSE; west, TRUE., stored as 5
  labyrinth[1][0] = 0b1100; // x = 1, y = 0, wall = north, TRUE; east, TRUE; south, FALSE; west, FALSE.
  labyrinth[2][0] = 0b1001; // x = 2, y = 0
  labyrinth[3][0] = 0b1100;
  labyrinth[4][0] = 0b1011;
  labyrinth[5][0] = 0b1000;
  labyrinth[6][0] = 0b1100;

  labyrinth[0][1] = 0b0111; // x = 0, y = 1
  labyrinth[1][1] = 0b0001;
  labyrinth[2][1] = 0b0110;
  labyrinth[3][1] = 0b0011;
  labyrinth[4][1] = 0b1100;
  labyrinth[5][1] = 0b0101;
  labyrinth[6][1] = 0b0101;

  labyrinth[0][2] = 0b1011; // x = 0, y = 2
  labyrinth[1][2] = 0b0010;
  labyrinth[2][2] = 0b1000;
  labyrinth[3][2] = 0b1110;
  labyrinth[4][2] = 0b0101;
  labyrinth[5][2] = 0b0111;
  labyrinth[6][2] = 0b0101;

  labyrinth[0][3] = 0b1101; // x = 0, y = 3
  labyrinth[1][3] = 0b1101;
  labyrinth[2][3] = 0b0101;
  labyrinth[3][3] = 0b1101;
  labyrinth[4][3] = 0b0011;
  labyrinth[5][3] = 0b1010;
  labyrinth[6][3] = 0b0100;

  labyrinth[0][4] = 0b0101; // x = 0, y = 4
  labyrinth[1][4] = 0b0101;
  labyrinth[2][4] = 0b0011;
  labyrinth[3][4] = 0b0110;
  labyrinth[4][4] = 0b1011;
  labyrinth[5][4] = 0b1000;
  labyrinth[6][4] = 0b0100;

  labyrinth[0][5] = 0b0101; // x = 0, y = 5
  labyrinth[1][5] = 0b0011;
  labyrinth[2][5] = 0b1000;
  labyrinth[3][5] = 0b1000;
  labyrinth[4][5] = 0b1010;
  labyrinth[5][5] = 0b0110;
  labyrinth[6][5] = 0b0111;

  labyrinth[0][6] = 0b0011; // x = 0, y = 6
  labyrinth[1][6] = 0b1010;
  labyrinth[2][6] = 0b0110;
  labyrinth[3][6] = 0b0011;
  labyrinth[4][6] = 0b1010;
  labyrinth[5][6] = 0b1010;
  labyrinth[6][6] = 0b1110;
  pathToCenterFrom00 = "12212210x";
  pathToCenterFrom06 = "222330030323212210x";
  pathToCenterFrom60 = "110111010330030323212210x";
  pathToCenterFrom66 = "333011010330030323212210x";

  pathToFollow = (char *)malloc(100);
}
void initializeTimers()
{
  // set required frequency for general-purpose timer
  // EM set timer configurations

  // EM Timer to activate stepper motor:
  __HAL_TIM_SET_PRESCALER(&htim3, SystemCoreClock / FREQUENCY_GOAL - 1);
   tick = 1500; // 1.5 ms
  __HAL_TIM_SET_AUTORELOAD(&htim3, tick - 1); // custom time between motor steps (speed adjustment)

  // EM Timer to calculate distance from echo pins signals:
  __HAL_TIM_SET_PRESCALER(&htim1, SystemCoreClock / FREQUENCY_GOAL - 1);
  __HAL_TIM_SET_AUTORELOAD(&htim1, 65535);

  __HAL_TIM_SET_PRESCALER(&htim2, SystemCoreClock / FREQUENCY_GOAL - 1);
  __HAL_TIM_SET_AUTORELOAD(&htim2, 1); // 1 mus reload

  // mv sp/rt

  __HAL_TIM_SET_AUTORELOAD(&htim15, 1500 - 1); //
  __HAL_TIM_SET_PRESCALER(&htim15, SystemCoreClock / FREQUENCY_GOAL - 1);

  usPassed = 0;
  msPassed = 0;
  sPassed = 0;

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim1);
  // HAL_TIM_Base_Start(&htim2); // EM: simple timer without interrupts
  HAL_TIM_Base_Start(&htim2);
}
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
    printToHterm(sprintf(uart_buf, "HAL error."));
    HAL_Delay(5000);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
