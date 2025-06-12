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
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

char choice;
char preset;
int steps;
uint8_t buffer[4];
uint8_t RPM[4];
uint8_t delay[4];
int j = 0;
int num=0;
int speed[4];
int time[4];
char run;



int currentSpeed = 1000;

uint8_t waveformVal;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sendNextionData(char s[])
{
    char cmd[100];  // Adjust size as needed
    uint8_t end_cmd[] = {0xFF, 0xFF, 0xFF};

    // Build the command string like: t0.txt="yourText"
    sprintf(cmd, "t0.txt=\"%s\"", s);

    // Send the command string
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 10);

    // Send the end command
    HAL_UART_Transmit(&huart1, end_cmd, 3, 10);
}

void sendWaveform(uint8_t waveform_id, uint8_t channel, uint8_t value)
{
    char cmd[30];
    uint8_t end_cmd[] = {0xFF, 0xFF, 0xFF};

    sprintf(cmd, "add %d,%d,%d", waveform_id, channel, value);
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 10);
    HAL_UART_Transmit(&huart1, end_cmd, 3, 10);
}


void ESC_SetThrottle(uint16_t pulse_us)
{
    if(pulse_us < 1000) pulse_us = 1000;
    if(pulse_us > 2000) pulse_us = 2000;

    if(pulse_us > currentSpeed)
    {
    	for(int i = currentSpeed; i<=pulse_us; i+=25)
    	{
    		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i);
    		waveformVal = (uint8_t)(((i - 1000) * 255) / 1000);
    		sendWaveform(2, 0, waveformVal);
    		HAL_Delay(25);
    	}
    }
    else
    {
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);
    	waveformVal = (uint8_t)(((pulse_us - 1000) * 255) / 1000);
    	sendWaveform(2, 0, waveformVal);
    	//sendWaveform(2, 1, pulse_us);
    }
    currentSpeed = pulse_us;
}

void flushUart1(void)
{
    uint8_t d;
    while (HAL_UART_Receive(&huart1, &d, 1, 1) == HAL_OK) { }
}

void transmitInt(int n)
{
	char msg[20];
	sprintf(msg, "%d\r\n", n);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  ESC_SetThrottle(1000);
  HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while(1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  while(1)
	  {
		  if(HAL_UART_Receive(&huart1, &choice, 1, HAL_MAX_DELAY) == HAL_OK)
		  {
			  if(choice == 'P')
			  {
				  HAL_UART_Transmit(&huart2, &choice, 1, 10);
				  flushUart1();
				  break;

			  }
			  else if(choice == 'N')
			  {
				  HAL_UART_Transmit(&huart2, &choice, 1, 10);
				  flushUart1();
				  break;
			  }
		  }
	  }

	  if(choice == 'N')
	  {
		  while(1)
		  {
			  if(HAL_UART_Receive(&huart1, buffer, 4, HAL_MAX_DELAY) == HAL_OK)
			  {
				  steps = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
				  flushUart1();
				  transmitInt(steps);
				  break;
			  }
		  }

		  int speeds[steps];
		  int times[steps];

		  for(int i=0; i<steps; i++)
		  {
			  j = 0;
			  while(j<4)
			  {
				  if(HAL_UART_Receive(&huart1, RPM, 4, HAL_MAX_DELAY) == HAL_OK)
				  {
					  int rec = RPM[0] | (RPM[1] << 8) | (RPM[2] << 16) | (RPM[3] << 24);
					  speed[j] = rec;
					  transmitInt(rec);
					  flushUart1();
					  j++;
				  }
			  }

			  num = 1000*speed[0]+100*speed[1]+10*speed[2]+speed[3];
			  transmitInt(num);
			  speeds[i] = num;
			  num=0;
			  j = 0;

			  while(j<4)
			  {
				  if(HAL_UART_Receive(&huart1, delay, 4, HAL_MAX_DELAY) == HAL_OK)
				  {
					  int rec = delay[0] | (delay[1] << 8) | (delay[2] << 16) | (delay[3] << 24);
					  time[j] = rec;
					  transmitInt(rec);
					  flushUart1();
					  j++;
				  }
			  }
			  num = 1000*time[0]+100*time[1]+10*time[2]+time[3];
			  transmitInt(num);
			  times[i] = num;
			  num=0;
		  }

		  while(1)
		  {
			  if(HAL_UART_Receive(&huart1, &run, 1, 500) == HAL_OK)
			  {
				  HAL_UART_Transmit(&huart2, &run, 1, 10);
				  flushUart1();
				  if(run == 'R')
				  {
					  for(int i = 0; i < steps; i++)
					  {
						  ESC_SetThrottle(speeds[i]);
						  waveformVal = (uint8_t)(((speeds[i] - 1000) * 255) / 1000);
						  uint32_t duration = times[i];
						  uint32_t interval = 50;
						  uint32_t count = duration / interval;

						  for (uint32_t j = 0; j < count; j++)
						  {
							  sendWaveform(2, 0, waveformVal);
							  HAL_Delay(interval);
						  }
					  }
					  ESC_SetThrottle(1000);
				  }
				  if(run == 'T')
				  {
					  break;
				  }
			  }
		  }
		  currentSpeed = 1000;
		  ESC_SetThrottle(1000);
		  flushUart1();
	  }
	  else if(choice == 'P')
	  {
		  while(1)
		  {
			  if(HAL_UART_Receive(&huart1, &preset, 1, 500) == HAL_OK)
			  {
				  switch(preset)
				  {
					case '1': break;
					case '2': break;
					case '3': break;
					case '4': break;
					case '5': break;
					case '6': break;
					case '7': break;
					case '8': break;
					case '9': break;
					case '10': break;
				  }
			  }
		  }
	  }
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
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
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
