/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//define sound speed in cm/uS
int lsr = 0;

uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t DistanceLeft  = 0;  // cm
uint16_t DistanceStraight  = 0;  // cm
uint16_t DistanceRight  = 0;  // cm

int count = 0;

uint32_t counter_1 = 0;
uint16_t count_1 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write (int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
	return len;
}

void forward(void)
{
  HAL_GPIO_WritePin(motorLeft1_GPIO_Port, motorLeft1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motorLeft2_GPIO_Port, motorLeft2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motorRight1_GPIO_Port, motorRight1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motorRight2_GPIO_Port, motorRight2_Pin, GPIO_PIN_RESET);
}

void left(void)
{
	HAL_GPIO_WritePin(motorLeft1_GPIO_Port, motorLeft1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motorLeft2_GPIO_Port, motorLeft2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motorRight1_GPIO_Port, motorRight1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motorRight2_GPIO_Port, motorRight2_Pin, GPIO_PIN_SET);
}

void right(void)
{
	HAL_GPIO_WritePin(motorLeft1_GPIO_Port, motorLeft1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motorLeft2_GPIO_Port, motorLeft2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motorRight1_GPIO_Port, motorRight1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motorRight2_GPIO_Port, motorRight2_Pin, GPIO_PIN_RESET);
}

void turnAround(void)
{
	right();
	right();
}

void stop(void)
{
	HAL_GPIO_WritePin(motorLeft1_GPIO_Port, motorLeft1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motorLeft2_GPIO_Port, motorLeft2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motorRight1_GPIO_Port, motorRight1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motorRight2_GPIO_Port, motorRight2_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//  while(count < des_count)
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(trigLeft_GPIO_Port, trigLeft_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
  HAL_GPIO_WritePin(trigStraight_GPIO_Port, trigStraight_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
  HAL_GPIO_WritePin(trigRight_GPIO_Port, trigRight_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL); //timer 1 channel 1&2 encoder mode for motor A
  TIM3->CNT = 0;



// counter_1 = __HAL_TIM_GET_COUNTER(&htim3);
//
// count_1 = (int16_t)counter_1; // encoder 1 pulse counter
//
// printf("%d\r\n", count_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  counter_1 = __HAL_TIM_GET_COUNTER(&htim3);

	  count_1 = (int16_t)counter_1; // encoder 1 pulse counter

	  printf("%d\r\n", count_1);


//	 forward();
//	 lsr = 0;
//
//	 // ultrasonic
//
//	 HAL_GPIO_WritePin(trigLeft_GPIO_Port, trigLeft_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
//	 __HAL_TIM_SET_COUNTER(&htim1, 0);
//	 while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
//	 HAL_GPIO_WritePin(trigLeft_GPIO_Port, trigLeft_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
//	 pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
//	 // wait for the echo pin to go high
//	 while (!(HAL_GPIO_ReadPin (echoLeft_GPIO_Port, echoLeft_Pin)) && pMillis + 10 >  HAL_GetTick());
//	 Value1 = __HAL_TIM_GET_COUNTER (&htim1);
//
//	 pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
//	 // wait for the echo pin to go low
//	 while ((HAL_GPIO_ReadPin (echoLeft_GPIO_Port, echoLeft_Pin)) && pMillis + 50 > HAL_GetTick());
//	 Value2 = __HAL_TIM_GET_COUNTER (&htim1);
//
//	 DistanceLeft = (Value2-Value1)* 0.034/2;
//
//	 HAL_GPIO_WritePin(trigStraight_GPIO_Port, trigStraight_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
//	 __HAL_TIM_SET_COUNTER(&htim1, 0);
//	 while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
//	 HAL_GPIO_WritePin(trigStraight_GPIO_Port, trigStraight_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
//	 pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
//	 // wait for the echo pin to go high
//	 while (!(HAL_GPIO_ReadPin (echoStraight_GPIO_Port, echoStraight_Pin)) && pMillis + 10 >  HAL_GetTick());
//	 Value1 = __HAL_TIM_GET_COUNTER (&htim1);
//	 pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
//	 // wait for the echo pin to go low
//	 while ((HAL_GPIO_ReadPin (echoStraight_GPIO_Port, echoStraight_Pin)) && pMillis + 50 > HAL_GetTick());
//	 Value2 = __HAL_TIM_GET_COUNTER (&htim1);
//	 DistanceStraight = (Value2-Value1)* 0.034/2;
//
//	 HAL_GPIO_WritePin(trigRight_GPIO_Port, trigRight_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
//	 __HAL_TIM_SET_COUNTER(&htim1, 0);
//	 while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
//	 HAL_GPIO_WritePin(trigRight_GPIO_Port, trigRight_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
//	 pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
//	 // wait for the echo pin to go high
//	 while (!(HAL_GPIO_ReadPin (echoRight_GPIO_Port, echoRight_Pin)) && pMillis + 10 >  HAL_GetTick());
//	 Value1 = __HAL_TIM_GET_COUNTER (&htim1);
//	 pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
//	 // wait for the echo pin to go low
//	 while ((HAL_GPIO_ReadPin (echoRight_GPIO_Port, echoRight_Pin)) && pMillis + 50 > HAL_GetTick());
//	 Value2 = __HAL_TIM_GET_COUNTER (&htim1);
//	 DistanceRight = (Value2-Value1)* 0.034/2;
//
////	 printf("%d, %d, %d\n", DistanceLeft, DistanceStraight, DistanceRight);
//
//	 if (DistanceLeft < 8) {lsr = 1;}
//	 else {lsr = 0;}
//
//	 if (DistanceStraight < 8) {lsr = (lsr << 1) | 1;}
//	 else {lsr = lsr << 1;}
//
//	 if (DistanceRight < 8) {lsr = (lsr << 1) | 1;}
//	 else {lsr = lsr << 1;}
//
//	 // LSR logic
//	 switch (lsr)
//	 {
//		 case 0b000:
//			 stop();
//			 break;
//		 case 0b001:
//			 left();
//			 break;
//		 case 0b010:
//			 left();
//			 break;
//		 case 0b011:
//			 left();
//			 break;
//		 case 0b100:
//			 forward();
//			 break;
//		 case 0b101:
//			 forward();
//			 break;
//		 case 0b110:
//			 right();
//			 break;
//		 case 0b111:
//			 turnAround();
//			 break;
//		 default:
//			 forward();
//			 break;
//	 }

//	 HAL_Delay(500);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, motorRight2_Pin|motorRight1_Pin|motorLeft2_Pin|motorLeft1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, trigRight_Pin|trigStraight_Pin|trigLeft_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_Encoder_A_Pin Motor_Encoder_B_Pin */
  GPIO_InitStruct.Pin = Motor_Encoder_A_Pin|Motor_Encoder_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : motorRight2_Pin motorRight1_Pin motorLeft2_Pin motorLeft1_Pin */
  GPIO_InitStruct.Pin = motorRight2_Pin|motorRight1_Pin|motorLeft2_Pin|motorLeft1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : trigRight_Pin trigStraight_Pin trigLeft_Pin */
  GPIO_InitStruct.Pin = trigRight_Pin|trigStraight_Pin|trigLeft_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : echoRight_Pin echoStraight_Pin echoLeft_Pin */
  GPIO_InitStruct.Pin = echoRight_Pin|echoStraight_Pin|echoLeft_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)

{

//  if(!FACTORY_TEST){

  static uint8_t counter = 0;

  if(GPIO_Pin == Motor_Encoder_A_Pin)

  {

      if(counter == 0)

      {

         //snprintf(rx_buffer, MAX_UART_MSG, "name:calibrate_motors\n");

//         snprintf(rx_buffer, MAX_UART_MSG, "name:go_forward,distance:100, use_ramp:1\n");

         counter++;

      }

      else if(counter == 1)

      {

//         snprintf(rx_buffer, MAX_UART_MSG, "name:angle_control,angle:90\n");

         counter = 0;

      }

//      flagUart = 1;

  }
              // new value from right encoder, need to update distance covered

  else if (GPIO_Pin == Motor_Encoder_B_Pin)

//    update_distance_dx();              // new value from left encoder, need to update distance covered
  {
	  if(HAL_GPIO_ReadPin(Motor_Encoder_A_GPIO_Port, Motor_Encoder_A_Pin))

//	  updated_distance = get_distance(DX) + DISTANCE_PER_TICK;
		  count++;
	  printf("%d\r\n", count);

  }


  // new code

//  if((GPIO_Pin == A_SX_Pin) || (GPIO_Pin == A_DX_Pin))
//
//    update_mvt(GPIO_Pin);
//
//       }
//
//       else{


  // new code end

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
}
  /* USER CODE END Error_Handler_Debug */


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
