/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define DIM_AMOUNT   8
//#define BUFFER_SIZE  9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t      dimPins[DIM_AMOUNT]  = {DIM_CH1_Pin, DIM_CH2_Pin, DIM_CH3_Pin, DIM_CH4_Pin,
								            DIM_CH5_Pin, DIM_CH6_Pin, DIM_CH7_Pin, DIM_CH8_Pin};
GPIO_TypeDef* dimPorts[DIM_AMOUNT] = {DIM_CH1_GPIO_Port, DIM_CH2_GPIO_Port, DIM_CH3_GPIO_Port, DIM_CH4_GPIO_Port,
	                                        DIM_CH5_GPIO_Port, DIM_CH6_GPIO_Port, DIM_CH7_GPIO_Port, DIM_CH8_GPIO_Port};

uint8_t transmitBuffer[BUFFER_SIZE];
uint8_t receiveBuffer[BUFFER_SIZE];


//struct Dimmer {
//    int mode;      // Режим дії (наприклад, 0 - автоматичний, 1 - ручний)
//    int dim_val;    // Значення диммера (наприклад, інтенсивність світла)
//};
//
struct Dimmer myDimmer[DIM_AMOUNT];
//const TIM_TypeDef*  TIMs[DIM_AMOUNT]     = {TIM1,TIM2,TIM3,TIM4,TIM5,TIM8,TIM9,TIM11};

//const TIM_HandleTypeDef* HandleTIMs[]    = {&htim1, &htim2, &htim3, &htim4, &htim5, &htim8, &htim9, &htim11};

//extern uint16_t adc_value = 0;

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
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, receiveBuffer, BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

//	for(uint8_t i = 0; i<DIM_AMOUNT; i++)
//	{
//		if(htim->Instance == TIMs[i])
//		{
////			if(i==0) HAL_TIM_Base_Stop_IT(&htim1);//  о�?танавливаем таймер
////			if(i==1) HAL_TIM_Base_Stop_IT(&htim2);
////			if(i==2) HAL_TIM_Base_Stop_IT(&htim3);
////			if(i==3) HAL_TIM_Base_Stop_IT(&htim4);
////			if(i==4) HAL_TIM_Base_Stop_IT(&htim5);
////			if(i==5) HAL_TIM_Base_Stop_IT(&htim8);
////			if(i==6) HAL_TIM_Base_Stop_IT(&htim9);
////			if(i==7) HAL_TIM_Base_Stop_IT(&htim11);
////
////			HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_SET);
//	        HAL_TIM_Base_Stop_IT(HandleTIMs[i]);  // О�?танній параметр автоматично визначаєть�?�? за допомогою індек�?у
//	        HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_SET);
//		}
//	}

	HAL_TIM_Base_Stop_IT(&htim14);
	HAL_GPIO_WritePin(DIM_CH1_GPIO_Port, DIM_CH1_Pin, GPIO_PIN_SET);
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    // USART1 завершил прием данных
	  HAL_UART_Receive_IT(&huart2, receiveBuffer, BUFFER_SIZE);

	for(uint8_t i = 0; i<DIM_AMOUNT; i++)
	{
//		if(dim_par.mode_flag[i] == FIRING_ANGLE_MODE)
//		{
//			dim_par.dimmer[i] = adc_value.adc_value * X_COEFICIENT + B_COEFICIENT;   // dimmer[1000:10000] adc_value[0:255] -35.12412
//		}
//		if(dim_par.mode_flag[i] == ZERO_CROSS_MODE)
//		{
//			dim_par.dimmer[i] = adc_value.adc_value; //dimmer[0:255] adc_value[0:255]
//		}

		myDimmer[i].mode    = (receiveBuffer[8] & (0b00000001)<<i)>>i;
		myDimmer[i].dim_val = receiveBuffer[i];
	}
  }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    // USART2 завершил отправку данных
  }
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
