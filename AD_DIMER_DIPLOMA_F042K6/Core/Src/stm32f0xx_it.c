/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t      dimPins[DIM_AMOUNT]  = {DIM_CH1_Pin, DIM_CH2_Pin, DIM_CH3_Pin, DIM_CH4_Pin,
								      DIM_CH5_Pin, DIM_CH6_Pin, DIM_CH7_Pin, DIM_CH8_Pin};
GPIO_TypeDef* dimPorts[DIM_AMOUNT] = {DIM_CH1_GPIO_Port, DIM_CH2_GPIO_Port, DIM_CH3_GPIO_Port, DIM_CH4_GPIO_Port,
	                                  DIM_CH5_GPIO_Port, DIM_CH6_GPIO_Port, DIM_CH7_GPIO_Port, DIM_CH8_GPIO_Port};
uint16_t transmitBuffer[BUFFER_SIZE];
uint16_t adc_value = 0;

struct Dimmer myDimmer[DIM_AMOUNT];

//uint8_t receiveBuffer[BUFFER_SIZE];
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */


	for(uint8_t i = 0; i < DIM_AMOUNT; i++)
	{
//--------------------FIRING_ANGLE---------------------------------------------
		if(myDimmer[i].mode == FIRING_ANGLE_MODE)
		{
			if(myDimmer[i].dim_val == MAX_DIM_VAL)
			{
				myDimmer[i].triac_status_flag = ON;
				HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_SET);
			}
			else if(myDimmer[i].dim_val == MIN_DIM_VAL)
			{
				myDimmer[i].triac_status_flag = OFF;
				HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_RESET);
				myDimmer[i].triac_status_flag = OFF;
				myDimmer[i].dim_tim_count    = 0;
			}
		}
//--------------------ZERO_CROS------------------------------------------------
		else if(myDimmer[i].mode == ZERO_CROS_MODE)
		{
			if(myDimmer[i].dim_val == MAX_DIM_VAL)
			{
				HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_SET);
				myDimmer[i].walves_counter = 0;
			}
			else if(myDimmer[i].dim_val == MIN_DIM_VAL)
			{
				HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_RESET);
				myDimmer[i].walves_counter = 0;
			}
			else
			{
				myDimmer[i].allowed_walves = myDimmer[i].dim_val * WALVES_COUFICIENT;

				if(myDimmer[i].walves_counter < myDimmer[i].allowed_walves)
				{
					HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_RESET);
				}
				myDimmer[i].walves_counter++;
				if(myDimmer[i].walves_counter == MAX_WALVES)
				{
					myDimmer[i].walves_counter = 0;
				}
			}
		}
	}

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ZERO_Pin);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles ADC interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
//---------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//---------------------------------------------------------
{
	if(htim->Instance == TIM14)
	{
		for(uint8_t i = 0; i < DIM_AMOUNT; i++)
		{
			if(myDimmer[i].curr_measur_status == ON)
			{
				HAL_ADC_Start(&hadc);
				HAL_ADC_PollForConversion(&hadc, 100);
				adc_value = HAL_ADC_GetValue(&hadc);
				HAL_ADC_Stop(&hadc);
				transmitBuffer[0] = adc_value;
				HAL_UART_Transmit_IT(&huart2, transmitBuffer, BUFFER_SIZE);
			}
			else
			{
				if((myDimmer[i].mode == FIRING_ANGLE_MODE) &&
				   (myDimmer[i].dim_val != MIN_DIM_VAL))
				{
					if(myDimmer[i].triac_status_flag == OFF)
					{
						myDimmer[i].dim_tim_count++;
					}
					if(HAL_GPIO_ReadPin(dimPorts[i], dimPins[i]) == OFF)
					{
						if(myDimmer[i].dim_tim_count == (MAX_DIM_VAL - myDimmer[i].dim_val))
						{
							HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_SET);
							myDimmer[i].triac_status_flag = ON;
						}
					}
				}
			}
		}
	}
	if(htim->Instance == TIM16)
	{
		HAL_GPIO_TogglePin(ST_LED_GPIO_Port, ST_LED_Pin);
	}
}
//---------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//---------------------------------------------------------
{
	if (huart->Instance == USART2)
	{
		// USART2 finished receive data
		HAL_UART_Receive_IT(&huart2, receiveBuffer, BUFFER_SIZE);

		for(uint8_t i = 0; i<DIM_AMOUNT; i++)
		{
			myDimmer[i].curr_measur_status    = (receiveBuffer[CURR_BYTE] & (0b00000001) << i) >> i;
			myDimmer[i].mode                  = (receiveBuffer[MODE_BYTE] & (0b00000001) << i) >> i;
			myDimmer[i].dim_val               =  receiveBuffer[i];
		}
	}
}
//---------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//---------------------------------------------------------
{
	if (huart->Instance == USART2)
	{
		// USART2 finished transmit data
	}
}
/* USER CODE END 1 */
