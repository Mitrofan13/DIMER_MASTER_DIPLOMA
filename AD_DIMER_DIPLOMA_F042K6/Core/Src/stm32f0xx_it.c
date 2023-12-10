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
//#define MAX_DIM_VAL       100
//#define MIN_DIM_VAL       0
//#define FIRING_ANGLE_MODE 1
//#define ZERO_CROS_MODE    2
//#define DEFAULT_MODE      1
//#define MAX_WALVES        64
//#define WALVES_COUFICIENT MAX_WALVES / 100
//#define DIM_AMOUNT        8
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

uint8_t allowed_walves[DIM_AMOUNT];
uint8_t walves_counter[DIM_AMOUNT];

uint8_t dim_tim_count[DIM_AMOUNT];
uint8_t dim_status_flag[DIM_AMOUNT];

uint8_t transmitBuffer[BUFFER_SIZE];

struct Dimmer myDimmer[DIM_AMOUNT];

//uint8_t receiveBuffer[BUFFER_SIZE];
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim14;
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
				dim_status_flag[i] = 1;
				HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_SET);
			}
			else if(myDimmer[i].dim_val == MIN_DIM_VAL)
			{
				dim_status_flag[i] = 1;
				HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_RESET);
				dim_status_flag[i] = 0;
				dim_tim_count[i] = 0;
			}
		}
//--------------------ZERO_CROS------------------------------------------------
		else if(myDimmer[i].mode == ZERO_CROS_MODE)
		{
			if(myDimmer[i].dim_val == MAX_DIM_VAL)
			{
				HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_SET);
				walves_counter[i] = 0;
			}
			else if(myDimmer[i].dim_val == MIN_DIM_VAL)
			{
				HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_RESET);
				walves_counter[i] = 0;
			}
			else
			{
				allowed_walves[i] = myDimmer[i].dim_val * WALVES_COUFICIENT;

				if(walves_counter[i] < allowed_walves[i])
				{
					HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_RESET);
				}
				walves_counter[i]++;
				if(walves_counter[i] == MAX_WALVES)
				{
					walves_counter[i] = 0;
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
	for(uint8_t i = 0; i < DIM_AMOUNT; i++)
	{
		if(dim_status_flag[i] == 0)
		{
			dim_tim_count[i]++;
		}
        if(HAL_GPIO_ReadPin(dimPorts[i], dimPins[i]) == 0)
        {
            if(dim_tim_count[i] == (100 - myDimmer[i].dim_val))
            {
            	HAL_GPIO_WritePin(dimPorts[i], dimPins[i], GPIO_PIN_SET);
            	dim_status_flag[i] = 1;
            }
        }

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
			myDimmer[i].mode    = (receiveBuffer[8] & (0b00000001)<<i)>>i;
			myDimmer[i].dim_val = receiveBuffer[i];
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
