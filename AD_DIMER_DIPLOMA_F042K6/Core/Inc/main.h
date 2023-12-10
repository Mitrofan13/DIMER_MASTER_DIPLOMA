/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIM_CH8_Pin GPIO_PIN_4
#define DIM_CH8_GPIO_Port GPIOA
#define DIM_CH7_Pin GPIO_PIN_5
#define DIM_CH7_GPIO_Port GPIOA
#define DIM_CH6_Pin GPIO_PIN_6
#define DIM_CH6_GPIO_Port GPIOA
#define DIM_CH5_Pin GPIO_PIN_7
#define DIM_CH5_GPIO_Port GPIOA
#define DIM_CH4_Pin GPIO_PIN_0
#define DIM_CH4_GPIO_Port GPIOB
#define DIM_CH3_Pin GPIO_PIN_1
#define DIM_CH3_GPIO_Port GPIOB
#define DIM_CH2_Pin GPIO_PIN_8
#define DIM_CH2_GPIO_Port GPIOA
#define DIM_CH1_Pin GPIO_PIN_9
#define DIM_CH1_GPIO_Port GPIOA
#define ST_LED_Pin GPIO_PIN_3
#define ST_LED_GPIO_Port GPIOB
#define ZERO_Pin GPIO_PIN_4
#define ZERO_GPIO_Port GPIOB
#define ZERO_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

#define BUFFER_SIZE       9
#define MAX_DIM_VAL       100
#define MIN_DIM_VAL       0
#define FIRING_ANGLE_MODE 0
#define ZERO_CROS_MODE    1
#define MAX_WALVES        64
#define WALVES_COUFICIENT MAX_WALVES / 100
#define DIM_AMOUNT        8

struct Dimmer
{
    uint8_t mode;
    uint8_t dim_val;
};

extern struct Dimmer myDimmer[DIM_AMOUNT];

extern uint16_t      dimPins[DIM_AMOUNT];
extern GPIO_TypeDef* dimPorts[DIM_AMOUNT];
extern uint8_t       dim_tim_count[DIM_AMOUNT];
extern uint8_t       dim_status_flag[DIM_AMOUNT];
extern uint8_t       receiveBuffer[BUFFER_SIZE];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
