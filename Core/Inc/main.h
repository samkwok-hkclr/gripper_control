/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdlib.h>
#include <math.h>

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Left_Direction_Pin GPIO_PIN_1
#define Left_Direction_GPIO_Port GPIOA
#define Right_Direction_Pin GPIO_PIN_7
#define Right_Direction_GPIO_Port GPIOA
#define Left_Feedback_Pin GPIO_PIN_0
#define Left_Feedback_GPIO_Port GPIOB
#define Left_Feedback_EXTI_IRQn EXTI0_IRQn
#define Right_Feedback_Pin GPIO_PIN_1
#define Right_Feedback_GPIO_Port GPIOB
#define Right_Feedback_EXTI_IRQn EXTI1_IRQn
#define Forward_Right_Pin GPIO_PIN_12
#define Forward_Right_GPIO_Port GPIOB
#define Forward_Left_Pin GPIO_PIN_13
#define Forward_Left_GPIO_Port GPIOB
#define Through_Sensor_Pin GPIO_PIN_14
#define Through_Sensor_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

uint16_t calc_duty_cycle(const uint16_t x);
void set_duty_cycle(const uint16_t left_value, const uint16_t right_value);
void init_values();

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
