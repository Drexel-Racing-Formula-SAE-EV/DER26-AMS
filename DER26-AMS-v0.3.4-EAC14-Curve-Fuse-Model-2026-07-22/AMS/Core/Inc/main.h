/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_A_Pin GPIO_PIN_2
#define CS_A_GPIO_Port GPIOE
#define CS_B_Pin GPIO_PIN_4
#define CS_B_GPIO_Port GPIOE
#define C_SNS_L_Pin GPIO_PIN_0
#define C_SNS_L_GPIO_Port GPIOC
#define FAN_5_Pin GPIO_PIN_0
#define FAN_5_GPIO_Port GPIOA
#define FAN_6_Pin GPIO_PIN_1
#define FAN_6_GPIO_Port GPIOA
#define C_SNS_H_Pin GPIO_PIN_3
#define C_SNS_H_GPIO_Port GPIOA
#define IMD_PWM_Pin GPIO_PIN_5
#define IMD_PWM_GPIO_Port GPIOA
#define FAN_1_Pin GPIO_PIN_7
#define FAN_1_GPIO_Port GPIOA
#define IMD_STAT_Pin GPIO_PIN_5
#define IMD_STAT_GPIO_Port GPIOC
#define FAN_2_Pin GPIO_PIN_1
#define FAN_2_GPIO_Port GPIOB
#define AIR_CTRL_Pin GPIO_PIN_11
#define AIR_CTRL_GPIO_Port GPIOF
#define TSAL_Pin GPIO_PIN_13
#define TSAL_GPIO_Port GPIOE
#define FAN_3_Pin GPIO_PIN_14
#define FAN_3_GPIO_Port GPIOD
#define FAN_4_Pin GPIO_PIN_15
#define FAN_4_GPIO_Port GPIOD
#define BMS_OK_Pin GPIO_PIN_0
#define BMS_OK_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
