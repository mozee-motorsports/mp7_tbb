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
#include "stm32g4xx_hal.h"

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
#define H_IN1_Pin GPIO_PIN_0
#define H_IN1_GPIO_Port GPIOC
#define H_NSLEEP_Pin GPIO_PIN_1
#define H_NSLEEP_GPIO_Port GPIOC
#define TRIM_1_Pin GPIO_PIN_2
#define TRIM_1_GPIO_Port GPIOC
#define TRIM_2_Pin GPIO_PIN_3
#define TRIM_2_GPIO_Port GPIOC
#define TPS_POT1_Pin GPIO_PIN_0
#define TPS_POT1_GPIO_Port GPIOA
#define TPS_POT2_Pin GPIO_PIN_1
#define TPS_POT2_GPIO_Port GPIOA
#define ECU_OUT_Pin GPIO_PIN_4
#define ECU_OUT_GPIO_Port GPIOA
#define H_IN2_Pin GPIO_PIN_7
#define H_IN2_GPIO_Port GPIOA
#define H_NFAULT_Pin GPIO_PIN_5
#define H_NFAULT_GPIO_Port GPIOC
#define H_DECAY_Pin GPIO_PIN_0
#define H_DECAY_GPIO_Port GPIOB
#define H_OCPM_Pin GPIO_PIN_1
#define H_OCPM_GPIO_Port GPIOB
#define H_MODE2_Pin GPIO_PIN_2
#define H_MODE2_GPIO_Port GPIOB
#define H_IPROPI_Pin GPIO_PIN_11
#define H_IPROPI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define USING_PID2			// Using PID2 library

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
