/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stm32g4xx_hal.h"
#include <stdint.h>
/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */
/* DLC sizes */
#define SHUTDOWN_DLC 			    ((uint8_t)0)
#define STATUS_REPORT_DLC 		((uint8_t)3)
#define ERROR_DLC 				    ((uint8_t)1)
#define THROTTLE_DLC 		      ((uint8_t)1)


#define DEFAULT_PRIORITY 		  ((uint8_t)0xFF)	// Default priority for CAN messages

/* Enum for Module field for custom CAN packet (inside of the 11b ID) */
typedef enum {
	saftey_system = 0,
	broadcast = 1,
	throttle_control_board = 2,
	pedal_box = 3,
	steering_wheel = 4,
	thermo_control_board = 5

} Module;

typedef enum {
	to = 0, from = 1
} Direction;

typedef enum {
	shutdown = 0,
	status_report = 1,
	send_error = 2,
	throttle_percentage = 3
	// calibrate = 4
} Command;

typedef struct{
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t data[8];
}CANMessage;

typedef enum
{
  ok                  = 0x00U,
  generic             = 0x01U,
  mismatch_dlc        = 0x02U,
  invalid_command     = 0x03U,
  fdcan_init_failure  = 0x04U,
  fdcan_rx_failure    = 0x05U,
  fdcan_tx_failure    = 0x06U,
  adc_failure         = 0x07U,
  dac_init_failure = 0x08U,
}Error;
/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef fdcanWrite(FDCAN_HandleTypeDef* hfdcan, FDCAN_TxHeaderTypeDef txHeader, uint8_t* txData, uint8_t len, Module module, Direction direction, uint8_t priority, Command command);
HAL_StatusTypeDef fdcanInit(FDCAN_HandleTypeDef* hfdcan);
HAL_StatusTypeDef fdcanFilterInit(FDCAN_HandleTypeDef* hfdcan, FDCAN_TxHeaderTypeDef* txHeader);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

