/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 14;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#define MAX_3BIT 0b111
#define MAX_4BIT 0b1111
#define MAX_FDCAN_RETRIES 3
HAL_StatusTypeDef fdcanWrite(FDCAN_HandleTypeDef* hfdcan, FDCAN_TxHeaderTypeDef txHeader, uint8_t* txData, uint8_t len,
		Module module, Direction direction, uint8_t priority, Command command)
{
	// Verify that the actual payload (txData) is not greater than 8 bytes (max for classic CAN)
	if(len > FDCAN_DLC_BYTES_8)
		return HAL_ERROR;

	// Verify that priority, module, and direction are not greater than 3 bits, and command isn't greater than 4 bits
	if(priority > MAX_3BIT || module > MAX_3BIT || direction > MAX_3BIT || command > MAX_4BIT)
		return HAL_ERROR;

	// Set DLC
	txHeader.DataLength = len;

	// Create 11b id
	txHeader.Identifier = ((uint16_t)priority << 8) | (uint8_t)(module << 5) | (uint8_t)(direction << 4) | (uint8_t)command;

	// Transmit message by putting it into 10 element TxFIFO queue
  int retry = 0;
  while (retry < MAX_FDCAN_RETRIES)
  {
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) == HAL_OK)
    {
      return HAL_OK; // Success
    }
    retry++;
    HAL_Delay(10); // Small delay before retrying
  }

  return HAL_ERROR;
}

HAL_StatusTypeDef fdcanInit(FDCAN_HandleTypeDef* hfdcan)
{
	// FDCAN 1 - Rx FIFO0
	if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
		return HAL_ERROR;

	// Triggers interrupt when new message appears in RX_FIFO0
	if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;

}

// CANFD macros
#define FILTER 	0b00001000000		// What to look for i.e. id (0b10) and direction (0b0) XXX_010_X_XXXX										// Steering Wheel ID
#define MASK 	0b00011110000		// 11b id. Look at Module bits [7..5] and direction (should be set to "to" = 0)


HAL_StatusTypeDef fdcanFilterInit(FDCAN_HandleTypeDef* hfdcan, FDCAN_TxHeaderTypeDef* txHeader)
{
	FDCAN_FilterTypeDef fdcan_filter_config;

	// 11b register with 11b id. We only care about bits [7..5]
	// FDCAN1 Mask Filter
	fdcan_filter_config.IdType = FDCAN_STANDARD_ID;						// Using standard IDs, not extended IDs
	fdcan_filter_config.FilterIndex = 0;								// We are only using 1 filter so index = 0
	fdcan_filter_config.FilterType = FDCAN_FILTER_MASK;					// Using mask filter
	fdcan_filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;			// Messages that pass through the filter should be sent to RX FIFO 0
	fdcan_filter_config.FilterID1 = FILTER;								// We only care about bits [7..5]
	fdcan_filter_config.FilterID2 = MASK;								// Mask bits
	if (HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter_config) != HAL_OK)
	  return HAL_ERROR; // Filter configuration Error

	// Configure TX Header for FDCAN1
//	txHeader.Identifier = SW_MODULE;									// Recall header format - changed dynamically
	txHeader->IdType = FDCAN_STANDARD_ID;								// Using standard IDs, not extended IDs
	txHeader->TxFrameType = FDCAN_DATA_FRAME;							// Sending a Data frame not a Remote frame
//	txHeader.DataLength = FDCAN_DLC_BYTES_8;							// Data length is classic CAN - 8 bytes
	txHeader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;					// Notify us if there is any error in transmission
	txHeader->BitRateSwitch = FDCAN_BRS_OFF;							// Will use same bit rate for both Arbitration and Data fields
	txHeader->FDFormat = FDCAN_CLASSIC_CAN;							// Using standard CAN not FDCAN
	txHeader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;					// Not using TxEvent
	txHeader->MessageMarker = 0;										// Not using MessageMarker
	return HAL_OK;
}
/* USER CODE END 1 */
