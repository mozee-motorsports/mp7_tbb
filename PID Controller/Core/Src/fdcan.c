/*
 * fdcan.c
 *
 *  Created on: Jan 19, 2025
 *      Author: kohlmanz
 */

#include "fdcan.h"

#define MAX_3BIT 0b111
#define MAX_4BIT 0b1111
HAL_StatusTypeDef fdcanWrite(FDCAN_HandleTypeDef* hfdcan, volatile FDCAN_TxHeaderTypeDef txHeader, uint8_t* txData, uint8_t len,
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
	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData)!= HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
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
#define FILTER 	0b00010000000										// Steering Wheel ID
#define MASK 	0b00011110000										// 11b id. Only look at Module bits [7..5] and direction (should be set to "to" = 0)
HAL_StatusTypeDef fdcanFilterInit(FDCAN_HandleTypeDef* hfdcan, volatile FDCAN_TxHeaderTypeDef* txHeader)
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
