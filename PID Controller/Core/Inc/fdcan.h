/*
 * fdcan.h
 *
 *  Created on: Jan 19, 2025
 *      Author: kohlmanz
 */

#ifndef INC_FDCAN_H_
#define INC_FDCAN_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define DEFAULT_ETC_PRIORITY 7

/* Error codes */
#define GENERIC_ERROR_CODE 0xFF

/* DLC sizes */
#define DLC_THROTTLE 1
#define DLC_ACCEL 3
#define DLC_ERROR 1
#define DLC_STATUS_REPORT 3
#define DLC_SHUTDOWN 0
#define DLC_CALIBRATE 0

/* Enum for Module field for custom CAN packet (inside of the 11b ID) */
typedef enum {
	SafteySystem = 0,
	Broadcast = 1,
	ThrottleControlBoard = 2,
	PedalBox = 3,
	SteeringWheel = 4,
	ThermoControlBoard = 5

} Module;

typedef enum {
	to = 0, from = 1
} Direction;

typedef enum {
	shutdown = 0,
	status_report = 1,
	error = 2,
	position = 3,
	throttle_percentage = 4,
	calibrate = 5
} Command;
typedef struct{
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t data[8];
}CANMessage;


HAL_StatusTypeDef fdcanWrite(FDCAN_HandleTypeDef* hfdcan, FDCAN_TxHeaderTypeDef txHeader, uint8_t* txData, uint8_t len, Module module, Direction direction, uint8_t priority, Command command);
HAL_StatusTypeDef fdcanInit(FDCAN_HandleTypeDef* hfdcan);
HAL_StatusTypeDef fdcanFilterInit(FDCAN_HandleTypeDef* hfdcan, FDCAN_TxHeaderTypeDef* txHeader);




#endif /* INC_FDCAN_H_ */
