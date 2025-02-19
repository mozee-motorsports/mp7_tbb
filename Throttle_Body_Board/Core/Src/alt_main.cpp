/*
 * alt_main.cpp
 *
 *  Created on: Feb 4, 2025
 *      Author: kohlmanz
 */

#include "alt_main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"

#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <queue>
#include <string.h>
#ifdef USING_PID2
#include "pid2.h"
#else
#include "pid.h"
#endif
#include "fdcan_queue.h"

#define MAX_ADC_OUTPUT 0xFFF // MAX is 4095 = 3V3
#define MAX_DUTY_CYCLE 100.0f

static int32_t set_point = MAX_ADC_OUTPUT >> 1; // Store set point, set in FDCAN ISR
static volatile int32_t tps_buffer[1];			// Store throttle position sensor value, set in memory via DMA
static volatile bool tps_ready = false;
static volatile bool pid_ready = false;
static volatile bool shutdown_req = false;

static int32_t pid_out;
static int32_t out[500]; // For debugging only

#ifdef USING_PID2
#define KP (int32_t)0	// Proportional gain
#define KI (int32_t)100 // Integral gain
#define KD (int32_t)1	// Derivative gain
#define QN (int32_t)5	// Fixed point precision
#define MAX_ERROR_QUEUE_SIZE  10 // Define a max queue size
Pid::PID pidController = Pid::PID(set_point, KP, KI, KD, QN, Pid::feedbackPositive, Pid::proportionalToError);
#else
#define KP 0.0f	 // Proportional gain
#define KI 10.0f // Integral gain
#define KD 0.0f	 // Derivative gain
static PID_TypeDef pid;
#endif

#define INTERVAL_MS (int32_t)10

// FDCAN Defines
static FDCAN_TxHeaderTypeDef tx_header;
static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t tx_data[8];
static uint8_t rx_data[8];

// Private function prototypes
static std::queue<CANMessage> fdcan_queue; // Queue to process non-critical tasks
static std::queue<Error> error_queue;	   // Queue to process errors
static Error handleError(Error code);
static Error handleThrottle(CANMessage *msg);
static Error handleStatusReport(void);
static Error processCANMessage(CANMessage *msg, Command command);
static void my_shutdown(void);
static void myprintf(const char *fmt, ...);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
	{
		tps_ready = true;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int last, current;
	if (htim->Instance == TIM2)
	{
		current = HAL_GetTick();
		int delta = current - last;
		pid_ready = true;
		last = current;
	}
}

static void my_shutdown(void)
{
	// Turn off enable
	HAL_GPIO_WritePin(HBRIDGE_EN_GPIO_Port, HBRIDGE_EN_Pin, GPIO_PIN_RESET);

	// Set pins to ground
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	// Stop PWM signals
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}

static void myprintf(const char *fmt, ...)
{
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&hlpuart1, (uint8_t *)buffer, len, -1);
}

// Process commands sent from the control board master to the throttle control board
static Error processCANMessage(CANMessage *msg, Command command)
{
	// Execute sent command
	Error code = ok;
	switch (command)
	{
	case shutdown: // Already handled in the ISR
		// Shutdown command, exit program
		my_shutdown();
		shutdown_req = true; // Set shutdown request flag
		break;
	case throttle_percentage:
		code = handleThrottle(msg);
		break;
	case send_error:
		code = handleError((Error)(msg->data[0]));
		break;
	case status_report:
		code = handleStatusReport();
		break;
	default: // Invalid command, return error
		code = invalid_command;
		break;
	}

	// If there is an error, then add it to the queue
	if (code != ok)
	{
		error_queue.push(code);
	}

	return code;
}

/**
 * Determines appropriate response given the error code. All error codes sent to control board master.
 * * @param code: Error code to handle
 */
static Error handleError(Error code)
{

	// Handle errors based on the error code
	switch (code)
	{
	case ok:
		return ok; // No error, do nothing
	case generic:  // Try to send error over CAN, but don't retry here since fdcanWrite already retries
		if (fdcanWrite(&hfdcan1, tx_header, (uint8_t *)&code, ERROR_DLC, throttle_control_board, from, DEFAULT_PRIORITY, send_error) != HAL_OK)
		{
			return fdcan_tx_failure; // FDCAN failure
		}
		break;
	case mismatch_dlc: // DLC mismatch try to send error over CAN, but don't retry here since fdcanWrite already retries
		if (fdcanWrite(&hfdcan1, tx_header, (uint8_t *)&code, ERROR_DLC, throttle_control_board, from, DEFAULT_PRIORITY, send_error) != HAL_OK)
		{
			return fdcan_tx_failure; // FDCAN failure
		}
		break;
	case invalid_command: // Invalid command, try to send error over CAN, but don't retry here since fdcanWrite already retries
		if (fdcanWrite(&hfdcan1, tx_header, (uint8_t *)&code, ERROR_DLC, throttle_control_board, from, DEFAULT_PRIORITY, send_error) != HAL_OK)
		{
			return fdcan_tx_failure; // FDCAN failure
		}
		break;
	case fdcan_init_failure: // Try to re-init
		if (fdcanInit(&hfdcan1) != HAL_OK)
		{
			return fdcan_init_failure; // FDCAN failure
		}
		break;
	case fdcan_rx_failure: // Try to re-init
		if (fdcanInit(&hfdcan1) != HAL_OK)
		{
			return fdcan_rx_failure; // FDCAN failure
		}
		break;
	case fdcan_tx_failure: // Try to re-init
		if (fdcanInit(&hfdcan1) != HAL_OK)
		{
			return fdcan_tx_failure; // FDCAN failure
		}
		break;
	case adc_failure: // Try to re-init
		if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)tps_buffer, 1) != HAL_OK)
		{
			return adc_failure; // ADC failure
		}
		break;
	default: // Unknown error, try to send error over CAN, but don't retry here since fdcanWrite already retries
		if (fdcanWrite(&hfdcan1, tx_header, (uint8_t *)&code, ERROR_DLC, throttle_control_board, from, DEFAULT_PRIORITY, send_error) != HAL_OK)
		{
			return generic; // FDCAN failure
		}
		break;
	}
}

/**
 * Gets the throttle position percentage from the current message in the queue, calculates the set point,
 * and sets it.
 */
static Error handleThrottle(CANMessage *msg)
{
	// Process received data - Only 1 byte for throttle percentage
	uint8_t throttle_position_percentage = msg->data[0];

	// Get position in terms of ADC levels based on percent.
	set_point = roundf(((float)throttle_position_percentage / MAX_DUTY_CYCLE) * MAX_ADC_OUTPUT);

#ifdef USING_PID2
	pidController.setSetpoint(set_point); // Set set point in PID controller
#else
	// Set PID structure set point from incoming data
	PID_Set_Setpoint(&pid, &set_point);
#endif

	return ok;
}

/*
 * Returns battery level of Throttle Control board by sampling with ADC - 0xFF if not implemented
 */
static Error handleStatusReport(void)
{
    std::queue<Error> temp = error_queue; // Create a copy

	// First verify dlc
	if(rx_header.DataLength != STATUS_REPORT_DLC)
		return mismatch_dlc;

	// Check if there are any errors
	while(!temp.empty())
	{
		Error error = temp.front(); // Get error from queue

		// Setup packet to send
		uint8_t tx_data[3] = {0xFF, 0xFF, (uint8_t)error};

		// Send error, if we can't then return error
		if (fdcanWrite(&hfdcan1, tx_header, tx_data, STATUS_REPORT_DLC, steering_wheel, from, DEFAULT_PRIORITY, status_report) != HAL_OK)
			return fdcan_tx_failure;

		temp.pop();				   // Remove error from queue
	}

	return ok;

}

int alt_main(void)
{
	/* Initialization */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)tps_buffer, 1) != HAL_OK)
		error_queue.push(adc_failure); // ADC failure

	// Init canfd instance
	if (fdcanInit(&hfdcan1) != HAL_OK)
		error_queue.push(fdcan_init_failure); // FDCAN failure

	// Sets up the CANFD filter
	if (fdcanFilterInit(&hfdcan1, &tx_header) != HAL_OK)
		error_queue.push(fdcan_init_failure); // FDCAN failure

#ifdef USING_PID2
	pidController.setOutputMin(0);				// 0
	pidController.setOutputMax(MAX_ADC_OUTPUT); // 4095
	pidController.init(tps_buffer[0]);
#else
	// Init PID structure - tps buffer in terms of adc steps, and so is set_point. pid_out also in terms of adc steps
	PID(&pid, &tps_buffer[0], &pid_out, &set_point, KP, KI, KD, _PID_P_ON_E, _PID_CD_DIRECT); // We compare the current tps value (from the throttle body) with the setpoint from CANopen
	PID_SetMode(&pid, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&pid, INTERVAL_MS);
	PID_SetOutputLimits(&pid, 0, MAX_ADC_OUTPUT);
#endif
	static int i = 0; // For debugging only
	static int32_t position_delta;

	while (shutdown_req == false)
	{
		/* Super loop */
		// Check for errors
		while (!error_queue.empty())
		{
			if (error_queue.size() > MAX_ERROR_QUEUE_SIZE)
			{
				error_queue.pop(); // Discard the oldest error
			}
			Error error = error_queue.front(); // Get error from queue
			shutdown_req = handleError(error) == ok? false : true;				   // Process error
			error_queue.pop();				   // Remove error from queue
			
			if(shutdown_req == true) // If shutdown requested, break out of loop
			{
				break;
			}
		}

		if(shutdown_req == true) // If shutdown requested, break out of loop
		{
			break;
		}


		// Process FDCAN messages in queue
		if (!fdcan_queue.empty())
		{
			Error error;
			CANMessage msg = fdcan_queue.front(); // Get message from queue
			fdcan_queue.pop();					  // Remove message from queue

			// Process message
			error = processCANMessage(&msg, (Command)(msg.rx_header.Identifier & 0x0F));

			// If there's an error processing it, handle the error
			if(error != ok)
			{
				shutdown_req = handleError(error) == ok? false : true; // Process error
			}
		}

		if(shutdown_req == true) // If shutdown requested, break out of loop
		{
			break;
		}

		if ((pid_ready) && tps_ready)
		{
			// Print to lpuart1
//			myprintf("TPS: %il\r\n", tps_buffer[0]); // For debugging only

			// What is the distance to set point? In terms of ADC steps.
			position_delta = set_point - tps_buffer[0];
#ifdef USING_PID2
			pid_out = pidController.compute(tps_buffer[0]);
#else
			PID_Compute(&pid);
#endif
			out[i] = pid_out;
			i++;

			// Duty variable for PWM output - recall it goes 0 -> 99
			static uint8_t duty_numerical = 0;

			// PWM_HIGH = PC0 = TIM1_CH1
			// PWM_LOW = PA7 = TIM3_CH2
			if (position_delta < 0)
			{									   // Set to backward by stopping PC0 and writing duty cycle to PA7
				if ((tps_buffer[0] + pid_out) < 0) // If the tps_value (in adc steps) + negative pid output is less than 0, the min value that the etc can be, then is an error
				{
					// Negative bounds error, set to to 0
					pid_out = 0;
				}
				// Stop PC0
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

				// Convert absolute value of pid_out to duty cycle
				double duty_cycle = ((fabs(pid_out) / MAX_ADC_OUTPUT) * MAX_DUTY_CYCLE);
				if (duty_cycle > 99)
					duty_cycle = 99;
				duty_numerical = (uint8_t)duty_cycle;

				// Set duty cycle for PA7
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_numerical);

				// Enable output for PA7
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			}
			else if (position_delta > 0)
			{ // Set forward by writing duty cycle to PC0 and setting PA7 to ground

				// Stop PA7
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);

				double duty_cycle = (((double)pid_out / MAX_ADC_OUTPUT) * MAX_DUTY_CYCLE);
				if (duty_cycle > 99)
					duty_cycle = 99;
				duty_numerical = (duty_cycle);

				// Set duty cycle for PC0
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);

				// Enable output for PC0
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			}
			else
			{ // An error has happend, turn off etc
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
				HAL_GPIO_WritePin(HBRIDGE_EN_GPIO_Port, HBRIDGE_EN_Pin, GPIO_PIN_RESET);
			}

			// For debugging only
			if (i == 500)
			{
				myprintf("End reached\r\n");
			}
//			HAL_Delay (INTERVAL);

			// Get a new TPS value
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)tps_buffer, 1);
		}
	}
	my_shutdown(); // Shutdown system
	exit(0);	  // Exit program
}

/**
 * @brief  Callback function for handling received messages in FDCAN Rx FIFO 0.
 * @param  hfdcan: Pointer to the FDCAN handle.
 * @param  RxFifo0ITs: Interrupt flags for Rx FIFO 0.
 * @retval None
 *
 * This function is called when a new message is received in the FDCAN Rx FIFO 0.
 * It processes the received data, calculates the throttle position percentage,
 * determines the set point in terms of ADC levels, computes the duty cycle percentage
 * using a PID controller, and updates the PWM duty cycle accordingly.
 * At this stage, all messages are filtered to have a direction bit = 0 (To Module)
 *
 * The function performs the following steps:
 * 1. Checks if the FDCAN instance is FDCAN1.
 * 2. Verifies if a new message is received in Rx FIFO 0.
 * 3. Retrieves the received message and processes the data.
 * 4. Calculates the throttle position percentage from the received data.
 * 5. Converts the throttle position percentage to an ADC set point.
 * 6. Computes the duty cycle percentage using the PID controller.
 * 7. Updates the PWM duty cycle based on the computed duty cycle percentage.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if (hfdcan->Instance == FDCAN1)
	{
		// If we are receiving a CAN signal and its NOT in reset, then get message
		if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
		{
			FDCAN_RxHeaderTypeDef rxHeader; // Struct to place data
			uint8_t rxData[8];				// 8 bit number expressed as a percentage -> 0 - 100%

			// Get Rx messages from RX FIFO0
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
				error_queue.push(fdcan_rx_failure); // FDCAN RX failure

			// Activate notification again in case HAL deactivates interrupt
			if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
				error_queue.push(fdcan_rx_failure); // FDCAN RX failure

			// Extract command
			Command command = (Command)(rx_header.Identifier & 0x0F);

			// Process critical commands
			if (command == shutdown)
			{
				shutdown_req= true; // Request shutdown in main loop
                return;
			}

			// Queue non-critical commands
			CANMessage msg = {rx_header, {0}};

			// Make deep copy of payload
			memcpy(msg.data, rx_data, sizeof(rx_data));

			// Add message to queue
			fdcan_queue.push(msg);
		}
	}
}
