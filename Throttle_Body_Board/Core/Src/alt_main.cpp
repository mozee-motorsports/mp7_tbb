/*
 * alt_main.cpp
 *
 *  Created on: Feb 4, 2025
 *      Author: kohlmanz
 ********************************************************************************
 *	Wiring - Chinesium
 *		Motor Connections
 *			VNH5019  OUTA 	-> Pin 4D (Motor +)
 *			VNH5019  OUTB  	-> Pin 1A (Motor -)
 *
 *		L298N Control Pins
 *			VNH5019  INA 	-> PA4 (Motor1Pin1)
 *			VNH5019  INB 	-> PA5 (Motor1Pin2)
 *			VNH5019  ENA 	-> PC0 (PWM_HIGH, for speed control via PWM)
 *
 *		Power Supply
 *			VNH5019  VDD 	-> 5V (logic supply)
 *			VNH5019  VIN 	-> 12V (motor supply)
 *			VNH5019  GND 	-> Common ground with STM32
 *			VNH5019  GND	-> Supply ground
 *
 *		Potentiometer Connections
 *			Pin 3C (Poti +) -> 3V3 (or 5V if your ADC is 5V-based)
 *			Pin 2B (Poti -) -> GND
 *			Pin 6F (Poti 1) -> PA0 (ADC1_IN1)
 *			Pin 5E (Poti 2) -> PA1 (ADC1_IN2)
 *
 *	Control Logic :
 *		Forward (Open Throttle):
 *			Set PA4 (IN1) high, PA5 (IN2) low.
 *			Apply PWM on PC0 (ENA) to control motor speed.
 *
 *		Reverse (Close Throttle):
 *			Set PA4 (IN1) low, PA5 (IN2) high.
 *			Apply PWM on PC0 (ENA).
 *
 *		Stop:
 *			Set PA4 and PA5 low (or disable ENA).
 ********************************************************************************
 *  ETC Outputs:
 *  	POT1						: Analog position forward, TPS+
 *  	POT2						: Analog position reverse, TPS-
 *
 *  ETC Inputs:
 *  	POT+						: + pot ref, 3V3
 *  	POT-						: - pot ref, GND
 *  	Motor+						: + motor control sig, Hbridge output+
 *  	Motor-						: - motor control sig, Hbrdige output-
 *
 ********************************************************************************
 *
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
#include "pid_controller.h"
#include "PID_V1.h"

#define OPEN_ADC_STEPS 3600		// Measured ADC steps for fully open - pots may measure farther

#define CHINESEIUM // Using chineseium h-bridge (L298N), uses Motor1Pin1 (PA4) -> IN1, Motor1Pin2 (PA5) -> IN2, PWM_High (PC0, TIM1 CH1) -> ENA
#define CUSTOM_PID

#define MAX_ADC_OUTPUT 0xFFF // MAX is 4095 = 3V3
#define MAX_DUTY_CYCLE 100.0f
#define TIM1_17_ARR MAX_DUTY_CYCLE
#define SAMPLES_PER_CHANNEL 256

static volatile bool tps_ready = false;
static volatile bool pid_ready = false;
static volatile bool shutdown_req = false;

static uint32_t tps_buffer[2];
static uint16_t set_point = 2867;
/* Adaptive tuning parameters */

// Aggressive tuning parameters for large error
#define AGR_KP 0.09f
#define AGR_KI 0.01f
#define AGR_KD 0.001f

// Non-adaptive tuning parameters
//#define KP 0
//#define KI 0
//#define KD 0
#define KP AGR_KP
#define KI AGR_KI
#define KD AGR_KD

// Medium tuning parameters for medium errors
#define MED_KP 1.0f
#define MED_KI 0.0f
#define MED_KD 0.0f

// Conservative tuning parameters for small errors
#define CONS_KP 0.008f
#define CONS_KI 0.005f
#define CONS_KD 0.01f

// Output from PID controller
static double pid_out;

// Double representation of set point- recall that it's in terms of ADC steps, but library requires double type
static double set_point_d = set_point;
// Pointer to double representation of set point
static double* set_ptr = &set_point_d;

static double pot1_d = static_cast<double>(tps_buffer[0]);
static double pot2_d = static_cast<double>(tps_buffer[1]);

// Create PID controller object
//static PIDController throttlePID(AGR_KP, AGR_KI, AGR_KD, -TIM1_17_ARR, TIM1_17_ARR);
PID throttlePID(&pot1_d, &pid_out, set_ptr, KP, KI, KD, DIRECT);

#define INTERVAL_MS (int32_t)10

// FDCAN Defines
static FDCAN_TxHeaderTypeDef tx_header;
static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t tx_data[8];
static uint8_t rx_data[8];

// Private function prototypes
static std::queue<CANMessage> fdcan_queue; // Queue to process non-critical tasks
static std::queue<Error> error_queue;	   // Queue to process errors

// Function prototypes
static void controlMotor(double control_signal, int32_t position_delta);
static void setThrottleSetpoint(double setpoint);
static void stopMotor(void);
static void adaptPIDTuning(void);		// Adaptive tuning
static Error handleError(Error code);
static Error handleThrottle(CANMessage *msg);
static Error handleStatusReport(void);
static Error processCANMessage(CANMessage *msg, Command command);
static void my_shutdown(void);
static void myprintf(const char *fmt, ...);
static double getSetpointSteps(uint8_t percentage);


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	// Cast adc to double to used as PID parameters
	pot1_d = static_cast<double>(tps_buffer[0]);
	pot2_d = static_cast<double>(tps_buffer[1]);
}

// Sample at fixed intervals: 10ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int last, current;
	if (htim->Instance == TIM2)
	{
		current = HAL_GetTick();
		pid_ready = true;
		last = current;

		// ADC flag
		tps_ready = true;
	}
}

// Implement after
static void adaptPIDTuning(void)
{
    // Calculate error magnitude in terms of adc steps
    double error = fabs(set_point_d - pot1_d);

    // Apply different tuning parameters based on error magnitude
    if (error > 500)
    {
        // Large error - aggressive tuning
        throttlePID.SetTunings(AGR_KP, AGR_KI, AGR_KD);
    }
    else if (error > 100)
    {
        // Medium error - moderate tuning
        throttlePID.SetTunings(MED_KP, MED_KI, MED_KD);
    }
    else
    {
        // Small error - conservative tuning
        throttlePID.SetTunings(CONS_KP, CONS_KI, CONS_KD);
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
		if (HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(tps_buffer), 2) != HAL_OK)
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
	return ok;
}

static double getSetpointSteps(uint8_t percentage)
{
	return static_cast<double>(roundf(((float)percentage / MAX_DUTY_CYCLE) * OPEN_ADC_STEPS));
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
	set_point_d = getSetpointSteps(throttle_position_percentage);
	return ok;
}

/*
 * Returns battery level of Throttle Control board by sampling with ADC - 0xFF if not implemented
 */
static Error handleStatusReport(void)
{
	std::queue<Error> temp = error_queue; // Create a copy

	// First verify dlc
	if (rx_header.DataLength != STATUS_REPORT_DLC)
		return mismatch_dlc;

	// Check if there are any errors
	while (!temp.empty())
	{
		Error error = temp.front(); // Get error from queue

		// Setup packet to send
		uint8_t tx_data[3] = {0xFF, 0xFF, (uint8_t)error};

		// Send error, if we can't then return error
		if (fdcanWrite(&hfdcan1, tx_header, tx_data, STATUS_REPORT_DLC, steering_wheel, from, DEFAULT_PRIORITY, status_report) != HAL_OK)
			return fdcan_tx_failure;

		temp.pop(); // Remove error from queue
	}

	return ok;
}

static void stopMotor(void)
{
#ifdef CHINESEIUM
	HAL_GPIO_WritePin(Motor1Pin1_GPIO_Port, Motor1Pin1_Pin, GPIO_PIN_RESET); // IN1 low
	HAL_GPIO_WritePin(Motor1Pin2_GPIO_Port, Motor1Pin2_Pin, GPIO_PIN_RESET); // IN2 low
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);								 // ENA off
#else
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);  // PWM_HIGH off
	HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1); // PWM_LOW off
#endif
}

// PWM LOW = TIM17 CH1
// PWM HIGH = TIM1 CH1
/**
 * @brief Sets motor direction and speed based on PID output, respecting physical limits.
 * @param pid_output PID controller output (positive for forward, negative for reverse).
 */
// In terms of ADC POT1 steps
#define MAX_PHYSICAL_LIMIT 3890
#define MIN_PHYSCIAL_LIMIT 640
static void controlMotor(double control_signal, int32_t position_delta)
{

	// Prevent driving beyond physical limits
	if ((pot1_d <= MIN_PHYSCIAL_LIMIT && control_signal < 0) || // At min, trying to close further
		(pot1_d >= MAX_PHYSICAL_LIMIT && control_signal > 0))
	{ // At max, trying to open further
		stopMotor();
		return;
	}

	// Without a deadband, small errors (e.g., ±10 ADC steps)
	// could keep the motor oscillating or running at low PWM, causing jitter and wear.
//	if (abs(position_delta) <= DEADBAND_DIFF)
//	{
//	    stopMotor();
//	    return;
//	}

	// Calculate duty cycle from PID output
	const double duty_cycle = fabs(control_signal);
	const uint8_t duty = (duty_cycle > MAX_DUTY_CYCLE) ? MAX_DUTY_CYCLE : static_cast<uint8_t>(duty_cycle);

#ifdef CHINESEIUM
	// CHINESEIUM H-bridge (L298N): Uses IN1/IN2 for direction, ENA for PWM
	if (control_signal > 0)			// Control signal = pid output = -100 -> 100 to be mapped to PWM
	{ // Forward (open throttle)
		// Set tuning to aggressive
		HAL_GPIO_WritePin(Motor1Pin1_GPIO_Port, Motor1Pin1_Pin, GPIO_PIN_SET);	 // IN1 high
		HAL_GPIO_WritePin(Motor1Pin2_GPIO_Port, Motor1Pin2_Pin, GPIO_PIN_RESET); // IN2 low
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);						 // ENA PWM
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	}
	else if (control_signal < 0) 	// Control signal = pid output = -100 -> 100 to be mapped to PWM
	{ // Reverse (close throttle)

		HAL_GPIO_WritePin(Motor1Pin1_GPIO_Port, Motor1Pin1_Pin, GPIO_PIN_RESET); // IN1 low
		HAL_GPIO_WritePin(Motor1Pin2_GPIO_Port, Motor1Pin2_Pin, GPIO_PIN_SET);	 // IN2 high
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);						 // ENA PWM
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	}


#else
	// Non-CHINESEIUM: Uses PWM_HIGH for forward, PWM_LOW for reverse
	if (control_signal > 0) // Control signal = pid output = -100 -> 100 to be mapped to PWM
	{ // Forward (open throttle)
		HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);			// PWM_LOW off
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty); // PWM_HIGH duty
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	}
	else if (control_signal < 0) // Control signal = pid output = -100 -> 100 to be mapped to PWM
	{ // Reverse (close throttle)
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);			 // PWM_HIGH off
		__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, duty); // PWM_LOW duty
		HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	}

#endif
	GPIO_PinState ina = HAL_GPIO_ReadPin(Motor1Pin1_GPIO_Port, Motor1Pin1_Pin);
	GPIO_PinState inb = HAL_GPIO_ReadPin(Motor1Pin2_GPIO_Port, Motor1Pin2_Pin);

	myprintf("POT1 : %f POT2 : %f PID: %f DUTY: %i, set point: %f, delta: %i, INA %i, INB %i\r\n",
			pot1_d,
			pot2_d,
			control_signal,
			duty,
			set_point_d,
			position_delta,
			(int)ina,
			(int)inb);

}

void setThrottleSetpoint(double setpoint)
{
	if (setpoint < 0)
		setpoint = 0;
	else if (setpoint > MAX_ADC_OUTPUT)
		setpoint = MAX_ADC_OUTPUT;

	set_point = setpoint;
	set_point_d = static_cast<double>(set_point);
}

int alt_main(void)
{
	set_point_d = getSetpointSteps(75);

	/* Initialization */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	if (HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(tps_buffer), 2) != HAL_OK)
		error_queue.push(adc_failure); // ADC failure

	// Init canfd instance
	if (fdcanInit(&hfdcan1) != HAL_OK)
		error_queue.push(fdcan_init_failure); // FDCAN failure

	// Sets up the CANFD filter
	if (fdcanFilterInit(&hfdcan1, &tx_header) != HAL_OK)
		error_queue.push(fdcan_init_failure); // FDCAN failure

//	throttlePID.setSetpoint(set_point);
	throttlePID.SetMode(AUTOMATIC);
	throttlePID.SetSampleTime(INTERVAL_MS);
	throttlePID.SetOutputLimits(-(double)TIM1_17_ARR, (double)TIM1_17_ARR);		// Normalized limits: -100 - 100% - map these directly to the pwm
	static double position_delta; // How close are we to the actual setpoint?

	shutdown_req = false;
	while (shutdown_req == false)
	{
		/* Super loop */
		if (shutdown_req == true) // If shutdown requested, break out of loop
		{
			break;
		}

		if (shutdown_req == true) // If shutdown requested, break out of loop
		{
			break;
		}

		if (pid_ready && tps_ready)
		{
			position_delta = set_point_d - pot1_d;
//			pid_out = throttlePID.calculate(tps_buffer[0], HAL_GetTick());
			throttlePID.Compute();

			controlMotor(pid_out, position_delta);
		}
	}
	my_shutdown(); // Shutdown system
	exit(0);	   // Exit program
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
				shutdown_req = true; // Request shutdown in main loop
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
