/*
 * alt_main.cpp
 *
 *  Created on: Feb 4, 2025
 *      Author: kohlmanz
 ********************************************************************************
 *	Wiring - Chinesium
 *		Motor Connections
 *			L298N OUT1 	-> Pin 4D (Motor +)
 *			L298N OUT2  -> Pin 1A (Motor -)
 *
 *		L298N Control Pins
 *			L298N IN1 	-> PA4 (Motor1Pin1)
 *			L298N IN2 	-> PA5 (Motor1Pin2)
 *			L298N ENA 	-> PC0 (PWM_HIGH, for speed control via PWM)
 *
 *		Power Supply
 *			REMOVE 5V REGULATOR JUMPER
 *			L298N VSS 	-> 5V (logic supply)
 *			L298N VS 	-> 12V (motor supply)
 *			L298N GND 	-> Common ground with STM32
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
 *  Pinout: L298N Chinesium Hbridge driver
 *		PC0, PWM_GEN				: PWM_HIGH -> pot+
 *		PA7, PWM_GEN				: PWM_LOW -> pot-
 *		PC1, Not used				: Not used
 *		PA0, ADC_IN					: POT1 -> fwd pot
 *		PA1, ADC_IN					: POT2 -> rev pot
 *		PA4, PWM_GEN, TIM3_CH2		: Motor1Pin1 -> IN1
 *		PA5, PWM_GEN, TIM2_CH1		: Motor1Pin2 -> IN2
 *
 ********************************************************************************
 *	L298N Inputs:
 *		VS							: +12V From motor/psu
 *		GND 						: From motor/psu
 *		VSS							: Not used, Logic level - 5V min
 *		ENA							: Speed control, not used. Jumper placed, will output at full speed
 *		IN1 						: PWM ETC velocity, duty cycle when forward, 0 when reverse
 *		IN2 						: PWM ETC velocity, duty cycle when reverse, 0 when forward
 *
 *	L298N Outputs:
 *		OUT1, Motor+				: To ETC
 *		OUT2, Motor-				: To ETC
 *
 ********************************************************************************
 *
 *	Note: Uses 5V logic, but G4 micro only has 3V3 logic. Add jumper at ENA pins, removing
 *		  the original PWM signal and send PID, PWM signal through active direction (Motor1Pin1, Motor1Pin2).
 *
 *		  L298N Documentation: https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/#:~:text=The%20L298N%20chip%20contains%20two,most%20of%20our%20DC%20motors.
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

#define DEADBAND_DIFF 10 // if (tps_buffer[0] - set_point > Deadband_diff) { ... determine direction, else we are at set point }

static void setMotor(double pid_output, int32_t position_delta);
static void setThrottleSetpoint(double setpoint);

//#define USING_PID_INT // Using PID2 library
#define CHINESEIUM // Using chineseium h-bridge (L298N), uses Motor1Pin1 (PA4) -> IN1, Motor1Pin2 (PA5) -> IN2, PWM_High (PC0, TIM1 CH1) -> ENA

#ifdef USING_PID_INT
#include "pid2.h"
#else
#include "PID_V1.h"
#endif

#define MAX_ADC_OUTPUT 0xFFF // MAX is 4095 = 3V3
#define MAX_DUTY_CYCLE 100.0f
#define SAMPLES_PER_CHANNEL 256

static volatile bool tps_ready = false;
static volatile bool pid_ready = false;
static volatile bool shutdown_req = false;
static int32_t tps_buffer[2]; // tps_buffer[0] << 2= POT1, tps_buffer[1] << 2= POT2


#ifdef USING_PID_INT
static int32_t set_point = 2048;	   // Store set point, set in FDCAN ISR, 3071 = 3/4
static int32_t pid_out;				   // Velocity of ETC, not position. Should not go negative.

// Scaling factor is 2^QN = 2^12 = 4096	... So do round(DESIRED_FRAC * 4096)
static const int32_t QN = 12;
static const int32_t SCALING = pow(2, QN);
static const int32_t KP = round(1 * SCALING); //round(2 * SCALING);
static const int32_t KI = 0;// round(0.01 * SCALING);
static const int32_t KD = 0;

//#define KP (int32_t)1				   // Proportional gain
//#define KI (int32_t)1				   // Integral gain
//#define KD (int32_t)1				   // Derivative gain
//#define QN (int32_t)12				   // Number of fractional bits (32 total bits = 12 fractional + 20 integer)
#define MAX_ERROR_QUEUE_SIZE 10		   // Define a max queue size
Pid::PID pidController = Pid::PID(set_point, KP, KI, KD, QN, Pid::feedbackPositive, Pid::proportionalToError);
#else
#define AGR_KP 1.8f
#define AGR_KI 0.01f
#define AGR_KD 0.002f
double* tps_value = reinterpret_cast<double*>(tps_buffer);

#define CONS_KP 0.008f
#define CONS_KI 0.005f
#define CONS_KD 0.01f

double set_point, pid_out;
PID myPID(tps_value, &pid_out, &set_point, AGR_KP, AGR_KI, AGR_KD, DIRECT);

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
	// Uses DMA, no code needed here
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
		if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)tps_buffer, 2) != HAL_OK)
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

#ifdef USING_PID_INT
	pidController.setSetpoint(set_point); // Set set point in PID controller
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

// PWM LOW = TIM17 CH1
// PWM HIGH = TIM1 CH1
/**
 * @brief Sets motor direction and speed based on PID output, respecting physical limits.
 * @param pid_output PID controller output (positive for forward, negative for reverse).
 */
static void setMotor(double pid_output, int32_t position_delta)
{
	// Get current throttle position from POT1
	const double current_position = tps_buffer[0];

	// Prevent driving beyond physical limits
	if ((current_position <= 0 && pid_output < 0) || // At min, trying to close further
		(current_position >= MAX_ADC_OUTPUT && pid_output > 0))
	{ // At max, trying to open further
#ifdef CHINESEIUM
		HAL_GPIO_WritePin(Motor1Pin1_GPIO_Port, Motor1Pin1_Pin, GPIO_PIN_RESET); // IN1 low
		HAL_GPIO_WritePin(Motor1Pin2_GPIO_Port, Motor1Pin2_Pin, GPIO_PIN_RESET); // IN2 low
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);								 // ENA off
#else
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);  // PWM_HIGH off
		HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1); // PWM_LOW off
#endif
		return;
	}

	// Calculate duty cycle from PID output
	const double abs_output = fabs(pid_output);
	const double duty_cycle = (abs_output / MAX_ADC_OUTPUT) * MAX_DUTY_CYCLE;
	const uint8_t duty = (duty_cycle > MAX_DUTY_CYCLE) ? MAX_DUTY_CYCLE : static_cast<uint8_t>(duty_cycle);

#ifdef CHINESEIUM
	// CHINESEIUM H-bridge (L298N): Uses IN1/IN2 for direction, ENA for PWM
	if (pid_output > DEADBAND_DIFF)
	{ // Forward (open throttle)
#ifndef USING_PID_INT
		// Set tuning to aggressive
		myPID.SetTunings(AGR_KP, AGR_KI, AGR_KD);
#endif
		HAL_GPIO_WritePin(Motor1Pin1_GPIO_Port, Motor1Pin1_Pin, GPIO_PIN_SET);	 // IN1 high
		HAL_GPIO_WritePin(Motor1Pin2_GPIO_Port, Motor1Pin2_Pin, GPIO_PIN_RESET); // IN2 low
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);						 // ENA PWM
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	}
	else if (pid_output < -DEADBAND_DIFF)
	{ // Reverse (close throttle)
#ifndef USING_PID_INT
		// Set tuning to aggressive
		myPID.SetTunings(AGR_KP, AGR_KI, AGR_KD);
#endif
		HAL_GPIO_WritePin(Motor1Pin1_GPIO_Port, Motor1Pin1_Pin, GPIO_PIN_RESET); // IN1 low
		HAL_GPIO_WritePin(Motor1Pin2_GPIO_Port, Motor1Pin2_Pin, GPIO_PIN_SET);	 // IN2 high
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);						 // ENA PWM
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	}
	else
	{																			 // Within deadband, stop
		HAL_GPIO_WritePin(Motor1Pin1_GPIO_Port, Motor1Pin1_Pin, GPIO_PIN_RESET); // IN1 low
		HAL_GPIO_WritePin(Motor1Pin2_GPIO_Port, Motor1Pin2_Pin, GPIO_PIN_RESET); // IN2 low
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);								 // ENA off
#ifndef USING_PID_INT
		// Near set point, so set conservative tuning
		myPID.SetTunings(CONS_KP, CONS_KI, CONS_KD);
#endif
	}
#else
	// Non-CHINESEIUM: Uses PWM_HIGH for forward, PWM_LOW for reverse
	if (pid_output > DEADBAND_DIFF)
	{ // Forward (open throttle)
#ifndef USING_PID_INT
		// Set aggressive tuning
		myPID.SetTunings(AGR_KP, AGR_KI, AGR_KD);
#endif
		HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);			// PWM_LOW off
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty); // PWM_HIGH duty
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	}
	else if (pid_output < -DEADBAND_DIFF)
	{ // Reverse (close throttle)
#ifndef USING_PID_INT
		// Set aggressive tuning
		myPID.SetTunings(AGR_KP, AGR_KI, AGR_KD);
#endif
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);			 // PWM_HIGH off
		__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, duty); // PWM_LOW duty
		HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	}
	else
	{ // Within deadband, stop
#ifndef USING_PID_INT
		// Near set point, so set conservative tuning
		myPID.SetTunings(CONS_KP, CONS_KI, CONS_KD);
#endif
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);  // PWM_HIGH off
		HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1); // PWM_LOW off
	}
#endif
	myprintf("POT1 : %i POT2 : %i PID: %i DUTY: %f, position: %i, INA %i, INB %i\r\n", tps_buffer[0], tps_buffer[1], pid_out, duty_cycle, position_delta, HAL_GPIO_ReadPin(GPIOA, Motor1Pin1_Pin), HAL_GPIO_ReadPin(GPIOA, Motor1Pin2_Pin));

}

void setThrottleSetpoint(double setpoint)
{
	if (setpoint < 0)
		setpoint = 0;
	else if (setpoint > MAX_ADC_OUTPUT)
		setpoint = MAX_ADC_OUTPUT;
#ifdef USING_PID_INT
	pidController.setSetpoint((uint32_t)setpoint);
#else // Uses pointer to setpoint, so setter not needed
	set_point = setpoint;
#endif
}

int alt_main(void)
{
	/* Initialization */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)tps_buffer, 2) != HAL_OK)
		error_queue.push(adc_failure); // ADC failure

	// Init canfd instance
	if (fdcanInit(&hfdcan1) != HAL_OK)
		error_queue.push(fdcan_init_failure); // FDCAN failure

	// Sets up the CANFD filter
	if (fdcanFilterInit(&hfdcan1, &tx_header) != HAL_OK)
		error_queue.push(fdcan_init_failure); // FDCAN failure

#ifdef USING_PID_INT
	pidController.setOutputMin(-MAX_ADC_OUTPUT);				// 0
	pidController.setOutputMax(MAX_ADC_OUTPUT); 				// 4095
	pidController.init(tps_buffer[0]);
	static int32_t position_delta; // How close are we to the actual setpoint?
#else
	myPID.SetMode(AUTOMATIC);
	myPID.SetSampleTime(INTERVAL_MS);
	myPID.SetOutputLimits(-(double)MAX_ADC_OUTPUT, (double)MAX_ADC_OUTPUT);
	static double position_delta; // How close are we to the actual setpoint?

#endif
	shutdown_req = false;
	while (shutdown_req == false)
	{
		/* Super loop */
		// Check for errors
		//		while (!error_queue.empty())
		//		{
		//			if (error_queue.size() > MAX_ERROR_QUEUE_SIZE)
		//			{
		//				error_queue.pop(); // Discard the oldest error
		//			}
		//			Error error = error_queue.front(); // Get error from queue
		//			shutdown_req = handleError(error) == ok? false : true;				   // Process error
		//			error_queue.pop();				   // Remove error from queue
		//
		//			if(shutdown_req == true) // If shutdown requested, break out of loop
		//			{
		//				break;
		//			}
		//		}

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
			position_delta = set_point - tps_buffer[0];
#ifdef USING_PID_INT
			pid_out = pidController.compute(tps_buffer[0]);

#else
			myPID.Compute();
//			myprintf("POT1 : %i POT2 : %i PID: %f position: %f, INA %i, INB %i\r\n", tps_buffer[0], tps_buffer[1], pid_out, position_delta, HAL_GPIO_ReadPin(GPIOA, Motor1Pin1_Pin), HAL_GPIO_ReadPin(GPIOA, Motor1Pin2_Pin));
#endif
			setMotor(pid_out, position_delta);
		}
		//		if (pid_ready && tps_ready)
		//		{
		//
		//
		//			position_delta = set_point - tps_buffer[0];
		// #ifdef USING_PID_INT
		//			pid_out = pidController.compute(tps_buffer[0]);
		//			myprintf("POT1 : %i POT2 : %i PID: %i position: %i, INA %i, INB %i\r\n", tps_buffer[0], tps_buffer[1], pid_out, position_delta, HAL_GPIO_ReadPin(GPIOA, Motor1Pin1_Pin), HAL_GPIO_ReadPin(GPIOA, Motor1Pin2_Pin));
		//
		// #else
		//			myPID.Compute();
		//			myprintf("POT1 : %f POT2 : %f PID: %f position: %f, INA %i, INB %i\r\n", tps_buffer[0], tps_buffer[1], pid_out, position_delta, HAL_GPIO_ReadPin(GPIOA, Motor1Pin1_Pin), HAL_GPIO_ReadPin(GPIOA, Motor1Pin2_Pin));
		// #endif
		//			// Duty variable for PWM output - recall it goes 0 -> 99
		//			static uint8_t duty_numerical = 0;
		//
		//			// PWM_HIGH = PC0 = TIM1_CH1
		//			// PWM_LOW = PA7 = TIM17_CH1
		//			// If position is negative, then we are overshooting set point and need to go in reverse
		//			if (position_delta < 0)
		////			if(pid_out < 0)
		//			{
		//				if ((tps_buffer[0] + pid_out) < 0) // If the tps_value (in adc steps) + negative pid output is less than 0, the min value that the etc can be, then is an error
		//				{
		//					// Negative bounds error, set to to 0
		//					pid_out = 0;
		//				} // Set to backward by stopping Motor1Pin1 and writing duty cycle to Motor1Pin2
		// #ifndef CHINESEIUM
		//				// Set PWM_HIGH LOW
		//				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		// #else
		//				// Instead of two PWM Signals, we only use PWM_HIGH and set direction with M1P1 and M1P2
		//				// GPIO_Output
		//				HAL_GPIO_WritePin(Motor1Pin1_GPIO_Port, Motor1Pin1_Pin, GPIO_PIN_RESET);
		//				HAL_GPIO_WritePin(Motor1Pin2_GPIO_Port, Motor1Pin2_Pin, GPIO_PIN_SET);
		// #endif
		//				// Convert absolute value of pid_out (in terms of ADC setps 0 - 4095) to duty cycle (0 - 100%)
		//				double duty_cycle = ((fabs(pid_out) / MAX_ADC_OUTPUT) * MAX_DUTY_CYCLE);
		//				if (duty_cycle > MAX_DUTY_CYCLE)
		//					duty_cycle = MAX_DUTY_CYCLE;
		//				duty_numerical = (uint8_t)duty_cycle;
		//
		// #ifndef CHINESEIUM
		//				// Recall we use PWM_LOW/HIGH -> In rev, PWM_LOW is duty cycle and PWM_HIGH is LOW
		//
		//				// Set duty cycle for PWM_LOW
		//				__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, duty_numerical);
		//
		//				// Enable output for PWM_LOW
		//				HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
		// #else
		//				// Chinesium hbridge uses direction bits and only 1 PWM signal (PWM_HIGH)
		//				// PWM_HIGH should be duty cycle
		//
		//				// Set duty cycle for ENA
		//				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_numerical);
		//
		//				// Enable output for ENA
		//				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		// #endif
		//			}
		//			// If positon is positive, then we are below set point and need to go forward
		//			else if (position_delta > 0)
		//			{ // Set forward by writing duty cycle to Motor1Pin1 to High and setting Motor1Pin2 to ground
		// #ifndef CHINESEIUM
		//			  // In forward -> PWM_LOW = 0, PWM_HIGH = duty cycle
		//				HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
		// #else
		//				// In forward, M1P1 = High, M1P2 = low
		//				HAL_GPIO_WritePin(Motor1Pin1_GPIO_Port, Motor1Pin1_Pin, GPIO_PIN_SET);
		//				HAL_GPIO_WritePin(Motor1Pin2_GPIO_Port, Motor1Pin2_Pin, GPIO_PIN_RESET);
		// #endif
		//				double duty_cycle = (((double)pid_out / MAX_ADC_OUTPUT) * MAX_DUTY_CYCLE);
		//				if (duty_cycle > MAX_DUTY_CYCLE)
		//					duty_cycle = MAX_DUTY_CYCLE;
		//				duty_numerical = (duty_cycle);
		//
		//				// Set duty cycle for PWM_HIGH - Chineseium H-Bridge uses TIM1_CH1
		//				// BOTH CHINESIUM AND REGULAR HBRIDGE ONLY USE THIS PWM IN FORWARD
		//				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
		//
		//				// Enable output for PWM_HIGH
		//				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		//			}
		//			else
		//			{ 	// We are effectively at the setpoint
		//				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		//				HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_2);
		//				HAL_GPIO_WritePin(HBRIDGE_EN_GPIO_Port, HBRIDGE_EN_Pin, GPIO_PIN_RESET);
		//				HAL_GPIO_WritePin(Motor1Pin1_GPIO_Port, Motor1Pin1_Pin, GPIO_PIN_RESET);
		//				HAL_GPIO_WritePin(Motor1Pin2_GPIO_Port, Motor1Pin2_Pin, GPIO_PIN_RESET);
		//			}
		//
		//
		//			//			HAL_Delay (INTERVAL);
		//		}
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
