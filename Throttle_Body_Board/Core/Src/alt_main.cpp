
/*
 * alt_main.cpp
 *
 *  Created on: Feb 4, 2025
 *      Author: kohlmanz
 ********************************************************************************
 *  Wiring - Chinesium
 *    Motor Connections
 *      VNH5019  OUTA   -> Pin 4D (Motor +)
 *      VNH5019  OUTB   -> Pin 1A (Motor -)
 *
 *    L298N Control Pins
 *      VNH5019  INA  -> PA4 (Motor1Pin1)
 *      VNH5019  INB  -> PA5 (Motor1Pin2)
 *      VNH5019  ENA  -> PC0 (PWM_HIGH, for speed control via PWM)
 *
 *    Power Supply
 *      VNH5019  VDD  -> 5V (logic supply)
 *      VNH5019  VIN  -> 12V (motor supply)
 *      VNH5019  GND  -> Common ground with STM32
 *      VNH5019  GND  -> Supply ground
 *
 *    Potentiometer Connections
 *      Pin 3C (Poti +) -> 3V3 (or 5V if your ADC is 5V-based)
 *      Pin 2B (Poti -) -> GND
 *      Pin 6F (Poti 1) -> PA0 (ADC1_IN1)
 *      Pin 5E (Poti 2) -> PA1 (ADC1_IN2)
 *
 *  Control Logic :
 *    Forward (Open Throttle):
 *      Set PA4 (IN1) high, PA5 (IN2) low.
 *      Apply PWM on PC0 (ENA) to control motor speed.
 *
 *    Reverse (Close Throttle):
 *      Set PA4 (IN1) low, PA5 (IN2) high.
 *      Apply PWM on PC0 (ENA).
 *
 *    Stop:
 *      Set PA4 and PA5 low (or disable ENA).
 ********************************************************************************
 *  ETC Outputs:
 *    POT1            : Analog position forward, TPS+
 *    POT2            : Analog position reverse, TPS-
 *
 *  ETC Inputs:
 *    POT+            : + pot ref, 3V3
 *    POT-            : - pot ref, GND
 *    Motor+            : + motor control sig, Hbridge output+
 *    Motor-            : - motor control sig, Hbrdige output-
 *
 ********************************************************************************
 *
 */

/* stdlib Includes */
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* HAL Includes */
#include "adc.h"
#include "alt_main.h"
#include "dac.h"
#include "dma.h"
#include "fdcan.h"
#include "gpio.h"
#include "main.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"

/* User Includes */
#include "PID_V1.h" // arduino library
using namespace std;

// PID Tuning Constants ===========================================================
// tuning parameters
#define P 250.0f
#define I 50.0f
#define D 1.0f

#define IDLE_PCT 6.5

// Dead band compensation
#define PWM_DEADBAND_OPEN   607.0 // minimum duty cycle when opening
#define PWM_DEADBAND_CLOSE  600.0 // ... when closing
#define PID_STOP_BAND       0.0

// testing =======================================================================
//TODO: remove after testing
static uint16_t msg_num = 0;
static uint16_t pbb_taps = 0;
static float throttle_pct = 0;
int PWM_test = 0;
int last_msg_num = 0;
// do not remove after testing ===
bool open_loop_testing = false; //make true to set system to open loop control
#define OPEN_LOOP_TARGET_PERCENT 18

// Hard ware Specifiers =========================================================
/* way low... but will be corrected by trim pot calibrated on 3/10 */
#define MAX_PHYSICAL_LIMIT 3418 //3890 // 100% val
#define MIN_PHYSICAL_LIMIT 387 // 0 % val TODO measure this exactly for precise idle
#define REST_PCT 7.433190 // pct of where the valve will sit without any PWM signal

// TODO: Whhat?
#define MAX_INT_INPUT 0xFFF // MAX is 4095 = 3V3 ?
#define MAX_DUTY_CYCLE 1000.0f
#define TIM1_17_ARR MAX_DUTY_CYCLE
#define SAMPLES_PER_CHANNEL 256

//Software Flags ================================================================
static volatile bool tps_ready = false;
static volatile bool pid_ready = false;
static volatile bool shutdown_req = false;

static uint32_t tps_buffer[4];
static volatile bool trim_sample_fresh = false;
static bool ready_to_drive = false;

static double pot1_d = static_cast<double>(tps_buffer[0]);
static double pot2_d = static_cast<double>(tps_buffer[1]);

// PID ==========================================================================
#define INTERVAL_MS (int32_t)1
// Output from PID controller
static double pid_out;
// Double representation of set point- recall that it's in percentage,
static double set_pct_d = IDLE_PCT;
// Pointer to double representation of set point
static double *set_pct_ptr = &set_pct_d;
// double for pct_throttle_output
static double pct_pot1_d = IDLE_PCT;
// Pointer to the pct of pot 1
static double *pct_pot1_ptr = &pct_pot1_d;
// Create PID controller object
PID throttlePID(pct_pot1_ptr, &pid_out, set_pct_ptr, P, I, D, DIRECT);

// CAN ===========================================================================

#ifndef CAN_PEDAL_POS
#define CAN_PEDAL_POS 0x7E7
#endif

#ifndef CAN_READY_TO_DRIVE
#define CAN_READY_TO_DRIVE 0x7E6
#endif

// FDCAN Defines
static FDCAN_TxHeaderTypeDef tx_header;
//static FDCAN_RxHeaderTypeDef rx_header;
//static uint8_t rx_data[8];

#include "fdcan_queue.hpp"
// Private function prototypes
static FDCANBuffer fdcan_queue(3); // Queue to process non-critical tasks

// static std::queue<Error> error_queue;    // Queue to process errors

// Function prototypes ============================================================
static void controlMotor(double control_signal);
static void stopMotor(void);
static Error handleError(Error code);
static Error handleThrottle(CANMessage *msg);
//static Error handleStatusReport(void);
static Error processCANMessage(CANMessage *msg);
static void myprintf(const char *fmt, ...);
static double getSetpointSteps(float percentage);
static void applyPWM(uint16_t  duty, bool forward);
static double applyThrottleDeadbandComp(double control_signal);

// Functions ======================================================================
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  // Cast adc to double to used as PID parameters
  if (hadc->Instance == ADC1) {
    pot1_d = tps_buffer[0];
    pot2_d = tps_buffer[1];
    trim_sample_fresh = true;
  }
}

// Sample at fixed intervals: 10ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  static int last, current; // Ignore this error
  if (htim->Instance == TIM2) {
    current = HAL_GetTick();
    pid_ready = true;
    last = current;

    // ADC flag
    tps_ready = true;
  }
  // Ready to drive watchdog
  if(htim->Instance == TIM4) {
	  ready_to_drive = false;
	  //stopMotor();
	  set_pct_d = IDLE_PCT; // go to idle
	  //myprintf("BARK!\n");
	  // this timer is started by processCANMessage on ready-to-drive heartbeat
	  HAL_TIM_Base_Stop_IT(&htim4);
	  __HAL_TIM_SET_COUNTER(&htim4, 0);
  }
}


static void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)buffer, len, -1);
}

// Process commands sent to the throttle control board
static Error processCANMessage(CANMessage *msg) {

  Error code = ok;

  switch (msg->rx_header.Identifier) {
    case CAN_READY_TO_DRIVE:
      // ready to drive heartbeat
      //myprintf("got heartbeat, petting watchdog\n");
      ready_to_drive = true;
      HAL_TIM_Base_Stop_IT(&htim4);
      __HAL_TIM_SET_COUNTER(&htim4, 0); // pet the watchdog
      HAL_TIM_Base_Start_IT(&htim4);
      break;
  
    case CAN_PEDAL_POS:
      code = handleThrottle(msg);
      break;

    default: // Invalid command, return error
      code = invalid_command;
      break;
  }

  // If there is an error, then add it to the queue
  if (code != ok) {
    //    error_queue.push(code);
  }

  return code;
}

static Error handleError(Error code) {
  // Handle errors based on the error code
  switch (code) {
  case ok:
    return ok;  // No error, do nothing
  case generic: // Try to send error over CAN, but don't retry here since
                // fdcanWrite already retries
    if (fdcanWrite(&hfdcan1, tx_header, (uint8_t *)&code, ERROR_DLC,
                   throttle_control_board, from, DEFAULT_PRIORITY,
                   send_error) != HAL_OK) {
      return fdcan_tx_failure; // FDCAN failure
    }
    break;
  case mismatch_dlc: // DLC mismatch try to send error over CAN, but don't retry
                     // here since fdcanWrite already retries
    if (fdcanWrite(&hfdcan1, tx_header, (uint8_t *)&code, ERROR_DLC,
                   throttle_control_board, from, DEFAULT_PRIORITY,
                   send_error) != HAL_OK) {
      return fdcan_tx_failure; // FDCAN failure
    }
    break;
  case invalid_command: // Invalid command, try to send error over CAN, but
                        // don't retry here since fdcanWrite already retries
    if (fdcanWrite(&hfdcan1, tx_header, (uint8_t *)&code, ERROR_DLC,
                   throttle_control_board, from, DEFAULT_PRIORITY,
                   send_error) != HAL_OK) {
      return fdcan_tx_failure; // FDCAN failure
    }
    break;
  case fdcan_init_failure: // Try to re-init
    if (fdcanInit(&hfdcan1) != HAL_OK) {
      return fdcan_init_failure; // FDCAN failure
    }
    break;
  case fdcan_rx_failure: // Try to re-init
    if (fdcanInit(&hfdcan1) != HAL_OK) {
      return fdcan_rx_failure; // FDCAN failure
    }
    break;
  case fdcan_tx_failure: // Try to re-init
    if (fdcanInit(&hfdcan1) != HAL_OK) {
      return fdcan_tx_failure; // FDCAN failure
    }
    break;
  case adc_failure: // Try to re-init
    if (HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t *>(tps_buffer),
                          2) != HAL_OK) {
      return adc_failure; // ADC failure
    }
    break;
  default: // Unknown error, try to send error over CAN, but don't retry here
           // since fdcanWrite already retries
    if (fdcanWrite(&hfdcan1, tx_header, (uint8_t *)&code, ERROR_DLC,
                   throttle_control_board, from, DEFAULT_PRIORITY,
                   send_error) != HAL_OK) {
      return generic; // FDCAN failure
    }
    break;
  }
  return ok;
}

#define NUM_SAMPLES 1
uint32_t samples[NUM_SAMPLES];
uint32_t idx = 0;
long double total = 0;
// filter for throttle taps
static double filterThrottleTaps(double raw_taps) {
  static double filtered_taps = MIN_PHYSICAL_LIMIT;
  const double alpha = 0.15;   // lower = smoother, slower

  total -= samples[idx];
  samples[idx] = raw_taps;
  total += samples[idx];
  idx = (idx + 1) % NUM_SAMPLES;
  filtered_taps = total / NUM_SAMPLES;


  // clamp first
  if (raw_taps <= MIN_PHYSICAL_LIMIT) {
    filtered_taps = MIN_PHYSICAL_LIMIT;
  } else if (raw_taps >= MAX_PHYSICAL_LIMIT) {
    filtered_taps = MAX_PHYSICAL_LIMIT;
  } else {
    filtered_taps = (raw_taps * alpha) + ((1.0f - alpha) * filtered_taps);
  }

  return filtered_taps;
}
// take throttle taps filter them and turn them into percentage
static double throttleTaps2Pct(double throttle_adc_taps) {
  double filtered_taps = filterThrottleTaps(throttle_adc_taps);
  double adc_range = (double)(MAX_PHYSICAL_LIMIT - MIN_PHYSICAL_LIMIT);

  double throttle_percentage = ((filtered_taps - MIN_PHYSICAL_LIMIT) / adc_range) * 100.0;

  //myprintf("taps filtered: %f, pct: %f \r\n", filtered_taps, throttle_percentage);
  return throttle_percentage;
}

static Error handleThrottle(CANMessage *msg) {

  // Process received data - Only 1 byte for throttle percentage
  uint16_t throttle_adc_taps = (((uint16_t)msg->data[1] << 8)) | msg->data[0];
  msg_num = (((uint16_t)msg->data[3] << 8)) | msg->data[2];
  float throttle_percentage = (throttle_adc_taps / 4095.0) * 100.0;

  /* Uncomment when READY to DRIVE +++++++++++++++++++++++++++++++
  if(ready_to_drive) {
    if (throttle_percentage <= 0.0) { // no throttle signal = idle
      set_pct_d = IDLE_PCT;
	} else if (throttle_percentage >= 100.0) { // clamp over 100%
	  set_pct_d = 100.0;
	} else { // scale from 0-100 to idle percent to 100
	  set_pct_d = IDLE_PCT + (throttle_percentage * (100.0 - IDLE_PCT) / 100.0);
	}
  } else {
   set_point_d = IDLE_PCT
  }++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
  if (throttle_percentage <= 0.0) { // no throttle signal = idle
      set_pct_d = IDLE_PCT;
  } else if (throttle_percentage >= 100.0) { // clamp over 100%
      set_pct_d = 100.0;
  } else { // scale from 0-100 to idle percent to 100
      set_pct_d = IDLE_PCT + (throttle_percentage * (100.0 - IDLE_PCT) / 100.0);
  }
  return ok;
}

static void stopMotor(void) {
  // applyPWM will turn these back on when ready_to_drive = true;
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);  // PWM_HIGH off
  HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1); // PWM_LOW off
}

static void applyPWM(uint16_t duty, bool forward) {
	if (forward) {
	  HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);           // PWM_LOW off
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty); // PWM_HIGH duty
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	} else {
	  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);             // PWM_HIGH off
	  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, duty); // PWM_LOW duty
	  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	}
}

static double applyThrottleDeadbandComp(double control_signal) {
    if (pct_pot1_d > REST_PCT) {
        // Above resting point, shift output toward open deadband
        throttlePID.SetOutputLimits(
            -1.0 * (TIM1_17_ARR + PWM_DEADBAND_OPEN),
             1.0 * (TIM1_17_ARR - PWM_DEADBAND_OPEN)
        );
            return control_signal + PWM_DEADBAND_OPEN;
    } else {  //(pot1_d_pct < REST_PCT)
        // Below resting point, shift output toward close deadband
        throttlePID.SetOutputLimits(
            -1.0 * (TIM1_17_ARR - PWM_DEADBAND_CLOSE),
             1.0 * (TIM1_17_ARR + PWM_DEADBAND_CLOSE)
        );
        return control_signal - PWM_DEADBAND_CLOSE;
    }
}

// PWM LOW = TIM17 CH1
// PWM HIGH = TIM1 CH1
/**
 * @brief Sets motor direction and speed based on PID output, respecting
 * physical limits.
 * @param pid_output PID controller output (positive for forward, negative for
 * reverse).
 */
// In terms of ADC POT1 steps

static void controlMotor(double control_signal) {
	control_signal = applyThrottleDeadbandComp(control_signal);

  // Prevent driving beyond physical limits
  if ((pot1_d <= MIN_PHYSICAL_LIMIT && control_signal < 0.0) ||
      (pot1_d >= MAX_PHYSICAL_LIMIT && control_signal > 0.0)) {
    stopMotor();
    return;
  }

  const bool forward = (control_signal >= 0.0);
  const double deadband_pwm = forward ? PWM_DEADBAND_OPEN : PWM_DEADBAND_CLOSE;
  double duty = fabs(control_signal);

  if (duty > (uint16_t)MAX_DUTY_CYCLE) {
    duty = (uint16_t)MAX_DUTY_CYCLE;
  }

  //testing
  if (forward){
	  PWM_test = duty;
  }else{
	  PWM_test = -1*duty;
  }

  applyPWM(duty, forward);
}


// main =======================================================================
int alt_main(void) {
  HAL_GPIO_WritePin(HBRIDGE_EN_GPIO_Port, HBRIDGE_EN_Pin,
	                    GPIO_PIN_RESET); // Disable H-bridge at start

  set_pct_d = IDLE_PCT;

  /* Initialization */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  if (HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t *>(tps_buffer), 4) !=
      HAL_OK) {
  }
  //    error_queue.push(adc_failure); // ADC failure

  if (fdcanInit(&hfdcan1) != HAL_OK) {
  }
  //    error_queue.push(fdcan_init_failure); // FDCAN failure

  if (fdcanFilterInit(&hfdcan1, &tx_header) != HAL_OK) {
  }
  //    error_queue.push(fdcan_init_failure); // FDCAN failure

  throttlePID.SetMode(AUTOMATIC);
  throttlePID.SetSampleTime(INTERVAL_MS);
  throttlePID.SetOutputLimits(
      -1 * static_cast<double>(TIM1_17_ARR - PWM_DEADBAND_CLOSE),
      static_cast<double>(TIM1_17_ARR - PWM_DEADBAND_OPEN)); // Normalized limits: to ARR limits with deadbands

  HAL_GPIO_WritePin(HBRIDGE_MODE2_GPIO_Port, HBRIDGE_MODE2_Pin,
                    GPIO_PIN_SET); // Set to PWM/PWM mode
  HAL_GPIO_WritePin(HBRIDGE_DECAY_GPIO_Port, HBRIDGE_DECAY_Pin,
                    GPIO_PIN_RESET); // Set Decay to motor mode
  HAL_GPIO_WritePin(HBRIDGE_OCPM_GPIO_Port, HBRIDGE_OCPM_Pin,
                    GPIO_PIN_SET); // Set OCPM to auto recovery

  HAL_GPIO_WritePin(HBRIDGE_EN_GPIO_Port, HBRIDGE_EN_Pin,
                    GPIO_PIN_SET); // Enable HBridge

  /* start the CAN watchdog */
  //HAL_TIM_Base_GetState(&htim4);
  HAL_TIM_Base_Start_IT(&htim4);

  //set tunings before entering forever loop
  throttlePID.SetTunings(P, I, D);

  /* Super loop */
  while (1) {

    // Process CAN messages in the queue
	if (fdcan_queue.size() != 0) {

		CANMessage msg;
	    fdcan_queue.get(0, msg);

	    Error code = processCANMessage(&msg);

	    if (code != ok) {
	    	myprintf("Error processing CAN message: %d\r\n", code);
	    	handleError(code);
	    }
	}

	// Do PID  ====================================
	// convert output (adc) to percentage
	pct_pot1_d = throttleTaps2Pct(pot1_d);
    throttlePID.Compute(); // compute pid_out

    if (open_loop_testing){
    	set_pct_d = OPEN_LOOP_TARGET_PERCENT;
    	controlMotor(set_pct_d);
    } else {
    	controlMotor(pid_out);
    }

    // logging
    if ((last_msg_num != msg_num) || (msg_num == 0) ){ // only print unique messages except for 0
    	myprintf("%d | R(s): %f, H(s): %f, PWM: %i, \r\n", msg_num , set_pct_d, pct_pot1_d, PWM_test);
    	last_msg_num = msg_num;
    }

  }
  // my_shutdown(); // Shutdown system
}

/**
 * NOTE: probably none of this is accurate - Caleb
 * @brief  Callback function for handling received messages in FDCAN Rx FIFO 0.
 * @param  hfdcan: Pointer to the FDCAN handle.
 * @param  RxFifo0ITs: Interrupt flags for Rx FIFO 0.
 * @retval None
 *
 * This function is called when a new message is received in the FDCAN Rx FIFO
 * 0. It processes the received data, calculates the throttle position
 * percentage, determines the set point in terms of ADC levels, computes the
 * duty cycle percentage using a PID controller, and updates the PWM duty cycle
 * accordingly. At this stage, all messages are filtered to have a direction bit
 * = 0 (To Module)
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
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if (hfdcan->Instance == FDCAN1) {
    // If we are receiving a CAN signal and its NOT in reset, then get message
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
      FDCAN_RxHeaderTypeDef rxHeader; // Struct to place data
      uint8_t rxData[8]; // 8 bit number expressed as a percentage -> 0 - 100%

      // Get Rx messages from RX FIFO0
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) !=
          HAL_OK) {
      }
      //        error_queue.push(fdcan_rx_failure); // FDCAN RX failure

      // Activate notification again in case HAL deactivates interrupt
      if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                         0) != HAL_OK) {
      };
      //        error_queue.push(fdcan_rx_failure); // FDCAN RX failure

      // Process critical commands

      /* NOTE: you should not need to do this but memcpy not worky ¯\_(ツ)_/¯ */
      FDCAN_RxHeaderTypeDef msgHeader = {
          .Identifier = rxHeader.Identifier,
          .IdType = rxHeader.IdType,
          .RxFrameType = rxHeader.RxFrameType,
          .DataLength = rxHeader.DataLength,
          .ErrorStateIndicator = rxHeader.ErrorStateIndicator,
          .BitRateSwitch = rxHeader.BitRateSwitch,
          .FDFormat = rxHeader.FDFormat,
          .RxTimestamp = rxHeader.RxTimestamp,
          .FilterIndex = rxHeader.FilterIndex,
          .IsFilterMatchingFrame = rxHeader.IsFilterMatchingFrame,

      };

      CANMessage msg = {
          .rx_header = msgHeader,
      };

      for (int i = 0; i < 8; i++) {
        msg.data[i] = rxData[i];
      }
      fdcan_queue.push(msg);
    }
  }
}
