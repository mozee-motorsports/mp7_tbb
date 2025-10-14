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

// Aggressive tuning parameters for large error
#define AGR_KP 0.11f
#define AGR_KI 0.02f
#define AGR_KD 0.003f

// Non-adaptive tuning parameters
#define KP AGR_KP
#define KI AGR_KI
#define KD AGR_KD

// Medium tuning parameters for medium errors
#define MED_KP 1.0f
#define MED_KI 0.0f
#define MED_KD 0.0f

// Conservative tuning parameters for small errors IDLE TUNE
#define CONS_KP 0.25f
#define CONS_KI 0.4f
#define CONS_KD 0.0009f

#define OPEN_ADC_STEPS                                                         \
  3600 // Measured ADC steps for fully open - pots may measure farther
#define IDLE_PCT 5

// TODO: Whhat?
#define MAX_ADC_OUTPUT 0xFFF // MAX is 4095 = 3V3 ?
#define MAX_DUTY_CYCLE 100.0f
#define TIM1_17_ARR MAX_DUTY_CYCLE
#define SAMPLES_PER_CHANNEL 256

static volatile bool tps_ready = false;
static volatile bool pid_ready = false;
static volatile bool shutdown_req = false;

static uint32_t tps_buffer[4];
static volatile bool trim_sample_fresh = false;

static uint16_t set_point = IDLE_PCT; // Percentage

// Output from PID controller
static double pid_out;

// Double representation of set point- recall that it's in terms of ADC steps,
// but library requires double type
static double set_point_d = set_point;
// Pointer to double representation of set point
static double *set_ptr = &set_point_d;

static double pot1_d = static_cast<double>(tps_buffer[0]);
static double pot2_d = static_cast<double>(tps_buffer[1]);

// Create PID controller object
PID throttlePID(&pot1_d, &pid_out, set_ptr, KP, KI, KD, DIRECT);

#define INTERVAL_MS (int32_t)10
#define MAX_PHYSICAL_LIMIT 3890

/* way low... but will be corrected by trim pot */
#define MIN_PHYSICAL_LIMIT 300 

static double min_limit_trimmed = MIN_PHYSICAL_LIMIT;

// FDCAN Defines
static FDCAN_TxHeaderTypeDef tx_header;
//static FDCAN_RxHeaderTypeDef rx_header;
//static uint8_t rx_data[8];

#include "fdcan_queue.hpp"
// Private function prototypes
static FDCANBuffer fdcan_queue(3); // Queue to process non-critical tasks

// static std::queue<Error> error_queue;    // Queue to process errors

// Function prototypes
static void controlMotor(double control_signal, int32_t position_delta);
static void stopMotor(void);
static Error handleError(Error code);
static Error handleThrottle(CANMessage *msg);
static Error handleStatusReport(void);
static Error processCANMessage(CANMessage *msg, Command command);
static void my_shutdown(void);
static void myprintf(const char *fmt, ...);
static double getSetpointSteps(float percentage);
static void applyPWM(uint8_t duty, bool forward);

static bool ready_to_drive = false;

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
  static int last, current;
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
	  set_point_d = getSetpointSteps(IDLE_PCT); // go to idle
	  //myprintf("BARK!\n");
	  // this timer is started by processCANMessage on ready-to-drive heartbeat
	  HAL_TIM_Base_Stop_IT(&htim4);
	  __HAL_TIM_SET_COUNTER(&htim4, 0);
  }
}

static void my_shutdown(void) {
  stopMotor();
  exit(1);
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

typedef enum {
    SAFETY_SYSTEM = 0,
    BROADCAST = 1,
    THROTTLE_CONTROL_BOARD = 2,
    PEDAL_BOX = 3,
    STEERING_WHEEL = 4,
    THERMO_CONTROL_BOARD = 5,
    ISOLATION_EXPANSION_DEVICE = 6,
} eModule;

typedef enum {
    TO = 0,
    FROM = 1,
} eDirection;

typedef struct {
    uint8_t priority;
    eModule module;
    eDirection direction;
    uint8_t command;
} sCAN_Header;


sCAN_Header parse_id(uint32_t id) {
    return sCAN_Header {
        .priority = (uint8_t)((id >> 8) & 0b0111),
        .module  = (eModule)((id >> 5) & 0b0111),
        .direction = (eDirection) ((id >> 4) & 0b01),
        .command = (uint8_t)(id & 0xF),
    };
}

uint32_t header2id(sCAN_Header header) {
    uint32_t id = 0;
    id |= (header.priority << 8);
    id |= (header.module << 5);
    id |= (header.direction << 4);
    id |= header.command;
    return id;
}
// Process commands sent from the control board master to the throttle control
// board
static Error processCANMessage(CANMessage *msg, Command command) {

  sCAN_Header id = parse_id(msg->rx_header.Identifier);
  if (id.module == BROADCAST && command == 0) {
	  // ready to drive heartbeat
	  //myprintf("got heartbeat, petting watchdog\n");
	  ready_to_drive = true;
	  HAL_TIM_Base_Stop_IT(&htim4);
	  __HAL_TIM_SET_COUNTER(&htim4, 0); // pet the watchdog
	  HAL_TIM_Base_Start_IT(&htim4);
	  return ok;
  }

  // Execute sent command
  Error code = ok;
  switch (command) {
  case shutdown: // Already handled in the ISR
    // Shutdown command, exit program
    // my_shutdown();
    break;
  case throttle_percentage:
    code = handleThrottle(msg);
    break;
  case send_error:
    code = handleError((Error)(msg->data[0]));
    break;
  case status_report:
    // code = handleStatusReport();
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

static double getSetpointSteps(float percentage) {
  // Unsigned integer, so negatives roll over to MAX - 1, if percentage is
  // greater than 100, clamp to fully open
  double duty = percentage < 100
                    ? static_cast<double>(roundf((percentage / MAX_DUTY_CYCLE) *
                                                 OPEN_ADC_STEPS))
                    : OPEN_ADC_STEPS;
  if (!duty)
    return min_limit_trimmed;
  else
    return duty < min_limit_trimmed ? min_limit_trimmed : duty;
}

static Error handleThrottle(CANMessage *msg) {


  // Process received data - Only 1 byte for throttle percentage
  uint16_t throttle_adc_taps = (((uint16_t)msg->data[1] << 8)) | msg->data[0];
  float throttle_percentage = (throttle_adc_taps / 4096.0) * 100.0;

  // Get position in terms of ADC levels based on percent.
  if(ready_to_drive) {
	  set_point_d = getSetpointSteps(throttle_percentage);
  } else {
	  set_point_d = getSetpointSteps(IDLE_PCT);
  }

  //  myprintf("throttle_percent = %f\n", throttle_percentage);
  //  myprintf("set_point_d = %lf\n", set_point_d);
  return ok;
}

/*
 * Returns battery level of Throttle Control board by sampling with ADC - 0xFF
 * if not implemented
 */
// static Error handleStatusReport(void)
//{
////  std::queue<Error> temp = error_queue; // Create a copy
//
//  // First verify dlc
//  if (rx_header.DataLength != STATUS_REPORT_DLC)
//    return mismatch_dlc;
//
//  // Check if there are any errors
//  while (!temp.empty())
//  {
//    Error error = temp.front(); // Get error from queue
//
//    // Setup packet to send
//    uint8_t tx_data[3] = {0xFF, 0xFF, (uint8_t)error};
//
//    // Send error, if we can't then return error
//    if (fdcanWrite(&hfdcan1, tx_header, tx_data, STATUS_REPORT_DLC,
//    steering_wheel, from, DEFAULT_PRIORITY, status_report) != HAL_OK)
//      return fdcan_tx_failure;
//
//    temp.pop(); // Remove error from queue
//  }
//
//  return ok;
//}
//

static void stopMotor(void) {
  // applyPWM will turn these back on when ready_to_drive = true;
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);  // PWM_HIGH off
  HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1); // PWM_LOW off
}

static void applyPWM(uint8_t duty, bool forward) {
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

// PWM LOW = TIM17 CH1
// PWM HIGH = TIM1 CH1
/**
 * @brief Sets motor direction and speed based on PID output, respecting
 * physical limits.
 * @param pid_output PID controller output (positive for forward, negative for
 * reverse).
 */
// In terms of ADC POT1 steps

static void controlMotor(double control_signal, int32_t position_delta) {

  // Prevent driving beyond physical limits
  if ((pot1_d <= min_limit_trimmed &&
       control_signal < 0) || // At min, trying to close further
      (pot1_d >= MAX_PHYSICAL_LIMIT &&
       control_signal > 0)) { // At max, trying to open further
    stopMotor();
    return;
  }

  // Calculate duty cycle from PID output
  const double duty_cycle = fabs(control_signal);
  const uint8_t duty = (duty_cycle > MAX_DUTY_CYCLE)
                           ? MAX_DUTY_CYCLE
                           : static_cast<uint8_t>(duty_cycle);

  applyPWM(duty, control_signal >= 0.0);

  //  myprintf("POT1 : %f POT2 : %f PID: %f DUTY: %i, set point: %f, delta:
  //  %i\r\n",
  //      pot1_d,
  //      pot2_d,
  //      control_signal,
  //      duty,
  //      set_point_d,
  //      position_delta);
}

int alt_main(void) {
  HAL_GPIO_WritePin(HBRIDGE_EN_GPIO_Port, HBRIDGE_EN_Pin,
	                    GPIO_PIN_RESET); // Disable H-bridge at start

  set_point_d = getSetpointSteps(set_point);

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
      -1 * static_cast<double>(TIM1_17_ARR),
      static_cast<double>(TIM1_17_ARR)); // Normalized limits: -100 - 100% - map
                                         // these directly to the pwm
  static double position_delta; // How close are we to the actual setpoint?

  //  if(HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) {
  //    error_queue.push(dac_init_failure);
  //  };
  //  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1670);

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
  /* Super loop */
  while (1) {
    // Process CAN messages in the queue

    if (trim_sample_fresh) {
      // set minimum position based on TRIM_1 
      double trimmer = (tps_buffer[2] / 4096.0)*350.0;
      min_limit_trimmed = MIN_PHYSICAL_LIMIT + trimmer; 
    }

    if (fdcan_queue.size() != 0) {

      CANMessage msg;
      fdcan_queue.get(0, msg);

      Error code = processCANMessage(
          &msg, static_cast<Command>((msg.rx_header.Identifier & 0x0F)));
      if (code != ok) {
        myprintf("Error processing CAN message: %d\r\n", code);
        handleError(code);
      }
    }

    if (pid_ready && tps_ready) {
      position_delta = set_point_d - pot1_d;

      if (set_point_d >= 650 && pot1_d >= 650) {
        throttlePID.SetTunings(AGR_KP, AGR_KI, AGR_KD);
        //myprintf("adap tunings: ");
      }

      if (set_point_d < 650 && pot1_d < 650) {
        throttlePID.SetTunings(CONS_KP, CONS_KI, CONS_KD);
        //myprintf("cons tunings: ");
      }


      myprintf("set_point_d: %lf\n", set_point_d);
      //myprintf("ready_to_drive: %d\n", ready_to_drive);
      //myprintf("set_point_d: %lf, pot1_d: %lf, min_limit: %lf\n", set_point_d, pot1_d, min_limit_trimmed);
      throttlePID.Compute();
      controlMotor(pid_out, position_delta);
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
