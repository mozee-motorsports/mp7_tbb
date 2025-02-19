/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * Info:
  * 		CLK: 170 MHz
  * 		PSC: 170
  *
  * 		Fclk: 170 MHz/ 170 = 1MHz
  * 		Desired frequency: 10 kHz
  * 		ARR: F_CLK / Desired frequency = 1 MHz/ 10kHz = 100 clock cycles
  * 		Duty = Period %: CRR/ARR * 100... TIMM11 ->CCR1 = ....
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include "pid.h"
#include "fdcan_queue.h"
#include "fdcan.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_ADC_OUTPUT 0xFFF			// MAX is 4095 = 3V3
#define MAX_DUTY_CYCLE 100.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
static int32_t set_point = MAX_ADC_OUTPUT;				// Store set point, set in FDCAN ISR
int32_t tps_buffer[1];

static PID_TypeDef pid;
static int32_t pid_out;
#define KP 1.0f
#define KI 1.0f
#define KD 1.0f

// FDCAN Defines
FDCAN_TxHeaderTypeDef   tx_header;
FDCAN_RxHeaderTypeDef   rx_header;
uint8_t               tx_data[8];
uint8_t               rx_data[8];

// Queue to process non-critical tasks
Queue fdcan_queue;

// State variable to keep track of state
uint8_t error_code = 0x00;	// 0 = OK, anything else = error

volatile bool adc_dma_processed = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static float PID_Controller(uint16_t set_point, uint16_t throttle_position);
static uint8_t handleError(uint8_t code);
static uint8_t handleThrottle(CANMessage* msg);
static uint8_t handleCalibration(void);
static uint8_t handleStatusReport(void);
static void processCANMessage(CANMessage* msg, Command command);
static void myprintf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void myprintf(const char *fmt, ...) {
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, len, -1);

}

static void processCANMessage(CANMessage* msg, Command command)
{
	// Execute sent command
	switch(command)
	{
		case shutdown:				// Already handled in ISR - here also to remove warning
			exit(EXIT_SUCCESS);
			break;
		case throttle_percentage:
			if(rx_header.DataLength != DLC_THROTTLE)
				error_code = handleError(GENERIC_ERROR_CODE);
			else
				error_code = handleThrottle(msg);

			break;
		case calibrate:
			if(rx_header.DataLength != DLC_CALIBRATE)
				error_code = handleError(GENERIC_ERROR_CODE);
			else
				error_code = handleCalibration();
			break;
		case status_report:
			if(rx_header.DataLength != DLC_STATUS_REPORT)
				error_code = handleError(GENERIC_ERROR_CODE);
			else
				error_code = handleStatusReport();
			break;
		default:	// Not a valid command, assume error
			error_code = handleError(GENERIC_ERROR_CODE);
			break;
	}
}

/**
 * Determines appropriate response given the error code. All error codes sent to control board master.
 * Currently only 1 error code defined - GENERIC_ERROR_CODE = 0xFF.
 */
static uint8_t handleError(uint8_t code)
{
	// When there is an error determine response
	switch(code)
	{
	case GENERIC_ERROR_CODE: // Set throttle to zero
		// An invalid command was sent, reset set point
		set_point = 0;
		PID_Set_Setpoint(&pid, &set_point);
		break;
	}

	// All codes need to be sent to ETC
	// Send error code - TODO verify error codes
	tx_data[0] = code;
	fdcanWrite(&hfdcan1, tx_header, tx_data, DLC_ERROR, throttle_control_board, from, DEFAULT_ETC_PRIORITY, error);

	// Always return ERROR
	return GENERIC_ERROR_CODE;
}

/**
 * Gets the throttle position percentage from the current message in the queue, calculates the set point,
 * and sets it.
 */
static uint8_t handleThrottle(CANMessage* msg)
{
	// Process received data - Only 1 byte for throttle percentage
	uint8_t throttle_position_percentage = msg->data[0];

	// Get position in terms of ADC levels based on percent.
	set_point = roundf(((float) throttle_position_percentage / MAX_DUTY_CYCLE) * MAX_ADC_OUTPUT);

	// Set PID structure set point from incoming data
	PID_Set_Setpoint(&pid, &set_point);

	return 0;
}

/*
 * Returns battery level of Throttle Control board by sampling with ADC - 0xFF if not implemented
 */
static uint8_t handleStatusReport(void)
{
	// Assuming not implemented for the time being
	tx_data[0] = 0xFF;
	tx_data[1] = 0xFF;
	tx_data[2] = error_code;		// Report any errors

	// Write status report
	return fdcanWrite(&hfdcan1, tx_header, tx_data, DLC_STATUS_REPORT, throttle_control_board, from, DEFAULT_ETC_PRIORITY, status_report);
}

/**
 * Currently not implemented, always return OK
 */
static uint8_t handleCalibration(void)
{
	return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Init canfd instance
  if (fdcanInit(&hfdcan1) != HAL_OK)
	  Error_Handler();

  // Sets up the CANFD filter
  if (fdcanFilterInit(&hfdcan1, &tx_header) != HAL_OK)
	  Error_Handler();

  // Init queue
  queueInit(&fdcan_queue);

  // Init PID structure - tps buffer in terms of adc steps, and so is set_point. pid_out also in terms of adc steps
  PID(&pid, &tps_buffer[0], &pid_out, &set_point, KP, KI, KD, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&pid, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&pid, 100);
  PID_SetOutputLimits(&pid, 0, MAX_ADC_OUTPUT);

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		set_point = MAX_ADC_OUTPUT;   			// incoming is 50% throttle
		PID_Set_Setpoint(&pid, &set_point);

		static double position_delta = 0.0;

		if(error_code != 0)
		{
			// Shutdown
		    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		}

		// While the queue isn't empty, perform command
		if(!isEmpty(&fdcan_queue))
		{
			// Get head element, and extract command
			CANMessage msg = peek(&fdcan_queue);
			Command command = msg.rx_header.Identifier & 0x0F;
			processCANMessage(&msg, command);

			// Remove element
			dequeue(&fdcan_queue);
		}

		// Perform pid only if there is a tps value
		if(!adc_dma_processed)
		{

			// What is the distance to set point? In terms of ADC steps.
			position_delta = set_point - tps_buffer[0];

			// Set point is expected to change very rapidly
			PID_Compute(&pid);

			// Duty variable for PWM output - recall it goes 0 -> 99
			static uint8_t duty_numerical = 0;

			// PWM_HIGH = PC0 = TIM1_CH1
			// PWM_LOW = PA7 = TIM3_CH2
			if (pid_out < 0) {// Set to backward by stopping PC0 and writing duty cycle to PA7

				// Stop PC0
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

			    // Convert absolute value of pid_out to duty cycle
			    double duty_cycle = ((fabs(pid_out) / MAX_ADC_OUTPUT) * MAX_DUTY_CYCLE);
			    if (duty_cycle > 99) duty_cycle = 99;
			    uint8_t duty_numerical = (uint8_t)duty_cycle;

				// Set duty cycle for PA7
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_numerical);

				// Enable output for PA7
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			} else {// Set forward by writing duty cycle to PC0 and setting PA7 to ground

				// Stop PA7
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);

			    double duty_cycle = ((pid_out/ MAX_ADC_OUTPUT) * MAX_DUTY_CYCLE);
			    if (duty_cycle > 99) duty_cycle = 99;
			    uint8_t duty_numerical = (uint8_t)duty_cycle;

				// Set duty cycle for PC0
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_numerical);

				// Enable output for PC0
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			}
			adc_dma_processed = true;
		}


	  //uint8_t duty_numerical = (uint8_t)((delta / MAX_DUTY_CYCLE) * htim1.Init.Period);
//	  // Ensure it stays within bounds
//	  if (duty_numerical > htim1.Init.Period)
//		  duty_numerical = htim1.Init.Period;
//	  else if (duty_numerical < 0)
//		  duty_numerical = 0;


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)tps_buffer, 1);

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  // Enable interrupts for CAN reception
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 1;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 170-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Base_Start(&htim2);

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 170-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ADC Interrupt handler - To quantize 3.3V throttle position from TPS sensor */
/**
 * @brief ADC conversion complete callback.
 *
 * This function is called when the ADC conversion is complete. It checks if the
 * ADC instance is ADC1, retrieves the 12-bit ADC value from the throttle position
 * sensor (TPS), and restarts the ADC interrupt.
 *
 * @param hadc Pointer to the ADC handle.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1)
	{
		// Get 12-bit ADC value from TPS sensor. 2^12 - 1 = 4095 max value
		// Reset flag variable
		adc_dma_processed = false;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)tps_buffer, 1);
	}
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
      uint8_t rxData[8];              // 8 bit number expressed as a percentage -> 0 - 100%

      // Get Rx messages from RX FIFO0
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        Error_Handler(); // Reception Error

      // Activate notification again in case HAL deactivates interrupt
      if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        Error_Handler(); // Notification Error

      // Extract command
      Command command = (Command)(rx_header.Identifier & 0x0F);

      // Process critical commands
      if (command == shutdown)
        exit(EXIT_SUCCESS);

      // Queue non-critical commands
      CANMessage msg = {rx_header, {0}};

      // Make deep copy of payload
      memcpy(msg.data, rx_data, sizeof(rx_data));

      // Add message to queue
      enqueue(&fdcan_queue, &msg);
    }
  }
}

/**
 * @brief PID Controller function to calculate the PWM duty cycle.
 *
 * This function implements a PID controller to adjust the throttle position
 * based on the given set point. It calculates the error, integral, and
 * derivative terms to compute the PID output, which is then converted to
 * a PWM duty cycle percentage.
 *
 * @param set_point The desired set point value.
 * @param throttle_position The current throttle position value.
 * @return uint8_t The calculated PWM duty cycle percentage (0-100%).
 *
 * The function performs the following steps:
 * 1. Calculates the error between the set point and the throttle position.
 * 2. Updates the integral term by adding the current error.
 * 3. Clamps the integral term to prevent integral windup.
 * 4. Calculates the derivative term based on the change in error.
 * 5. Computes the PID output using the proportional, integral, and derivative terms.
 * 6. Updates the previous error for the next iteration.
 * 7. Converts the PID output to a PWM duty cycle percentage.
 * 8. Clamps the PWM duty cycle to the range of 0-100%.
 */

static float integral_prev = 0.0f;
static uint32_t time_prev = 0;
static int32_t error_prev = 0;

// PID constants - Change for tuning
//static const float KP = 1.0f;
//static const float KI = 0.0f;
//static const float KD = 0.00f;
static const float MICRO_TO_S = 0.000001f;

static float PID_Controller(uint16_t set_point, uint16_t throttle_position) {
	uint32_t current_tick = __HAL_TIM_GET_COUNTER(&htim2);

	// Convert timer ticks into seconds
	// Each tick is 1 microsecond (1e-6 seconds)
	float dt;

	// IF current_tick >= time_prev -> set dt in terms of seconds, ELSE, Handle timer overflow
	dt = (float) (current_tick - time_prev) * MICRO_TO_S; // Convert microseconds to seconds
														  // maybe remove this?

	// Make sure there is no 0 dt
	if (!dt)
		dt = 1;

	int32_t error = ((int32_t) set_point) - ((int32_t) throttle_position);

	// Calculate the integral
	float integral = integral_prev + ((error + error_prev) / 2.0f) * dt;

	float derivative = (error - error_prev) / dt;

	// Prevent integral windup by clamping the integral term
	if (integral > MAX_ADC_OUTPUT) {
		integral = MAX_ADC_OUTPUT;
	} else if (integral < -MAX_ADC_OUTPUT) {
		integral = -MAX_ADC_OUTPUT;
	}

	// Calculate the PID output
	float output = (KP * error) + (KI * integral) + (KD * derivative);

	// Convert the PID output to a PWM duty cycle percentage
	int32_t pwm_delta = (output / MAX_ADC_OUTPUT) * MAX_DUTY_CYCLE;

	integral_prev = integral;
	time_prev = current_tick;
	error_prev = error;

	// Return duty cycle percentage
	return pwm_delta;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
