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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include<stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	SELF_TEST,
	INIT,
	MEASURE,
	CHARGING,
	CHARGING_W_CB,
	FAULT,
	SLEEP,
	SELF_DISCHARGE
} BMS_STATES;

typedef struct {
	float voltage;
	float current; // + values means I_out > I_in
	float hotspot_temp;
} PACK_DATA;

typedef struct {
	float voltage;
	float temperature;
} CELL_DATA;

typedef struct {
	uint16_t voltage;
	uint16_t temperature;
} CELL_DATA_RAW;

// 128 bytes (no byte stuffing)
typedef struct {
	bool over_voltage;
	bool over_power;
	bool over_temperature;
	bool under_voltage;
	uint16_t cell_over_voltage;
	uint16_t cell_under_voltage;
	uint16_t cell_over_temperature;
	uint16_t cell_under_temperature;
	bool discharging;
	bool fault_triggered;
	bool charging;
	bool sleep;
} PACK_FLAGS;

// 32 bytes (no byte stuffing)
typedef struct {
	bool over_voltage;
	bool under_voltage;
	bool over_temperature;
	bool under_temperature;
} CELL_FLAGS;

// 224 bytes
typedef struct {
	uint32_t timestamp_ms;
	uint32_t packet_counter;
	PACK_FLAGS flags;
	PACK_DATA data;
} CAN_DATA;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_TO_VOLT(n)	 	(float)(((float)n * 5) / (1 << 12)) // 5V reference, 12 bit precision
/* Linreg applied on data given in Cell Pack Table 2. moved to be centered on (1.51,60) */
#define VOLT_TO_DEG_C(n) 	(float)(-116.5918987*((float)n - 1.51) + 60)
#define MIN_VOLT			(float)2.5 // 2.5 Volts as per Cell Pack Table 1
#define MAX_MEASURE_VOLT	(float)4.2 // 4.2 Volts as per Cell Pack Table 1
#define MAX_CHARGING_VOLT	(float)4.2 // Sometimes this may be set differently (4.3V)
#define MAX_CELL_TEMP		(float)60  // 60 degC as per Cell Pack Table 1
#define MIN_CELL_TEMP		(float)-20  // -20 degC as per Cell Pack Table 1
#define MAX_PACK_POWER		(float)80*1000 // 80kW as per EV.3.3.1
#define MAX_PACK_VOLTAGE	(float)600 // 600V DC as per EV.3.3.2
#define NUM_IC				(uint8_t)1 // could be any value
#define CELL_PER_IC			(uint8_t)1 // could be any value <= 12
#define TIME_TO_SLEEP		(uint32_t)10 // time (in s) where I_sleep > I_out after which enter sleep mode (can be anything)
#define TIME_TO_CHARGE		(uint32_t)1 // time (in s) where I_out > I_in after which enter charge mode (can be anything)
#define SLEEP_CURRENT		(float)4 // I_sleep (can be any value)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

static BMS_STATES bms_state = SELF_TEST;
static PACK_DATA pack_data;
static CELL_DATA_RAW cell_data_raw[NUM_IC][CELL_PER_IC];
static CELL_DATA cell_data[NUM_IC][CELL_PER_IC];
static PACK_FLAGS pack_flags;
static CELL_FLAGS cell_flags[NUM_IC][CELL_PER_IC];
static CAN_DATA can_data;
static bool has_initialized;
static bool has_initialized_sleep;
static bool tim1_on;
static bool tim3_on;
static uint8_t current_cs;
static uint8_t current_cell[NUM_IC];
static uint32_t charging_timer;
static uint32_t sleep_timer;
static float highest_voltage;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void BMS_State_Machine(void);
static void Init_Peripherals(void);
static bool Peripheral_Check(void);
static bool Proper_Peripheral_Init(void);
static void Enable_TS(void);
static void Disable_TS(void);
static void Read_Current(void);
static void SPI_Read(void);
static void SPI_CS_Toggle(void);
static void Convert_Data(void);
static void Check_Data(void);
static void CAN_Send(void);
static void Enable_Low_Power(void);
static void Disable_Low_Power(void);
static bool Balance_Cells(void);
static void Discharge_Cell(void);
static void Discharge_Pack(void);
static void ConvertEndian(uint8_t *result, uint32_t original);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Begin State Machine
	  BMS_State_Machine();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
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
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_CS1_Pin|SPI2_CS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS1_Pin SPI2_CS2_Pin */
  GPIO_InitStruct.Pin = SPI2_CS1_Pin|SPI2_CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BALANCE_CELLS_Pin WAKE_Pin */
  GPIO_InitStruct.Pin = BALANCE_CELLS_Pin|WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DISCHARGE_Pin */
  GPIO_InitStruct.Pin = DISCHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DISCHARGE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Main state machine for the BMS where bms_state is the current state
static void BMS_State_Machine(void) {
	switch(bms_state) {
		case SELF_TEST:
			printf("TESTING\n\r");
			// To avoid repeated initializations
			if ( !has_initialized ) {
				Init_Peripherals();
			}

			// Check that peripherals are reachable, if so -> INIT
			if ( Peripheral_Check() ) {
				bms_state = INIT;
			}

			break;
		case INIT:

			if ( has_initialized_sleep ) {
				Disable_Low_Power();
			}

			// Check peripheral fault registers and configuration, if so -> MEASURE
			if ( Proper_Peripheral_Init() ) {
				Enable_TS();
				bms_state = MEASURE;
			}

			break;
		case MEASURE:
			if ( tim3_on ) {
				HAL_TIM_Base_Stop_IT(&htim3);  // Stop 5 ms timer
				tim3_on = false;
			}

			if ( !tim1_on ) {
				HAL_TIM_Base_Start_IT(&htim1);  // Start 1 ms timer
				tim1_on = true;
			}

			break;
		case CHARGING:
			break;
		case CHARGING_W_CB:
			break;
		case FAULT:

			if ( !pack_flags.fault_triggered ) {
				Disable_TS();
				pack_flags.fault_triggered = true;
			}

			if ( tim3_on ) {
				HAL_TIM_Base_Stop_IT(&htim3);  // Stop 5 ms timer
				tim3_on = false;
			}

			if ( !tim1_on ) {
				HAL_TIM_Base_Start_IT(&htim1);  // Start 1 ms timer
				tim1_on = true;
			}

			break;
		case SLEEP:

			if ( !has_initialized_sleep ) {
				Enable_Low_Power();
			}

			if ( tim1_on ) {
				HAL_TIM_Base_Stop_IT(&htim1);  // Stop 1 ms timer
				tim1_on = false;
			}

			if ( !tim3_on ) {
				HAL_TIM_Base_Start_IT(&htim3);  // Start 5 ms timer
				tim3_on = true;
			}

			break;
		case SELF_DISCHARGE:

			if ( !pack_flags.discharging ) {
				Disable_TS();
				Discharge_Pack();
				pack_flags.discharging = true;
			}

			if ( tim3_on ) {
				HAL_TIM_Base_Stop_IT(&htim3);  // Stop 5 ms timer
				tim3_on = false;
			}

			if ( !tim1_on ) {
				HAL_TIM_Base_Start_IT(&htim1);  // Start 1 ms timer
				tim1_on = true;
			}

			break;
	}
}

static void Init_Peripherals(void) {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

	has_initialized = true;
}

static bool Peripheral_Check(void) {
	SPI_Read();

	return true;
}

static bool Proper_Peripheral_Init(void) {
	return true;
}

static void Enable_TS(void) {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

static void Disable_TS(void) {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

static void Read_Current(void) {
	pack_data.current = (int16_t)htim4.Instance->CNT;
}

static void SPI_Read(void) {

	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, 100);

	cell_data_raw[0][0].voltage = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);

	HAL_ADC_Start(&hadc2);

	HAL_ADC_PollForConversion(&hadc2, 100);

	cell_data_raw[0][0].temperature = HAL_ADC_GetValue(&hadc2);

	HAL_ADC_Stop(&hadc2);
}

static void SPI_CS_Toggle(void) {

}

static void Convert_Data(void) {
	cell_data[0][0].voltage = ADC_TO_VOLT(cell_data_raw[0][0].voltage);
	cell_data[0][0].temperature = VOLT_TO_DEG_C(ADC_TO_VOLT(cell_data_raw[0][0].temperature));
}

static void Check_Data(void) {
	if ( cell_data[0][0].voltage >= MAX_MEASURE_VOLT ) {
		bms_state = FAULT;
		pack_flags.cell_over_voltage = 1;
		cell_flags[0][0].over_voltage = true;
		printf("MAX_VOLT\n\r");
	}

	if ( cell_data[0][0].voltage < MIN_VOLT ) {
		bms_state = FAULT;
		pack_flags.cell_under_voltage = 1;
		cell_flags[0][0].under_temperature = true;
		printf("MIN_VOLT\n\r");
	}

	if ( cell_data[0][0].temperature > MAX_CELL_TEMP ) {
		bms_state = FAULT;
		pack_flags.cell_over_temperature = 1;
		cell_flags[0][0].over_temperature = true;
		printf("TEMP\n\r");
	}
}

static void CAN_Send(void) {
	printf("State: %i, Current: %f, Voltage: %f, Temperature: %f\n\r", bms_state, pack_data.current, cell_data[0][0].voltage, cell_data[0][0].temperature);
}

static void Enable_Low_Power(void) {

}

static void Disable_Low_Power(void) {

}

static bool Balance_Cells(void) {
	return false;
}

static void Discharge_Cell(void) {

}

static void Discharge_Pack(void) {

}

static void ConvertEndian(uint8_t *result, uint32_t original) {
  // Example Edian Conversion for 4 byte read/writes where SPI sends over 16 bit words
  // Simpler for 2 byte, where upper byte swaps with lower byte
  // o3 o2 o1 o0 --> o2 o3 o0 o1 as r3 r2 r1 r0

  result[0] = (original >> 16) & 0xFF;
  result[1] = (original >> 24) & 0xFF;
  result[2] = original & 0xFF;
  result[3] = (original >> 8) & 0xFF;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if ( htim == &htim1 ) {
		// Rollover must be handled by master
		can_data.timestamp_ms++;
		can_data.timestamp_ms =
				(can_data.timestamp_ms == 0xFFFF) ? 0 : can_data.timestamp_ms;

		SPI_Read();
		Read_Current();

		Convert_Data();
		Check_Data();
		CAN_Send();

		if ( bms_state == CHARGING_W_CB ) {
			if ( Balance_Cells() ) {
				bms_state = CHARGING;
			}
		}

		float diff = highest_voltage - MAX_CHARGING_VOLT;
		if ( bms_state == CHARGING && ((diff > 0) ? diff < 1e-9f : diff > -1e-9f) ) {
			bms_state = MEASURE;
			pack_flags.charging = false;
		}

		if ( bms_state == CHARGING && pack_data.current >= 0 && pack_flags.charging ) {
			bms_state = MEASURE;
			pack_flags.charging = false;
		}

		if ( bms_state == MEASURE && pack_data.current < 0 && !pack_flags.charging ) {
			if ( charging_timer >=  TIME_TO_CHARGE ) {
				charging_timer = 0;
				bms_state = CHARGING;
				pack_flags.charging = true;
			}

			charging_timer++;
		}
		else {
			charging_timer = 0;
		}

		if ( bms_state == MEASURE && pack_data.current >= 0 && pack_data.current < SLEEP_CURRENT && !pack_flags.sleep ) {
			if ( sleep_timer >=  TIME_TO_SLEEP ) {
				sleep_timer = 0;
				bms_state = SLEEP;
				pack_flags.sleep = true;
			}

			sleep_timer++;
		}
		else {
			sleep_timer = 0;
		}
	}

	else if ( htim == &htim3 ) {
		// Rollover must be handled by master
		can_data.timestamp_ms += 5;
		can_data.timestamp_ms =
				(can_data.timestamp_ms == 0xFFFF) ? 0 : can_data.timestamp_ms;

		SPI_Read();
		Read_Current();

		Convert_Data();
		Check_Data();
		CAN_Send();

		if ( bms_state == SLEEP && (pack_data.current < 0 || pack_data.current > SLEEP_CURRENT) ) {
			bms_state = INIT;
			pack_flags.sleep = false;
		}
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if ( hspi == &hspi2 ) {

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if ( GPIO_Pin == WAKE_Pin ) {
    	if ( bms_state == SLEEP ) {
    		bms_state = MEASURE;
    		pack_flags.sleep = false;
    	}
    }

    if ( GPIO_Pin == BALANCE_CELLS_Pin ) {
    	if ( bms_state == CHARGING ) {
    		bms_state = CHARGING_W_CB;
    	}
    	else if ( bms_state == CHARGING_W_CB ) {
			bms_state = CHARGING;
		}
	}

    if ( GPIO_Pin == DISCHARGE_Pin ) {
		if ( bms_state == MEASURE || bms_state == SLEEP ) {
			bms_state = SELF_DISCHARGE;
		}
	}
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
