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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include "ICM20948.h"
#include "math.h"
#include "string_func.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t aRxBuffer[14];
#define CENTER 149
#define LEFT 90
#define RIGHT 210
#define PI 3.14159266359
#define sleeping 400

// Motor variables
// global ack flag, if 2= default state, 0 = buffer received, triggered ISR, 1 = executed command, ready to send ACK back to RPI
int Task2 = 0;
int ack = 2; // global ack flag
uint8_t Command[4] = "";
uint8_t TargetDistance = 0;
uint8_t TargetDistanceDebug = 0;
double XOffsetDistance = 0;

// IR Sensor Variable
uint16_t LeftIR = 0, RightIR = 0;

bool isMoving = false;
bool isStraight = false;
int isForward = 2;
int pwmLeft = 0, pwmRight = 0;
uint16_t newDutyL, newDutyR;

// calculate distance
int leftcount = 0, rightcount = 0;
int previous_leftcount = 0, previous_rightcount = 0;
double averagedistance = 0;
double averagecount = 0;
double offsetdistance = 0;

// Encoder Variables
float time_per_loop = 0;
int cnt1A, cnt2A, cnt1B, cnt2B;
int motorA_encoder_value_persecond = 0, motorB_encoder_value_persecond = 0;
double distance_travelledA = 0, distance_travelledB = 0;

// Gyro Variables
double total_angle = 0;
uint8_t gyroBuffer[20];
ICM20948 imu;
const uint8_t ICMAddress = 0x68;
uint16_t newDutyL, newDutyR;
int correction = 0;
float angleNow = 0;
int16_t angular_speed = 0;

// PID for keep straight
typedef struct _pidConfig
{
    float Kp;
    float Ki;
    float Kd;
    float ek1;
    float ekSum;
} PIDConfig;

PIDConfig SpeedSlow, SpeedFast;

int8_t dir = 1; // 0 for forward, 1 for backwards, 2 is just init state

// Ultra-Sonic Variables
int Is_First_Captured = 0;
int32_t IC_Val1 = 0;
int32_t IC_Val2 = 0;
double Difference = 0;
double Distance = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Encoder */
osThreadId_t EncoderHandle;
const osThreadAttr_t Encoder_attributes = {
  .name = "Encoder",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Gyro */
osThreadId_t GyroHandle;
const osThreadAttr_t Gyro_attributes = {
  .name = "Gyro",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Straighten */
osThreadId_t StraightenHandle;
const osThreadAttr_t Straighten_attributes = {
  .name = "Straighten",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OLEDDisplay */
osThreadId_t OLEDDisplayHandle;
const osThreadAttr_t OLEDDisplay_attributes = {
  .name = "OLEDDisplay",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DistanceCalcula */
osThreadId_t DistanceCalculaHandle;
const osThreadAttr_t DistanceCalcula_attributes = {
  .name = "DistanceCalcula",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UltrasoundReadT */
osThreadId_t UltrasoundReadTHandle;
const osThreadAttr_t UltrasoundReadT_attributes = {
  .name = "UltrasoundReadT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IRTask */
osThreadId_t IRTaskHandle;
const osThreadAttr_t IRTask_attributes = {
  .name = "IRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void EncoderCheck(void *argument);
void GyroReadTask(void *argument);
void KeepStraight(void *argument);
void OLEDPrint(void *argument);
void Calculate_Dist(void *argument);
void UltraSoundRead(void *argument);
void IRReadPolling(void *argument);

/* USER CODE BEGIN PFP */
void Motor_Init();
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
    OLED_Init();
    Motor_Init();
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Encoder */
  EncoderHandle = osThreadNew(EncoderCheck, NULL, &Encoder_attributes);

  /* creation of Gyro */
  GyroHandle = osThreadNew(GyroReadTask, NULL, &Gyro_attributes);

  /* creation of Straighten */
  StraightenHandle = osThreadNew(KeepStraight, NULL, &Straighten_attributes);

  /* creation of OLEDDisplay */
  OLEDDisplayHandle = osThreadNew(OLEDPrint, NULL, &OLEDDisplay_attributes);

  /* creation of DistanceCalcula */
  DistanceCalculaHandle = osThreadNew(Calculate_Dist, NULL, &DistanceCalcula_attributes);

  /* creation of UltrasoundReadT */
  UltrasoundReadTHandle = osThreadNew(UltraSoundRead, NULL, &UltrasoundReadT_attributes);

  /* creation of IRTask */
  IRTaskHandle = osThreadNew(IRReadPolling, NULL, &IRTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DE_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DE_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DE_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == GPIO_PIN_12) {
        NVIC_SystemReset();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
    HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 5);
    ack = 0; // global ack flag, if 2= default state, 0 = buffer received, triggered ISR, 1 = executed command, ready to send ACK back to RPI
}

// Gyro
void readByte(uint8_t addr, uint8_t *data)
{
    gyroBuffer[0] = addr;
    HAL_I2C_Master_Transmit(&hi2c1, ICMAddress << 1, gyroBuffer, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, ICMAddress << 1, data, 2, 20);
}

void writeByte(uint8_t addr, uint8_t data)
{
    gyroBuffer[0] = addr;
    gyroBuffer[1] = data;
    HAL_I2C_Master_Transmit(&hi2c1, ICMAddress << 1, gyroBuffer, 2, 20);
}

void gyroInit()
{
    writeByte(0x06, 0x00);
    osDelay(10);
    writeByte(0x03, 0x80);
    osDelay(10);
    writeByte(0x07, 0x07);
    osDelay(10);
    writeByte(0x06, 0x01);
    osDelay(10);
    writeByte(0x7F, 0x20);
    osDelay(10);
    writeByte(0x01, 0x2F);
    osDelay(10);
    writeByte(0x0, 0x00);
    osDelay(10);
    writeByte(0x7F, 0x00);
    osDelay(10);
    writeByte(0x07, 0x00);
    osDelay(10);
}
// UltraSonic
void HAL_Delay_uS(uint16_t time)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    while (__HAL_TIM_GET_COUNTER(&htim4) < time)
        ;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt source is channel1
    {
        if (Is_First_Captured == 0) // if the first value is not captured
        {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
            Is_First_Captured = 1;                                    // set the first captured as true
            // Now change the polarity to falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }

        else if (Is_First_Captured == 1) // if the first is already captured
        {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value
            __HAL_TIM_SET_COUNTER(htim, 0);                           // reset the counter

            if (IC_Val2 > IC_Val1)
            {
                Difference = IC_Val2 - IC_Val1;
            }

            else if (IC_Val1 > IC_Val2)
            {
                Difference = (0xffff - IC_Val1) + IC_Val2;
            }

            Distance = Difference * .034 / 2;
            Is_First_Captured = 0; // set it back to false

            // set polarity to rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
        }
    }
}
void Ultrasonic_Read(void)
{
    // Code for Ultrasonic Sensor
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET); // pull the TRIG pin HIGH
    HAL_Delay_uS(10);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET); // pull the TRIG pin low

    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
}

// PID for Straight line
void PIDConfigInit(PIDConfig *cfg, const float Kp, const float Ki, const float Kd)
{
    cfg->Kp = Kp;
    cfg->Ki = Ki;
    cfg->Kd = Kd;
    cfg->ek1 = 0;
    cfg->ekSum = 0;
}

void PIDConfigReset(PIDConfig *cfg)
{
    cfg->ek1 = 0;
    cfg->ekSum = 0;
}

void start_encoder()
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // encoder for motor A(left)
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // encoder for motor B(right)
}

void Motor_Init()
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // left
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // right
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // servo
}

void setSpeed(int left, int right)
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, left);  // set left pwm to speed
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, right); // set right pwm to speed
}

void setDirection(int left, int right)
{
    if (left == 1)
    {
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // set direction forward
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
    }
    else if (left == 0)
    {
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // set direction forward
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
    }
    if (right == 1)
    {
        HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
    }
    else if (right == 0)
    {
        HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
    }
}

void MotorStop()
{
    isMoving = false;
    setSpeed(0, 0);
    //	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);//set left pwm to 0
    //	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);//set right pwm to 0
}

int MotorForward(int speed, int distance)
{
    isMoving = true;
    isStraight = true;
    // total_angle=0;//set angle so that it can keep straight
    htim1.Instance->CCR4 = 149; // set servo to center

    total_angle = 0;

    TargetDistanceDebug = distance; // only for debug mode

    pwmLeft = speed;
    pwmRight = speed;

    setSpeed(pwmLeft, pwmRight);

    if (speed == 1200)
    {
        PIDConfigInit(&SpeedSlow, 100, 0.0, 0.0);
    }
    else if (speed == 3000)
    {
        PIDConfigInit(&SpeedFast, 100, 0.0 , 0.0);
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0);     // Set the encoder counter value of right to 0
    __HAL_TIM_SET_COUNTER(&htim2, 65535); // Set the encoder counter value of left to 65535
    leftcount = 0, rightcount = 0;
    distance_travelledA = 0;
    distance_travelledB = 0;
    averagedistance = 0;

    while (averagedistance < distance)
    {
        // set right pwm to speed
        setDirection(1, 1);
        isForward = 1;
    }
    offsetdistance = averagedistance - distance;
    isForward = 2;
    MotorStop();
    //    osDelay(500);
    //    if (offsetdistance>0){
    //    	MotorBack(1200,offsetdistance);
    //    }
    return 1;
}

int MotorBack(int speed, int distance)
{
    isMoving = true;
    isStraight = true;
    // total_angle=0;
    htim1.Instance->CCR4 = 149; // set servo to center

    total_angle = 0;

    TargetDistanceDebug = distance; // only for debug mode

    pwmLeft = speed;
    pwmRight = speed;

    setSpeed(pwmLeft, pwmRight);

    __HAL_TIM_SET_COUNTER(&htim3, 65535); // Set the encoder counter value of right to 0
    __HAL_TIM_SET_COUNTER(&htim2, 0);     // Set the encoder counter value of left to 0
    leftcount = 0, rightcount = 0;
    distance_travelledA = 0;
    distance_travelledB = 0;
    averagedistance = 0;

    while (averagedistance < distance)
    {
        setDirection(0, 0);
        isForward = 0;
    }
    offsetdistance = averagedistance - distance;
    isForward = 2;
    MotorStop();
    osDelay(sleeping);
    //    if (offsetdistance>0) MotorForward(1200,offsetdistance);
    return 1;
}

int MotorLeft(int speed, int degreeTarget)
{
    isMoving = true;
    isStraight = false;

    distance_travelledA = 0;
    distance_travelledB = 0;
    total_angle = 0;

    htim1.Instance->CCR4 = 100; // hard left

    // switch (lab,inside isaac etc)
    setSpeed(0, speed);

    while (total_angle < degreeTarget)
    { // for now, degreeTarget is set as distance
        setDirection(0, 1);
    }
    MotorStop();
    htim1.Instance->CCR4 = 149; // centre
    //	osDelay(500);
    return 1;
}

int MotorBackLeft(int speed, int degreeTarget)
{
    isMoving = true;
    isStraight = false;
    htim1.Instance->CCR4 = 100; // hard left

    distance_travelledA = 0;
    distance_travelledB = 0;
    total_angle = 0;

    setSpeed(0, speed);

    //	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);//set left pwm to 0
    //	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed);//set right pwm to speed

    while (total_angle > degreeTarget)
    { // for now, degreeTarget is set as distance
        setDirection(1, 0);
    }
    MotorStop();
    htim1.Instance->CCR4 = 149; // centre
    osDelay(sleeping);
    return 1;
}

int MotorRight(int speed, int degreeTarget)
{
    isMoving = true;
    isStraight = false;
    htim1.Instance->CCR4 = 200; // hard right

    distance_travelledA = 0;
    distance_travelledB = 0;
    total_angle = 0;

    setSpeed(speed, 0);

    while (total_angle > degreeTarget)
    { // for now, degreeTarget is set as distance
        setDirection(1, 0);
    }
    MotorStop();
    htim1.Instance->CCR4 = 149; // centre
    //	osDelay(500);
    return 1;
}

int MotorBackRight(int speed, int degreeTarget)
{
    isMoving = true;
    isStraight = false;
    htim1.Instance->CCR4 = 200; // hard right

    distance_travelledA = 0;
    distance_travelledB = 0;
    total_angle = 0;

    setSpeed(speed, 0);

    while (total_angle < degreeTarget)
    { // for now, degreeTarget is set as distance
        setDirection(0, 1);
    }
    MotorStop();
    htim1.Instance->CCR4 = 149; // centre
    osDelay(500);
    return 1;
}

int forwardbiasedtwopointTurnLeft(int speed, int degreeTarget)
{
    isMoving = true;
    isStraight = false;
    htim1.Instance->CCR4 = 100; // hard left

    distance_travelledA = 0;
    distance_travelledB = 0;
    total_angle = 0;

    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed); // set right pwm to speed
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);     // set left pwm to 0

    while (total_angle < degreeTarget / 2)
    {                                                     // for now, degreeTarget is set as distance
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // set direction forward
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
    }
    MotorStop();

    total_angle = 0;
    //	osDelay(500);
    htim1.Instance->CCR4 = 200;                         // hard right
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed); // set left pwm to speed
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);     // set right pwm to 0

    while (total_angle < degreeTarget / 2)
    {                                                       // for now, degreeTarget is set as distance
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // set direction back
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET); // set direction back
        HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
    }
    MotorStop();
    htim1.Instance->CCR4 = 149; // hard left
    return 1;
}

int backbiasedtwopointTurnLeft(int speed, int degreeTarget)
{
    isMoving = true;
    isStraight = false;
    htim1.Instance->CCR4 = 200; // hard left

    distance_travelledA = 0;
    distance_travelledB = 0;
    total_angle = 0;

    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed); // set left pwm to speed
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);     // set right pwm to 0

    while (total_angle < degreeTarget / 2)
    {                                                       // for now, degreeTarget is set as distance
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // set direction back
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET); // set direction back
        HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
    }
    MotorStop();

    total_angle = 0;
    //	osDelay(500);
    htim1.Instance->CCR4 = 100; // hard right

    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed); // set right pwm to speed
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);     // set left pwm to 0

    while (total_angle < degreeTarget / 2)
    {                                                     // for now, degreeTarget is set as distance
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // set direction forward
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
    }
    MotorStop();
    htim1.Instance->CCR4 = 149; // hard left
    return 1;
}

int forwardbiasedtwopointTurnRight(int speed, int degreeTarget)
{
    isMoving = true;
    isStraight = false;
    htim1.Instance->CCR4 = 200; // hard left

    distance_travelledA = 0;
    distance_travelledB = 0;
    total_angle = 0;

    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);     // set right pwm to speed
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed); // set left pwm to 0

    while (total_angle > degreeTarget / 2)
    {                                                     // for now, degreeTarget is set as distance
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // set direction forward
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
    }
    MotorStop();

    total_angle = 0;
    //	osDelay(500);
    htim1.Instance->CCR4 = 100;                         // hard left
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);     // set left pwm to speed
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed); // set right pwm to 0

    while (total_angle > degreeTarget / 2)
    {                                                       // for now, degreeTarget is set as distance
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // set direction back
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET); // set direction back
        HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
    }
    MotorStop();
    htim1.Instance->CCR4 = 149; // hard left

    return 1;
}

int backbiasedtwopointTurnRight(int speed, int degreeTarget)
{
    isMoving = true;
    isStraight = false;
    htim1.Instance->CCR4 = 100; // hard left

    distance_travelledA = 0;
    distance_travelledB = 0;
    total_angle = 0;

    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);     // set left pwm to speed
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speed); // set right pwm to 0

    while (total_angle > degreeTarget / 2)
    {                                                       // for now, degreeTarget is set as distance
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // set direction back
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET); // set direction back
        HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
    }
    MotorStop();

    total_angle = 0;
    //	osDelay(500);
    htim1.Instance->CCR4 = 200; // hard right

    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed); // set left pwm to 0
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);     // set right pwm to speed

    while (total_angle > degreeTarget / 2)
    {                                                     // for now, degreeTarget is set as distance
        HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // set direction forward
        HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
    }
    MotorStop();
    htim1.Instance->CCR4 = 149; // hard left

    return 1;
}

double T1MotorForward(int speed)
{
    isMoving = true;
    isStraight = true;
    // total_angle=0;//set angle so that it can keep straight
    htim1.Instance->CCR4 = 149; // set servo to center
    distance_travelledA = 0;
    distance_travelledB = 0;
    total_angle = 0;

    pwmLeft = speed;
    pwmRight = speed;

    setSpeed(pwmLeft, pwmRight);

    if (speed == 1200)
    {
        PIDConfigInit(&SpeedSlow, 2.5, 2.0, 0.8);
    }
    else if (speed == 3000)
    {
        PIDConfigInit(&SpeedFast, 100.0, 0.0, 0.0);
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0);     // Set the encoder counter value of right to 0
    __HAL_TIM_SET_COUNTER(&htim2, 65535); // Set the encoder counter value of left to 65535
    leftcount = 0, rightcount = 0;
    averagedistance = 0;
    averagecount = 0;

    int Ydists = 0;
    while (Distance > 6.5)
    {
        // set right pwm to speed
        setDirection(1, 1);
        Ydists = averagecount;
        isForward = 1;
    }
    isForward = 2;
    MotorStop();
    osDelay(sleeping);
    //    if (offsetdistance>0){
    //    	MotorBack(1200,offsetdistance);
    //    }
    return Ydists;
}

int T2MotorForward(int speed)
{
    isMoving = true;
    isStraight = true;
    // total_angle=0;//set angle so that it can keep straight
    htim1.Instance->CCR4 = 149; // set servo to center
    distance_travelledA = 0;
    distance_travelledB = 0;
    total_angle = 0;

    pwmLeft = speed;
    pwmRight = speed;

    setSpeed(pwmLeft, pwmRight);

    if (speed == 1200)
    {
        //    	PIDConfigInit(&SpeedSlow, 2.5, 2.0, 0.8);
        PIDConfigInit(&SpeedSlow, 100.0, 0.0, 0.0);
    }
    else if (speed == 3000)
    {
        PIDConfigInit(&SpeedFast, 100.0, 0.0, 0.0);
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0);     // Set the encoder counter value of right to 0
    __HAL_TIM_SET_COUNTER(&htim2, 65535); // Set the encoder counter value of left to 65535
    leftcount = 0, rightcount = 0;
    averagedistance = 0;
    averagecount = 0;

    while (Distance > 3)
    {
        // set right pwm to speed
    	if (RightIR<10){
    	    htim1.Instance->CCR4 = 100; // hard left
    	}
    	else if(LeftIR<10){
    	    htim1.Instance->CCR4 = 200; // hard right
    	}
    	else{
    	    htim1.Instance->CCR4 = 149; // center
    	}
        setDirection(1, 1);

        isForward = 1;
    }
    isForward = 2;
    MotorStop();
    osDelay(sleeping);
    //    if (offsetdistance>0){
    //    	MotorBack(1200,offsetdistance);
    //    }
    return averagecount;
}

int ForwardCheckIR(int speed, char direction)
{
    isMoving = true;
    isStraight = true;
    htim1.Instance->CCR4 = 149; // set servo to center
    distance_travelledA = 0;
    distance_travelledB = 0;
    total_angle = 0;
    int IRdist = 50;

    pwmLeft = speed;
    pwmRight = speed;

    setSpeed(pwmLeft, pwmRight);

    if (speed == 1200)
    {
        PIDConfigInit(&SpeedSlow, 2.5, 2.0, 0.8);
    }
    else if (speed == 3000)
    {
        PIDConfigInit(&SpeedFast, 100.0, 0.0, 0.0);
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0);     // Set the encoder counter value of right to 0
    __HAL_TIM_SET_COUNTER(&htim2, 65535); // Set the encoder counter value of left to 65535
    leftcount = 0, rightcount = 0;
    averagedistance = 0;
    averagecount = 0;

    int Ydist = 0;
    if (direction == 'L')
    {
        while (RightIR <= IRdist)
        {
            // set right pwm to speed
            setDirection(1, 1);
            Ydist = averagecount;
            isForward = 1;
        }
    }
    else if (direction == 'R')
    {
        while (LeftIR <= IRdist)
        {
            // set right pwm to speed
            setDirection(1, 1);
            Ydist = averagecount;
            isForward = 1;
        }
    }

    isForward = 2;
    isStraight = false;
    MotorStop();
    osDelay(sleeping);
    //    if (offsetdistance>0){
    //    	MotorBack(1200,offsetdistance);
    //    }
    return Ydist;
}
// temp gamepad code implementation
int go = 10;

void customMotor(char *dir, int val)
{
    htim1.Instance->CCR4 = 149; // set servo to center
    int speed = 3000;
    int turnspeed = 3000;
    int turnangle = 90;
    while (ack == 0)
    {
        //		if (strcmp(dir, "FW")==0)
        //			ack = MotorForward(speed,val);
        //		else if (strcmp(dir, "BW")==0)
        //			ack = MotorBack(speed,val);
        //		else if (strcmp(dir, "FL")==0)
        //			ack = MotorLeft(turnspeed, 87);
        //		else if (strcmp(dir, "FR")==0)
        //			ack = MotorRight(turnspeed, -89);
        //		else if (strcmp(dir, "BL")==0)
        //			ack = MotorBackLeft(turnspeed, -87);
        //		else if (strcmp(dir, "BR")==0)
        //			ack = MotorBackRight(turnspeed, 90);
        //		else{
        //			MotorStop();
        //			ack=2;
        //		}
        if (strcmp(dir, "FW") == 0)
            ack = MotorForward(speed, val);
        else if (strcmp(dir, "BW") == 0)
            ack = MotorBack(speed, val);
        else if (strcmp(dir, "FL") == 0)
            ack = MotorLeft(turnspeed, 87);
        else if (strcmp(dir, "FR") == 0)
            ack = MotorRight(turnspeed, -87);
        else if (strcmp(dir, "BL") == 0)
            ack = MotorBackLeft(turnspeed, -87);
        else if (strcmp(dir, "BR") == 0)
            ack = MotorBackRight(turnspeed, 88);
        else if (strcmp(dir, "TT") == 0)
        {
            Task2 = 1;
            ack = 1;
        }
        else
        {
            MotorStop();
            ack = 2;
        }

        MotorStop();
        htim1.Instance->CCR4 = 149;
        osDelay(1);
    }
}

int gooo = 0;
double Ydist = 0;

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
double turnDistance = 0;
double cmDistance = 0;
double offsetdistx = 0;
double offsetdisty = 0;
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
    uint8_t OLEDstr[20] = "";
    uint8_t OLEDstr1[20] = "";
    uint8_t integer1_str[4] = ""; // aaa-aaa-aaa-a
    uint8_t old_Speed, old_Turnangle = 0;
    int aaaaa = 0;

    int flag = 0;

    htim1.Instance->CCR4 = 149; // set servo to center
    osDelay(6000);              // approx time taken to initiate gyro
    // Tested movements/ and calibrated
    //	MotorLeft(3000,88);
    //	osDelay(500);
    //	MotorForward(3000,300);

    //		T1MotorForward(3000);//record distance1
    //		MotorLeft(3000,45);
    ////		osDelay(500);
    //		MotorForward(3000,10);
    //		MotorRight(3000,-90);
    ////		osDelay(500);
    //		MotorForward(3000,20);
    //		MotorLeft(3000,42);
    //
    //		if(Distance<7){
    //			while(Distance<7){// if distance too near, move back
    //				isForward = true;
    //				isStraight = true;
    //				setSpeed(3000,3000);
    //				setDirection(0,0);
    //				isForward=0;
    //				flag = 1;
    //			}
    //			isForward=2;
    //		}else{
    //			offsetdisty += T1MotorForward(3000);//record distance1
    //		}
    //	  Ydist = (float)Ydist/1552.0*22;//get 1552 from cnt1B for one

    // right
    //		MotorRight(3000,-45);
    //		osDelay(500);
    //		MotorForward(3000,15);
    //		MotorLeft(3000,90);
    //		osDelay(500);
    //		MotorForward(3000,35);
    //		MotorRight(3000,-43);

    //	MotorLeft(3000,90);
    ////	osDelay(500);
    ////	MotorForward(3000,10);
    //	MotorRight(3000,-90);
    //	osDelay(500);
    //	MotorForward(3000,3);
    //	MotorRight(3000,-90);
    ////	osDelay(500);
    ////	MotorForward(3000,10);
    //	MotorLeft(3000,90);

    //	ForwardCheckIR(3000,'L');
    //	MotorLeft(3000,87);
    //	MotorRight(3000,-87);
    //	MotorRight(3000,-87);
    //	MotorLeft(3000,87);
    //	MotorBack(3000,20);
    HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 5);

    for (;;)
    {
        if (Task2 == 0)
        {
            // task 1
            if (ack == 0)
            { // if have ISR

                slice(aRxBuffer, Command, 0, 2);
                slice(aRxBuffer, integer1_str, 2, 4);
                TargetDistance = strtoint(integer1_str);

                if (strcmp(aRxBuffer, "FINN") != 0)
                {
                    customMotor(Command, TargetDistance);
                }
                else
                {
                    MotorStop();
                    htim1.Instance->CCR4 = 149;
                }
                if (ack == 1)
                { // check if done with custom motor
                    MotorStop();
                    htim1.Instance->CCR4 = 149;
                    HAL_UART_Transmit(&huart3, "ACK\n\r", 5, 0xFFFF);
                    ack = 2; // set back to default
                    memset(aRxBuffer, '0', strlen(aRxBuffer));
                }
            } // end ack check
        }     // end check for task 2
        else if (Task2 == 1)
        {
            // task 2
            // move to first obstacle
            double initialDistance = T1MotorForward(3000); // record distance1

            HAL_UART_Transmit(&huart3, "SNAP\n\r", 6, 0xFFFF);
            HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 5);
            for (;;)
            { // wait for rpi to sent L/R
                if (ack == 0)
                {
                    if (aRxBuffer[0] == 'L')
                    {
                        // Left
//							MotorLeft(3000,55);
//					//		osDelay(500);
//							MotorForward(3000,5);
//							MotorRight(3000,-55);
//							MotorRight(3000,-55);
//					//		osDelay(500);
//							MotorForward(3000,10);
//							MotorLeft(3000,51);

                        // 45degree left
                        MotorLeft(3000, 45);
                        //		osDelay(500);
                        MotorForward(3000, 10);
                        MotorRight(3000, -90);
                        //		osDelay(500);
                        MotorForward(3000, 20);
                        MotorLeft(3000, 42);

                        //Set some distance
                        turnDistance += 90;
                    }
                    else if (aRxBuffer[0] == 'R')
                    {
                        // Right
                        MotorRight(3000, -45);
                        MotorForward(3000, 15);
                        MotorLeft(3000, 88);
                        MotorForward(3000, 20);
                        MotorRight(3000, -43);

                        //Set some distance to be set
                        turnDistance += 90;
                    }
                    ack = 2;
                    break;
                } // end ack check
            }     // end for loop
            // move towards 2nd obstacle
            // Initial move distance

            osDelay(500);
            if (Distance < 7)
            {
            	//Store the initial distance
            	double initialDistance = Distance;
                while (Distance < 7)
                { // if distance too near, move back
                    isForward = true;
                    isStraight = true;
                    setSpeed(3000, 3000);
                    setDirection(0, 0);
                    //Set the is forward flag to be false
                    isForward = 0;
                    //Update reverse to be true
                    flag = 1;
                }
                //After reversing, we get that the forward motion is there
                isForward = 2;
                //3.85 is the multiplier for ultrasonic
                turnDistance -= (7 - initialDistance) * 3.85;
            }
            else
            {
            	//This is the distance between 1st obstacle to 2nd obstacle
                offsetdisty += T1MotorForward(3000); // record distance1
            }
            //After stopping we try to snap
            MotorStop();

            // check 2nd obstacle
            HAL_UART_Transmit(&huart3, "SNAP\n\r", 6, 0xFFFF);
            HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 5);
            for (;;)
            { // wait for rpi
                if (ack == 0)
                {
                    if (aRxBuffer[0] == 'L')
                    {
                        // Left
                        // MotorLeft(3000,89);
                        MotorLeft(3000, 86);
                    }
                    else if (aRxBuffer[0] == 'R')
                    {
                        // Right
                        MotorRight(3000, -90);
                    }
                    ack = 2;
                    break;
                } // end ack check
            }     // end for loop
            // wait for a while to calibrate gyro
            osDelay(sleeping);

            // 1/2 of long side of obstacle
            ForwardCheckIR(3000, aRxBuffer[0]);

            osDelay(sleeping);

            if (aRxBuffer[0] == 'L'){
            	if (RightIR<40){
                    //cmDistance += 7;
            		MotorForward(3000, 7);
            	}
            }
            else if (aRxBuffer[0] == 'R'){
            	if (LeftIR<40){
                    //cmDistance += 7;
            		MotorForward(3000, 7);
            	}
			}

            osDelay(50);
            // turn opp of arrow direction
            // short side of obstacle
            // turn
            // long side of obstacle
            // turn
            if (aRxBuffer[0] == 'L')
            {
                // Left
                MotorRight(3000, -87);
                osDelay(sleeping);
//                MotorForward(3000, 10);
                osDelay(sleeping);
                // short side
                offsetdisty += ForwardCheckIR(3000, aRxBuffer[0]);
                osDelay(100);
                MotorForward(3000, 10);
                MotorRight(3000, -87);
                osDelay(sleeping);
                // long side
                MotorForward(3000, 10);
                cmDistance += 12;
                offsetdistx += ForwardCheckIR(3000, aRxBuffer[0]);
                MotorForward(3000, 10);
                MotorRight(3000, -87);
            }
            else if (aRxBuffer[0] == 'R')
            {
                // Right
                MotorLeft(3000, 85);
                osDelay(sleeping);
                // short side
                offsetdisty+= ForwardCheckIR(3000, aRxBuffer[0]);
                osDelay(100);
                MotorForward(3000, 10);
                MotorLeft(3000, 85);
                osDelay(sleeping);
                // long side
                MotorForward(3000, 10);
                cmDistance += 12;
                offsetdistx += ForwardCheckIR(3000, aRxBuffer[0]);
                MotorForward(3000, 10);
                MotorLeft(3000, 85);
            }

            // go back to parking
            // move forward
            //osDelay(500);
            gooo = (float)(offsetdisty) / 1552.0 * 22; // get 1552 from cnt1B for one
            //If reversed, we have to add the turn success distance
            double OFFSET = 40;
            gooo += turnDistance + cmDistance;

            int fwdMovement = ceil((double)gooo * 1.22);
            osDelay(sleeping);
            MotorForward(3000, fwdMovement + OFFSET);

            // turn in
            if (aRxBuffer[0] == 'L')
            {
                // Left
                MotorRight(3000, -90);
            }
            else if (aRxBuffer[0] == 'R')
            {
                // Right
                MotorLeft(3000, 89);
            }
            gooo = (float)(offsetdistx) / 1552.0 * 22;
            gooo /= 2;

            double constantOffset = 15;//as length of obstacle increases, this need to decrease....

            if (gooo >= 24){
            	constantOffset=7;
            }
            else if(gooo >=45){
            	constantOffset=0;
            }
            else {//if goo is less than 24
            	constantOffset = 17.5;
            }
            //If gooo is higher than 0 and past some offset
            if (gooo > 0 && gooo - constantOffset > 0){
            	MotorForward(3000, gooo - constantOffset);

            }
            else
            {
                MotorBack(3000, 10);
            }

            if (aRxBuffer[0] == 'L')
            {
                // Left
                MotorLeft(3000, 90);
            }
            else if (aRxBuffer[0] == 'R')
            {
                // Right
                MotorRight(3000, -90);
            }

            // park
            osDelay(sleeping);
            T2MotorForward(1200);
            HAL_UART_Transmit(&huart3, "WOOF\n\r", 6, 0xFFFF);//signal RPI end of Task 2
            Task2 = 0;
        } // task 2==1
    }     // end for
    //	MotorForward(3000,90);
    //	osDelay(2000);//approx time taken to initiate gyro
    //	MotorBack(3000,90);
    //	osDelay(2000);//approx time taken to initiate gyro
    //
    //
    //
    //
    //	MotorLeft(3000,89);
    //	osDelay(2000);//approx time taken to initiate gyro
    //
    //	//	MotorLeft(1200,89);
    ////	MotorLeft(1200,89);
    ////	MotorLeft(1200,89);
    //
    //	MotorRight(3000,-90);
    //	osDelay(6000);//approx time taken to initiate gyro
    //
    //	//	MotorLeft(3000,85);
    ////	MotorRight(3000,-87);
    //	MotorBackLeft(3000,-90);
    //	osDelay(6000);//approx time taken to initiate gyro
    //
    //	MotorBackRight(3000,91);
    //	osDelay(6000);//approx time taken to initiate gyro

    /////////////////////////////////////NOT TESTED YET////////////////////////////////////////////////
    //	MotorRight(1200,-90);
    //	MotorBack(10000,100);
    //	MotorForward(10000,100);
    //	MotorLeft(3000,90);
    //	MotorBackLeft(3000,90);
    //	MotorBackRight(3000,-90);
    //	forwardbaisedtwopointTurnRight(3000,-90);
    //	osDelay(1000);
    //	forwardbaisedtwopointTurnRight(3000,-90);
    //	MotorLeft(3000,90);
    //	MotorRight(3000,-90);
    //	MotorBack(10000,100);
    //	forwardbaisedtwopointTurn(1000,90);
    //	osDelay(1000);
    //	backbiasedtwopointTurn(1000,90);
    /////////////////////////////////////////////////////////////////////////////////////////////////
    /* Infinite loop */
    //	for(;;)
    //	{
    //
    //	}
    //	Gampad

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_EncoderCheck */
/**
 * @brief Function implementing the Encoder thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_EncoderCheck */
void EncoderCheck(void *argument)
{
  /* USER CODE BEGIN EncoderCheck */
    /* Infinite loop */
    start_encoder();

    // Initialize local variables for encoder
    uint32_t time_elasped = 0; // in terms of ticks, where 1000 ticks = 1 s
    uint32_t tick;

    cnt1A = __HAL_TIM_GET_COUNTER(&htim2);
    cnt1B = __HAL_TIM_GET_COUNTER(&htim3);
    tick = HAL_GetTick();

    // Gets the encoder count for both motors per second
    for (;;)
    {
        // counts the number of encoder pulses per second
        if (HAL_GetTick() - tick > 100L)
        {
            time_per_loop = 100;

            cnt2A = __HAL_TIM_GET_COUNTER(&htim2);
            cnt2B = __HAL_TIM_GET_COUNTER(&htim3);

            // motor A encoder count
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
            {
                if (cnt2A < cnt1A)
                {
                    motorA_encoder_value_persecond = cnt1A - cnt2A;
                }
                else
                {
                    motorA_encoder_value_persecond = (65535 - cnt2A) + cnt1A;
                }
            }
            else
            {
                if (cnt2A > cnt1A)
                {
                    motorA_encoder_value_persecond = cnt2A - cnt1A;
                }
                else
                {
                    motorA_encoder_value_persecond = (65535 - cnt1A) + cnt2A;
                }
            }

            // motor B encoder count
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
            {
                if (cnt2B < cnt1B)
                {
                    motorB_encoder_value_persecond = cnt1B - cnt2B;
                }
                else
                {
                    motorB_encoder_value_persecond = (65535 - cnt2B) + cnt1B;
                }
            }
            else
            {
                if (cnt2B > cnt1B)
                {
                    motorB_encoder_value_persecond = cnt2B - cnt1B;
                }
                else
                {
                    motorB_encoder_value_persecond = (65535 - cnt1B) + cnt2B;
                }
            }

            time_elasped = time_elasped + time_per_loop;

            // Move set distance(1) or constantly move straight(0).
            // By default, target_distance is set to 0;
            // target_distance = 100; //Manually input or get from Rpi
            // move_distance(1, target_distance);

            cnt1A = __HAL_TIM_GET_COUNTER(&htim2);
            cnt1B = __HAL_TIM_GET_COUNTER(&htim3);
            tick = HAL_GetTick();
        }
    } // end for
  /* USER CODE END EncoderCheck */
}

/* USER CODE BEGIN Header_GyroReadTask */
/**
 * @brief Function implementing the Gyro thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GyroReadTask */
void GyroReadTask(void *argument)
{
  /* USER CODE BEGIN GyroReadTask */
    /* Infinite loop */
    gyroInit();
    uint8_t val[2] = {0, 0};
    double tick_angle = 0;
    uint32_t tick = 0;
    double offset = 0;
    int i = 0;

    //	uint8_t OLEDStringint[20]="";
    //	int heartbeat=0;

    while (i < 100)
    {
        osDelay(50);
        readByte(0x37, val);
        angular_speed = (val[0] << 8) | val[1]; // combine val[0]+val[1] into angular speed //calculate current angle speed
        //				trash += (double)((double)angular_speed) * ((HAL_GetTick() - tick) / 16400.0);
        offset += angular_speed; // accumulating angular speed
        tick = HAL_GetTick();
        HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); // toggle LED pin
        i++;
    }
    offset = offset / i;
    tick = HAL_GetTick();
    osDelay(10);
    /* Infinite loop */
    for (;;)
    {
        //	if(HAL_GetTick()-miliOld < 1000L) continue;

        if (HAL_GetTick() - tick >= 50) // poll every 50ms
        {
            readByte(0x37, val);
            angular_speed = (val[0] << 8) | val[1];                                                     // combine val[0]+val[1] into angular speed //calculate current angle speed
            tick_angle = (double)((double)angular_speed - offset) * ((HAL_GetTick() - tick) / 16400.0); // convert angular speed to current angle by deducting offset

            if ((tick_angle < -0.1) || (tick_angle > 0.1)) // smoothing factor
            {
                if (tick_angle > 0)
                {
                    total_angle += tick_angle * 1.01; // make it slower
                }
                else
                {
                    total_angle += tick_angle * 1.01;
                }
            }

            if (total_angle >= 360)
            {
                total_angle = 0;
            }
            if (total_angle <= -360)
            {
                total_angle = 0;
            }

            i -= angular_speed;
            i++;
            tick = HAL_GetTick();
        }
        /////Debugging Only to print and see
        //			  	  sprintf(OLEDStringint, "Alive: %d\0",heartbeat);
        //			  	  OLED_ShowString(0, 0, OLEDStringint);
        //			  	  heartbeat++;
        //
        //
        //	      sprintf(sbuf2,"Yaw: %5.2f ",total_angle);
        //	      OLED_ShowString(0,10,sbuf2);
        //		  OLED_Refresh_Gram();
        //		  sprintf(sbuf2, "Yaw: %5.2f\n\r\0", total_angle);
        //		  HAL_UART_Transmit(&huart3,sbuf2,14,0xFFFF);//count the length
        //	      osDelay(1);
    }
  /* USER CODE END GyroReadTask */
}

/* USER CODE BEGIN Header_KeepStraight */
/**
 * @brief Function implementing the Straighten thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_KeepStraight */
void KeepStraight(void *argument)
{
  /* USER CODE BEGIN KeepStraight */
    /* Infinite loop */
    for (;;)
    {
        //		  htim1.Instance -> CCR4 = 150;
        //		  osDelay(1);
        // Going to the right
        dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? 1 : -1; // use only one of the wheel to determine car direction

        if (isMoving && isStraight && isForward == 1)
        {
            if (pwmLeft == 1200)
            {                                                                                   // slow
                angleNow += ((angular_speed >= -4 && angular_speed <= 11) ? 0 : angular_speed); // / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
                __PID_SPEED_T(SpeedSlow, angleNow, correction, dir, newDutyL, newDutyR);
                setSpeed(newDutyL, newDutyR);
            }
            else if (pwmLeft == 3000)
            {                                                                                   // fast
                angleNow += ((angular_speed >= -4 && angular_speed <= 11) ? 0 : angular_speed); // / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
                __PID_SPEED_2(SpeedFast, angleNow, correction, dir, newDutyL, newDutyR);
                setSpeed(newDutyL, newDutyR);
            }

            //			__SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);
        } // back
        else if (isMoving && isStraight && isForward == 0)
        {
            if (pwmLeft == 1200)
            {
                if (total_angle == 0.0)
                {                               // if middle
                    htim1.Instance->CCR4 = 149; // keep straight
                    osDelay(1);
                }
                else if (total_angle < -0.01)
                { // if go right
                    //				  htim1.Instance -> CCR4 = 154;//keep left
                    htim1.Instance->CCR4 = 159; // keep left
                    osDelay(1);
                }
                else if (total_angle > 0.01)
                {                               // if go left
                    htim1.Instance->CCR4 = 139; // keep right
                    osDelay(1);
                }
            }
            else if (pwmLeft == 3000)
            {
                if (total_angle == 0.0)
                {                               // if middle
                    htim1.Instance->CCR4 = 149; // keep straight
                    osDelay(1);
                }
                else if (total_angle < -0.01)
                { // if go right
                    //				  htim1.Instance -> CCR4 = 154;//keep left
                    htim1.Instance->CCR4 = 154; // keep left
                    osDelay(1);
                }
                else if (total_angle > 0.01)
                {                               // if go left
                    htim1.Instance->CCR4 = 145; // keep right
                    osDelay(1);
                }
            }
        }
        //	    osDelay(1);
        osDelay(1);
    }
  /* USER CODE END KeepStraight */
}

/* USER CODE BEGIN Header_OLEDPrint */
/**
 * @brief Function implementing the OLEDDisplay thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_OLEDPrint */
void OLEDPrint(void *argument)
{
  /* USER CODE BEGIN OLEDPrint */
    /* Infinite loop */
    uint8_t OLEDStringint[20] = "";
    int heartbeat = 0;
    for (;;)
    {
        sprintf(OLEDStringint, "Alive: %d\0", heartbeat);
        OLED_ShowString(0, 0, OLEDStringint);
        heartbeat++;
        //	  //print speed
        //	  sprintf(OLEDStringint, "%5d | %5d\0", cnt1A,cnt1B);//show encoder reading
        sprintf(OLEDStringint, "Buf: %s | %d\0", Command, TargetDistance);
        OLED_ShowString(0, 10, OLEDStringint);

        //	  sprintf(OLEDStringint, "DistA:%5d\0", (int) distance_travelledA);
        //	  OLED_ShowString(0, 20, OLEDStringint);
        sprintf(OLEDStringint, "IR: %4d %4d", LeftIR, RightIR);
        OLED_ShowString(0, 20, OLEDStringint);

        sprintf(OLEDStringint, "Ydist:%5d\0", (int)gooo);
        OLED_ShowString(0, 30, OLEDStringint);
        //	  HAL_UART_Transmit(&huart3, (int) distance_travelled  , 20, 0xFFFF);

        //	  if (isStraight){
        //	      sprintf(OLEDStringint,"isStraight: 1\0 ");
        //	      OLED_ShowString(0, 40,OLEDStringint);
        //	  }
        //	  else{
        //	      sprintf(OLEDStringint,"isStraight: 0\0 ");
        //	      OLED_ShowString(0, 40,OLEDStringint);
        //	  }
        sprintf(OLEDStringint, "aveCount: %5d\0 ", (int)averagecount);
        OLED_ShowString(0, 40, OLEDStringint);

        sprintf(OLEDStringint, "US: %5d\0 ", (int)Distance);
        OLED_ShowString(0, 50, OLEDStringint);

        OLED_Refresh_Gram();

        sprintf(OLEDStringint, "Yaw: %5d\n\r\0", (int)total_angle);
        //	  HAL_UART_Transmit(&huart3, "connected\n\r",  11, 0xFFFF);

        //      HAL_UART_Transmit(&huart3,OLEDStringint,14,0xFFFF);//count the length

        osDelay(1);
    }
  /* USER CODE END OLEDPrint */
}

/* USER CODE BEGIN Header_Calculate_Dist */
/**
 * @brief Function implementing the DistanceCalcula thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Calculate_Dist */
void Calculate_Dist(void *argument)
{
  /* USER CODE BEGIN Calculate_Dist */
    /* Infinite loop */
    int index = 0;
    double IsaacHallcircumference[9] = {29.0, 26.5, 23.0, 22.95, 22.1, 22.0, 22.3, 22.0, 21.95}; // calibrated distance 10cm to 90cm. assuming circumference of wheel since we cant change tick/rev
    double InsideLabCircumference[9] = {29.0, 24.2, 22.5, 22.1, 21.6, 21.4, 21.3, 21.2, 21.15};  // calibrated distance 10cm to 90cm. assuming circumference of wheel since we cant change tick/rev
    double OutsideLabCircumference[9] = {31.0, 25.0, 23.0, 22.3, 22.1, 21.8, 21.5, 21.2, 21.2};  // calibrated distance 10cm to 90cm. assuming circumference of wheel since we cant change tick/rev
    // the lower the further it travels
    for (;;)
    {
        index = (TargetDistance / 10)-1;
        if (index > 9)
        {
            index = 8;
        }
        else if(index < 0){
            index = 0;
        }

        //      circumference= 20.5cm
        if (isForward == 1)
        { // going forward
            rightcount = __HAL_TIM_GET_COUNTER(&htim3);
            leftcount = 65535 - __HAL_TIM_GET_COUNTER(&htim2);
        }
        else if (isForward == 2)
        { // not going forward
            rightcount = 0;
            leftcount = 0;
        }
        else if (isForward == 0)
        { // going back
            rightcount = 65535 - __HAL_TIM_GET_COUNTER(&htim3);
            leftcount = __HAL_TIM_GET_COUNTER(&htim2);
        }
        if (ack == 0)
        {                                                                                       // while executing command
            distance_travelledA = ((float)leftcount) / 1535.0 * OutsideLabCircumference[index]; // diameter is 6.5cm
            distance_travelledB = (float)rightcount / 1552.0 * OutsideLabCircumference[index];  // get 1552 from cnt1B for one
        }
        else if (ack == 2)
        {                                                                                       // no command sent, only in debug mode
            distance_travelledA = ((float)leftcount) / 1535.0 * OutsideLabCircumference[index]; // diameter is 6.5cm
            distance_travelledB = (float)rightcount / 1552.0 * OutsideLabCircumference[index];  // get 1552 from cnt1B for one
        }
        //			distance_travelledA=((float)leftcount)/1535.0*20.42;//diameter is 6.5cm
        //			distance_travelledB= (float)rightcount/1552.0*20.42;//get 1552 from cnt1B for one revolution
        averagecount = (leftcount + rightcount) / 2;
        averagedistance = (distance_travelledA + distance_travelledB) / 2;
    }
  /* USER CODE END Calculate_Dist */
}

/* USER CODE BEGIN Header_UltraSoundRead */
/**
 * @brief Function implementing the UltrasoundReadT thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UltraSoundRead */
void UltraSoundRead(void *argument)
{
  /* USER CODE BEGIN UltraSoundRead */
    /* Infinite loop */
    uint8_t OLEDStringint[20] = "";
    int heartbeat = 0;

    for (;;)
    {
        //		  if (isMoving) {
        //		  sprintf(OLEDStringint, "Alive: %d\0",heartbeat);
        //		  OLED_ShowString(0, 0, OLEDStringint);
        //		  heartbeat++;

        Ultrasonic_Read();

        //		      sprintf(OLEDStringint,"US: %5d\0 ",(int)Distance);
        //		      OLED_ShowString(0,50,OLEDStringint);
        //
        //			  OLED_Refresh_Gram();

        //		  }
        osDelay(20);
        if (Distance <= 10)
        {
            //				  MotorStop();
        }
    }
  /* USER CODE END UltraSoundRead */
}

/* USER CODE BEGIN Header_IRReadPolling */
/**
 * @brief Function implementing the IRTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_IRReadPolling */
void IRReadPolling(void *argument)
{
  /* USER CODE BEGIN IRReadPolling */
    /* Infinite loop */
    uint32_t adcVal1, adcVal2;
    float voltage1, voltage2;
    /* Infinite loop */
    for (;;)
    {
        if (1)
        {
            //	Left IR Sensor
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 100);
            adcVal1 = HAL_ADC_GetValue(&hadc1); // Raw data
            voltage1 = (adcVal1 / pow(2, 12)) * 3.3;
            LeftIR = 1 / (0.0140817 * pow(voltage1, 2) + 0.00685361 * voltage1 + 0.012403);
            //		sprintf(IR_Left_Str, "Left IR: %2d", IR_Left);
            //		OLED_ShowString(10, 40, IR_Left_Str);

            //	Right IR Sensor
            HAL_ADC_Start(&hadc2);
            HAL_ADC_PollForConversion(&hadc2, 100);
            adcVal2 = HAL_ADC_GetValue(&hadc2); // Raw data
            voltage2 = (adcVal2 / pow(2, 12)) * 3.3;
            RightIR = 1 / (0.0140817 * pow(voltage2, 2) + 0.00685361 * voltage2 + 0.012403);
            //		sprintf(IR_Right_Str, "Right IR: %2d", RightIR);
            //		OLED_ShowString(10, 50, IR_Right_Str);
        }
        osDelay(30);
    }
  /* USER CODE END IRReadPolling */
}

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
