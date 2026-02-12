/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  * この順で保存
  * git status
    git add .
    git commit -m "tune injection map"
    git push
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "liquidcrystal_i2c.h"
#include "ecu_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BASE_ANGLE 40//センサー基準位置27では小さすぎた
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

osThreadId IG_TaskHandle;
osThreadId fuel_TaskHandle;
osThreadId Throttle_TaskHandle;
osThreadId TMP_TaskHandle;
osThreadId UI_TaskHandle;
osMessageQId sensor_QueueHandle;
osMessageQId command_QueueHandle;
osMessageQId configQueueHandle;
osMutexId Uart_mutexHandle;
osMutexId I2C_mutexHandle;
osSemaphoreId Crank_SemHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
void ig_Task(void const * argument);
void fuel_task(void const * argument);
void Throttle_task(void const * argument);
void TMP_task(void const * argument);
void UI_task(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int rpm_axis[RPM_SIZE] = {1200, 2500, 3500, 4500, 5500, 6500, 7500, 8500};
int tps_axis[TPS_SIZE] = {0, 10, 25, 40, 60, 100};

double map_fuel[RPM_SIZE][TPS_SIZE] = {{16.5,16.0,15.5,14.7,14.2,13.8},
		                               {16.8,16.2,15.0,14.7,14.0,13.6},
									   {17.0,16.3,15.0,14.5,13.8,13.2},
									   {17.0,16.2,14.8,14.2,13.5,13.0},
									   {16.8,16.0,14.5,14.0,13.2,12.8},
									   {16.5,15.8,14.2,13.8,13.0,12.6},
									   {16.2,15.5,14.0,13.6,12.8,12.5},
									   {16.0,15.2,13.8,13.5,12.7,12.4}}; //空燃比表示(デフォルト)

int map_ign[RPM_SIZE][TPS_SIZE] = {
 {8,10,12,14,16,16},
 {12,16,20,22,24,24},
 {14,20,24,26,28,28},
 {16,22,26,28,30,30},
 {16,24,28,30,32,32},
 {14,22,26,28,30,30},
 {12,20,24,26,28,28},
 {10,18,22,24,26,26}
};//進角角度表示(デフォルト)上死点前



//可変進角管理用
int rpm_pred = 1000;//予測RPM
int tps_prev = 0;
// TPS → 加速度予測係数（調整ポイント）
float accel_gain = 3.0;  // 数字を大きくすると加速予測が強くなる

//各変数管理
volatile int THper = 0;
volatile int rpm_A = 0;
volatile int fdeg = 0;
volatile float tmp = 0;
//速度、タイムなど
double speedKmh = 0.0;
double timesecmin = 0.0;



// ピン定義 ※書き換えてね→使わない
/*
const int Cranksensor_PIN = 3;
const int Camsensor_PIN = 4;
const int Igoutput_PIN = 10;
const int Fueloutput_PIN = 11;
const int Throttlesensor_PIN = 26;//ADC 0
const int Tmpsensor_PIN = 27;//ADC 1
const int serial_1_TX = 0;//メインサブ通信用 GP0
const int serial_1_RX = 1;//メインサブ通信用 GP1
*/

// ===== 点火用 =====
volatile uint32_t crank_last_us = 0;
volatile uint32_t crank_period_us = 15000;
volatile int32_t delay_us = 0;
int dwell_us = 0;
volatile int crank_flag = 0;
volatile int32_t next_delay_us = 2000;




void UIprint_int(volatile int com1,volatile int last_com1,char comm1[16],int yoko1,int tate1){
	if(com1 != last_com1){
		last_com1 = com1;
		sprintf(comm1,"%d",com1);
		osMutexWait(I2C_mutexHandle,osWaitForever);
		HD44780_SetCursor(yoko1,tate1);
		HD44780_PrintStr(comm1);
		osDelay(500);
		HD44780_PrintStr("    ");
		osMutexRelease(I2C_mutexHandle);

	}
}
void UIprint_float(volatile float com2,volatile int last_com2,char comm2[16],int yoko2,int tate2){
	if(com2 != last_com2){
		last_com2 = com2;
		sprintf(comm2,"%f",com2);
		osMutexWait(I2C_mutexHandle,osWaitForever);
		HD44780_SetCursor(yoko2,tate2);
		HD44780_PrintStr(comm2);
		osDelay(500);
		HD44780_PrintStr("    ");
		osMutexRelease(I2C_mutexHandle);

	}
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
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);   // 周期計測用
  HAL_TIM_Base_Start(&htim3);   // 点火遅延用

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of Uart_mutex */
  osMutexDef(Uart_mutex);
  Uart_mutexHandle = osMutexCreate(osMutex(Uart_mutex));

  /* definition and creation of I2C_mutex */
  osMutexDef(I2C_mutex);
  I2C_mutexHandle = osMutexCreate(osMutex(I2C_mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of Crank_Sem */
  osSemaphoreDef(Crank_Sem);
  Crank_SemHandle = osSemaphoreCreate(osSemaphore(Crank_Sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of sensor_Queue */
  osMessageQDef(sensor_Queue, 16, uint16_t);
  sensor_QueueHandle = osMessageCreate(osMessageQ(sensor_Queue), NULL);

  /* definition and creation of command_Queue */
  osMessageQDef(command_Queue, 16, uint16_t);
  command_QueueHandle = osMessageCreate(osMessageQ(command_Queue), NULL);

  /* definition and creation of configQueue */
  osMessageQDef(configQueue, 16, uint16_t);
  configQueueHandle = osMessageCreate(osMessageQ(configQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of IG_Task */
  osThreadDef(IG_Task, ig_Task, osPriorityRealtime, 0, 128);
  IG_TaskHandle = osThreadCreate(osThread(IG_Task), NULL);

  /* definition and creation of fuel_Task */
  osThreadDef(fuel_Task, fuel_task, osPriorityRealtime, 0, 128);
  fuel_TaskHandle = osThreadCreate(osThread(fuel_Task), NULL);

  /* definition and creation of Throttle_Task */
  osThreadDef(Throttle_Task, Throttle_task, osPriorityHigh, 0, 128);
  Throttle_TaskHandle = osThreadCreate(osThread(Throttle_Task), NULL);

  /* definition and creation of TMP_Task */
  osThreadDef(TMP_Task, TMP_task, osPriorityAboveNormal, 0, 128);
  TMP_TaskHandle = osThreadCreate(osThread(TMP_Task), NULL);

  /* definition and creation of UI_Task */
  osThreadDef(UI_Task, UI_task, osPriorityLow, 0, 256);
  UI_TaskHandle = osThreadCreate(osThread(UI_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sConfig.Channel = ADC_CHANNEL_1;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sConfig.Channel = ADC_CHANNEL_4;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 179;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 179;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 179;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|IG_output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : EXTI0_crank_Pin */
  GPIO_InitStruct.Pin = EXTI0_crank_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(EXTI0_crank_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin IG_output_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|IG_output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == EXTI0_crank_Pin)   // ← クランク入力ピンに合わせる
  {
	uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);

	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // デバッグ用

	crank_period_us = now - crank_last_us;
	crank_last_us = now;
	//if (crank_period_us < 3000) return;   // ≒ 10000rpm以上
	//if (crank_period_us > 100000) return; // クランキング異常


	rpm_A = 60000000UL / crank_period_us;

	// RTOSタスクに渡すだけ
	 //osSemaphoreRelease(Crank_SemHandle);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(Crank_SemHandle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

  }
}
*/
//クランク信号割り込み
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == EXTI0_crank_Pin)   // ← クランク入力ピンに合わせる
  {
	  uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
	  crank_period_us = now - crank_last_us;
	  crank_last_us = now;

	  uint32_t target = now + next_delay_us;

	 // __HAL_TIM_SET_COUNTER(&htim3, 0);
	  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, target);
	  //HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, target);
	  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{//点火開始
  if (htim->Instance == TIM2)
  {
    HAL_GPIO_WritePin(IG_output_GPIO_Port, IG_output_Pin, GPIO_PIN_SET); // IGN ON
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_SET); // デバッグ用

    // TIM4カウンタリセット
        __HAL_TIM_SET_COUNTER(&htim4, 0);

        // dwell時間セット
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, dwell_us);

        HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);


   // HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
  }//点火終わり
  else if (htim->Instance == TIM4)
    {
        // Spark OFF
        HAL_GPIO_WritePin(IG_output_GPIO_Port, IG_output_Pin, GPIO_PIN_RESET);
        HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
    }
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_ig_Task */
/**
  * @brief  Function implementing the IG_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ig_Task */
void ig_Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */


  for(;;)
  {
	        rpm_A = 60000000UL / crank_period_us;
	        if (rpm_A < 3000)      dwell_us = 3000;
	        else if (rpm_A < 6000) dwell_us = 2000;
	        else                   dwell_us = 1500;


	        fdeg = getValue_i(rpm_A, THper, map_ign);
	        if (fdeg < 0)  fdeg = 0;
	        if (fdeg > 35) fdeg = 35;

	        int32_t sdeg = BASE_ANGLE - fdeg;
	        delay_us = (sdeg * crank_period_us) / 360;
	        if (delay_us < 50) delay_us = 50;
	        if (delay_us > 60000) delay_us = 60000;
	        next_delay_us = delay_us;

	        //__HAL_TIM_SET_COUNTER(&htim3, 0);
	        //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, delay_us);
	        //HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);


	      osDelay(1);

  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_fuel_task */
/**
* @brief Function implementing the fuel_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fuel_task */
void fuel_task(void const * argument)
{
  /* USER CODE BEGIN fuel_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END fuel_task */
}

/* USER CODE BEGIN Header_Throttle_task */
/**
* @brief Function implementing the Throttle_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Throttle_task */
void Throttle_task(void const * argument)
{
  /* USER CODE BEGIN Throttle_task */
  /* Infinite loop */
  for(;;)
  {
	  // ADCの開始
	  	          HAL_ADC_Start(&hadc1);

	  	          // ADC変換が完了するまで待機
	  	          if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	  	          {
	  	              // ADCの値を取得
	  	              uint32_t TH = HAL_ADC_GetValue(&hadc1);

	  	              // ADCの最大値（12ビット分解能の場合：4095）を基に百分率に変換
	  	              THper = (TH * 100) / 4095;
	  	          }

	  	          // ADCの停止
	  	          HAL_ADC_Stop(&hadc1);

	  	          osDelay(5);
  }
  /* USER CODE END Throttle_task */
}

/* USER CODE BEGIN Header_TMP_task */
/**
* @brief Function implementing the TMP_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TMP_task */
void TMP_task(void const * argument)
{
  /* USER CODE BEGIN TMP_task */
  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_Start(&hadc2);

	  	  	          // ADC変換が完了するまで待機
	  	  	          if (HAL_ADC_PollForConversion(&hadc2, 100) == HAL_OK)
	  	  	          {
	  	  	              // ADCの値を取得
	  	  	              uint32_t tmpp = HAL_ADC_GetValue(&hadc2);

	  	  	              // 温度℃変換
	  	  	              tmp = GetTMP(tmpp);
	  	  	          }

	  	  	          // ADCの停止
	  	  	          HAL_ADC_Stop(&hadc2);

	  	  	          osDelay(5);
  }
  /* USER CODE END TMP_task */
}

/* USER CODE BEGIN Header_UI_task */
/**
* @brief Function implementing the UI_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UI_task */
void UI_task(void const * argument)
{
  /* USER CODE BEGIN UI_task */

	static int last_rpm = -1;
	char rpmm[16];

	static int last_deg = -1;
	char degg[16];

	static int last_tps = -1;
	char tpss[16];

	static int last_tmp = -1;
	char tmmp[16];
	osMutexWait(I2C_mutexHandle, osWaitForever);
	HD44780_Init(2);
	HD44780_Clear();
	osMutexRelease(I2C_mutexHandle);
  /* Infinite loop */
  for(;;)
  {
	  UIprint_int(rpm_A,last_rpm,rpmm,0,0);
	  UIprint_int(fdeg,last_deg,degg,0,1);
	  UIprint_int(THper,last_tps,tpss,3,1);
	  UIprint_float(tmp,last_tmp,tmmp,6,1);
  }
  /* USER CODE END UI_task */
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
