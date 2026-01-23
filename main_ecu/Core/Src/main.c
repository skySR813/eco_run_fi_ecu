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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RPM_SIZE 8//マップサイズ変更はここで
#define TPS_SIZE 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

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
osSemaphoreId Crank_SemHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_I2C1_Init(void);
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

int map_fuel[RPM_SIZE][TPS_SIZE]; //空燃比表示
int map_ign[RPM_SIZE][TPS_SIZE];//進角角度表示

//可変進角管理用
int rpm_pred = 1000;//予測RPM
int tps_prev = 0;
// TPS → 加速度予測係数（調整ポイント）
float accel_gain = 3.0;  // 数字を大きくすると加速予測が強くなる


float THper = 0;
volatile int rpm_A = 0;

//速度、タイムなど
double speedKmh = 0.0;
double timesecmin = 0.0;

//温度管理用（センサーの定数など)
const float BETA = 3435.0;     // サーミスタのB定数 103AT-2-34119
const float R25 = 10000.0;     // 25℃でのサーミスタの抵抗値 (10kΩ)
const float T0 = 298.15;       // 25℃ = 298.15K
const float VREF = 3.3;        // 参照電圧
const float R_FIXED = 10000.0; // 分圧抵抗 (10kΩ)

// ピン定義 ※書き換えてね
const int Cranksensor_PIN = 3;
const int Camsensor_PIN = 4;
const int Igoutput_PIN = 10;
const int Fueloutput_PIN = 11;
const int Throttlesensor_PIN = 26;//ADC 0
const int Tmpsensor_PIN = 27;//ADC 1
const int serial_1_TX = 0;//メインサブ通信用 GP0
const int serial_1_RX = 1;//メインサブ通信用 GP1

// t = 0～256（固定小数点）で線形補間
static inline int lerp_int(int a, int b, int t)
{
    return a + (((b - a) * t) >> 8);   // >>8 は ÷256
}

static int findIndex(int valuea, const int *axis, int sizea)
{
    for (int i = 0; i < sizea - 1; i++) {
        if (valuea >= axis[i] && valuea <= axis[i + 1]) {
            return i;
        }
    }
    return sizea - 2;
}


int getValue(int rpm, int tps, int mapp)
{
    // 区間インデックス
    int i = findIndex(rpm, rpm_axis, RPM_SIZE);
    int j = findIndex(tps,  tps_axis, TPS_SIZE);

    // 補間係数を 0～256 に変換（固定小数点）
    int t_rpm = ((rpm - rpm_axis[i]) << 8) / (rpm_axis[i+1] - rpm_axis[i]);
    int t_tps = ((tps - tps_axis[j]) << 8) / (tps_axis[j+1] - tps_axis[j]);

    // まずRPM方向補間
    int v1 = lerp_int(mapp[i][j],     mapp[i+1][j],     t_rpm);
    int v2 = lerp_int(mapp[i][j+1],   mapp[i+1][j+1],   t_rpm);

    // 次にTPS方向補間
    int result = lerp_int(v1, v2, t_tps);

    return result;   // int のまま返す
}

//温度計算
float GetTMP(int TMPinput){
  float voltage = (TMPinput / 4095.0) * VREF;//ADC値を電圧に変換
  float resistance = (R_FIXED * voltage) / (VREF - voltage); // サーミスタの抵抗値計算

  // Steinhart-Hart 方程式を用いて温度 (K) を算出
    float temperatureK = 1.0f / ((1.0f / T0) + (1.0f / BETA) * logf(resistance / R25));

    // 絶対温度 (K) から摂氏 (°C) に変換
    return temperatureK - 273.15;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of Uart_mutex */
  osMutexDef(Uart_mutex);
  Uart_mutexHandle = osMutexCreate(osMutex(Uart_mutex));

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
  osThreadDef(UI_Task, UI_task, osPriorityNormal, 0, 128);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
    osDelay(1);
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
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UI_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
