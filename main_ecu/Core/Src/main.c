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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "liquidcrystal_i2c.h"
#include "ecu_math.h"
#include "ee24.h"
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

/* USER CODE BEGIN PV */
EE24_HandleTypeDef hee24;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int rpm_axis[RPM_SIZE] = {1200, 2500, 3500, 4500, 5500, 6500, 7500, 8500};
int tps_axis[TPS_SIZE] = {0, 10, 25, 40, 60, 100};

float map_fuel[RPM_SIZE][TPS_SIZE] = {{16.5,16.0,15.5,14.7,14.2,13.8},
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
//int rpm_pred = 1000;//予測RPM
//int tps_prev = 0;
// TPS → 加速度予測係数（調整ポイント）
//float accel_gain = 3.0;  // 数字を大きくすると加速予測が強くなる

//各変数管理
volatile int THper = 0;
volatile int rpm_A = 0;
volatile int fdeg = 0;
volatile float AFR_target = 0;
volatile float T_inj_ms = 0;
volatile float T_inj_us = 0;




//燃料用
//インジェクター無効噴射時間
float inj_inv_ms = 1.0;
float AFR_base = 14.7f;
float T_base = 6.5f;

volatile float tmp = 0;
//速度、タイムなど
//float speedKmh = 0.0;
float timesecmin = 0.0;


// ===== 点火用 =====
volatile uint32_t crank_last_us = 0;
volatile uint32_t crank_period_us = 15000;
volatile int32_t delay_us = 0;
int dwell_us = 0;
volatile int crank_flag = 0;
volatile int32_t next_delay_us = 2000;




void UIprint_int(int com1,int *last_com1,char comm1[16],int yoko1,int tate1){
	if(com1 != *last_com1){
		*last_com1 = com1;
		sprintf(comm1,"%4d",com1);
		//osMutexWait(I2C_mutexHandle,osWaitForever);
		HD44780_SetCursor(yoko1,tate1);
		HD44780_PrintStr(comm1);
		//osDelay(100);
		//HD44780_PrintStr("    ");
		//osMutexRelease(I2C_mutexHandle);

	}
}

void UIprint_float(float com2,float *last_com2,char comm2[16],int yoko2,int tate2)
{
    if (fabsf(com2 - *last_com2) > 0.05f)
    {
        snprintf(comm2,16,"%4.1f",com2);
        HD44780_SetCursor(yoko2,tate2);
        HD44780_PrintStr(comm2);
        *last_com2 = com2;
    }
}
/*
void UIprint_float(float com2,float *last_com2,char comm2[16],int yoko2,int tate2){
	if(com2 != *last_com2){
		*last_com2 = com2;
		sprintf(comm2,"%4.1f",com2);
		osMutexWait(I2C_mutexHandle,osWaitForever);
		HD44780_SetCursor(yoko2,tate2);
		HD44780_PrintStr(comm2);
		osDelay(50);
		HD44780_PrintStr("    ");
		osMutexRelease(I2C_mutexHandle);

	}
}
*/

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
  MX_TIM5_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);   // 周期計測用
  HAL_TIM_Base_Start(&htim3);   // 点火遅延用
  HAL_TIM_Base_Start(&htim5);   //燃料噴射時間用
  // EEPROM を初期化




  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

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

	  // オーバーフロー対策
	        if(now >= crank_last_us) {
	            crank_period_us = now - crank_last_us;
	        } else {
	            crank_period_us = (0xFFFFFFFF - crank_last_us) + now + 1;
	        }
	        crank_last_us = now;

	        if(crank_period_us > 0) {
	          rpm_A = 60000000UL / crank_period_us;
	        }else{
	          rpm_A = 0;
	        }

	  	    if (rpm_A < 3000)      dwell_us = 3000;
	  	    else if (rpm_A < 6000) dwell_us = 2000;
	  	    else                   dwell_us = 1500;


	  uint32_t target1 = now + next_delay_us;

	 // __HAL_TIM_SET_COUNTER(&htim3, 0);
	  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, target);
	  //HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, target1);
	  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
  }else if(GPIO_Pin == EXTI1_cam_Pin){
	  HAL_GPIO_WritePin(fuel_output_GPIO_Port,fuel_output_Pin,GPIO_PIN_SET);
	  uint32_t now1 = __HAL_TIM_GET_COUNTER(&htim5);
	  uint32_t target2 = now1 + (uint32_t)T_inj_us;

	  //噴射時間セット

		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, target2);

		 HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);




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
      HAL_GPIO_WritePin(IG_output_GPIO_Port, IG_output_Pin, GPIO_PIN_RESET);
      HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
    }
  else if (htim->Instance == TIM5){
	  HAL_GPIO_WritePin(fuel_output_GPIO_Port, fuel_output_Pin, GPIO_PIN_RESET);
	  HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_1);
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
