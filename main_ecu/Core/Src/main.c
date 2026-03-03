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
#include "ecu_data.h"
#include "ecu_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



//↑バージョン情報も保存させているときはその分のサイズも足す
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
  if( EE24_Init(&hee24, &hi2c3, EE24_ADDRESS_DEFAULT) ){

          EE24_Read(&hee24, 0x0000, raw_map, MAP_SIZE, 1000);
  }
  uint32_t index = 0;
  //RPM軸
  for(int i=0;i<RPM_SIZE;i++){
	  current_map.rpm_axis_ee[i] = raw_map[index] | (raw_map[index+1] << 8);
	  index += 2;
  }
  // TPS軸
  for(int i=0;i<TPS_SIZE;i++){
      current_map.tps_axis_ee[i] = raw_map[index] | (raw_map[index+1] << 8);
      index += 2;
  }

  // AFRマップ
  for(int r=0;r<RPM_SIZE;r++){
      for(int t=0;t<TPS_SIZE;t++){
          current_map.map_fuel_raw_ee[r][t] =
              raw_map[index] | (raw_map[index+1] << 8);

          current_map.fuel_map_ee[r][t] =
              current_map.map_fuel_raw_ee[r][t] / 1000.0f;

          index += 2;
      }
  }

  // 点火マップ
  for(int r=0;r<RPM_SIZE;r++){
      for(int t=0;t<TPS_SIZE;t++){
          current_map.map_ign_ee[r][t] =
              raw_map[index] | (raw_map[index+1] << 8);
          index += 2;
      }
  }

  // CRC
  uint16_t crc_read =
      raw_map[index] | (raw_map[index+1] << 8);

  //デフォルトを使うかEEPROM版を使うか判断
  if(index != MAP_SIZE)
  {
	  HD44780_SetCursor(0,0);
	  HD44780_PrintStr("error! use dmap");
	  HAL_Delay(100);
	  current_map = default_map;
  }else{
	  for(int i=0;i<RPM_SIZE;i++)
	          current_map.rpm_axis[i] = current_map.rpm_axis_ee[i];

	      for(int i=0;i<TPS_SIZE;i++)
	          current_map.tps_axis[i] = current_map.tps_axis_ee[i];

	      for(int r=0;r<RPM_SIZE;r++){
	          for(int t=0;t<TPS_SIZE;t++){
	              current_map.map_fuel[r][t] = current_map.fuel_map_ee[r][t];
	              current_map.map_ign[r][t]  = current_map.map_ign_ee[r][t];
	          }
	      }
  }



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
