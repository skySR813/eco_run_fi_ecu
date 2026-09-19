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
#include "crc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "liquidcrystal_i2c.h"
#include "ecu_math.h"
#include "ee24.h"
#include "ecu_data.h"
#include "ecu_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "xbee_ecu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
EE24_HandleTypeDef hee24;
extern XBee_ECU_Handle_t xbee_ecu;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



//↑バージョン情報も保存させているときはその分のサイズも足す
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static uint16_t map_crc16(const uint8_t *data, uint32_t length);
static uint8_t load_map_from_payload(const uint8_t *data, uint32_t length);
extern osMutexId I2C_mutexHandle;

int XBee_GetEngineRPM(void)
{
    /* A previous RPM value is not evidence that the engine is still rotating. */
    if (!crank_is_synchronised ||
        (HAL_GetTick() - crank_last_edge_ms) > CRANK_TIMEOUT_MS ||
        rpm_A <= 0)
        return 0;

    if (rpm_A > 65535)
        return 65535;

    return (uint16_t)rpm_A;
}


HAL_StatusTypeDef XBee_WriteBin(
    const uint8_t *data,
    uint16_t length
)
{
    if (data == NULL || length != MAP_SIZE)
        return HAL_ERROR;

    if (osMutexWait(I2C_mutexHandle, osWaitForever) != osOK)
        return HAL_ERROR;
    uint8_t write_ok = EE24_Write(&hee24, 0x0000U, (uint8_t *)data, length, 1000U);
    (void)osMutexRelease(I2C_mutexHandle);
    return write_ok ? HAL_OK : HAL_ERROR;
}


HAL_StatusTypeDef XBee_VerifyBin(
    const uint8_t *data,
    uint16_t length
)
{
    uint8_t verify[MAP_SIZE];

    if (data == NULL || length != MAP_SIZE ||
        osMutexWait(I2C_mutexHandle, osWaitForever) != osOK)
    {
        return HAL_ERROR;
    }
    uint8_t read_ok = EE24_Read(&hee24, 0x0000U, verify, MAP_SIZE, 1000U);
    (void)osMutexRelease(I2C_mutexHandle);
    if (!read_ok ||
        memcmp(data, verify, MAP_SIZE) != 0 ||
        !load_map_from_payload(verify, MAP_SIZE))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */






static uint16_t map_crc16(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFU;

    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint32_t bit = 0; bit < 8U; bit++)
        {
            crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
        }
    }

    return (uint16_t)((crc ^ 0xFFFFFFFFU) & 0xFFFFU);
}

static uint8_t load_map_from_payload(const uint8_t *data, uint32_t length)
{
    mapdata candidate = default_map;
    uint32_t index = 0U;
    uint16_t stored_crc;

    if (data == NULL || length != MAP_SIZE)
        return 0U;

    stored_crc = (uint16_t)data[MAP_DATA_SIZE] |
                 ((uint16_t)data[MAP_DATA_SIZE + 1U] << 8U);
    if (map_crc16(data, MAP_DATA_SIZE) != stored_crc)
        return 0U;

    for (int i = 0; i < RPM_SIZE; i++)
    {
        candidate.rpm_axis[i] = (int)((uint16_t)data[index] |
                                      ((uint16_t)data[index + 1U] << 8U));
        index += 2U;
        if (candidate.rpm_axis[i] > 12000 ||
            (i > 0 && candidate.rpm_axis[i] <= candidate.rpm_axis[i - 1]))
            return 0U;
    }

    for (int i = 0; i < TPS_SIZE; i++)
    {
        candidate.tps_axis[i] = (int)((uint16_t)data[index] |
                                      ((uint16_t)data[index + 1U] << 8U));
        index += 2U;
        if (candidate.tps_axis[i] > 100 ||
            (i > 0 && candidate.tps_axis[i] <= candidate.tps_axis[i - 1]))
            return 0U;
    }

    for (int r = 0; r < RPM_SIZE; r++)
    {
        for (int t = 0; t < TPS_SIZE; t++)
        {
            uint16_t value = (uint16_t)data[index] |
                             ((uint16_t)data[index + 1U] << 8U);
            index += 2U;
            candidate.map_fuel_raw_ee[r][t] = value;
            candidate.map_fuel[r][t] = (float)value / 1000.0f;
            if (candidate.map_fuel[r][t] < 10.0f || candidate.map_fuel[r][t] > 18.0f)
                return 0U;
        }
    }

    for (int r = 0; r < RPM_SIZE; r++)
    {
        for (int t = 0; t < TPS_SIZE; t++)
        {
            int16_t value = (int16_t)((uint16_t)data[index] |
                                      ((uint16_t)data[index + 1U] << 8U));
            index += 2U;
            candidate.map_ign_ee[r][t] = value;
            candidate.map_ign[r][t] = (int)value;
            if (value < -20 || value > 40)
                return 0U;
        }
    }

    if (index != MAP_DATA_SIZE)
        return 0U;

    current_map = candidate;
    return 1U;
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_I2C3_Init();
  MX_CRC_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  XBee_ECU_Init(
  	        &xbee_ecu,
  	        &huart3,
  			XBee_GetEngineRPM,
  	        XBee_WriteBin,
  	        XBee_VerifyBin
  	    );
  HAL_TIM_Base_Start(&htim2);   // 周期計測用
  HAL_TIM_Base_Start(&htim3);   // 点火遅延用
  HAL_TIM_Base_Start(&htim5);   //燃料噴射時間用
  HD44780_Init(2);
  HD44780_Clear();
  HD44780_PrintStr("1");
  HAL_Delay(500);
  for(int addr=0; addr<128; addr++)
  {
      if(HAL_I2C_IsDeviceReady(&hi2c3, addr<<1, 1, 10) == HAL_OK)
      {
          printf("I2C device: 0x%02X\n",addr);
      }
  }
  //HAL_Delay(10000);
  /* Invalid, erased, or partially written EEPROM must never become a map. */
  if (!EE24_Init(&hee24, &hi2c3, EE24_ADDRESS_DEFAULT) ||
      !EE24_Read(&hee24, 0x0000U, raw_map, MAP_SIZE, 1000U) ||
      !load_map_from_payload(raw_map, MAP_SIZE))
  {
      current_map = default_map;
      HD44780_Clear();
      HD44780_PrintStr("default map");
  }
  else
  {
      HD44780_Clear();
      HD44780_PrintStr("map loaded");
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
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
      uint32_t period;

      if (!crank_is_synchronised)
      {
          crank_last_us = now;
          crank_last_edge_ms = HAL_GetTick();
          crank_is_synchronised = 1U;
          return;
      }

      /* Unsigned subtraction is correct across a 32-bit timer wrap. */
      period = now - crank_last_us;
      if (period < CRANK_PERIOD_MIN_US || period > CRANK_PERIOD_MAX_US)
      {
          return;
      }

      crank_period_us = period;
      crank_last_us = now;
      crank_last_edge_ms = HAL_GetTick();
      rpm_A = (int)(60000000UL / crank_period_us);

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
	  //カム信号割り込み
	  //インジェクターテストモードでは割り込み無効化
	  if(injector_test_mode)
	      {
	          return;
	      }

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
    }//燃料噴射終わり
  else if (htim->Instance == TIM5){
	  HAL_GPIO_WritePin(fuel_output_GPIO_Port, fuel_output_Pin, GPIO_PIN_RESET);
	  HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_1);
  }
}







void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        XBee_ECU_RxCpltCallback(&xbee_ecu, huart);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        XBee_ECU_ErrorCallback(&xbee_ecu, huart);
    }
}






/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
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
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(200);
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
