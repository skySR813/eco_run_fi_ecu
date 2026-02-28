/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ig_Task(void const * argument);
void fuel_task(void const * argument);
void Throttle_task(void const * argument);
void TMP_task(void const * argument);
void UI_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
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

}

/* USER CODE BEGIN Header_ig_Task */
/**
  * @brief  Function implementing the IG_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ig_Task */
void ig_Task(void const * argument)
{
  /* USER CODE BEGIN ig_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ig_Task */
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
