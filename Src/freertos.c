/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
osThreadId defaultTaskHandle;
osThreadId chassis_taskHandle;
osThreadId rc_taskHandle;
osThreadId led_taskHandle;
osThreadId buzzer_taskHandle;
osThreadId plot_taskHandle;
osThreadId auto_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartChassisTask(void const * argument);
void StartRcTask(void const * argument);
void StartLedTask(void const * argument);
void StartBuzzerTask(void const * argument);
void StartPlotTask(void const * argument);
void StartAutoTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of chassis_task */
  osThreadDef(chassis_task, StartChassisTask, osPriorityRealtime, 0, 256);
  chassis_taskHandle = osThreadCreate(osThread(chassis_task), NULL);

  /* definition and creation of rc_task */
  osThreadDef(rc_task, StartRcTask, osPriorityNormal, 0, 256);
  rc_taskHandle = osThreadCreate(osThread(rc_task), NULL);

  /* definition and creation of led_task */
  osThreadDef(led_task, StartLedTask, osPriorityLow, 0, 128);
  led_taskHandle = osThreadCreate(osThread(led_task), NULL);

  /* definition and creation of buzzer_task */
  osThreadDef(buzzer_task, StartBuzzerTask, osPriorityLow, 0, 128);
  buzzer_taskHandle = osThreadCreate(osThread(buzzer_task), NULL);

  /* definition and creation of plot_task */
  osThreadDef(plot_task, StartPlotTask, osPriorityLow, 0, 256);
  plot_taskHandle = osThreadCreate(osThread(plot_task), NULL);

  /* definition and creation of auto_task */
  osThreadDef(auto_task, StartAutoTask, osPriorityNormal, 0, 256);
  auto_taskHandle = osThreadCreate(osThread(auto_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the chassis_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
  extern void chassis_task(void);
  chassis_task();
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartRcTask */
/**
* @brief Function implementing the rc_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRcTask */
void StartRcTask(void const * argument)
{
  /* USER CODE BEGIN StartRcTask */
  /* Infinite loop */
  extern void rc_task(void);
  rc_task();
  /* USER CODE END StartRcTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the led_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  extern void led_task(void);
  led_task();
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartBuzzerTask */
/**
* @brief Function implementing the buzzer_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBuzzerTask */
void StartBuzzerTask(void const * argument)
{
  /* USER CODE BEGIN StartBuzzerTask */
  /* Infinite loop */
  extern void buzzer_task(void);
  buzzer_task();
  /* USER CODE END StartBuzzerTask */
}

/* USER CODE BEGIN Header_StartPlotTask */
/**
* @brief Function implementing the plot_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPlotTask */
void StartPlotTask(void const * argument)
{
  /* USER CODE BEGIN StartPlotTask */
  /* Infinite loop */
  extern void plot_task(void);
  plot_task();
  /* USER CODE END StartPlotTask */
}

/* USER CODE BEGIN Header_StartAutoTask */
/**
* @brief Function implementing the auto_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAutoTask */
void StartAutoTask(void const * argument)
{
  /* USER CODE BEGIN StartAutoTask */
  /* Infinite loop */
  extern void auto_task(void);
  auto_task();
  /* USER CODE END StartAutoTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
