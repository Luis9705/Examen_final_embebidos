/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
/* Definitions for readSensor_Task */
osThreadId_t readSensor_TaskHandle;
const osThreadAttr_t readSensor_Task_attributes = {
  .name = "readSensor_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for printOutput_Tas */
osThreadId_t printOutput_TasHandle;
const osThreadAttr_t printOutput_Tas_attributes = {
  .name = "printOutput_Tas",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for alarm_Task */
osThreadId_t alarm_TaskHandle;
const osThreadAttr_t alarm_Task_attributes = {
  .name = "alarm_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for debouncing_Task */
osThreadId_t debouncing_TaskHandle;
const osThreadAttr_t debouncing_Task_attributes = {
  .name = "debouncing_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for updateThreshold */
osThreadId_t updateThresholdHandle;
const osThreadAttr_t updateThreshold_attributes = {
  .name = "updateThreshold",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for distanceQueue */
osMessageQueueId_t distanceQueueHandle;
const osMessageQueueAttr_t distanceQueue_attributes = {
  .name = "distanceQueue"
};
/* Definitions for buttonCMDQueue */
osMessageQueueId_t buttonCMDQueueHandle;
const osMessageQueueAttr_t buttonCMDQueue_attributes = {
  .name = "buttonCMDQueue"
};
/* Definitions for distanceAlarmQueue */
osMessageQueueId_t distanceAlarmQueueHandle;
const osMessageQueueAttr_t distanceAlarmQueue_attributes = {
  .name = "distanceAlarmQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void readSensorTask(void *argument);
void printOutputTask(void *argument);
void alarmTask(void *argument);
void debouncingTask(void *argument);
void updateThresholdTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the queue(s) */
  /* creation of distanceQueue */
  distanceQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &distanceQueue_attributes);

  /* creation of buttonCMDQueue */
  buttonCMDQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &buttonCMDQueue_attributes);

  /* creation of distanceAlarmQueue */
  distanceAlarmQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &distanceAlarmQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of readSensor_Task */
  readSensor_TaskHandle = osThreadNew(readSensorTask, NULL, &readSensor_Task_attributes);

  /* creation of printOutput_Tas */
  printOutput_TasHandle = osThreadNew(printOutputTask, NULL, &printOutput_Tas_attributes);

  /* creation of alarm_Task */
  alarm_TaskHandle = osThreadNew(alarmTask, NULL, &alarm_Task_attributes);

  /* creation of debouncing_Task */
  debouncing_TaskHandle = osThreadNew(debouncingTask, NULL, &debouncing_Task_attributes);

  /* creation of updateThreshold */
  updateThresholdHandle = osThreadNew(updateThresholdTask, NULL, &updateThreshold_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_readSensorTask */
/**
  * @brief  Function implementing the readSensor_Task thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_readSensorTask */
void readSensorTask(void *argument)
{
  /* USER CODE BEGIN readSensorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END readSensorTask */
}

/* USER CODE BEGIN Header_printOutputTask */
/**
* @brief Function implementing the printOutput_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_printOutputTask */
void printOutputTask(void *argument)
{
  /* USER CODE BEGIN printOutputTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END printOutputTask */
}

/* USER CODE BEGIN Header_alarmTask */
/**
* @brief Function implementing the alarm_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_alarmTask */
void alarmTask(void *argument)
{
  /* USER CODE BEGIN alarmTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END alarmTask */
}

/* USER CODE BEGIN Header_debouncingTask */
/**
* @brief Function implementing the debouncing_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_debouncingTask */
void debouncingTask(void *argument)
{
  /* USER CODE BEGIN debouncingTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END debouncingTask */
}

/* USER CODE BEGIN Header_updateThresholdTask */
/**
* @brief Function implementing the updateThreshold thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_updateThresholdTask */
void updateThresholdTask(void *argument)
{
  /* USER CODE BEGIN updateThresholdTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END updateThresholdTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
