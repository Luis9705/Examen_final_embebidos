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
typedef enum  {
  UPDATE_UP_THR,
  UPDATE_DOWN_THR
}update_thr_Type;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THR_INTERVAL 10
#define MAX_DISTANCE 330
#define MIN_DISTANCE 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint16_t D_THRESH = 100;
char button_pressed = 0;

/* USER CODE END Variables */
/* Definitions for readSensor_Task */
osThreadId_t readSensor_TaskHandle;
const osThreadAttr_t readSensor_Task_attributes = {
  .name = "readSensor_Task",
  .priority = (osPriority_t) osPriorityHigh,
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
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for debouncing_Task */
osThreadId_t debouncing_TaskHandle;
const osThreadAttr_t debouncing_Task_attributes = {
  .name = "debouncing_Task",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for updateThreshold */
osThreadId_t updateThresholdHandle;
const osThreadAttr_t updateThreshold_attributes = {
  .name = "updateThreshold",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for oneSecond_Task */
osThreadId_t oneSecond_TaskHandle;
const osThreadAttr_t oneSecond_Task_attributes = {
  .name = "oneSecond_Task",
  .priority = (osPriority_t) osPriorityNormal,
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
/* Definitions for printMutex */
osMutexId_t printMutexHandle;
const osMutexAttr_t printMutex_attributes = {
  .name = "printMutex"
};
/* Definitions for oneSecondSemaphore */
osSemaphoreId_t oneSecondSemaphoreHandle;
const osSemaphoreAttr_t oneSecondSemaphore_attributes = {
  .name = "oneSecondSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osEventFlagsId_t buttonEventFlags;                    // event flags id
/* USER CODE END FunctionPrototypes */

void readSensorTask(void *argument);
void printOutputTask(void *argument);
void alarmTask(void *argument);
void debouncingTask(void *argument);
void updateThresholdTask(void *argument);
void oneSecondTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of printMutex */
  printMutexHandle = osMutexNew(&printMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of oneSecondSemaphore */
  oneSecondSemaphoreHandle = osSemaphoreNew(1, 1, &oneSecondSemaphore_attributes);

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
  buttonCMDQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &buttonCMDQueue_attributes);

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

  /* creation of oneSecond_Task */
  oneSecond_TaskHandle = osThreadNew(oneSecondTask, NULL, &oneSecond_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  buttonEventFlags = osEventFlagsNew(NULL);
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
	  //sensing distance
	  uint16_t  distance = LIDAR_read();

	  osMessageQueuePut(distanceQueueHandle, &distance, 0U, osWaitForever);
	  osMessageQueuePut(distanceAlarmQueueHandle, &distance, 0U, osWaitForever);

	  osDelay(10);
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

	uint16_t distance;
	osStatus_t status;
  for(;;)
  {
	  status = osMessageQueueGet(distanceQueueHandle, &distance, NULL,  osWaitForever);   // wait for message

		if (status == osOK) {

			osSemaphoreAcquire(oneSecondSemaphoreHandle, osWaitForever); //wait for 1 second
			  osStatus_t result = osMutexAcquire(printMutexHandle, osWaitForever);
			  if (result == osOK) {
				  print("Distance: %d cm\r\n", distance);
			    }
			    osMutexRelease(printMutexHandle);

		}
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
	uint16_t distance;
	osStatus_t status;
	  for(;;)
	  {
		  status = osMessageQueueGet(distanceAlarmQueueHandle, &distance, NULL,  osWaitForever);   // wait for message

			if (status == osOK) {

				if(distance < D_THRESH ){
					  osStatus_t result = osMutexAcquire(printMutexHandle, osWaitForever);
					  if (result == osOK) {
						  print("Sensor is too close!!\r\n");
					    }
					    osMutexRelease(printMutexHandle);
				}

			}
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
		uint32_t flags;
		uint8_t updateCmd;
	  for(;;)
	  {
		 flags = osEventFlagsWait(buttonEventFlags,
				 DOWN_THR_MASK |
				 UP_THR_MASK , osFlagsWaitAny, osWaitForever);
	    if(flags!=0){ //A Button was pressed
	    	osDelay(200); //Wait for 10ms
	    	//HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	        if(flags & DOWN_THR_MASK){ //The Button was pressed
	        	updateCmd = UPDATE_DOWN_THR;
	        	osEventFlagsClear(buttonEventFlags, DOWN_THR_MASK);
	        }
	        else if(flags & UP_THR_MASK){ //The Button was pressed
	        	updateCmd = UPDATE_UP_THR;
	        	osEventFlagsClear(buttonEventFlags, UP_THR_MASK);
	        }

	        osMessageQueuePut(buttonCMDQueueHandle, &updateCmd, 0U, osWaitForever);
	        button_pressed = 0;
	    }

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
	uint8_t updateCmd ;
	osStatus_t status;
	  for(;;)
	  {
		status = osMessageQueueGet(buttonCMDQueueHandle, &updateCmd, NULL, osWaitForever);   // wait for message
		if (status == osOK) {
			switch(updateCmd){
				case UPDATE_UP_THR:
					if (D_THRESH + 1 <= MAX_DISTANCE) D_THRESH += THR_INTERVAL;
					break;
				case UPDATE_DOWN_THR:
					if (D_THRESH - 1  >= MIN_DISTANCE) D_THRESH -= THR_INTERVAL;
					break;
				default:
					break;
			}
			  osStatus_t result = osMutexAcquire(printMutexHandle, osWaitForever);
			  if (result == osOK) {
				  print("Distance Threshold updated to: %d\r\n", D_THRESH);
			    }
			    osMutexRelease(printMutexHandle);
		}
	  }
  /* USER CODE END updateThresholdTask */
}

/* USER CODE BEGIN Header_oneSecondTask */
/**
* @brief Function implementing the oneSecond_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_oneSecondTask */
void oneSecondTask(void *argument)
{
  /* USER CODE BEGIN oneSecondTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);

    osSemaphoreRelease(oneSecondSemaphoreHandle);
  }
  /* USER CODE END oneSecondTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
