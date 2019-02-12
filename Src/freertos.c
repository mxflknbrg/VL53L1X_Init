/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "utilities.h"

// Watchdog #includes
#include "stm32l4xx_hal.h"
#include "wwdg.h"

// VL53L1X #includes
#include "vl53l1_api.h"

// SEGGER RTT
#include "SEGGER_RTT.h"

//#define USE_WATCHDOG

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
// Watchdog Variables
extern uint8_t wwdg_flag;
extern WWDG_HandleTypeDef hwwdg;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId RangingTaskHandle;
osThreadId CANTaskHandle;
osThreadId BluetoothTaskHandle;
osMessageQId rangingqueueHandle;
osTimerId wwdgRefreshTimerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void RangingTaskEntry(void const * argument);
void CANTaskEntry(void const * argument);
void BluetoothTaskEntry(void const * argument);
void wwdgRefreshTimerCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);
void vApplicationDaemonTaskStartupHook(void);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	/* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	for(;;){
		__asm("NOP");						// For Debugging
	}
	osThreadSuspendAll();					// WWDG will not be kicked again -> MCU reset
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	for(;;){
		__asm("NOP");						// For Debugging
	}
	osThreadSuspendAll();					// WWDG will not be kicked again -> MCU reset
}
/* USER CODE END 5 */

/* USER CODE BEGIN DAEMON_TASK_STARTUP_HOOK */
void vApplicationDaemonTaskStartupHook(void)
{
#ifdef USE_WATCHDOG
	osTimerStart(wwdgRefreshTimerHandle, 20);
#endif		// USE_WATCHDOG
	//	osTimerStart(ledTimerHandle, 100);
}
/* USER CODE END DAEMON_TASK_STARTUP_HOOK */

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

  /* Create the timer(s) */
  /* definition and creation of wwdgRefreshTimer */
  osTimerDef(wwdgRefreshTimer, wwdgRefreshTimerCallback);
  wwdgRefreshTimerHandle = osTimerCreate(osTimer(wwdgRefreshTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of RangingTask */
  osThreadDef(RangingTask, RangingTaskEntry, osPriorityHigh, 0, 1280);
  RangingTaskHandle = osThreadCreate(osThread(RangingTask), NULL);

  /* definition and creation of CANTask */
  osThreadDef(CANTask, CANTaskEntry, osPriorityAboveNormal, 0, 128);
  CANTaskHandle = osThreadCreate(osThread(CANTask), NULL);

  /* definition and creation of BluetoothTask */
  osThreadDef(BluetoothTask, BluetoothTaskEntry, osPriorityNormal, 0, 128);
  BluetoothTaskHandle = osThreadCreate(osThread(BluetoothTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of rangingqueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(rangingqueue, 8, uint8_t);
  rangingqueueHandle = osMessageCreate(osMessageQ(rangingqueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
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

  /* USER CODE BEGIN StartDefaultTask */
	WWDGSetTaskActive(kDefaultTask);
	/* Infinite loop */
	for(;;)
	{
		SEGGER_RTT_WriteString(0, "Default Task\n\r");

		WWDGAlive(kDefaultTask);

		osDelay(osWaitForever);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_RangingTaskEntry */
/**
 * @brief Function implementing the RangingTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_RangingTaskEntry */
void RangingTaskEntry(void const * argument)
{
  /* USER CODE BEGIN RangingTaskEntry */
	WWDGSetTaskActive(kRangingTask);

	VL53L1_Dev_t 							lidararray[3];
	VL53L1_Error 							status;
	VL53L1_RangingMeasurementData_t 		rangingdata[3];
	uint8_t									rangestatus				= 0;

	WWDGSetTaskInactive(kRangingTask);

	status = VL53L1XInitModules(lidararray);

	__asm("NOP");

	WWDGSetTaskActive(kRangingTask);

	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

		WWDGAlive(kRangingTask);

		status = VL53L1XSequentialRanging(lidararray, rangingdata);

		rangestatus = 0;

		for(uint8_t i = 0; i < NUMBER_OF_MODULES; i++)
			if (rangingdata[i].RangeStatus == 0){
				rangestatus |= 1<<(7-i);
				asm("NOP");
			}

		osMessagePut(rangingqueueHandle, rangestatus, 0);

		for(uint8_t i = 0; i < NUMBER_OF_MODULES; i++)
			osMessagePut(rangingqueueHandle, (rangingdata[i].RangeMilliMeter / 10), 0);

		osSignalSet(CANTaskHandle, kVL53L1XSendCANMessage);

		WWDGAlive(kRangingTask);

		osDelay(1);
	}
  /* USER CODE END RangingTaskEntry */
}

/* USER CODE BEGIN Header_CANTaskEntry */
/**
 * @brief Function implementing the CANTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CANTaskEntry */
void CANTaskEntry(void const * argument)
{
  /* USER CODE BEGIN CANTaskEntry */
	uint8_t						message[NUMBER_OF_MODULES + 1]		= {0};
	osEvent						queueevent;

	MCP2515Init();

	osDelay(5);

	/* Infinite loop */
	for(;;)
	{
		osSignalWait(kVL53L1XSendCANMessage, osWaitForever);

		for(uint8_t i = 0; i <= NUMBER_OF_MODULES; i++){
			queueevent = osMessageGet(rangingqueueHandle, osWaitForever);
			message[i] = (uint8_t)queueevent.value.v;
		}

		MCP2515SendMessage(0, CAN_ID, kCANHighPriority, message, sizeof message);

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		osDelay(1);
	}
  /* USER CODE END CANTaskEntry */
}

/* USER CODE BEGIN Header_BluetoothTaskEntry */
/**
 * @brief Function implementing the BluetoothTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_BluetoothTaskEntry */
void BluetoothTaskEntry(void const * argument)
{
  /* USER CODE BEGIN BluetoothTaskEntry */
	/* Infinite loop */
	for(;;)
	{
		osDelay(50);
	}
  /* USER CODE END BluetoothTaskEntry */
}

/* wwdgRefreshTimerCallback function */
void wwdgRefreshTimerCallback(void const * argument)
{
  /* USER CODE BEGIN wwdgRefreshTimerCallback */
	if(wwdg_flag == WWDGGetActiveTasks()){
		HAL_WWDG_Refresh(&hwwdg);
		wwdg_flag = 0;
	}
  /* USER CODE END wwdgRefreshTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
