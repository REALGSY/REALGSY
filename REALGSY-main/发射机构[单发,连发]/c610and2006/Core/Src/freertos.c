/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"
#include "remoter.h"
#include "motor.h"
#include "shot.h"
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
osThreadId ControlHandle;
osThreadId DieHandle;
/* USER CODE END Variables */
osThreadId M3508Handle;
osThreadId GM6020Handle;
osThreadId shotHandle;


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

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
  /* definition and creation of M3508 */
  osThreadDef(M3508, StartDefaultTask, osPriorityNormal, 0, 128);  //3508??????
  M3508Handle = osThreadCreate(osThread(M3508), NULL);
  
  /* definition and creation of GM6020 */
  osThreadDef(GM6020, StartTask02, osPriorityNormal, 0, 128);  //6020??????
  GM6020Handle = osThreadCreate(osThread(GM6020), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(Control, StartTask03, osPriorityRealtime, 0, 128);   //????????????
  ControlHandle = osThreadCreate(osThread(Control), NULL);

  osThreadDef(Die, StartTask04, osPriorityNormal, 0, 128);   //??????
  DieHandle = osThreadCreate(osThread(Die), NULL);

  osThreadDef(shot, StartTask05, osPriorityNormal, 0, 128);   //??????
  shotHandle = osThreadCreate(osThread(shot), NULL);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  M3508
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument) 
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		DJI_GO();
		moter_TIM2_IRQHandler();  
		osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief GM6020
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)  
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    Motor_SendData(0,0,0,0);
		DJI_GO();
		moter_M6020();
		osDelay(10);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
* @brief choice
* @param argument: Not used
* @retval None
*/
void StartTask03(void const * argument)  
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {  
    if(rc.sw1 == 2 &&rc.sw2 == 2) //??????
    {
      //????????????
      vTaskSuspend(M3508Handle); 
      vTaskSuspend(GM6020Handle);
      vTaskResume(DieHandle);
      vTaskSuspend(shotHandle);
    }
	  if(rc.sw1 == 0 &&rc.sw2 == 0) //??????
    {
      //????????????
      vTaskSuspend(M3508Handle); 
      vTaskSuspend(GM6020Handle);
      vTaskResume(DieHandle);
      vTaskSuspend(shotHandle);
    }
    if(rc.sw1 == 3 &&rc.sw2 == 3) //3508
    {
      //????????????
      vTaskResume(M3508Handle);
      vTaskSuspend(GM6020Handle);
      vTaskSuspend(DieHandle);
      vTaskSuspend(shotHandle);
    }
    if(rc.sw1 == 2 &&rc.sw2 == 3) //6020
    {
      //6020??????
      vTaskSuspend(M3508Handle);
      vTaskResume(GM6020Handle);
      vTaskSuspend(DieHandle);
      vTaskSuspend(shotHandle);
    }
    if(rc.sw1 == 1 &&rc.sw2 == 1)
    {
      //????????????
      vTaskSuspend(M3508Handle);
      vTaskSuspend(GM6020Handle);
      vTaskSuspend(DieHandle);
      vTaskResume(shotHandle);
    }
		osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/**
* @brief godie
* @param argument: Not used
* @retval None
*/
void StartTask04(void const * argument)  
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {    
    godie(); 
		osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/**
* @brief shot
* @param argument: Not used
* @retval None
*/
void StartTask05(void const * argument)  
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {   
    DJI_GO();
    roll_shot_go();
		osDelay(10);
  }
  /* USER CODE END StartTask02 */
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
