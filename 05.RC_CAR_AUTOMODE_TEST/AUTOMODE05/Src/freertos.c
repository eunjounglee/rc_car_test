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
#include "cmsis_os2.h"
#include "ultrasonic.h"        // US_Left_cm/Right/Center
#include "automode.h"          // 자동주행 상태 getter (아래 참고)
#include "stdio.h"
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
/* Definitions for sonic */
osThreadId_t sonicHandle;
const osThreadAttr_t sonic_attributes = {
  .name = "sonic",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for autocontrol */
osThreadId_t autocontrolHandle;
const osThreadAttr_t autocontrol_attributes = {
  .name = "autocontrol",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for debug */
osThreadId_t debugHandle;
const osThreadAttr_t debug_attributes = {
  .name = "debug",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ultrasonic(void *argument);
void automode(void *argument);
void debugtask(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of sonic */
  sonicHandle = osThreadNew(ultrasonic, NULL, &sonic_attributes);

  /* creation of autocontrol */
  autocontrolHandle = osThreadNew(automode, NULL, &autocontrol_attributes);

  /* creation of debug */
  debugHandle = osThreadNew(debugtask, NULL, &debug_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ultrasonic */
/**
  * @brief  Function implementing the sonic thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ultrasonic */
void ultrasonic(void *argument)
{
  /* USER CODE BEGIN ultrasonic */
  /* Infinite loop */
	US_Init();
  for(;;)
  {
  	US_Update();
    osDelay(10);
  }
  /* USER CODE END ultrasonic */
}

/* USER CODE BEGIN Header_automode */
/**
* @brief Function implementing the autocontrol thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_automode */
void automode(void *argument)
{
  /* USER CODE BEGIN automode */
  /* Infinite loop */
  	AutoMode_Init();
  for(;;)
  {
  	AutoMode_Update();
  	osDelay(60);
  }
  /* USER CODE END automode */
}

/* USER CODE BEGIN Header_debugtask */
/**
* @brief Function implementing the debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_debugtask */
void debugtask(void *argument)
{
  /* USER CODE BEGIN debugtask */
  /* Infinite loop */
  for(;;)
  {

//  		printf("[TURN] F=%u L=%u R=%u\n", US_Center_cm(), US_Left_cm(), US_Right_cm());
//
//      osDelay(200);
  }
  /* USER CODE END debugtask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

