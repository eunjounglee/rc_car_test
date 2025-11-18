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
#include "ultrasonic.h"
#include "automode.h"
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
/* Definitions for ultrasonicTask */
osThreadId_t ultrasonicTaskHandle;
const osThreadAttr_t ultrasonicTask_attributes = {
  .name = "ultrasonicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ultra(void *argument);
void ctrl(void *argument);

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
  /* creation of ultrasonicTask */
  ultrasonicTaskHandle = osThreadNew(ultra, NULL, &ultrasonicTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(ctrl, NULL, &controlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ultra */
/**
  * @brief  Function implementing the ultrasonicTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ultra */
void ultra(void *argument)
{
  /* USER CODE BEGIN ultra */
  /* Infinite loop */
  // 센서/필터 초기화 (각도 30°, 통로폭 50cm, SMA 5, STALE 300ms)
  US_Init(30.0f, 50.0f, 5, 300);

  TickType_t last = xTaskGetTickCount();

  for (;;)
  {
      US_Update();                              // 센서 모듈이 트리거/캡처/필터까지 모두 처리
      vTaskDelayUntil(&last, pdMS_TO_TICKS(10)); // 10 ms 주기
  }
  /* USER CODE END ultra */
}

/* USER CODE BEGIN Header_ctrl */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ctrl */
void ctrl(void *argument)
{
  /* USER CODE BEGIN ctrl */
  /* Infinite loop */
  AutoMode_Init();  // motor_dir_init/motor_speed_init 내부에서 처리됨

  TickType_t last = xTaskGetTickCount();
  for (;;)
  {
      AutoMode_Update();                        // 정제 프레임만 소비하여 속도/조향 수행
      vTaskDelayUntil(&last, pdMS_TO_TICKS(10)); // 10 ms 주기
  }
  /* USER CODE END ctrl */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

