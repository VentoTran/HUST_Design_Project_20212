/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

#include <stdio.h>

#include "rtc.h"
#include "lcd.h"
#include "tim.h"
#include "font.h"

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
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TouchTask */
osThreadId_t TouchTaskHandle;
const osThreadAttr_t TouchTask_attributes = {
  .name = "TouchTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void MainTask(void *argument);
void TouchHandler(void *argument);

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
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(MainTask, NULL, &mainTask_attributes);

  /* creation of TouchTask */
  TouchTaskHandle = osThreadNew(TouchHandler, NULL, &TouchTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_MainTask */
/**
 * @brief  Function implementing the mainTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_MainTask */
void MainTask(void *argument)
{
  /* USER CODE BEGIN MainTask */
  HAL_RTCEx_SetSecond_IT(&hrtc);
  uint32_t tickNow = 0;
  uint32_t count = 0;
  // ILI9341_Init();

  // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 100);
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // ILI9341_FillScreen(ILI9341_YELLOW);

  /* Infinite loop */
  for (;;)
  {
    // HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    // HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    osDelay(1000);

    // tickNow = HAL_GetTick();
    // ILI9341_FillScreen(ILI9341_RED);
    // ILI9341_FillScreen(ILI9341_GREEN);
    // ILI9341_FillScreen(ILI9341_BLUE);
    // tickNow = HAL_GetTick() - tickNow;

    // char tempStr[15] = "";
    // sprintf(tempStr, "Took %ld ms", tickNow);
    // ILI9341_WriteString(80, 120, tempStr, Font_16x26, ILI9341_WHITE, ILI9341_BLACK);

    count++;
    if (count == 20)
    {
      HAL_GPIO_WritePin(PCTRL0_GPIO_Port, PCTRL0_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(PCTRL1_GPIO_Port, PCTRL1_Pin, GPIO_PIN_RESET);
    }
  }
  /* USER CODE END MainTask */
}

/* USER CODE BEGIN Header_TouchHandler */
/**
* @brief Function implementing the TouchTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TouchHandler */
void TouchHandler(void *argument)
{
  /* USER CODE BEGIN TouchHandler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TouchHandler */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)
{
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
}

/* USER CODE END Application */

