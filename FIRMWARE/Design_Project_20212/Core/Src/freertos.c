/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "sim.h"
#include "l70.h"
#include "lcd.h"
#include "touch.h"
#include "mpu6050.h"
#include "ds1307.h"

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
// extern IWDG_HandleTypeDef hiwdg;
extern I2C_HandleTypeDef hi2c1;

MPU6050_t MPU6050;
RTC_t myRTC;

/* USER CODE END Variables */
/* Definitions for LCD_Task */
osThreadId_t LCD_TaskHandle;
const osThreadAttr_t LCD_Task_attributes = {
  .name = "LCD_Task",
  .stack_size = 180 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SIM_Task */
osThreadId_t SIM_TaskHandle;
const osThreadAttr_t SIM_Task_attributes = {
  .name = "SIM_Task",
  .stack_size = 180 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SEN_Task */
osThreadId_t SEN_TaskHandle;
const osThreadAttr_t SEN_Task_attributes = {
  .name = "SEN_Task",
  .stack_size = 100 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GPS_Task */
osThreadId_t GPS_TaskHandle;
const osThreadAttr_t GPS_Task_attributes = {
  .name = "GPS_Task",
  .stack_size = 180 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LCDTASK(void *argument);
void SIMTASK(void *argument);
void SENSOR_Task(void *argument);
void GPS_TASK(void *argument);

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
  /* creation of LCD_Task */
  LCD_TaskHandle = osThreadNew(LCDTASK, NULL, &LCD_Task_attributes);

  /* creation of SIM_Task */
  SIM_TaskHandle = osThreadNew(SIMTASK, NULL, &SIM_Task_attributes);

  /* creation of SEN_Task */
  SEN_TaskHandle = osThreadNew(SENSOR_Task, NULL, &SEN_Task_attributes);

  /* creation of GPS_Task */
  GPS_TaskHandle = osThreadNew(GPS_TASK, NULL, &GPS_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_LCDTASK */
/**
  * @brief  Function implementing the LCD_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LCDTASK */
void LCDTASK(void *argument)
{
  /* USER CODE BEGIN LCDTASK */
  /* Infinite loop */
  for(;;)
  {
    // HAL_IWDG_Refresh(&hiwdg);
    osDelay(1000);
  }
  /* USER CODE END LCDTASK */
}

/* USER CODE BEGIN Header_SIMTASK */
/**
* @brief Function implementing the SIM_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SIMTASK */
void SIMTASK(void *argument)
{
  /* USER CODE BEGIN SIMTASK */
  SIM_Init();
  osDelay(5000);
  SIM_startGPRS();
  osDelay(2000);
  SIM_sendSMS("0914989855", "HELLO :P");
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(500);
  }
  /* USER CODE END SIMTASK */
}

/* USER CODE BEGIN Header_SENSOR_Task */
/**
* @brief Function implementing the SEN_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SENSOR_Task */
void SENSOR_Task(void *argument)
{
  /* USER CODE BEGIN SENSOR_Task */
  while (MPU6050_Init(&hi2c1))
  {
    osDelay(50);
  }
  osDelay(500);

  // myRTC.Date.year = 22;
  // myRTC.Date.month = 6;
  // myRTC.Date.day = FRIDAY;
  // myRTC.Date.date = 17;

  // myRTC.Time.hours = 2;
  // myRTC.Time.minutes = 52;
  // myRTC.Time.seconds = 0;
  // myRTC.Time.time_format = TIME_FORMAT_24HRS;

  // ds1307_set_current_date(&myRTC.Date);
  // ds1307_set_current_time(&myRTC.Time);

  // osDelay(500);

  /* Infinite loop */
  for(;;)
  {
    MPU6050_Read_All(&hi2c1, &MPU6050);
    if ((MPU6050.Ax == 0.0) && (MPU6050.Ay == 0.0) && (MPU6050.Az == 0.0))
    {
      MPU6050_Init(&hi2c1);
      osDelay(100);
      MPU6050_Read_All(&hi2c1, &MPU6050);
    }
    osDelay(1000);
    ds1307_get_current_date(&myRTC.Date);
    ds1307_get_current_time(&myRTC.Time);
    osDelay(1000);
  }
  /* USER CODE END SENSOR_Task */
}

/* USER CODE BEGIN Header_GPS_TASK */
/**
* @brief Function implementing the GPS_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPS_TASK */
void GPS_TASK(void *argument)
{
  /* USER CODE BEGIN GPS_TASK */
  char *GPSResponse;
  char latData[15] = {0};
  char longData[15] = {0};
  l70_init();
  // char data[100] = {0};
  // HAL_UART_Receive(&huart2, data, 100, 3000);
  osDelay(1000);
  /* Infinite loop */
  for(;;)
  {
    osThreadSuspend(GPS_TaskHandle);
    GPSResponse = l70_receiveGPS();
    l70_handleGPS(latData, longData, GPSResponse);
  }
  /* USER CODE END GPS_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    SIM_RXCallback();
  }
  if (huart->Instance == USART2)
  {
    l70_callback();
  }
}

/* USER CODE END Application */

