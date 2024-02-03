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
#include "max30102.h"
#include "lcd.h"
#include "touch.h"
#include "helperFunc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PLOT

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

static uint32_t sample_count = 0U;

// MAX30102 object
max30102_t max30102;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCD */
osThreadId_t LCDHandle;
const osThreadAttr_t LCD_attributes = {
  .name = "LCD",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Timer1s */
osTimerId_t Timer1sHandle;
const osTimerAttr_t Timer1s_attributes = {
  .name = "Timer1s"
};
/* Definitions for Display_Event */
osEventFlagsId_t Display_EventHandle;
const osEventFlagsAttr_t Display_Event_attributes = {
  .name = "Display_Event"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

// printf() function
int __io_putchar(int ch)
{
  uint8_t temp = ch;
  HAL_UART_Transmit(&huart1, &temp, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END FunctionPrototypes */

void DefaultTask(void *argument);
void Display_Task(void *argument);
void Timer1sFunc(void *argument);

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

  /* Create the timer(s) */
  /* creation of Timer1s */
  Timer1sHandle = osTimerNew(Timer1sFunc, osTimerPeriodic, NULL, &Timer1s_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(DefaultTask, NULL, &defaultTask_attributes);

  /* creation of LCD */
  // LCDHandle = osThreadNew(Display_Task, NULL, &LCD_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Display_Event */
  Display_EventHandle = osEventFlagsNew(&Display_Event_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_DefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_DefaultTask */
void DefaultTask(void *argument)
{
  /* USER CODE BEGIN DefaultTask */
  printf("Enter task MAX30102\n");

  // Initiation
  max30102_init(&max30102, &hi2c1);
  max30102_reset(&max30102);
  // uint8_t device_id = 0x00U;
  // max30102_read(&max30102, 0xFFU, &device_id, 1U);
  // if (device_id != 21U) while(1);
  max30102_clear_fifo(&max30102);
  max30102_set_fifo_config(&max30102, max30102_smp_ave_2, 1, 10);

  // Sensor settings
  max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
  max30102_set_adc_resolution(&max30102, max30102_adc_2048);
  max30102_set_sampling_rate(&max30102, max30102_sr_800);
  // RED
  max30102_set_led_current_1(&max30102, 6.4);
  // IR
  max30102_set_led_current_2(&max30102, 6.4);

  // Enter SpO2 mode
  max30102_set_mode(&max30102, max30102_spo2);
  max30102_set_a_full(&max30102, 0);

  // Initiate 1 temperature measurement
  max30102_set_die_temp_en(&max30102, 0);
  max30102_set_die_temp_rdy(&max30102, 0);

  // max30102_interrupt_handler(&max30102);

  osTimerStart(Timer1sHandle, 1000);

  /* Infinite loop */
  for (;;)
  {
    osDelay(100);
    // max30102_interrupt_handler(&max30102);
    max30102_read_fifo(&max30102);
    // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
  /* USER CODE END DefaultTask */
}

/* USER CODE BEGIN Header_Display_Task */
/**
 * @brief Function implementing the LCD thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Display_Task */
void Display_Task(void *argument)
{
  /* USER CODE BEGIN Display_Task */
  printf("Enter task LCD\n");
  vTaskSuspend(LCDHandle);
  Display_SetEventHandler(Display_EventHandle);
  ILI9341_Init();
  ILI9341_FillScreen(ILI9341_BLACK);
  uint16_t color = 0x0001;
  uint32_t tick = HAL_GetTick();
  uint32_t tickElapsed = 0;
  /* Infinite loop */
  for (;;)
  {
    tick = HAL_GetTick();
    ILI9341_FillScreen(color);
    if (color == 0x8000)  color = 0x0001;
    color <<= 1;
    tickElapsed = HAL_GetTick() - tick;
    printf("Took %ld ms\n", tickElapsed);
  }
  /* USER CODE END Display_Task */
}

/* Timer1sHandle function */
void Timer1sFunc(void *argument)
{
  /* USER CODE BEGIN Timer1sHandle */
  // printf("Sample Rate: %ld SPS\n", sample_count);
  sample_count = 0;

  /* USER CODE END Timer1sHandle */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void max30102_plot(uint32_t ir_sample, uint32_t red_sample)
{
#ifdef PLOT
  printf("%li,%li\n", red_sample, ir_sample);
  sample_count++;

#else
  UNUSED(ir_sample);
  UNUSED(red_sample);
#endif
}

/* USER CODE END Application */

