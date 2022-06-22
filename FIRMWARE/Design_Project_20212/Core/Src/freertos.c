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
#include "max30102.h"
#include "debug.h"
#include "stdlib.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  SLEEPING = 0,
  RUNNING = 1
} Mode_t;

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

static MPU6050_t MPU6050;
static RTC_t myRTC;
static Mode_t DeviceState = RUNNING;


volatile bool isTouch = false;

static max30102_t MAX30102;
static uint8_t HeartRate = 0;
static uint8_t tHR[4] = {0};
static uint32_t IR_Value[250] = {0};
static uint8_t IR_Count = 0;
static uint32_t RD_Value[250] = {0};
static uint8_t RD_Count = 0;

/* USER CODE END Variables */
/* Definitions for LCD_Task */
osThreadId_t LCD_TaskHandle;
const osThreadAttr_t LCD_Task_attributes = {
  .name = "LCD_Task",
  .stack_size = 150 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SIM_Task */
osThreadId_t SIM_TaskHandle;
const osThreadAttr_t SIM_Task_attributes = {
  .name = "SIM_Task",
  .stack_size = 150 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SEN_Task */
osThreadId_t SEN_TaskHandle;
const osThreadAttr_t SEN_Task_attributes = {
  .name = "SEN_Task",
  .stack_size = 150 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GPS_Task */
osThreadId_t GPS_TaskHandle;
const osThreadAttr_t GPS_Task_attributes = {
  .name = "GPS_Task",
  .stack_size = 150 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCD_Touch */
osSemaphoreId_t LCD_TouchHandle;
const osSemaphoreAttr_t LCD_Touch_attributes = {
  .name = "LCD_Touch"
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

  /* Create the semaphores(s) */
  /* creation of LCD_Touch */
  LCD_TouchHandle = osSemaphoreNew(1, 1, &LCD_Touch_attributes);

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
  // osThreadSuspend(LCD_TaskHandle);
  ILI9341_Unselect();
  ILI9341_TouchUnselect();
  osDelay(1000);
  ILI9341_Init();
  osDelay(1000);
  ILI9341_FillScreen(ILI9341_BLACK);
  ILI9341_WriteString(50, 10, "SIM: ", Font_16x26, ILI9341_WHITE, ILI9341_BLACK);
  ILI9341_WriteString(130, 10, "GPRS: ", Font_16x26, ILI9341_WHITE, ILI9341_BLACK);

  uint16_t x, y;
  // uint32_t currentTick = 0, lastTick = 0;
  /* Infinite loop */
  for(;;)
  {
    // HAL_IWDG_Refresh(&hiwdg);
    osDelay(50);
//  ----------------------------------- Wait for event --------------------------------------------
    if (osSemaphoreAcquire(LCD_TouchHandle, portMAX_DELAY) == osOK)
    {
      if ((HAL_GPIO_ReadPin(TCH_IRQ_GPIO_Port, TCH_IRQ_Pin) == GPIO_PIN_RESET) && (isTouch == true))
      {
        osDelay(50);
        if (HAL_GPIO_ReadPin(TCH_IRQ_GPIO_Port, TCH_IRQ_Pin) == GPIO_PIN_RESET)
        {
          while(ILI9341_TouchGetCoordinates(&y, &x) != true);
          y = 240 - y;
          x = 320 - x;
        }
        logPC("Touch Coordinate: (%i,%i)\n", x, y);
        ILI9341_DrawPixel(x, y, ILI9341_WHITE);
        osDelay(50);
      }
    }
    isTouch = false;
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(50);
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
  osThreadSuspend(SIM_TaskHandle);
  SIM_Init();
  osDelay(5000);
  SIM_startGPRS();
  osDelay(2000);
  // SIM_sendSMS("0914989855", "HELLO :P");
  /* Infinite loop */
  for(;;)
  {
    // osThreadSuspend(LCD_TaskHandle);
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
  // osThreadSuspend(SEN_TaskHandle);
  // osDelay(5000);
  while (MPU6050_Init(&hi2c1))
  {
    osDelay(50);
  }

  osDelay(50);

  max30102_init(&MAX30102, &hi2c1);
  max30102_reset(&MAX30102);
  max30102_clear_fifo(&MAX30102);
  max30102_set_fifo_config(&MAX30102, max30102_smp_ave_8, 1, 5);
  
  // Sensor settings
  max30102_set_led_pulse_width(&MAX30102, max30102_pw_16_bit);
  max30102_set_adc_resolution(&MAX30102, max30102_adc_2048);
  max30102_set_sampling_rate(&MAX30102, max30102_sr_1000);
  max30102_set_led_current_1(&MAX30102, 6.2);
  max30102_set_led_current_2(&MAX30102, 6.2);

  // Enter SpO2 mode
  max30102_set_mode(&MAX30102, max30102_spo2);
  max30102_set_a_full(&MAX30102, 0);
  
  // Initiate 1 temperature measurement
  max30102_set_die_temp_en(&MAX30102, 0);
  max30102_set_die_temp_rdy(&MAX30102, 0);
  
  uint8_t en_reg[2] = {0};
  max30102_read(&MAX30102, 0x00, en_reg, 1);
  osDelay(100);

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

  uint8_t count = 0;
  bool isDataValid = false;
  MAX30102.Peak.maxPeak = 7;
  MAX30102.Peak.minGap = 40;
  MAX30102.Peak.nPeak = 0;
  memset(MAX30102.Peak.peakLoc, '\0', sizeof(MAX30102.Peak.peakLoc));
  memset(tHR, '\0', sizeof(tHR));
  uint32_t lastTime = HAL_GetTick();

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

    ds1307_get_current_date(&myRTC.Date);
    ds1307_get_current_time(&myRTC.Time);

    if ((HAL_GetTick() - lastTime) >= 2000)
    {
      logPC("HELLO! Now is %d/%d/%d %d:%d:%d\n", myRTC.Date.date, myRTC.Date.month, myRTC.Date.year, myRTC.Time.hours, myRTC.Time.minutes, myRTC.Time.seconds);
      lastTime = HAL_GetTick();
    }
#if PLOT == 1
    logPC("$%d %d;", (uint8_t)MPU6050.KalmanAngleX, (uint8_t)MPU6050.KalmanAngleY);
#endif

    max30102_read_fifo(&MAX30102);

    if ((IR_Count >= 200) || (RD_Count >= 200))
    {
      isDataValid = true;
      for (uint8_t i = 0; ((i < 200) && (isDataValid == true)); i++)
      {
        if (IR_Value[i] < 45000*5)
        {
          isDataValid = false;
        }
        else
        {
          if ((i >= 2) && (i <= 197))
          {
            IR_Value[i-1] = (IR_Value[i-2] + IR_Value[i-1] + IR_Value[i]) / 3;
            IR_Value[i] = (IR_Value[i-2] + IR_Value[i-1] + IR_Value[i] + IR_Value[i+1] + IR_Value[i+2]) / 5;
            IR_Value[i+1] = (IR_Value[i+2] + IR_Value[i+1] + IR_Value[i]) / 3;
            RD_Value[i-1] = (RD_Value[i-2] + RD_Value[i-1] + RD_Value[i]) / 3;
            RD_Value[i] = (RD_Value[i-2] + RD_Value[i-1] + RD_Value[i] + RD_Value[i+1] + RD_Value[i+2]) / 5;
            RD_Value[i+1] = (RD_Value[i+2] + RD_Value[i+1] + RD_Value[i]) / 3;
          }
#if PLOT == 1
          logPC("$%i %i;", IR_Value[i]/5, RD_Value[i]/5);
#endif
        }
      }

      if (isDataValid == true)
      {
        // HeartRate = getHeartRate(&MAX30102, IR_Value);
        MAX30102.Peak.nPeak = 0;
        memset(MAX30102.Peak.peakLoc, '\0', sizeof(MAX30102.Peak.peakLoc));

        maxim_find_peaks(&MAX30102, IR_Value, (uint8_t)200);

        uint32_t gap = 1;
        for (int i = 0; i < (MAX30102.Peak.nPeak-1); i++)
        {
          gap = gap + (MAX30102.Peak.peakLoc[i+1] - MAX30102.Peak.peakLoc[i]);
        }
        // logPC("%d", (uint8_t)avr_gap);

        HeartRate = (uint8_t)((60000.0 * MAX30102.Peak.nPeak) / (gap * (MAX30102.deltaTSample + 7)));
        if ((HeartRate >= 55) && (HeartRate <= 140))
        {
          tHR[0] = tHR[1];
          tHR[1] = tHR[2];
          tHR[2] = tHR[3];
          tHR[3] = HeartRate;
        }
        if (tHR[0] != '\0')
        {
          HeartRate = (uint8_t)((tHR[0] + tHR[1] + tHR[2] + tHR[3]) / 4);
          logPC("Heart Rate: %d BPM\n", HeartRate);
        }
        osDelay(50);
      }

      memset(IR_Value, '\0', sizeof(IR_Value));
      memset(RD_Value, '\0', sizeof(RD_Value));
      IR_Count = 0;
      RD_Count = 0;
    }

    count = 0;
    while ((MAX30102._ir_samples[count] != '\0') && (MAX30102._red_samples[count] != '\0'))
    {
      IR_Value[IR_Count++] = MAX30102._ir_samples[count];
      RD_Value[RD_Count++] = MAX30102._red_samples[count];
      count++;
    }

    osDelay(50);
  
    
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
  osThreadSuspend(GPS_TaskHandle);
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == TCH_IRQ_Pin) && (isTouch != true))
  {
    isTouch = true;
    osSemaphoreRelease(LCD_TouchHandle);
  }
}

/* USER CODE END Application */

