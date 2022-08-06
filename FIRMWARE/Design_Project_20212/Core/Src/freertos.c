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
#include "mqtt.h"
#include "DFPLAYER_MINI.h"
#include "timers.h"
#include "topic.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  STOP = 0,
  RUNNING = 1,
  PAUSE = 2
} Mode_t;

typedef struct
{
  uint16_t Voltage_12bits[1];
  float Voltage_V;
  float Voltage_Batt;
  uint8_t Perc_Batt;
  char cBatt[4];
} Battery_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_GAIN      (1.46527)
#define MAX_VOL       (4.22)
#define IR_THRESHOLD  (30000)

#define MAIN_PAGE   0
#define MP3_PAGE    1
#define DATA_PAGE   2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// extern IWDG_HandleTypeDef hiwdg;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;

static Mode_t DeviceState = STOP;
static MQTT_ST_t MQTT_Status = {
    .SIM_ST = SIM_SIMCARD_NOK,
    .MQTT_ST = MQTT_NOK
};

extern MQTT_t MQTT;
static char *GPSResponse;
static char latData[16] = "22.992335";
static char longData[16] = "105.886839";

static MPU6050_t MPU6050;
static RTC_t myRTC;
static max30102_t MAX30102 = {
  .Peak.maxPeak = 7,
  .Peak.minGap = 40
};
static uint8_t HeartRate = 0;
static uint32_t Step = 0;
static uint8_t TimeRun = 0;
static uint32_t IR_Value[250] = {0};
static uint8_t IR_Count = 0;
static uint32_t RD_Value[250] = {0};
static uint8_t RD_Count = 0;
static uint8_t count = 0;
static uint8_t tHR[4] = {0};
static bool isDataValid = false;
static bool isNewTime = false;

static Battery_t Batt = {
  .Perc_Batt = 0,
  .Voltage_12bits[0] = 0,
  .Voltage_Batt = 0.0,
  .Voltage_V = 0.0,
  .cBatt = {0}
};

static uint8_t CurrentPage = MAIN_PAGE;
static volatile bool isTouch = false;
static volatile bool isSleep = false;
const static myButton_t toPageLeft = {
  .pos_x = 30,
  .pos_y = 208,
  .color = ILI9341_LIGHTBLUE,
  .shape_r = 28
};
const static myButton_t toPageRight = {
  .pos_x = 290,
  .pos_y = 208,
  .color = ILI9341_LIGHTBLUE,
  .shape_r = 28
};
const static myButton_t toRightPage = {
  .pos_x = 90,
  .pos_y = 208,
  .color = ILI9341_GRAYBLUE,
  .shape_r = 28
};
const static myButton_t bStart = {
  .pos_x = 130,
  .pos_y = 193,
  .color = ILI9341_GRAYBLUE,
  .shape_w = 60,
  .shape_h = 30
};
const static myButton_t bPause = {
  .pos_x = 90,
  .pos_y = 193,
  .color = ILI9341_GRAYBLUE,
  .shape_w = 60,
  .shape_h = 30
};
const static myButton_t bStop = {
  .pos_x = 170,
  .pos_y = 193,
  .color = ILI9341_GRAYBLUE,
  .shape_w = 60,
  .shape_h = 30
};
const static myButton_t bPlay = {
  .pos_x = 30,
  .pos_y = 208,
  .color = ILI9341_GRAYBLUE,
  .shape_r = 40
};
const static myButton_t bPrev = {
  .pos_x = 30,
  .pos_y = 208,
  .color = ILI9341_GRAYBLUE,
  .shape_r = 28
};
const static myButton_t bNext = {
  .pos_x = 30,
  .pos_y = 208,
  .color = ILI9341_GRAYBLUE,
  .shape_r = 28
};
const static myButton_t incVol = {
  .pos_x = 30,
  .pos_y = 208,
  .color = ILI9341_GRAYBLUE,
  .shape_r = 28
};
const static myButton_t decVol = {
  .pos_x = 30,
  .pos_y = 208,
  .color = ILI9341_GRAYBLUE,
  .shape_r = 28
};
static char DateFormat[22] = {0};
static char TimeFormat[8] = {0};
static char HRFormat[4] = {0};
static char StepFormat[7] = {0};
static char TimeRunFormat[5] = {0};

/* USER CODE END Variables */
/* Definitions for LCD_Task */
osThreadId_t LCD_TaskHandle;
const osThreadAttr_t LCD_Task_attributes = {
  .name = "LCD_Task",
  .stack_size = 200 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SIM_Task */
osThreadId_t SIM_TaskHandle;
const osThreadAttr_t SIM_Task_attributes = {
  .name = "SIM_Task",
  .stack_size = 220 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SEN_Task */
osThreadId_t SEN_TaskHandle;
const osThreadAttr_t SEN_Task_attributes = {
  .name = "SEN_Task",
  .stack_size = 200 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCD_Timer */
osTimerId_t LCD_TimerHandle;
const osTimerAttr_t LCD_Timer_attributes = {
  .name = "LCD_Timer"
};
/* Definitions for Touch_Timer */
osTimerId_t Touch_TimerHandle;
const osTimerAttr_t Touch_Timer_attributes = {
  .name = "Touch_Timer"
};
/* Definitions for LCD_Sleep */
osTimerId_t LCD_SleepHandle;
const osTimerAttr_t LCD_Sleep_attributes = {
  .name = "LCD_Sleep"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

bool Connect_MQTT(void);
void updateParameter(uint8_t page);
void handleTouch(uint16_t x, uint16_t y, uint8_t page);

void Main_page(void);
void Mp3_page(void);
void Graph_page(void);

/* USER CODE END FunctionPrototypes */

void LCDTASK(void *argument);
void SIMTASK(void *argument);
void SENSOR_Task(void *argument);
void LCD_Timer_Callback(void *argument);
void Touch_Timer_Callback(void *argument);
void LCD_Sleep_Callback(void *argument);

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
  /* creation of LCD_Timer */
  LCD_TimerHandle = osTimerNew(LCD_Timer_Callback, osTimerPeriodic, NULL, &LCD_Timer_attributes);

  /* creation of Touch_Timer */
  Touch_TimerHandle = osTimerNew(Touch_Timer_Callback, osTimerPeriodic, NULL, &Touch_Timer_attributes);

  /* creation of LCD_Sleep */
  LCD_SleepHandle = osTimerNew(LCD_Sleep_Callback, osTimerOnce, NULL, &LCD_Sleep_attributes);

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
  uint32_t TimeRefresh = HAL_GetTick();

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  ILI9341_Unselect();
  ILI9341_TouchUnselect();
  ILI9341_Init();
  isTouch = false;

  do
  {
    ds1307_get_current_date(&myRTC.Date);
    ds1307_get_current_time(&myRTC.Time);
  }
  while (myRTC.Time.minutes >= 60);
  
  ILI9341_FillRectangle(0, 0, 320, 25, ILI9341_BLACK);
  ILI9341_WriteString(10, 10, "SIM: ", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
  ILI9341_WriteString(80, 10, "GPRS: ", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
  ILI9341_WriteString(150, 10, "MQTT: ", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);  
  ILI9341_WriteString(225, 10, "Battery: ", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
  
  Main_page();

  uint16_t x = 0;
  uint16_t y = 0;

  TimeRefresh = HAL_GetTick();
  osTimerStart(LCD_TimerHandle, 5000);
  osTimerStart(Touch_TimerHandle, 50);
  osTimerStart(LCD_SleepHandle, 120000);
  // osThreadSuspend(LCD_TaskHandle);
  /* Infinite loop */
  for(;;)
  {
    
    osDelay(1);
    if ((HAL_GPIO_ReadPin(TCH_IRQ_GPIO_Port, TCH_IRQ_Pin) == GPIO_PIN_RESET) && (isTouch == true))
    {
      osDelay(100);
      if (HAL_GPIO_ReadPin(TCH_IRQ_GPIO_Port, TCH_IRQ_Pin) == GPIO_PIN_RESET)
      {
        x = 0;
        y = 0;
        while(ILI9341_TouchGetCoordinates(&y, &x) != true);
        y = 240 - y;
        x = 320 - x;
      }
      logPC("Touch Coordinate: (%i,%i)\n", x, y);
      xTimerReset(LCD_SleepHandle, 100);
      if (isSleep == true)
      {
        logPC("LCD awake!\n");
        ILI9341_LCD_LED(true);
        osTimerStart(LCD_TimerHandle, 5000);
        isSleep = false;
      }
      else
      {
        uint8_t Prev = CurrentPage;
        handleTouch(x, y, CurrentPage);
        if (Prev != CurrentPage)
        {
          if (CurrentPage == MAIN_PAGE)
          {
            Main_page();
          }
          else if (CurrentPage == MP3_PAGE)
          {
            Mp3_page();
          }
          else if (CurrentPage == DATA_PAGE)
          {
            Graph_page();
          }
        }
        else
        {
          updateButton(CurrentPage);
        }
      }
      // ILI9341_DrawPixel(x, y, ILI9341_WHITE);
      osDelay(200);
    }
    else
    {
      updateParameter(CurrentPage);
    }
    isTouch = false;
    osThreadSuspend(LCD_TaskHandle);
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
  // osThreadSuspend(SIM_TaskHandle);
  // osDelay(2000);
  uint32_t timeSIM = HAL_GetTick();
  uint32_t timePUB = HAL_GetTick();

  while ((SIM_Init() != true) && ((HAL_GetTick() - timeSIM) <= 20000))
  {
    osDelay(1000);
  }
  
  DF_Init(INIT_VOL);

  MQTT_Status.SIM_ST = SIM_SIMCARD_NOK;

  if (Connect_MQTT() == true)
  {
    osDelay(1000);
    MQTT_Pub(STATUS_TOPIC, "1");
    // osDelay(1000);
    // MQTT_Sub(PING_TOPIC);
    osDelay(1000);
    MQTT_Sub(TIME_TOPIC);
  }

  osDelay(500);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  l70_init();

  DF_Sleep();

  // osTimerStart(MQTT_TimerHandle, 30000);
  timePUB = HAL_GetTick();

  /* Infinite loop */
  for(;;)
  {

    osDelay(100);
    // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    if (((HAL_GetTick() - timeSIM) >= 90000) && (MQTT.mqttServer.connect == 1))
    {
      if (MQTT_PingReq() != true)
      {
        MQTT_Status.MQTT_ST = MQTT_NOK;
        MQTT.mqttServer.connect = 0;
      }
      else
      {
        MQTT.mqttServer.connect = 1;
        MQTT_Status.MQTT_ST = MQTT_OK;
      }
      timeSIM = HAL_GetTick();
    }
    if ((MQTT_Status.MQTT_ST != MQTT_OK) || (MQTT.mqttServer.connect == 0) || (MQTT_Status.SIM_ST != SIM_GPRS_OK))
    {
      if (Connect_MQTT() == true)
      {
        // osDelay(1000);
        // MQTT_Sub(PING_TOPIC);
        osDelay(1000);
        MQTT_Sub(TIME_TOPIC);
        timeSIM = HAL_GetTick();
      }
    }
    if (MQTT.mqttReceive.newEvent == 1)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      logPC("MQTT Received from topic \"%s\", message is \"%s\"\n", MQTT.mqttReceive.topic, MQTT.mqttReceive.payload);
      MQTT.mqttReceive.newEvent = 0;
      SIM_clearRX();
      if (strstr(MQTT.mqttReceive.topic, "time") != NULL)
      {
        char* token = strtok(MQTT.mqttReceive.payload, ",");
        myRTC.Date.year = atoi(token);
        token = strtok(NULL, ",");
        myRTC.Date.month = atoi(token);
        token = strtok(NULL, ",");
        myRTC.Date.day = atoi(token);
        token = strtok(NULL, ",");
        myRTC.Date.date = atoi(token);
        token = strtok(NULL, ",");
        myRTC.Time.hours = atoi(token);
        token = strtok(NULL, ",");
        myRTC.Time.minutes = atoi(token);
        token = strtok(NULL, ",");
        myRTC.Time.seconds = atoi(token);
        isNewTime = true;
      }
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      timeSIM = HAL_GetTick();
    }
    if ((DeviceState == RUNNING) && ((HAL_GetTick() - timePUB) >= 15000))
    {
      char Upload[50] = {0};
      sprintf(Upload, "{\"HeartRate\":%03d,\"Step\":%06d,\"Period\":%03d}", HeartRate, Step, TimeRun);
      logPC("Data packet is %s\n", Upload);
      MQTT_Pub(DATA_TOPIC, Upload);
      timeSIM = HAL_GetTick();
      timePUB = HAL_GetTick();
    }
    else if ((DeviceState == STOP) || (DeviceState == PAUSE))
    {
      timePUB = HAL_GetTick();
    }
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
  uint32_t lastTime = HAL_GetTick();
  uint32_t TimeHR = HAL_GetTick();
  uint32_t getTime = HAL_GetTick();
  uint32_t runTick = HAL_GetTick();

  Debug_Init();

  while (MPU6050_Init(&hi2c1) && ((HAL_GetTick() - lastTime) <= 10000))
  {
    osDelay(50);
  }
  if ((HAL_GetTick() - lastTime) > 10000)
  {
    logPC("MPU6050 FAILED!\n");
  }
  else
  {
    logPC("MPU6050 OK\n");
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
  max30102_set_led_current_1(&MAX30102, 4);
  max30102_set_led_current_2(&MAX30102, 4);

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
  // myRTC.Date.month = 8;
  // myRTC.Date.day = SATURDAY;
  // myRTC.Date.date = 6;

  // myRTC.Time.hours = 17;
  // myRTC.Time.minutes = 2;
  // myRTC.Time.seconds = 0;
  // myRTC.Time.time_format = TIME_FORMAT_24HRS;

  // ds1307_set_current_date(&myRTC.Date);
  // ds1307_set_current_time(&myRTC.Time);

  MAX30102.Peak.nPeak = 0;
  memset(MAX30102.Peak.peakLoc, '\0', sizeof(MAX30102.Peak.peakLoc));
  memset(tHR, '\0', sizeof(tHR));
  lastTime = HAL_GetTick();
  TimeHR = HAL_GetTick();
  getTime = HAL_GetTick();
  runTick = HAL_GetTick();

  // osDelay(500);
  /* Infinite loop */
  for(;;)
  {
    if ((HAL_GetTick() - getTime) >= 5000)
    {
      ds1307_get_current_date(&myRTC.Date);
      ds1307_get_current_time(&myRTC.Time);
      Batt.Voltage_V =  ((float) Batt.Voltage_12bits[0] / 4095) * 3.3;
      Batt.Voltage_Batt = Batt.Voltage_V * ADC_GAIN;
      Batt.Perc_Batt = (uint8_t) (Batt.Voltage_Batt * 100 / MAX_VOL);
      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)Batt.Voltage_12bits, 1);
      getTime = HAL_GetTick();
    }

    if (isNewTime == true)
    {
      ds1307_set_current_date(&myRTC.Date);
      ds1307_set_current_time(&myRTC.Time);
      isNewTime = false;
    }

    if ((HAL_GetTick() - lastTime) >= 30000)
    {
      char buf[5] = {0};
      logPC("HELLO! Now is %02d/%02d/%02d %02d:%02d:%02d\n", myRTC.Date.date, myRTC.Date.month, myRTC.Date.year, myRTC.Time.hours, myRTC.Time.minutes, myRTC.Time.seconds);
      // logPC("ADC 12-bit value is %hu\n", Batt.Voltage_12bits[0]);
      // ftoa(Batt.Voltage_V, buf, 2);
      // logPC("ADC Voltage is %s\n", buf);
      // memset(buf, '\0', sizeof(buf));
      ftoa0(Batt.Voltage_Batt, buf, 2);
      logPC("Batt Voltage is %s\n", buf);
      logPC("Percent Battery is %hd\n", Batt.Perc_Batt);
      lastTime = HAL_GetTick();
    }

    if ((HAL_GetTick() - TimeHR) >= 15000)
    {
      HeartRate = 0;
    }

    if (DeviceState == RUNNING)
    {
      if ((HAL_GetTick() - runTick) >= 60000)
      {
        TimeRun++;
        runTick = HAL_GetTick();
      }
      MPU6050_Read_All(&hi2c1, &MPU6050);
      if ((MPU6050.Ax == 0.0) || (MPU6050.Ay == 0.0) || (MPU6050.Az == 0.0))
      {
        MPU6050_Init(&hi2c1);
        osDelay(100);
        MPU6050_Read_All(&hi2c1, &MPU6050);
      }
#if PLOT == 1
      // logPC("$%i %i %i;", (int)MPU6050.KalmanAngleX, (int)MPU6050.KalmanAngleY, (int)MPU6050.KalmanAngleZ);
#endif

      max30102_read_fifo(&MAX30102);

      count = 0;
      while (MAX30102._ir_samples[count] != '\0')
      {
        IR_Value[IR_Count++] = MAX30102._ir_samples[count++];
      }

      if (IR_Count >= 200)
      {
          isDataValid = true;
          for (uint8_t i = 0; ((i < 200) && (isDataValid == true)); i++)
          {
            if (IR_Value[i] < IR_THRESHOLD*5)
            {
              isDataValid = false;
              logPC("Finger OFF\n");
            }
            else
            {
              if ((i >= 2) && (i <= 197))
              {
                IR_Value[i-1] = (IR_Value[i-2] + IR_Value[i-1] + IR_Value[i]) / 3;
                IR_Value[i] = (IR_Value[i-2] + IR_Value[i-1] + IR_Value[i] + IR_Value[i+1] + IR_Value[i+2]) / 5;
                IR_Value[i+1] = (IR_Value[i+2] + IR_Value[i+1] + IR_Value[i]) / 3;
              }
#if PLOT == 1
              logPC("$%i %i;", IR_Value[i]/5, RD_Value[i]/5);
#endif
            }
          }
        
        if (isDataValid == true)
        {
          logPC("Finger ON\n");
          MAX30102.Peak.nPeak = 0;
          for (uint8_t i = 0; i < 10; i++)
          {
            MAX30102.Peak.peakLoc[i] = '\0';
          }

          maxim_find_peaks(&MAX30102, IR_Value, (uint8_t)200);

          if (MAX30102.Peak.nPeak > 1)
          {
            uint32_t gap = 1;
            for (uint8_t i = 0; i < (MAX30102.Peak.nPeak-1); i++)
            {
              gap += (MAX30102.Peak.peakLoc[i+1] - MAX30102.Peak.peakLoc[i]);
            }

            uint8_t tHeartRate = (uint8_t)((60000.0 * MAX30102.Peak.nPeak) / (gap * (MAX30102.deltaTSample*2)));
            if ((tHeartRate >= 55) && (tHeartRate <= 140))
            {
              TimeHR = HAL_GetTick();
              tHR[0] = tHR[1];
              tHR[1] = tHR[2];
              tHR[2] = tHR[3];
              tHR[3] = tHeartRate;
              if (tHR[0] != '\0')
              {
                HeartRate = (uint8_t)((tHR[0] + tHR[1] + tHR[2] + tHR[3]) / 4);
                logPC("Heart Rate: %d BPM\n", HeartRate);
              }
            }
          }
        }

        memset(IR_Value, '\0', 200*4);
        memcpy(IR_Value, IR_Value+200, 50*4);
        IR_Count -= 200;
      }
    
    }
    else
    {
      runTick = HAL_GetTick();
    }
    osDelay(50);
  }
  /* USER CODE END SENSOR_Task */
}

/* LCD_Timer_Callback function */
void LCD_Timer_Callback(void *argument)
{
  /* USER CODE BEGIN LCD_Timer_Callback */
  if (osThreadGetState(LCD_TaskHandle) == osThreadBlocked)
  {
    osThreadResume(LCD_TaskHandle);
  }
  /* USER CODE END LCD_Timer_Callback */
}

/* Touch_Timer_Callback function */
void Touch_Timer_Callback(void *argument)
{
  /* USER CODE BEGIN Touch_Timer_Callback */
  if ((isTouch == true) && (osThreadGetState(LCD_TaskHandle) == osThreadBlocked))
  {
    osThreadResume(LCD_TaskHandle);
  }
  /* USER CODE END Touch_Timer_Callback */
}

/* LCD_Sleep_Callback function */
void LCD_Sleep_Callback(void *argument)
{
  /* USER CODE BEGIN LCD_Sleep_Callback */
  logPC("LCD asleep!\n");
  ILI9341_LCD_LED(false);
  osTimerStop(LCD_TimerHandle);
  isSleep = true;
  /* USER CODE END LCD_Sleep_Callback */
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
  if (huart->Instance == USART1)
  {
    debug_callback();
  }
  if (huart->Instance == UART4)
  {
    DF_RX_Callback();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == TCH_IRQ_Pin) && (isTouch != true))
  {
    isTouch = true;
    // osSemaphoreRelease(LCD_TouchHandle);
  }
}

//============================================================================================

bool Connect_MQTT(void)
{
  uint32_t TIMEOUT = HAL_GetTick();
  if (MQTT_Status.SIM_ST == SIM_SIMCARD_NOK)
  {
    TIMEOUT = HAL_GetTick();
    while ((SIM_checkSIMCard() != true) && ((HAL_GetTick() - TIMEOUT) <= 15000))
    {
      osDelay(1000);
    }
    
    if ((HAL_GetTick() - TIMEOUT) <= 10000)
    {
      MQTT_Status.SIM_ST = SIM_SIMCARD_OK;
      logPC("SIM OK\n");
      osDelay(1000);
    }
    else
    {
      logPC("SIM FAIL\n");
      return false;
    }
  }

  if ((MQTT_Status.SIM_ST == SIM_SIMCARD_OK) || (MQTT_Status.SIM_ST == SIM_GPRS_NOK))
  {
    if (SIM_startGPRS() == true)
    {
      logPC("GPRS OK\n");
      MQTT_Status.SIM_ST = SIM_GPRS_OK;
      osDelay(1000);
    }
    else
    {
      logPC("GPRS FAIL\n");
      MQTT_Status.SIM_ST = SIM_GPRS_NOK;
      return false;
    }

  }

  if (MQTT_Status.SIM_ST == SIM_GPRS_OK)
  {
    MQTT.mqttServer.host = "test.mosquitto.org";
    MQTT.mqttServer.port = 1883;
    MQTT.mqttClient.username = "";
    MQTT.mqttClient.clientID = "hmmm";
    MQTT.mqttClient.pass = "";
    MQTT.mqttClient.keepAliveInterval = 120;

    MQTT_Connect();

    osDelay(1000);

    if (MQTT.mqttServer.connect == 1)
    {
      logPC("MQTT OK\n");
      MQTT_Status.MQTT_ST = MQTT_OK;
      return true;
    }
    else
    {
      logPC("MQTT FAIL\n");
      MQTT_Status.MQTT_ST = MQTT_NOK;
      return false;
    }
  }

  return false;
}

void updateParameter(uint8_t page)
{
  switch (CurrentPage)
  {
    case MAIN_PAGE:
    {
      if ((MQTT_Status.SIM_ST == SIM_SIMCARD_OK) || (MQTT_Status.SIM_ST == SIM_GPRS_OK))
      {ILI9341_WriteString(45, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
      else  
      {ILI9341_WriteString(45, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}
      if (MQTT_Status.SIM_ST == SIM_GPRS_OK)
      {ILI9341_WriteString(122, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
      else  
      {ILI9341_WriteString(122, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}
      if ((MQTT_Status.MQTT_ST == MQTT_OK) && (MQTT_Status.SIM_ST == SIM_GPRS_OK))
      {ILI9341_WriteString(192, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
      else  
      {ILI9341_WriteString(192, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}
      
      ILI9341_WriteString(225, 10, "Battery: ", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
      intToStr(Batt.Perc_Batt, Batt.cBatt, 3);
      ILI9341_WriteString(288, 10, Batt.cBatt, Font_7x10, ILI9341_YELLOW, ILI9341_BLACK);
      ILI9341_WriteString(309, 10, "%", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

      memset(DateFormat, '\0', sizeof(DateFormat));
      sprintf(DateFormat, "~~~ %02d/%02d/%4d ~~~\0", myRTC.Date.date, myRTC.Date.month, myRTC.Date.year+2000);
      ILI9341_WriteString(97, 30, DateFormat, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

      memset(TimeFormat, '\0', sizeof(TimeFormat));
      sprintf(TimeFormat, "%02d:%02d\0", myRTC.Time.hours, myRTC.Time.minutes);
      ILI9341_WriteString(133, 45, TimeFormat, Font_11x18, ILI9341_CYAN, ILI9341_BLACK);

      memset(HRFormat, '\0', sizeof(HRFormat));
      sprintf(HRFormat, "%03d", HeartRate);
      ILI9341_WriteString(37, 95, HRFormat, Font_11x18, ILI9341_RED, ILI9341_BLACK);

      memset(StepFormat, '\0', sizeof(StepFormat));
      sprintf(StepFormat, "%06ld", Step);
      ILI9341_WriteString(177, 95, StepFormat, Font_11x18, ILI9341_ORANGE, ILI9341_BLACK);

      memset(TimeRunFormat, '\0', sizeof(TimeRunFormat));
      sprintf(TimeRunFormat, "%03d", TimeRun);
      ILI9341_WriteString(36, 150, TimeRunFormat, Font_11x18, ILI9341_PINK, ILI9341_BLACK);

      ILI9341_WriteString(229, 147, latData, Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
      ILI9341_WriteString(222, 160, longData, Font_7x10, ILI9341_GREEN, ILI9341_BLACK);

      break;
    }
    case MP3_PAGE:
    {


      break;
    }
    case DATA_PAGE:
    {


      break;
    }
    default:
    {
      break;
    }
  }
}

void updateButton(uint8_t page)
{
  if (page == MAIN_PAGE)
  {
    ILI9341_FillRectangle(bPause.pos_x, bPause.pos_y, 140, 30, ILI9341_BLACK);

    if (DeviceState == STOP)
    {
      ILI9341_FillRectangle(bStart.pos_x, bStart.pos_y, bStart.shape_w, bStart.shape_h, bStart.color);
      ILI9341_WriteString(bStart.pos_x+3, bStart.pos_y+6, "START", Font_11x18, ILI9341_WHITE, bStart.color);
    }
    else if (DeviceState == RUNNING)
    {
      ILI9341_FillRectangle(bPause.pos_x, bPause.pos_y, bPause.shape_w, bPause.shape_h, bPause.color);
      ILI9341_WriteString(bPause.pos_x+3, bPause.pos_y+6, "PAUSE", Font_11x18, ILI9341_WHITE, bPause.color);
      ILI9341_FillRectangle(bStop.pos_x, bStop.pos_y, bStop.shape_w, bStop.shape_h, bStop.color);
      ILI9341_WriteString(bStop.pos_x+8, bStop.pos_y+6, "STOP", Font_11x18, ILI9341_WHITE, bStop.color);
    }
    else
    {
      ILI9341_FillRectangle(bPause.pos_x, bPause.pos_y, bPause.shape_w, bPause.shape_h, bPause.color);
      ILI9341_WriteString(bPause.pos_x+3, bPause.pos_y+6, "START", Font_11x18, ILI9341_WHITE, bPause.color);
      ILI9341_FillRectangle(bStop.pos_x, bStop.pos_y, bStop.shape_w, bStop.shape_h, bStop.color);
      ILI9341_WriteString(bStop.pos_x+8, bStop.pos_y+6, "STOP", Font_11x18, ILI9341_WHITE, bStop.color);
    }
  }
  else if (page == MP3_PAGE)
  {

  }
}

void handleTouch(uint16_t x, uint16_t y, uint8_t page)
{
  switch (page)
  {
    case MAIN_PAGE:
    {
      if (ILI9341_checkButton(x, y, &toPageLeft))
      {
        CurrentPage = MP3_PAGE;
      }
      else if (ILI9341_checkButton(x, y, &toPageRight))
      {
        CurrentPage = DATA_PAGE;
      }
      else if (DeviceState == STOP)
      {
        if (ILI9341_checkButton(x, y, &bStart))
        {
          DeviceState = RUNNING;
          max30102_clear_fifo(&MAX30102);
        }
      }
      else if (DeviceState == RUNNING)
      {
        if (ILI9341_checkButton(x, y, &bPause))
        {
          DeviceState = PAUSE;
        }
        else if (ILI9341_checkButton(x, y, &bStop))
        {
          DeviceState = STOP;
        }
      }
      else
      {
        if (ILI9341_checkButton(x, y, &bPause))
        {
          DeviceState = RUNNING;
          max30102_clear_fifo(&MAX30102);
        }
        else if (ILI9341_checkButton(x, y, &bStop))
        {
          DeviceState = STOP;
        }
      }
      break;
    }
    case MP3_PAGE:
    {
      if (ILI9341_checkButton(x, y, &toPageLeft))
      {
        CurrentPage = DATA_PAGE;
      }
      else if (ILI9341_checkButton(x, y, &toPageRight))
      {
        CurrentPage = MAIN_PAGE;
      }
      break;
    }
    case DATA_PAGE:
    {
      if (ILI9341_checkButton(x, y, &toPageLeft))
      {
        CurrentPage = MAIN_PAGE;
      }
      else if (ILI9341_checkButton(x, y, &toRightPage))
      {
        CurrentPage = MP3_PAGE;
      }
      break;
    }
    default:
    {
      break;
    }
  }
}

void Main_page(void)
{
  //========================================== CLEAR ================================================

  // ILI9341_FillScreen(ILI9341_BLACK);
  ILI9341_FillRectangle(0, 25, 320, 240, ILI9341_BLACK);

  //========================================= HEADER ========================================================

  if ((MQTT_Status.SIM_ST == SIM_SIMCARD_OK) || (MQTT_Status.SIM_ST == SIM_GPRS_OK))
  {ILI9341_WriteString(45, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
  else  
  {ILI9341_WriteString(45, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}
  if (MQTT_Status.SIM_ST == SIM_GPRS_OK)
  {ILI9341_WriteString(122, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
  else  
  {ILI9341_WriteString(122, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}
  if ((MQTT_Status.MQTT_ST == MQTT_OK) && (MQTT_Status.SIM_ST == SIM_GPRS_OK))
  {ILI9341_WriteString(192, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
  else  
  {ILI9341_WriteString(192, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}

  intToStr(Batt.Perc_Batt, Batt.cBatt, 3);
  ILI9341_WriteString(288, 10, Batt.cBatt, Font_7x10, ILI9341_YELLOW, ILI9341_BLACK);
  ILI9341_WriteString(309, 10, "%", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

  //============================================ DATE TIME =========================================

  memset(DateFormat, '\0', sizeof(DateFormat));
  sprintf(DateFormat, "~~~ %02d/%02d/%4d ~~~", myRTC.Date.date, myRTC.Date.month, myRTC.Date.year+2000);
  ILI9341_WriteString(97, 30, DateFormat, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
  
  memset(TimeFormat, '\0', sizeof(TimeFormat));
  sprintf(TimeFormat, "%02d:%02d", myRTC.Time.hours, myRTC.Time.minutes);
  ILI9341_WriteString(133, 45, TimeFormat, Font_11x18, ILI9341_CYAN, ILI9341_BLACK);

  //============================================ PARAMETERs =========================================

  ILI9341_WriteString(25, 70, "Heart Rate", Font_11x18, ILI9341_RED, ILI9341_BLACK);
  memset(HRFormat, '\0', sizeof(HRFormat));
  sprintf(HRFormat, "%03d", HeartRate);
  ILI9341_WriteString(37, 95, HRFormat, Font_11x18, ILI9341_RED, ILI9341_BLACK);
  ILI9341_WriteString(70, 95, " BPM", Font_11x18, ILI9341_RED, ILI9341_BLACK);

  ILI9341_WriteString(218, 70, "Step", Font_11x18, ILI9341_ORANGE, ILI9341_BLACK);
  memset(StepFormat, '\0', sizeof(StepFormat));
  sprintf(StepFormat, "%06ld", Step);
  ILI9341_WriteString(177, 95, StepFormat, Font_11x18, ILI9341_ORANGE, ILI9341_BLACK);
  ILI9341_WriteString(243, 95, " steps", Font_11x18, ILI9341_ORANGE, ILI9341_BLACK);

  ILI9341_WriteString(36, 125, "Run Time", Font_11x18, ILI9341_PINK, ILI9341_BLACK);
  memset(TimeRunFormat, '\0', sizeof(TimeRunFormat));
  sprintf(TimeRunFormat, "%03d", TimeRun);
  ILI9341_WriteString(36, 150, TimeRunFormat, Font_11x18, ILI9341_PINK, ILI9341_BLACK);
  ILI9341_WriteString(69, 150, " mins", Font_11x18, ILI9341_PINK, ILI9341_BLACK);

  ILI9341_WriteString(179, 125, "Coordinates", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
  ILI9341_WriteString(187, 147, "Lat: ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
  ILI9341_WriteString(229, 147, latData, Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
  ILI9341_WriteString(187, 160, "Lon: ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
  ILI9341_WriteString(222, 160, longData, Font_7x10, ILI9341_GREEN, ILI9341_BLACK);

  //======================================= BUTTONs ================================================

  ILI9341_FillRectangle(bPause.pos_x, bPause.pos_y, 140, 30, ILI9341_BLACK);

  if (DeviceState == STOP)
  {
    ILI9341_FillRectangle(bStart.pos_x, bStart.pos_y, bStart.shape_w, bStart.shape_h, bStart.color);
    ILI9341_WriteString(bStart.pos_x+3, bStart.pos_y+6, "START", Font_11x18, ILI9341_WHITE, bStart.color);
  }
  else if (DeviceState == RUNNING)
  {
    ILI9341_FillRectangle(bPause.pos_x, bPause.pos_y, bPause.shape_w, bPause.shape_h, bPause.color);
    ILI9341_WriteString(bPause.pos_x+3, bPause.pos_y+6, "PAUSE", Font_11x18, ILI9341_WHITE, bPause.color);
    ILI9341_FillRectangle(bStop.pos_x, bStop.pos_y, bStop.shape_w, bStop.shape_h, bStop.color);
    ILI9341_WriteString(bStop.pos_x+8, bStop.pos_y+6, "STOP", Font_11x18, ILI9341_WHITE, bStop.color);
  }
  else
  {
    ILI9341_FillRectangle(bPause.pos_x, bPause.pos_y, bPause.shape_w, bPause.shape_h, bPause.color);
    ILI9341_WriteString(bPause.pos_x+3, bPause.pos_y+6, "START", Font_11x18, ILI9341_WHITE, bPause.color);
    ILI9341_FillRectangle(bStop.pos_x, bStop.pos_y, bStop.shape_w, bStop.shape_h, bStop.color);
    ILI9341_WriteString(bStop.pos_x+8, bStop.pos_y+6, "STOP", Font_11x18, ILI9341_WHITE, bStop.color);
  }

  ILI9341_FillCircle(toPageLeft.pos_x, toPageLeft.pos_y, toPageLeft.shape_r, ILI9341_LIGHTBLUE);
  ILI9341_FillRectangle(toPageLeft.pos_x, toPageLeft.pos_y-10, 15, 20, ILI9341_WHITE);
  ILI9341_FillTriangle(toPageLeft.pos_x, toPageLeft.pos_y-15, toPageLeft.pos_x-20, toPageLeft.pos_y, toPageLeft.pos_x, toPageLeft.pos_y+15, ILI9341_WHITE);

  ILI9341_FillCircle(toPageRight.pos_x, toPageRight.pos_y, toPageRight.shape_r, ILI9341_LIGHTBLUE);
  ILI9341_FillRectangle(toPageRight.pos_x-15, toPageRight.pos_y-10, 15, 20, ILI9341_WHITE);
  ILI9341_FillTriangle(toPageRight.pos_x, toPageRight.pos_y-15, toPageRight.pos_x+20, toPageRight.pos_y, toPageRight.pos_x, toPageRight.pos_y+15, ILI9341_WHITE);

}

void Mp3_page(void)
{
  //========================================== CLEAR ================================================

  // ILI9341_FillScreen(ILI9341_BLACK);
  ILI9341_FillRectangle(0, 25, 320, 240, ILI9341_BLACK);

  //========================================= HEADER ========================================================

  if ((MQTT_Status.SIM_ST == SIM_SIMCARD_OK) || (MQTT_Status.SIM_ST == SIM_GPRS_OK))
  {ILI9341_WriteString(45, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
  else  
  {ILI9341_WriteString(45, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}
  if (MQTT_Status.SIM_ST == SIM_GPRS_OK)
  {ILI9341_WriteString(122, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
  else  
  {ILI9341_WriteString(122, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}
  if ((MQTT_Status.MQTT_ST == MQTT_OK) && (MQTT_Status.SIM_ST == SIM_GPRS_OK))
  {ILI9341_WriteString(192, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
  else  
  {ILI9341_WriteString(192, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}

  intToStr(Batt.Perc_Batt, Batt.cBatt, 3);
  ILI9341_WriteString(288, 10, Batt.cBatt, Font_7x10, ILI9341_YELLOW, ILI9341_BLACK);
  ILI9341_WriteString(309, 10, "%", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

  //=========================================== MP3 INTERFACE =================================================

  ILI9341_FillRectangle(20, 55, 280, 95, ILI9341_WHITE);
  ILI9341_DrawLine(20, 120, 300, 120, ILI9341_BLACK);
  ILI9341_DrawLine(160, 120, 160, 150, ILI9341_BLACK);

  ILI9341_WriteString(110, 60, "Ten bai hat", Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
  ILI9341_WriteString(24, 90, "Am tham ben em-SonTungMTP", Font_11x18, ILI9341_BLACK, ILI9341_WHITE);

  ILI9341_WriteString(40, 125, "THOI LUONG CHOI", Font_7x10, ILI9341_BLACK, ILI9341_WHITE);


  ILI9341_WriteString(200, 125, "AM LUONG", Font_7x10, ILI9341_BLACK, ILI9341_WHITE);    

  ILI9341_FillCircle(160, 180, 28, ILI9341_WHITE);
  ILI9341_FillTriangle(145, 160, 180, 180, 145, 200, ILI9341_BLUE);

  ILI9341_FillCircle(110, 180, 20, ILI9341_WHITE);
  ILI9341_FillTriangle(105, 165, 92, 180, 105, 195, ILI9341_BLUE);
  ILI9341_FillTriangle(118, 165, 105, 180, 118, 195, ILI9341_BLUE);

  ILI9341_FillCircle(210, 180, 20, ILI9341_WHITE);
  ILI9341_FillTriangle(202, 165, 215, 180, 202, 195, ILI9341_BLUE);
  ILI9341_FillTriangle(215, 165, 228, 180, 215, 195, ILI9341_BLUE);

  ILI9341_FillCircle(126, 217, 17, ILI9341_WHITE);
  ILI9341_WriteString(121, 210, "-", Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
  ILI9341_FillCircle(194, 217, 17, ILI9341_WHITE);
  ILI9341_WriteString(190, 210, "+", Font_11x18, ILI9341_BLACK, ILI9341_WHITE);

  ILI9341_FillCircle(toPageLeft.pos_x, toPageLeft.pos_y, toPageLeft.shape_r, ILI9341_LIGHTBLUE);
  ILI9341_FillRectangle(toPageLeft.pos_x, toPageLeft.pos_y-10, 15, 20, ILI9341_WHITE);
  ILI9341_FillTriangle(toPageLeft.pos_x, toPageLeft.pos_y-15, toPageLeft.pos_x-20, toPageLeft.pos_y, toPageLeft.pos_x, toPageLeft.pos_y+15, ILI9341_WHITE);

  ILI9341_FillCircle(toPageRight.pos_x, toPageRight.pos_y, toPageRight.shape_r, ILI9341_LIGHTBLUE);
  ILI9341_FillRectangle(toPageRight.pos_x-15, toPageRight.pos_y-10, 15, 20, ILI9341_WHITE);
  ILI9341_FillTriangle(toPageRight.pos_x, toPageRight.pos_y-15, toPageRight.pos_x+20, toPageRight.pos_y, toPageRight.pos_x, toPageRight.pos_y+15, ILI9341_WHITE);
}

void Graph_page(void)
{
  //========================================== CLEAR ================================================

  // ILI9341_FillScreen(ILI9341_BLACK);
  ILI9341_FillRectangle(0, 25, 320, 240, ILI9341_BLACK);

  //========================================= HEADER ========================================================

  if ((MQTT_Status.SIM_ST == SIM_SIMCARD_OK) || (MQTT_Status.SIM_ST == SIM_GPRS_OK))
  {ILI9341_WriteString(45, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
  else  
  {ILI9341_WriteString(45, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}
  if (MQTT_Status.SIM_ST == SIM_GPRS_OK)
  {ILI9341_WriteString(122, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
  else  
  {ILI9341_WriteString(122, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}
  if ((MQTT_Status.MQTT_ST == MQTT_OK) && (MQTT_Status.SIM_ST == SIM_GPRS_OK))
  {ILI9341_WriteString(192, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
  else  
  {ILI9341_WriteString(192, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}
  
  intToStr(Batt.Perc_Batt, Batt.cBatt, 3);
  ILI9341_WriteString(288, 10, Batt.cBatt, Font_7x10, ILI9341_YELLOW, ILI9341_BLACK);
  ILI9341_WriteString(309, 10, "%", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

  //=====================================================================================================

  ILI9341_FillCircle(toPageLeft.pos_x, toPageLeft.pos_y, toPageLeft.shape_r, ILI9341_LIGHTBLUE);
  ILI9341_FillRectangle(toPageLeft.pos_x, toPageLeft.pos_y-10, 15, 20, ILI9341_WHITE);
  ILI9341_FillTriangle(toPageLeft.pos_x, toPageLeft.pos_y-15, toPageLeft.pos_x-20, toPageLeft.pos_y, toPageLeft.pos_x, toPageLeft.pos_y+15, ILI9341_WHITE);

  ILI9341_FillCircle(toRightPage.pos_x, toRightPage.pos_y, toRightPage.shape_r, ILI9341_LIGHTBLUE);
  ILI9341_FillRectangle(toRightPage.pos_x-15, toRightPage.pos_y-10, 15, 20, ILI9341_WHITE);
  ILI9341_FillTriangle(toRightPage.pos_x, toRightPage.pos_y-15, toRightPage.pos_x+20, toRightPage.pos_y, toRightPage.pos_x, toRightPage.pos_y+15, ILI9341_WHITE);

}

bool isPeak()
{
  
}

/* USER CODE END Application */

