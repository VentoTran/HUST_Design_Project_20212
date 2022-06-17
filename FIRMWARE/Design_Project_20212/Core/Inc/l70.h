/**
 * @file l70.h
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-06-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __INC_L70_H__
#define __INC_L70_H__

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "usart.h"

#define UART_L70    &huart2
#define FREERTOS    1

#define L70_CHECK           0
#define L70_NOT_CHECK       1

typedef enum
{
    L70_NOT_OK,
    L70_OK
} l70State_t;

typedef enum
{
    L70_CALLBACK_CMD,
    L70_CALLBACK_GPS
} l70Callback_t;

typedef enum
{
    L70_START_DATA,
    L70_DATA,
    L70_END_DATA
} l70Data_t;

typedef enum
{
    L70_NOT_RX_CPLT,
    L70_RX_CPLT
} l70RxCplt_t;

typedef struct 
{
    char utcTime[12];
    char latitude[12];
    char NorS;
    char longitude[12];
    char EorW;
    char fixStatus;
    char numberOfSatellites[2];
} l70GPGGAType_t;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

void l70_callback(void);

l70State_t l70_sendCommand(char *l70Command, char *trueResponse, int checkOrNot);

void l70_init(void);

void l70_handleGPS(char *latData, char *longData, char *GPSResponse);

char* l70_receiveGPS(void);


// void l70_debug(uint8_t *pData, uint16_t Size, uint32_t Timeout);
// void l70_timerCallback(void);
// void l70_errorHandle(void);
// void l70_receiveGPSData(void);


#endif

