/**
 * @file l70.c
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-06-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "l70.h"
#include "helperFunc.h"

#if FREERTOS == 1
#include "cmsis_os2.h"
#endif

l70GPGGAType_t l70GPGGA = {0};
int k = 0;
char l70RxData[100] = {0};
char l70Data = 0;
l70Data_t l70RxState;

void l70_callback(void)
{
    //Start receive Data
    if(l70Data == '$' && l70RxState == L70_START_DATA)
    {
        l70RxData[k++] = l70Data;
        l70RxState = L70_DATA;
        HAL_UART_Receive_IT(UART_L70, (uint8_t*)&l70Data, 1);
    }
    //End process
    else if(l70Data == '\n' && l70RxState == L70_DATA)
    {
        l70RxData[k++] = l70Data;
        l70RxState = L70_END_DATA;
    }
    //Receving Data
    else if(l70RxState == L70_DATA)
    {
        l70RxData[k++] = l70Data;
        HAL_UART_Receive_IT(UART_L70, (uint8_t*)&l70Data, 1);
    }
    else 
    {
        HAL_UART_Receive_IT(UART_L70, (uint8_t*)&l70Data, 1);
    }
}

char *l70_receiveGPS(void)
{
    k = 0;
    memset(l70RxData, '\0', sizeof(l70RxData));
    l70RxState = L70_START_DATA;
    HAL_UART_Receive_IT(UART_L70, (uint8_t*)&l70Data, 1);
    while(!(l70RxState == L70_END_DATA));
    return l70RxData;
}

void l70_init(void)
{
    //Output once every one position fix
    //GGA interval fix data
    l70RxState = L70_START_DATA;
    HAL_UART_Receive_IT(UART_L70, (uint8_t*)&l70Data, 1);
    
    HAL_UART_Transmit_IT(UART_L70 ,(uint8_t*)"$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n", 51);

    osDelay(1000);
    k = 0;
    memset(l70RxData, '\0', sizeof(l70RxData));
    l70RxState = L70_START_DATA;
    
    HAL_UART_Receive_IT(UART_L70, (uint8_t*)&l70Data, 1);

    HAL_UART_Transmit_IT(UART_L70 ,(uint8_t*)"$PMTK185,0*22\r\n", 15);
    osDelay(1000);
    k = 0;
    memset(l70RxData, '\0', sizeof(l70RxData));
    HAL_UART_Receive_IT(UART_L70, (uint8_t*)&l70Data, 1);
    l70RxState = L70_START_DATA;

    HAL_UART_Transmit_IT(UART_L70 ,(uint8_t*)"$PMTK285,2,100*3E\r\n", 19);
    osDelay(10000);
    k = 0;
    memset(l70RxData, '\0', sizeof(l70RxData));
}

void l70_handleGPS(char *latData, char *longData, char *GPSResponse)
{
    int temp = 0;
    double decPart = 0;
    double intPart = 0;
    double l70latitude = 0;
    double l70longitude = 0;
    GPSResponse = strstr(GPSResponse, "$GPGGA");
    sscanf( GPSResponse, "$GPGGA,%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,]",\
            l70GPGGA.utcTime, l70GPGGA.latitude, &l70GPGGA.NorS, l70GPGGA.longitude,\
            &l70GPGGA.EorW, &l70GPGGA.fixStatus, l70GPGGA.numberOfSatellites);
    //convert str to double
    l70latitude = atof(l70GPGGA.latitude);
   //convert ddmm.mmmm to dddd.dddd (d: degree; m: minute)
    temp = (int)l70latitude;
    intPart = (double)(temp / 100);
    decPart = (double)(temp % 100) + l70latitude - (double)temp;
    l70latitude = intPart + decPart / 60;
    l70longitude = atof(l70GPGGA.longitude);
    temp = (int)l70longitude;
    intPart = (double)(temp / 100);
    decPart = (double)(temp % 100) + l70longitude - (double)temp;
    l70longitude = intPart + decPart / 60;
    ftoa(l70latitude, latData, 6);
    ftoa(l70longitude, longData, 6);
}

