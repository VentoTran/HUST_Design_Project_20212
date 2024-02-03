/* vim: set ai et ts=4 sw=4: */
#ifndef __ILI9341_TOUCH_H__
#define __ILI9341_TOUCH_H__

#include <stdbool.h>
#include "lcd.h"

/*** Redefine if necessary ***/

#define USE_CALIB       0
#define USE_FREERTOS    1

#if USE_FREERTOS == 1
#include "cmsis_os2.h"
#define DISPLAY_EVENT_FLAG_TOUCH    (0x02)
#endif  //  USE_RTOS

// Warning! Use SPI bus with < 1.3 Mbit speed, better ~650 Kbit to be save.
#define ILI9341_TOUCH_SPI_PORT hspi1
extern SPI_HandleTypeDef ILI9341_TOUCH_SPI_PORT;

#if USE_CALIB == 1
extern UART_HandleTypeDef huart1;
#endif

#define ILI9341_TOUCH_IRQ_Pin       Touch_EXT_Pin
#define ILI9341_TOUCH_IRQ_GPIO_Port Touch_EXT_GPIO_Port
#define ILI9341_TOUCH_CS_Pin        SPI1_CS_Pin
#define ILI9341_TOUCH_CS_GPIO_Port  SPI1_CS_GPIO_Port

// change depending on screen orientation
#define ILI9341_TOUCH_SCALE_X 240
#define ILI9341_TOUCH_SCALE_Y 320

// to calibrate uncomment UART_Printf line in ili9341_touch.c
#define ILI9341_TOUCH_MIN_RAW_X 1500
#define ILI9341_TOUCH_MAX_RAW_X 31000
#define ILI9341_TOUCH_MIN_RAW_Y 2000
#define ILI9341_TOUCH_MAX_RAW_Y 31500

//------------ define type --------------


typedef struct
{
    const uint16_t pos_x;
    const uint16_t pos_y;

    const uint16_t shape_r;
    const uint16_t shape_w;
    const uint16_t shape_h;

    const uint16_t color;

    bool state;

} myButton_t;



// call before initializing any SPI devices
void ILI9341_TouchSelect();
void ILI9341_TouchUnselect();

bool ILI9341_TouchPressed();
bool ILI9341_TouchGetCoordinates(uint16_t* x, uint16_t* y);

bool ILI9341_checkButton(uint16_t x, uint16_t y, const myButton_t* button);

#endif // __ILI9341_TOUCH_H__
