/* vim: set ai et ts=4 sw=4: */

#include "stm32f1xx_hal.h"
#include "touch.h"
#include <stdio.h>

#define READ_X 0xD0
#define READ_Y 0x90

#define ABS(x) ((x) > 0 ? (x) : -(x))

static void ILI9341_TouchSelect()
{
    HAL_GPIO_WritePin(ILI9341_TOUCH_CS_GPIO_Port, ILI9341_TOUCH_CS_Pin, GPIO_PIN_RESET);
}

void ILI9341_TouchUnselect()
{
    HAL_GPIO_WritePin(ILI9341_TOUCH_CS_GPIO_Port, ILI9341_TOUCH_CS_Pin, GPIO_PIN_SET);
}

bool ILI9341_TouchPressed()
{
    return HAL_GPIO_ReadPin(ILI9341_TOUCH_IRQ_GPIO_Port, ILI9341_TOUCH_IRQ_Pin) == GPIO_PIN_RESET;
}

bool ILI9341_TouchGetCoordinates(uint16_t* x, uint16_t* y)
{
    static const uint8_t cmd_read_x[] = { READ_X };
    static const uint8_t cmd_read_y[] = { READ_Y };
    static const uint8_t zeroes_tx[] = { 0x00, 0x00 };

    ILI9341_TouchSelect();

    uint32_t avg_x = 0;
    uint32_t avg_y = 0;
    uint8_t nsamples = 0;
    for(uint8_t i = 0; i < 30; i++) 
    {
        if(!ILI9341_TouchPressed())
            break;

        nsamples++;

        HAL_SPI_Transmit(&ILI9341_TOUCH_SPI_PORT, (uint8_t*)cmd_read_y, sizeof(cmd_read_y), HAL_MAX_DELAY);
        uint8_t y_raw[2];
        HAL_SPI_TransmitReceive(&ILI9341_TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, y_raw, sizeof(y_raw), HAL_MAX_DELAY);

        HAL_SPI_Transmit(&ILI9341_TOUCH_SPI_PORT, (uint8_t*)cmd_read_x, sizeof(cmd_read_x), HAL_MAX_DELAY);
        uint8_t x_raw[2];
        HAL_SPI_TransmitReceive(&ILI9341_TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, x_raw, sizeof(x_raw), HAL_MAX_DELAY);

        avg_x += (((uint16_t)x_raw[0]) << 8) | ((uint16_t)x_raw[1]);
        avg_y += (((uint16_t)y_raw[0]) << 8) | ((uint16_t)y_raw[1]);
    }

    ILI9341_TouchUnselect();

    if(nsamples < 30)
        return false;

    uint32_t raw_x = (avg_x / 30);
    // if(raw_x < ILI9341_TOUCH_MIN_RAW_X) raw_x = ILI9341_TOUCH_MIN_RAW_X;
    // if(raw_x > ILI9341_TOUCH_MAX_RAW_X) raw_x = ILI9341_TOUCH_MAX_RAW_X;

    uint32_t raw_y = (avg_y / 30);
    // if(raw_y < ILI9341_TOUCH_MIN_RAW_X) raw_y = ILI9341_TOUCH_MIN_RAW_Y;
    // if(raw_y > ILI9341_TOUCH_MAX_RAW_Y) raw_y = ILI9341_TOUCH_MAX_RAW_Y;

    //char data[50];
    // Uncomment this line to calibrate touchscreen:
    //sprintf(data, "raw_x = %ld, raw_y = %ld\n\0", raw_x, raw_y);
    //uint32_t data[] = {raw_x, raw_y};
    //HAL_UART_Transmit(&huart3, data, sizeof(data), 10);

    *x = (raw_x - ILI9341_TOUCH_MIN_RAW_X) * ILI9341_TOUCH_SCALE_X / (ILI9341_TOUCH_MAX_RAW_X - ILI9341_TOUCH_MIN_RAW_X);
    *y = (raw_y - ILI9341_TOUCH_MIN_RAW_Y) * ILI9341_TOUCH_SCALE_Y / (ILI9341_TOUCH_MAX_RAW_Y - ILI9341_TOUCH_MIN_RAW_Y);

    return true;
}

bool ILI9341_checkButton(uint16_t x, uint16_t y, myButton_t* button)
{
    if ((button->shape_r != 0) && (button->shape_w == 0) && (button->shape_h == 0))
    {
        if ((((ABS(x - button->pos_x))^2) + ((ABS(y - button->pos_y))^2)) <= ((button->shape_r)^2))
        {
            return true;
        }
    }
    else
    {
        if ((y >= button->pos_y) && (y <= (button->pos_y + button->shape_h)) && (x >= button->pos_x) && (x <= (button->pos_x + button->shape_w)))
        {
            return true;
        }
    }
    return false;
}


