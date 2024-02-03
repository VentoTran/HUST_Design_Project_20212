#include "lcd.h"
#include "helperFunc.h"
#include "string.h"

#if USE_DMA == 1
static osEventFlagsId_t evt_id;
#endif

inline void ILI9341_LCD_LED(bool state)
{
    if (state == true)
    {
        HAL_GPIO_WritePin(ILI9341_LED_GPIO_Port, ILI9341_LED_Pin, SET);
    }
    else
    {
        HAL_GPIO_WritePin(ILI9341_LED_GPIO_Port, ILI9341_LED_Pin, RESET);
    }
}

static inline void ILI9341_Select()
{
    HAL_GPIO_WritePin(ILI9341_CS_GPIO_Port, ILI9341_CS_Pin, GPIO_PIN_RESET);
    
}

inline void ILI9341_Unselect()
{
    HAL_GPIO_WritePin(ILI9341_CS_GPIO_Port, ILI9341_CS_Pin, GPIO_PIN_SET);
}

static void ILI9341_Reset()
{
    HAL_GPIO_WritePin(ILI9341_RES_GPIO_Port, ILI9341_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(ILI9341_RES_GPIO_Port, ILI9341_RES_Pin, GPIO_PIN_SET);
}

static inline void ILI9341_WriteCommand(uint8_t cmd)
{
    // ILI9341_Select();
    HAL_GPIO_WritePin(ILI9341_DC_GPIO_Port, ILI9341_DC_Pin, GPIO_PIN_RESET);
#if USE_DMA == 1
    // Transmit DMA
    HAL_SPI_Transmit_DMA(&ILI9341_SPI_PORT, &cmd, sizeof(cmd));
    // Wait in non-blocking till DMA done
    osEventFlagsWait(evt_id, DISPLAY_EVENT_FLAG_DMA_DONE, osFlagsWaitAny, osWaitForever);
#else
    HAL_SPI_Transmit(&ILI9341_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
#endif
    // ILI9341_Unselect();
}

static inline void ILI9341_WriteData(uint8_t* buff, size_t buff_size)
{
    // ILI9341_Select();
    HAL_GPIO_WritePin(ILI9341_DC_GPIO_Port, ILI9341_DC_Pin, GPIO_PIN_SET);
#if USE_DMA == 1
    // Transmit DMA
    HAL_SPI_Transmit_DMA(&ILI9341_SPI_PORT, buff, buff_size);
    // Wait in non-blocking till DMA done
    osEventFlagsWait(evt_id, DISPLAY_EVENT_FLAG_DMA_DONE, osFlagsWaitAny, osWaitForever);
#else
    // split data in small chunks because HAL can't send more then 64K at once
    while(buff_size > 0) 
    {
        uint16_t chunk_size = buff_size > 32768 ? 32768 : buff_size;
        HAL_SPI_Transmit(&ILI9341_SPI_PORT, buff, chunk_size, HAL_MAX_DELAY);
        // HAL_SPI_Transmit_DMA(&ILI9341_SPI_PORT, buff, chunk_size);
        buff += chunk_size;
        buff_size -= chunk_size;
    }
#endif
    // ILI9341_Unselect();
}

#if USE_DMA == 1
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    osEventFlagsSet(evt_id, DISPLAY_EVENT_FLAG_DMA_DONE);
}
#endif

static void ILI9341_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    // column address set
    ILI9341_WriteCommand(0x2A); // CASET
    {
        uint8_t data[] = { (x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1 & 0xFF };
        ILI9341_WriteData(data, sizeof(data));
    }

    // row address set
    ILI9341_WriteCommand(0x2B); // RASET
    {
        uint8_t data[] = { (y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1 & 0xFF };
        ILI9341_WriteData(data, sizeof(data));
    }

    // write to RAM
    ILI9341_WriteCommand(0x2C); // RAMWR
}

void ILI9341_Init()
{
    ILI9341_Select();
    ILI9341_Reset();

    // command list is based on https://github.com/martnak/STM32-ILI9341

    // SOFTWARE RESET
    ILI9341_WriteCommand(0x01);
    HAL_Delay(200);
        
    // POWER CONTROL A
    ILI9341_WriteCommand(0xCB);
    {
        uint8_t data[] = { 0x39, 0x2C, 0x00, 0x34, 0x02 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // POWER CONTROL B
    ILI9341_WriteCommand(0xCF);
    {
        uint8_t data[] = { 0x00, 0xC1, 0x30 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // DRIVER TIMING CONTROL A
    ILI9341_WriteCommand(0xE8);
    {
        uint8_t data[] = { 0x85, 0x00, 0x78 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // DRIVER TIMING CONTROL B
    ILI9341_WriteCommand(0xEA);
    {
        uint8_t data[] = { 0x00, 0x00 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // POWER ON SEQUENCE CONTROL
    ILI9341_WriteCommand(0xED);
    {
        uint8_t data[] = { 0x64, 0x03, 0x12, 0x81 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // PUMP RATIO CONTROL
    ILI9341_WriteCommand(0xF7);
    {
        uint8_t data[] = { 0x20 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // POWER CONTROL,VRH[5:0]
    ILI9341_WriteCommand(0xC0);
    {
        uint8_t data[] = { 0x23 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // POWER CONTROL,SAP[2:0];BT[3:0]
    ILI9341_WriteCommand(0xC1);
    {
        uint8_t data[] = { 0x10 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // VCM CONTROL
    ILI9341_WriteCommand(0xC5);
    {
        uint8_t data[] = { 0x3E, 0x28 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // VCM CONTROL 2
    ILI9341_WriteCommand(0xC7);
    {
        uint8_t data[] = { 0x86 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // MEMORY ACCESS CONTROL
    ILI9341_WriteCommand(0x36);
    {
        uint8_t data[] = { 0x48 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // PIXEL FORMAT
    ILI9341_WriteCommand(0x3A);
    {
        uint8_t data[] = { 0x55 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // FRAME RATIO CONTROL, STANDARD RGB COLOR
    ILI9341_WriteCommand(0xB1);
    {
        uint8_t data[] = { 0x00, 0x18 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // DISPLAY FUNCTION CONTROL
    ILI9341_WriteCommand(0xB6);
    {
        uint8_t data[] = { 0x08, 0x82, 0x27 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // 3GAMMA FUNCTION DISABLE
    ILI9341_WriteCommand(0xF2);
    {
        uint8_t data[] = { 0x00 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // GAMMA CURVE SELECTED
    ILI9341_WriteCommand(0x26);
    {
        uint8_t data[] = { 0x01 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // POSITIVE GAMMA CORRECTION
    ILI9341_WriteCommand(0xE0);
    {
        uint8_t data[] = { 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
                            0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // NEGATIVE GAMMA CORRECTION
    ILI9341_WriteCommand(0xE1);
    {
        uint8_t data[] = { 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
                            0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F };
        ILI9341_WriteData(data, sizeof(data));
    }

    // EXIT SLEEP
    ILI9341_WriteCommand(0x11);
    HAL_Delay(120);

    // TURN ON DISPLAY
    ILI9341_WriteCommand(0x29);

    // MADCTL
    ILI9341_WriteCommand(0x36);
    {
        uint8_t data[] = { ILI9341_ROTATION };
        ILI9341_WriteData(data, sizeof(data));
    }

    ILI9341_Unselect();
    
    // ILI9341_FillScreen(ILI9341_WHITE);
    ILI9341_LCD_LED(true);
}

void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if((x >= ILI9341_WIDTH) || (y >= ILI9341_HEIGHT))
        return;

    ILI9341_Select();

    ILI9341_SetAddressWindow(x, y, x+1, y+1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    ILI9341_WriteData(data, sizeof(data));

    ILI9341_Unselect();
}

void ILI9341_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t t; 
	int xerr = 0, yerr = 0, delta_x, delta_y, distance; 
	int incx, incy, uRow, uCol;

	delta_x = x2 - x1;
	delta_y = y2 - y1; 
	uRow = x1; 
	uCol = y1;

	if (delta_x > 0)
        incx = 1;
	else if (delta_x == 0)
        incx = 0;
	else 
    {
        incx = -1;
        delta_x = -delta_x;
    } 
	
    if (delta_y > 0)
        incy = 1; 
	else if (delta_y==0)
        incy = 0;
	else
    {
        incy = -1;
        delta_y = -delta_y;
    } 

	if (delta_x > delta_y)
        distance = delta_x;
	else 
        distance = delta_y; 
    
	for(t = 0; t <= distance + 1; t++)
	{  
		ILI9341_DrawPixel(uRow, uCol, color);

		xerr += delta_x; 
		yerr += delta_y; 

		if (xerr > distance) 
		{ 
			xerr -= distance; 
			uRow += incx; 
		} 
		if (yerr > distance) 
		{ 
			yerr -= distance; 
			uCol += incy; 
		} 
	}  
}

static void ILI9341_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
    uint32_t i, b, j;

    ILI9341_SetAddressWindow(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                uint8_t data[] = { color >> 8, color & 0xFF };
                ILI9341_WriteData(data, sizeof(data));
            } else {
                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
                ILI9341_WriteData(data, sizeof(data));
            }
        }
    }
}

void ILI9341_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor)
{
    ILI9341_Select();

    while(*str) {
        if(x + font.width >= ILI9341_WIDTH) {
            x = 0;
            y += font.height;
            if(y + font.height >= ILI9341_HEIGHT) {
                break;
            }

            if(*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }

        ILI9341_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

    ILI9341_Unselect();
}

void ILI9341_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    ILI9341_DrawLine(x, y, x + w, y, color);
    ILI9341_DrawLine(x + w, y, x + w, y + h, color);
    ILI9341_DrawLine(x, y, x, y + h, color);
    ILI9341_DrawLine(x, y + h, x + w, y + h, color);
}

void ILI9341_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // clipping
    if((x >= ILI9341_WIDTH) || (y >= ILI9341_HEIGHT)) return;
    if((x + w - 1) >= ILI9341_WIDTH) w = ILI9341_WIDTH - x;
    if((y + h - 1) >= ILI9341_HEIGHT) h = ILI9341_HEIGHT - y;

    ILI9341_Select();
    ILI9341_SetAddressWindow(x, y, x+w-1, y+h-1);

    uint8_t data[] = { color >> 8, color & 0xFF };
    HAL_GPIO_WritePin(ILI9341_DC_GPIO_Port, ILI9341_DC_Pin, GPIO_PIN_SET);
    for(y = h; y > 0; y--) {
        for(x = w; x > 0; x--) {
            HAL_SPI_Transmit(&ILI9341_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
        }
    }

    ILI9341_Unselect();
}

void ILI9341_FillScreen(uint16_t color)
{
    ILI9341_FillRectangle(0, 0, ILI9341_WIDTH, ILI9341_HEIGHT, color);
}

void ILI9341_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data)
{
    if((x >= ILI9341_WIDTH) || (y >= ILI9341_HEIGHT)) return;
    if((x + w - 1) >= ILI9341_WIDTH) return;
    if((y + h - 1) >= ILI9341_HEIGHT) return;

    ILI9341_Select();
    ILI9341_SetAddressWindow(x, y, x+w-1, y+h-1);
    ILI9341_WriteData((uint8_t*)data, sizeof(uint16_t)*w*h);
    ILI9341_Unselect();
}

void ILI9341_InvertColors(bool invert)
{
    ILI9341_Select();
    ILI9341_WriteCommand(invert ? 0x21 /* INVON */ : 0x20 /* INVOFF */);
    ILI9341_Unselect();
}

void ILI9341_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
    ILI9341_DrawLine(x1, y1, x2, y2, color);
    ILI9341_DrawLine(x2, y2, x3, y3, color);
    ILI9341_DrawLine(x3, y3, x1, y1, color);
}

static void _swap(uint16_t *a, uint16_t *b)
{
	uint16_t tmp;
    tmp = *a;
	*a = *b;
	*b = tmp;
}

void ILI9341_FillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t a, b, y, last;
	int dx01, dy01, dx02, dy02, dx12, dy12;
	long sa = 0;
	long sb = 0;

    if (y0 > y1) 
	{
        _swap(&y0,&y1); 
		_swap(&x0,&x1);
    }
    if (y1 > y2) 
	{
        _swap(&y2,&y1); 
		_swap(&x2,&x1);
    }
    if (y0 > y1) 
	{
        _swap(&y0,&y1); 
		_swap(&x0,&x1);
    }
	
    if (y0 == y2) 
	{ 
		a = b = x0;
		if(x1 < a)
        {
			a = x1;
        }
        else if (x1 > b)
        {
            b = x1;
        }

        if (x2 < a)
        {
            a = x2;
        }
        else if (x2 > b)
        {
            b = x2;
        }

		ILI9341_FillRectangle(a, y0, b - a + 1, y0 - y0 + 1, color);

        return;
	}

	dx01 = x1 - x0;
	dy01 = y1 - y0;
	dx02 = x2 - x0;
	dy02 = y2 - y0;
	dx12 = x2 - x1;
	dy12 = y2 - y1;
	
	if(y1 == y2)
	{
		last = y1; 
	}
    else
	{
		last = y1 - 1; 
	}

	for(y = y0; y <= last; y++) 
	{
		a = x0 + sa / dy01;
		b = x0 + sb / dy02;
		sa += dx01;
        sb += dx02;
        if(a > b)
        {
			_swap(&a,&b);
		}
		ILI9341_FillRectangle(a, y, b - a + 1, y - y + 1, color);
	}

	sa = dx12 * (y - y1);
	sb = dx02 * (y - y0);

	for(; y <=y2 ; y++) 
	{
		a = x1 + sa / dy12;
		b = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		if(a > b)
		{
			_swap(&a,&b);
		}
		ILI9341_FillRectangle(a, y, b - a + 1, y - y + 1, color);
	}
}

static int sqrt_t(int x)
{
    if (x == 0)
        return 0;
    double last = 0;
    double res = 1;
    while (res != last)
    {
        last = res;
        res = (res + x / res) / 2;
    }
    return (int)res;
}

void ILI9341_DrawCircle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
    int a, b, pre_b;
    a = r-1;
    b = 0;
    pre_b = 0;

    for(; a >= -r; a--)
    {   
        b = sqrt_t(r*r - a*a);
        b = (b > (r-1))? (r-1) : b;
        ILI9341_DrawLine(x + b, y - a, x + pre_b, y - a, color);
        ILI9341_DrawLine(x - b, y - a, x - pre_b, y - a, color);
        pre_b = b;
    }
}

void ILI9341_FillCircle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
    int a, b;
    a = r-1;
    b = 0;

    for(; a > -r; a--)
    {   
        b = sqrt_t(r*r - a*a);
        b = (b > (r-1))? (r-1) : b;
        ILI9341_DrawLine(x + b, y - a, x - b, y - a, color);
    }
}

void ILI9341_PlotTimeGraph32( uint16_t x0, uint16_t y0, uint8_t graphSizeX, 
                            uint32_t* Ydata, uint8_t graphSizeY, uint8_t Ymin, uint8_t DataToPixelRatio,
                            uint16_t YlabelMin, uint16_t YlabelMid, uint16_t YlabelMax, 
                            uint16_t XlabelMin, uint16_t XlabelMid, uint16_t XlabelMax, 
                            char* Xunit, char* Yunit, uint16_t color)
{
    char temp[5] = {0};
    
    //================================================= Y Axis =====================================================
    ILI9341_DrawLine(x0, y0, x0, y0 - graphSizeY, ILI9341_WHITE);
    ILI9341_DrawLine(x0, y0 - graphSizeY, x0, y0 - graphSizeY - 10, ILI9341_WHITE);
    ILI9341_DrawLine(x0 - 4, y0 - graphSizeY - 6, x0, y0 - graphSizeY - 10, ILI9341_WHITE);
    ILI9341_DrawLine(x0 + 4, y0 - graphSizeY - 6, x0, y0 - graphSizeY - 10, ILI9341_WHITE);

    memset(temp, '\0', sizeof(temp));
    intToStr(YlabelMin, temp, 3);
    ILI9341_DrawLine(x0, y0, x0 - 3, y0, ILI9341_WHITE);
    ILI9341_WriteString(x0 - 5 - 21, y0 - 5, temp, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
    memset(temp, '\0', sizeof(temp));
    intToStr(YlabelMid, temp, 4);
    ILI9341_DrawLine(x0, y0 - graphSizeY/2, x0 - 3, y0 - graphSizeY/2, ILI9341_WHITE);
    ILI9341_WriteString(x0 - 5 - 28, y0 - graphSizeY/2 - 5, temp, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
    memset(temp, '\0', sizeof(temp));
    intToStr(YlabelMax, temp, 5);
    ILI9341_DrawLine(x0, y0 - graphSizeY, x0 - 3, y0 - graphSizeY, ILI9341_WHITE);
    ILI9341_WriteString(x0 - 5 - 35, y0 - graphSizeY - 5, temp, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

    ILI9341_WriteString(x0, y0 - graphSizeY - 5 - 20, Yunit, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);


    //================================================= X Axis =====================================================
    ILI9341_DrawLine(x0, y0, x0 + graphSizeX, y0, ILI9341_WHITE);
    ILI9341_DrawLine(x0 + graphSizeX, y0, x0 + graphSizeX + 10, y0, ILI9341_WHITE);
    ILI9341_DrawLine(x0 + graphSizeX + 6, y0 - 4, x0 + graphSizeX + 10, y0, ILI9341_WHITE);
    ILI9341_DrawLine(x0 + graphSizeX + 6, y0 + 4, x0 + graphSizeX + 10, y0, ILI9341_WHITE);

    memset(temp, '\0', sizeof(temp));
    intToStr(XlabelMin, temp, 1);
    ILI9341_DrawLine(x0, y0, x0, y0 + 3, ILI9341_WHITE);
    ILI9341_WriteString(x0 - 3, y0 + 10, temp, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
    memset(temp, '\0', sizeof(temp));
    intToStr(XlabelMid, temp, 3);
    ILI9341_DrawLine(x0 + graphSizeX/2, y0, x0 + graphSizeX/2, y0 + 3, ILI9341_WHITE);
    ILI9341_WriteString(x0 + graphSizeX/2 - 10, y0 + 10, temp, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
    memset(temp, '\0', sizeof(temp));
    intToStr(XlabelMax, temp, 4);
    ILI9341_DrawLine(x0 + graphSizeX, y0, x0 + graphSizeX, y0 + 3, ILI9341_WHITE);
    ILI9341_WriteString(x0 + graphSizeX - 14, y0 + 10, temp, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

    ILI9341_WriteString(x0 + graphSizeX + 5 + 10, y0 - 5, Xunit, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

    //================================================= Plot Data ==================================================
    for (uint8_t i = 0; i < graphSizeX; i++)
    {
        if ((Ydata[i] >= YlabelMin) && (Ydata[i] <= YlabelMax))
        {ILI9341_DrawPixel(x0 + i, y0 - (Ydata[i] - Ymin)/DataToPixelRatio, color);}
        else if (Ydata[i] < YlabelMin)
        {ILI9341_DrawPixel(x0 + i, y0, color);}
        else if (Ydata[i] > YlabelMax)
        {ILI9341_DrawPixel(x0 + i, y0 + graphSizeY, color);}
        
    }
}

void ILI9341_PlotTimeGraph8( uint16_t x0, uint16_t y0, uint8_t graphSizeX, 
                            uint8_t* Ydata, uint8_t graphSizeY, uint8_t Ymin, uint8_t DataToPixelRatio,
                            uint16_t YlabelMin, uint16_t YlabelMid, uint16_t YlabelMax, 
                            uint16_t XlabelMin, uint16_t XlabelMid, uint16_t XlabelMax, 
                            char* Xunit, char* Yunit, uint16_t color)
{
    char temp[5] = {0};
    
    //================================================= Y Axis =====================================================
    ILI9341_DrawLine(x0, y0, x0, y0 - graphSizeY, ILI9341_WHITE);
    ILI9341_DrawLine(x0, y0 - graphSizeY, x0, y0 - graphSizeY - 10, ILI9341_WHITE);
    ILI9341_DrawLine(x0 - 4, y0 - graphSizeY - 6, x0, y0 - graphSizeY - 10, ILI9341_WHITE);
    ILI9341_DrawLine(x0 + 4, y0 - graphSizeY - 6, x0, y0 - graphSizeY - 10, ILI9341_WHITE);

    memset(temp, '\0', sizeof(temp));
    intToStr(YlabelMin, temp, 3);
    ILI9341_DrawLine(x0, y0, x0 - 3, y0, ILI9341_WHITE);
    ILI9341_WriteString(x0 - 5 - 21, y0 - 5, temp, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
    memset(temp, '\0', sizeof(temp));
    intToStr(YlabelMid, temp, 4);
    ILI9341_DrawLine(x0, y0 - graphSizeY/2, x0 - 3, y0 - graphSizeY/2, ILI9341_WHITE);
    ILI9341_WriteString(x0 - 5 - 28, y0 - graphSizeY/2 - 5, temp, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
    memset(temp, '\0', sizeof(temp));
    intToStr(YlabelMax, temp, 5);
    ILI9341_DrawLine(x0, y0 - graphSizeY, x0 - 3, y0 - graphSizeY, ILI9341_WHITE);
    ILI9341_WriteString(x0 - 5 - 35, y0 - graphSizeY - 5, temp, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

    ILI9341_WriteString(x0, y0 - graphSizeY - 5 - 20, Yunit, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);


    //================================================= X Axis =====================================================
    ILI9341_DrawLine(x0, y0, x0 + graphSizeX, y0, ILI9341_WHITE);
    ILI9341_DrawLine(x0 + graphSizeX, y0, x0 + graphSizeX + 10, y0, ILI9341_WHITE);
    ILI9341_DrawLine(x0 + graphSizeX + 6, y0 - 4, x0 + graphSizeX + 10, y0, ILI9341_WHITE);
    ILI9341_DrawLine(x0 + graphSizeX + 6, y0 + 4, x0 + graphSizeX + 10, y0, ILI9341_WHITE);

    memset(temp, '\0', sizeof(temp));
    intToStr(XlabelMin, temp, 1);
    ILI9341_DrawLine(x0, y0, x0, y0 + 3, ILI9341_WHITE);
    ILI9341_WriteString(x0 - 3, y0 + 10, temp, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
    memset(temp, '\0', sizeof(temp));
    intToStr(XlabelMid, temp, 3);
    ILI9341_DrawLine(x0 + graphSizeX/2, y0, x0 + graphSizeX/2, y0 + 3, ILI9341_WHITE);
    ILI9341_WriteString(x0 + graphSizeX/2 - 10, y0 + 10, temp, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
    memset(temp, '\0', sizeof(temp));
    intToStr(XlabelMax, temp, 4);
    ILI9341_DrawLine(x0 + graphSizeX, y0, x0 + graphSizeX, y0 + 3, ILI9341_WHITE);
    ILI9341_WriteString(x0 + graphSizeX - 14, y0 + 10, temp, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

    ILI9341_WriteString(x0 + graphSizeX + 5 + 10, y0 - 5, Xunit, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);

    //================================================= Plot Data ==================================================
    for (uint8_t i = 0; i < graphSizeX; i++)
    {
        if ((Ydata[i] >= YlabelMin) && (Ydata[i] <= YlabelMax))
        {ILI9341_DrawPixel(x0 + i, y0 - (Ydata[i] - Ymin)/DataToPixelRatio, color);}
        else if (Ydata[i] < YlabelMin)
        {ILI9341_DrawPixel(x0 + i, y0, color);}
        else if (Ydata[i] > YlabelMax)
        {ILI9341_DrawPixel(x0 + i, y0 + graphSizeY, color);}
        
    }
}

inline void ILI9341_DrawBitmapDMA(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint8_t * pdata)
{
    ILI9341_Select();
    ILI9341_SetAddressWindow(x0, y0, x0 + width - 1, y0 + height - 1);
    ILI9341_WriteData((uint8_t*)pdata, sizeof(uint16_t)*width*height);
    ILI9341_Unselect();
}

inline void Display_SetEventHandler(osEventFlagsId_t eventFlag)
{
#if USE_DMA == 1
    evt_id = eventFlag;
#else
    UNUSED(eventFlag);
#endif  //  USE_DMA
}



