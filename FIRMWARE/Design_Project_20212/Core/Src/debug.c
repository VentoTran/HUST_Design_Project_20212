/**
 * @file debug.c
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-06-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "debug.h"
#include "stdio.h"
#include <errno.h>
#include <sys/unistd.h>
#include <stdarg.h>
#include <string.h>

static uint8_t rx_debug = 0;
bool isNaize = false;


void Debug_Init(void)
{
    HAL_UART_Receive_IT(UART_DEBUG, &rx_debug, 1);
}

void debug_callback(void)
{
    if (rx_debug == '^')
    {
        HAL_NVIC_SystemReset();
    }
    else if (rx_debug == 'F')
    {
        isNaize = !isNaize;
    }
    HAL_UART_Receive_IT(UART_DEBUG, &rx_debug, 1);
}

#if DEBUG == 1
/**
 * @brief 
 * 
 * @param fmt 
 * @param argp 
 */
void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        HAL_UART_Transmit(UART_DEBUG, (uint8_t*)string, strlen(string), 100); // send message via UART
    }
}
/**
 * @brief custom printf() function
 * 
 * @param fmt 
 * @param ... 
 */
void logPC(const char *fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}
#else
void logPC(const char *fmt, ...)
{
    UNUSED(fmt);
}
#endif


// int fputc(int ch, FILE *f) 
// {

//     HAL_UART_Transmit(UART_DEBUG, (uint8_t*)ch, 1, 100);

//     return ch;
//     /* If character is not correct, you can return EOF (-1) to stop writing */
//     //return -1;
// }

// int _write(int file, char *data, int len)
// {
//     if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
//     {
//         errno = EBADF;
//         return -1;
//     }

//     // arbitrary timeout 1000
//     HAL_StatusTypeDef status = HAL_UART_Transmit(UART_DEBUG, (uint8_t*)data, len, 100);

//     // return # of bytes written - as best we can tell
//     return (status == HAL_OK ? len : 0);
// }

// int __io_putchar(int ch)
// {
//     /* Place your implementation of fputc here */
//       /* e.g. write a character to the LCD */
//     HAL_UART_Transmit(UART_DEBUG, (uint8_t*)ch, 1, 100);

//     return ch;
// }

