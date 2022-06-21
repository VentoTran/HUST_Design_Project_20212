/**
 * @file debug.h
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-06-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include "main.h"
#include "usart.h"

#define UART_DEBUG  &huart1

void logPC(const char *fmt, ...);


// void log(char* message);

#endif /*_DEBUG_H_*/
