/**
 * @file sim.c
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "sim.h"
#include "string.h"
#include "MQTTPacket.h"

#ifdef FREERTOS
#include "cmsis_os2.h"
#endif


uint8_t rx_char = 0;
uint8_t rx_index = 0;
uint8_t rx_buffer[200] = {0};





