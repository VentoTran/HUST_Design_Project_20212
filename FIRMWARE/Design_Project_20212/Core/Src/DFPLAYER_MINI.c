/**
 * @file DFPLAYER_MINI.c
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-06-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "DFPLAYER_MINI.h"
#include "usart.h"

extern UART_HandleTypeDef huart4;
#define DF_UART &huart4

static uint8_t df_rx_char = 0;
static uint8_t df_rx_index = 0;
uint8_t df_rx_buffer[50] = {0};

DF_PLAYER DF = {false, INIT_VOL};

void Send_cmd (uint8_t cmd, uint8_t Parameter1, uint8_t Parameter2)
{
	uint16_t Checksum = Version + Cmd_Len + cmd + Feedback + Parameter1 + Parameter2;
	Checksum = 0-Checksum;

	uint8_t CmdSequence[10] = {Start_Byte, Version, Cmd_Len, cmd, Feedback, Parameter1, Parameter2, (Checksum>>8)&0x00ff, (Checksum&0x00ff), End_Byte};

	HAL_UART_Transmit(DF_UART, CmdSequence, 10, HAL_MAX_DELAY);
}

void DF_RX_Callback(void)
{

}

void DF_Clear_RX(void)
{

}

void DF_PlayFromStart(void)
{
	Send_cmd(0x03, 0x00, 0x01);
	HAL_Delay(200);
}

void DF_Reset(void)
{
	Send_cmd(0x0C, 0x00, 0x00);
	HAL_Delay(1500);
}

void DF_Init(uint8_t volume)
{
	DF_Reset();
	Send_cmd(0x3F, 0x00, Source);
	HAL_Delay(200);
	Send_cmd(0x06, 0x00, volume);
	HAL_Delay(500);
}

void DF_Sleep(void)
{
	Send_cmd(0x0A, 0x00, 0x00);
	HAL_Delay(500);
}

void DF_Next(void)
{
	Send_cmd(0x01, 0x00, 0x00);
	HAL_Delay(200);
}

void DF_Pause(void)
{
	Send_cmd(0x0E, 0, 0);
	DF.status = false;
	HAL_Delay(200);
}

void DF_Previous(void)
{
	Send_cmd(0x02, 0, 0);
	HAL_Delay(200);
}

void DF_Playback(void)
{
	Send_cmd(0x0D, 0, 0);
	HAL_Delay(200);
}

uint8_t DF_getVol(void)
{
	return DF.volume;
}

bool DF_getState(void)
{
	return DF.status;
}










