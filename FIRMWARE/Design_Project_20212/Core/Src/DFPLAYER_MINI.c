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
#include "string.h"

extern UART_HandleTypeDef huart4;
#define DF_UART &huart4

static uint8_t df_rx_char = 0;
static uint8_t df_rx_index = 0;
uint8_t df_rx_buffer[50] = {0};

DF_PLAYER DF = {
	.status = false, 
	.volume = INIT_VOL,
	.currentSongNumber = 1,
	.SDCardOK = false,
	.totalSongs = 0
};

//----------------------------------------------------------------- Configure -----------------------------------------------------------------

void Send_cmd (uint8_t cmd, uint8_t Parameter1, uint8_t Parameter2)
{
	uint16_t Checksum = Version + Cmd_Len + cmd + Feedback + Parameter1 + Parameter2;
	Checksum = 0-Checksum;

	uint8_t CmdSequence[10] = {Start_Byte, Version, Cmd_Len, cmd, Feedback, Parameter1, Parameter2, (Checksum>>8)&0x00ff, (Checksum&0x00ff), End_Byte};

	HAL_UART_Transmit(DF_UART, CmdSequence, 10, HAL_MAX_DELAY);
}

void DF_RX_Callback(void)
{
	df_rx_buffer[df_rx_index++] = df_rx_char;
	if (df_rx_char == 0xEF)
	{
		if ((df_rx_buffer[df_rx_index-10] == 0x7E) && (df_rx_buffer[df_rx_index-9] == 0xFF) && (df_rx_buffer[df_rx_index-8] == 0x06) && (df_rx_buffer[df_rx_index-1] == 0xEF))
		{
			if (df_rx_buffer[df_rx_index-7] == 0x3D)
			{
				DF.currentSongNumber = df_rx_buffer[df_rx_index-5]*255 + df_rx_buffer[df_rx_index-4] + 1;
				DF.status = PAUSING;
			}
			else if (df_rx_buffer[df_rx_index-7] == 0x3F)
			{
				if (df_rx_buffer[df_rx_index-4] != 0x02)
				{DF.SDCardOK = false;}
				else
				{DF.SDCardOK = true;}
			}
			else if (df_rx_buffer[df_rx_index-7] == 0x3B)
			{
				DF.SDCardOK = false;
			}
			else if (df_rx_buffer[df_rx_index-7] == 0x48)
			{
				DF.totalSongs = df_rx_buffer[df_rx_index-5]*255 + df_rx_buffer[df_rx_index-4];
			}
		}
		DF_Clear_RX();
	}
	HAL_UART_Receive_IT(DF_UART, &df_rx_char, 1);
}

void DF_Clear_RX(void)
{
	memset(df_rx_buffer, '\0', sizeof(df_rx_buffer));
	df_rx_index = 0;
	df_rx_char = '\0';
}

void DF_Reset(void)
{
	Send_cmd(0x0C, 0x00, 0x00);
	HAL_Delay(1500);
}

void DF_Init(uint8_t volume)
{
	DF_Clear_RX();
	HAL_UART_Receive_IT(DF_UART, &df_rx_char, 1);
	DF_Reset();
	Send_cmd(0x09, 0x00, 0x01);
	HAL_Delay(200);
	Send_cmd(0x06, 0x00, volume);
	HAL_Delay(500);
	Send_cmd(0x48, 0x00, 0x00);
	HAL_Delay(500);
	DF_Clear_RX();
}

void DF_Sleep(void)
{
	Send_cmd(0x0A, 0x00, 0x00);
	HAL_Delay(500);
}

//--------------------------------------------------------------------- Interface ---------------------------------------------------------------

void DF_PlayFromStart(void)
{
	Send_cmd(0x03, 0x00, 0x01);
	HAL_Delay(200);
}

void DF_Next(void)
{
	Send_cmd(0x01, 0x00, 0x00);
	// DF.currentSongNumber++;
	// if (DF.currentSongNumber > DF.totalSongs)
	// {
	// 	DF.currentSongNumber = 1;
	// }
	HAL_Delay(200);
}

void DF_Previous(void)
{
	Send_cmd(0x02, 0, 0);
	// DF.currentSongNumber--;
	// if ((DF.currentSongNumber == 0) || (DF.currentSongNumber > DF.totalSongs))
	// {
	// 	DF.currentSongNumber = DF.totalSongs;
	// }
	HAL_Delay(200);
}

void DF_Pause(void)
{
	Send_cmd(0x0E, 0, 0);
	DF.status = PAUSING;
	HAL_Delay(200);
}

void DF_Playback(void)
{
	Send_cmd(0x0D, 0, 0);
	DF.status = PLAYING;
	HAL_Delay(200);
}

void DF_SetVol(uint8_t Level)
{
	DF.volume = Level;
	Send_cmd(0x06, 0x00, Level);
	HAL_Delay(200);
}

// void DF_setRandom(void)
// {
// 	Send_cmd(0x08, 0x00, 0x03);
// 	HAL_Delay(200);
// }

// void DF_setRepeat(void)
// {
// 	Send_cmd(0x08, 0x00, 0x00);
// 	HAL_Delay(200);
// }

// void DF_setSingle(void)
// {
// 	Send_cmd(0x08, 0x00, 0x02);
// 	HAL_Delay(200);
// }

//------------------------------------------------------------------- Query -------------------------------------------------------------

uint8_t DF_getVol(void)
{
	return DF.volume;
}

bool DF_getState(void)
{
	return DF.status;
}

void DF_setState(bool state)
{
	DF.status = state;
}

uint32_t DF_getTotalSongs(void)
{
	return DF.totalSongs;
}

uint32_t DF_getCurrentSongNumber(void)
{
	return DF.currentSongNumber;
}

void DF_setCurrentSongNumber(uint32_t number)
{
	DF.currentSongNumber = number;
}










