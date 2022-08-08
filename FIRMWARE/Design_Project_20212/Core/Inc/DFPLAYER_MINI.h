/**
 * @file DFPLAYER_MINI.h
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-06-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _DFPLAYER_MINI_H_
#define _DFPLAYER_MINI_H_

#include "main.h"
#include "stdbool.h"

#define Source      0x02  // TF CARD
#define Start_Byte  0x7E
#define End_Byte    0xEF
#define Version     0xFF
#define Cmd_Len     0x06
#define Feedback    0x01    //If need for Feedback: 0x01,  No Feedback: 0x00

#define INIT_VOL    0x01
#define MAX_VOLUME  0x09
#define MIN_VOLUME  0x00

#define PLAYING     (true)
#define PAUSING     (false)

typedef struct DF_PLAYER
{
    bool status;
    uint8_t volume;
    uint32_t totalSongs;
    uint32_t currentSongNumber;
    bool SDCardOK;
} DF_PLAYER;

void Send_cmd(uint8_t cmd, uint8_t Parameter1, uint8_t Parameter2);

void DF_Init(uint8_t volume);
void DF_Reset(void);
void DF_Sleep(void);
void DF_RX_Callback(void);
void DF_Clear_RX(void);

void DF_PlayFromStart(void);
void DF_Next(void);
void DF_Pause(void);
void DF_Previous(void);
void DF_Playback(void);
// void DF_IncVol(void);
// void DF_DecVol(void);
void DF_SetVol(uint8_t Level);
// void DF_setRandom(void);
// void DF_setRepeat(void);
// void DF_setSingle(void);

void DF_setState(bool state);
void DF_setCurrentSongNumber(uint32_t number);

uint8_t DF_getVol(void);
bool DF_getState(void);
uint32_t DF_getTotalSongs(void);
uint32_t DF_getCurrentSongNumber(void);


#endif /* _DFPLAYER_MINI_H_ */
