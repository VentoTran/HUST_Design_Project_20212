/**
 * @file sim.h
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _SIM_H_
#define _SIM_H_

#include <main.h>


// === CONFIG ===
#define UART_SIM    &huart3
#define FREERTOS    1
#define CMD_DELAY_LONG          5000
#define CMD_DELAY_MEDIUM        2000
#define CMD_DELAY_SHORT         1000
#define CMD_DELAY_VERYSHORT     100
// ==============

typedef struct 
{
    char *apn;
    char *apn_user;
    char *apn_pass;
} SIM_t;

bool SIM_Init(void);

bool SIM_Deinit(void);

void SIM_Reset(void);

void SIM_RXCallback(void);

void SIM_clearRX(void);

void SIM_checkSIMCard(void);

void SIM_sendSMS(char* numer, char* message);

void SIM_call(char* number);

uint32_t SIM_checkBalance(void);

void SIM_sendATCommand(char* command);

bool SIM_sendATCommandResponse(char* command, char* response);




#endif



