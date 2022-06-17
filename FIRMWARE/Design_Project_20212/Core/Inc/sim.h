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
#include "helperFunc.h"
#include "usart.h"


// === CONFIG ===
#define UART_SIM    &huart3
#define FREERTOS    1
#define CMD_DELAY_LONG          5000
#define CMD_DELAY_MEDIUM        2000
#define CMD_DELAY_SHORT         1000
#define CMD_DELAY_VERYSHORT     200


// ============================================

#define PWRKEY_PORT SIM_PWR_GPIO_Port
#define PWRKEY_PIN  SIM_PWR_Pin

// ============================================
typedef struct 
{
    char *apn;
    char *apn_user;
    char *apn_pass;
    uint8_t signalQuality;
    uint8_t SIMCardStatusCode;
    uint32_t Balance;
} SIM_t;

typedef enum
{
    SIM_RX_START,
    SIM_RX_RECIEVING,
    SIM_RX_END
} SIM_RX_t;

typedef struct 
{
    uint8_t Octec1;
    uint8_t Octec2;
    uint8_t Octec3;
    uint8_t Octec4;
} SIM_IP_t;

bool SIM_Init(void);

bool SIM_Deinit(void);

void SIM_Reset(void);

void SIM_RXCallback(void);

void SIM_clearRX(void);

bool SIM_checkSIMCard(void);

void SIM_sendSMS(char* number, char* message);

void SIM_call(char* number);

uint32_t SIM_checkBalance(void);

void SIM_sendATCommand(char* command);

bool SIM_sendATCommandResponse(char* command, char* response);

bool SIM_startGPRS(void);

bool SIM_getIP(void);

uint8_t SIM_checkSignalStrength(void);

#endif



