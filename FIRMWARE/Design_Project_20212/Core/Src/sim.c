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
#include "helperFunc.h"


#if FREERTOS == 1
#include "cmsis_os2.h"
#endif

#if USE_MQTT == 1
#include "mqtt.h"
static bool isReceivingMQTT = false;
static uint16_t MQTTByteCount = 0;
extern MQTT_t MQTT;
#endif

volatile SIM_RX_t SIM_RX_STATUS = SIM_RX_START;
static SIM_IP_t SIM_IP = {0, 0, 0, 0};
static SIM_t SIM = {"m3-world", "mms", "mms", 0, 0, 0};


static uint8_t rx_char = 0;
static uint8_t rx_index = 0;
uint8_t rx_buffer[200] = {0};


/**
 * @brief Initialize SIM module
 * 
 * @return true if successful,
 * @return false if fail
 */
bool SIM_Init(void)
{   
    // SIM_sendATCommand("ATE0\r\n");
    osDelay(200);
    if (HAL_UART_Abort_IT(UART_SIM) == HAL_OK)
    {
        HAL_UART_Receive_IT(UART_SIM, &rx_char, 1);
    }
    if ((SIM_sendATCommandResponse("AT+CFUN?\r\n", "1") == false) && (SIM_sendATCommandResponse("ATE0\r\n", "OK") == false))
    {
        HAL_GPIO_WritePin(PWRKEY_PORT, PWRKEY_PIN, GPIO_PIN_SET);
#if FREERTOS == 1
        osDelay(CMD_DELAY_MEDIUM);
#else
        HAL_Delay(CMD_DELAY_MEDIUM);
#endif
        HAL_GPIO_TogglePin(PWRKEY_PORT, PWRKEY_PIN);
    }
#if FREERTOS == 1
    osDelay(CMD_DELAY_VERYSHORT);
#else
    HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
    if (SIM_sendATCommandResponse("AT\r\n", "OK") == false)
    {
        return false;
    }
    else
    {
        SIM_sendATCommand("ATE0\r\n");
        return true;
    }
}

/**
 * @brief De-initalize SIM
 * 
 * @return true if successful,
 * @return false if fail
 */
bool SIM_Deinit(void)
{
    if (SIM_sendATCommandResponse("AT\r\n", "OK") == true)
    {
        HAL_GPIO_WritePin(PWRKEY_PORT, PWRKEY_PIN, GPIO_PIN_SET);
#if FREERTOS == 1
        osDelay(CMD_DELAY_MEDIUM);
#else
        HAL_Delay(CMD_DELAY_MEDIUM);
#endif
        HAL_GPIO_TogglePin(PWRKEY_PORT, PWRKEY_PIN);
    }
#if FREERTOS == 1
    osDelay(CMD_DELAY_VERYSHORT);
#else
    HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
    if (SIM_sendATCommandResponse("AT\r\n", "OK") == true)
    {
        return false;
    }
    else
    {
        return true;
    }
}

/**
 * @brief SIM Reset by firmware
 * 
 */
void SIM_Reset(void)
{
    if (SIM_sendATCommandResponse("AT\r\n", "OK") == true)
    {
        SIM_Deinit();
        SIM_Init();
    }
    else
    {
        SIM_Init();
    }
}

/**
 * @brief RX Interrupt Handler for module SIM.
 * Add this to RX complete callback
 * 
 */
void SIM_RXCallback(void)
{
    rx_buffer[rx_index++] = rx_char;

    if ((SIM_RX_STATUS == SIM_RX_START) && (rx_index == 1))
    {
#if USE_MQTT == 1
        if (rx_char == '0')
        {
            isReceivingMQTT = true;
            MQTTByteCount = 0xFFFF;
        }
#endif
        SIM_RX_STATUS = SIM_RX_RECIEVING;
    }
    else if (SIM_RX_STATUS == SIM_RX_RECIEVING)
    {
        if ((strstr((char *)rx_buffer, "\r\n") != NULL) && (rx_index == 2))
        {
            rx_index = 0;
        }
        else if (strstr((char *)rx_buffer, "\r\n") != NULL)
        {
            if (strstr((char*)rx_buffer, "CLOSE") != NULL)
            {
                MQTT.mqttServer.connect = 0;
            }
            SIM_RX_STATUS = SIM_RX_END;
        }
        else if ((rx_index > 100) && (isReceivingMQTT == false))
        {
            SIM_clearRX();
            SIM_RX_STATUS = SIM_RX_START;
        }
#if USE_MQTT == 1
        else if (isReceivingMQTT == true)
        {
            if (MQTTByteCount == 0xFFFF)
            {
                MQTTByteCount = rx_char;
            }
            else if (MQTTByteCount > 0)
            {
                MQTTByteCount--;
            }
            if (MQTTByteCount == 0)
            {
                MQTT_Receive(rx_buffer);
                isReceivingMQTT = false;
                SIM_clearRX();
                SIM_RX_STATUS = SIM_RX_START;
            }
        }
#endif
    }
    rx_char = '\0';
    HAL_UART_Receive_IT(UART_SIM, &rx_char, 1);
}


void SIM_clearRX(void)
{
    rx_index = 0;
    memset(rx_buffer, '\0', sizeof(rx_buffer));
    rx_char = '\0';
    SIM_RX_STATUS = SIM_RX_START;
}


bool SIM_checkSIMCard(void)
{
    bool status = true;
    status &= SIM_sendATCommandResponse("AT+CPIN?\r\n", "READY");
    status &= SIM_sendATCommandResponse("AT+CREG?\r\n", ",1");
    return status;
}


void SIM_sendSMS(char* number, char* message)
{
    char str[30] = {0};
    uint32_t timeOut = 0;

    snprintf(str, sizeof(str), "AT+CMGS=\"%s\"\r\n", number);
    SIM_sendATCommandResponse("AT+CMGF=1\r\n", "OK");
    if (SIM_sendATCommandResponse(str, ">") == true)
    {
#if FREERTOS == 1
        osDelay(CMD_DELAY_VERYSHORT);
#else
        HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
        SIM_sendATCommand(message);
#if FREERTOS == 1
        osDelay(CMD_DELAY_VERYSHORT);
#else
        HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
        SIM_clearRX();
        SIM_RX_STATUS = SIM_RX_START;
        *UART_SIM.Instance->DR = 0x1A;
        
        timeOut = HAL_GetTick();
        while ((SIM_RX_STATUS != SIM_RX_END) && ((HAL_GetTick() - timeOut) <= CMD_DELAY_LONG))
        {
#if FREERTOS == 1
        osDelay(CMD_DELAY_VERYSHORT);
#else
        HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
        }
#if FREERTOS == 1
        osDelay(CMD_DELAY_VERYSHORT);
#else
        HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
    }
}


void SIM_call(char* number)
{
    char str[30] = {0};
    snprintf(str, sizeof(str), "ATD%s;\r\n", number);
    if (SIM_sendATCommandResponse(str, "OK") == true)
    {
#if FREERTOS == 1
        osDelay(30000);
#else
        HAL_Delay(30000);
#endif
        SIM_sendATCommandResponse("ATH\r\n", "OK");
    }
}


uint32_t SIM_checkBalance(void)
{
    SIM_sendATCommand("AT+CUSD=1,\"*101#\"\r\n");
#if FREERTOS == 1
        osDelay(CMD_DELAY_LONG);
#else
        HAL_Delay(CMD_DELAY_LONG);
#endif
    char* p = strstr(rx_buffer, "TK chinh");
    sscanf(p, "TK chinh=%[^ ] ", SIM.Balance);
    return SIM.Balance;
}


void SIM_sendATCommand(char* command)
{
    SIM_clearRX();
    SIM_RX_STATUS = SIM_RX_START;
    HAL_UART_Transmit(UART_SIM, (unsigned char *)command, (uint16_t)strlen(command), 10);

#if FREERTOS == 1
        osDelay(CMD_DELAY_VERYSHORT);
#else
        HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
}


bool SIM_sendATCommandResponse(char* command, char* response)
{
    SIM_clearRX();
    SIM_RX_STATUS = SIM_RX_START;
    HAL_UART_Transmit(UART_SIM, (unsigned char *)command, (uint16_t)strlen(command), 100);

    uint32_t timeOut = 0;
    timeOut = HAL_GetTick();

    while ((SIM_RX_STATUS != SIM_RX_END) && ((HAL_GetTick() - timeOut) <= CMD_DELAY_LONG))
    {
#if FREERTOS == 1
        osDelay(CMD_DELAY_VERYSHORT);
#else
        HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
    }

    if (strstr(rx_buffer, response) != NULL)
    {
        SIM_clearRX();
        SIM_RX_STATUS = SIM_RX_START;
        return true;
    }
    SIM_clearRX();
    SIM_RX_STATUS = SIM_RX_START;
    return false;
}


bool SIM_startGPRS(void)
{
    bool status = true;
    char str[32] = {0};
    snprintf(str, sizeof(str), "AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n", SIM.apn, SIM.apn_user, SIM.apn_pass);

    status &= SIM_sendATCommandResponse("AT\r\n", "OK");
    status &= SIM_sendATCommandResponse("AT+CIPSHUT\r\n", "OK");
    status &= SIM_sendATCommandResponse("AT+CGATT=1\r\n", "OK");
    status &= SIM_sendATCommandResponse("AT+CIPMODE=0\r\n", "OK");
    status &= SIM_sendATCommandResponse(str, "OK");
    status &= SIM_sendATCommandResponse("AT+CIICR\r\n", "OK");
    status &= SIM_getIP();

    return status;
}


bool SIM_getIP(void)
{
    SIM_clearRX();
    SIM_RX_STATUS = SIM_RX_START;
    HAL_UART_Transmit(UART_SIM, (unsigned char *)"AT+CIFSR\r\n", (uint16_t) 10, 100);

    uint32_t timeOut = 0;
    timeOut = HAL_GetTick();

    while ((SIM_RX_STATUS != SIM_RX_END) && ((HAL_GetTick() - timeOut) <= CMD_DELAY_LONG));

    uint8_t dot = 0;
    uint8_t idx = 0;
    SIM_IP.Octec1 = 0;
    SIM_IP.Octec2 = 0;
    SIM_IP.Octec3 = 0;
    SIM_IP.Octec4 = 0;
    timeOut = HAL_GetTick();
    while ((rx_buffer[idx] != '\r') && ((HAL_GetTick() - timeOut) <= CMD_DELAY_LONG))
    {
        if(rx_buffer[idx] == '.')
        {
            dot++;
        }
        else if (dot == 0)
        {
            SIM_IP.Octec1 = SIM_IP.Octec1 * 10 + (rx_buffer[idx] - '0');
        }
        else if (dot == 1)
        {
            SIM_IP.Octec2 = SIM_IP.Octec2 * 10 + (rx_buffer[idx] - '0');
        }
        else if (dot == 2)
        {
            SIM_IP.Octec3 = SIM_IP.Octec3 * 10 + (rx_buffer[idx] - '0');
        }
        else if (dot == 3)
        {
            SIM_IP.Octec4 = SIM_IP.Octec4 * 10 + (rx_buffer[idx] - '0');
        }
        idx++;
    }

    if (dot == 3)
    {
        return true;
    }
    else
    {
        return false;
    }

}


uint8_t SIM_checkSignalStrength(void)
{
    SIM_sendATCommand("AT+CSQ\r\n");
    uint32_t timeOut = 0;
    timeOut = HAL_GetTick();
    while ((SIM_RX_STATUS != SIM_RX_END) && ((HAL_GetTick() - timeOut) <= CMD_DELAY_LONG))
    {
#if FREERTOS == 1
        osDelay(CMD_DELAY_VERYSHORT);
#else
        HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
    }
    sscanf(rx_buffer, "+CSQ: %[^,],", SIM.signalQuality);
    return SIM.signalQuality;
}



