/**
 * @file mqtt.c
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-06-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "mqtt.h"


extern uint8_t rx_buffer[200];
extern SIM_RX_t SIM_RX_STATUS;

MQTT_t MQTT;

/**
 * @brief 
 * 
 * @param mqtt 
 * @param len 
 * @param response 
 * @return true 
 * @return false 
 */
bool MQTT_SendMQTT(char* mqtt, uint8_t len, char* response)
{
    SIM_clearRX();
    SIM_RX_STATUS = SIM_RX_START;
    HAL_UART_Transmit(UART_SIM, (unsigned char *)mqtt, (uint16_t)len, 100);

    osDelay(200);

    *UART_SIM.Instance->DR = 0x1A;

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
        return true;
    }
    SIM_clearRX();
    SIM_RX_STATUS = SIM_RX_START;
    return false;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool MQTT_Connect(void)
{
    MQTT.mqttReceive.newEvent = 0;
    MQTT.mqttServer.connect = 0;
    bool state = true;
    char str1[128] = {0};
    unsigned char buf[128] = {0};
    snprintf(str1, sizeof(str1), "AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n", MQTT.mqttServer.host, MQTT.mqttServer.port);
    state &= SIM_sendATCommandResponse(str1, "OK\r\n");

#if FREERTOS == 1
    osDelay(200);
#else
    HAL_Delay(200);
#endif
    
    if (state == true)
    {
        MQTTPacket_connectData datas = MQTTPacket_connectData_initializer;
        datas.username.cstring = MQTT.mqttClient.username;
        datas.password.cstring = MQTT.mqttClient.pass;
        datas.clientID.cstring = MQTT.mqttClient.clientID;
        datas.keepAliveInterval = MQTT.mqttClient.keepAliveInterval;
        datas.cleansession = 1;
        int mqtt_len = MQTTSerialize_connect(buf, sizeof(buf), &datas);
        SIM_sendATCommand("AT+CIPSEND\r\n");
        osDelay(200);
        if (strstr(rx_buffer, ">") != NULL)
        {
            state &= MQTT_SendMQTT((char*)buf, mqtt_len, "SEND OK");
            osDelay(100);
        }
        
#if FREERTOS == 1
        osDelay(200);
#else
        HAL_Delay(200);
#endif
        return state;
    }
    return state;
}

/**
 * @brief 
 * 
 * @param topic 
 * @param payload 
 * @return true 
 * @return false 
 */
bool MQTT_Pub(char *topic, char *payload)
{
    unsigned char buf[256] = {0};
    bool state = true;

    MQTTString topicString = MQTTString_initializer;
    topicString.cstring = topic;
    
    int mqtt_len = MQTTSerialize_publish(buf, sizeof(buf), 0, 0, 1, 0, topicString, (unsigned char *)payload, (int)strlen(payload));

    SIM_sendATCommand("AT+CIPSEND\r\n");
    osDelay(200);
    if (strstr(rx_buffer, ">") != NULL)
    {
        state &= MQTT_SendMQTT((char*)buf, mqtt_len, "SEND OK");
        osDelay(100);
        return state;
    }
    return state;
}


void MQTT_PubUint8(char *topic, uint8_t data);


void MQTT_PubUint16(char *topic, uint16_t data);


void MQTT_PubUint32(char *topic, uint32_t data);


void MQTT_PubFloat(char *topic, float payload, uint8_t digit);


void MQTT_PubDouble(char *topic, double data, uint8_t digit);


void MQTT_PingReq(void);


void MQTT_Sub(char *topic);


void MQTT_Receive(unsigned char *buf);


int MQTTstrlen(MQTTString mqttstring);


int MQTTSerialize_connect(unsigned char* buf, int buflen, MQTTPacket_connectData* options);


int MQTTSerialize_publish(unsigned char* buf, int buflen, unsigned char dup, int qos, unsigned char retained, unsigned short packetid, MQTTString topicName, unsigned char* payload, int payloadlen);






