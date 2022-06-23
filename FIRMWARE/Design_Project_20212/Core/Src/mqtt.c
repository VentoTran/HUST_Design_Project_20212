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
#include "string.h"
#include "MQTTPacket.h"

extern uint8_t rx_buffer[200];
extern SIM_RX_t SIM_RX_STATUS;

#if FREERTOS == 1
#include "cmsis_os2.h"
#endif

MQTT_t MQTT;

//-------------------------------------------------------------------------------------------------------------------------------------

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
	char temp[1]= {26};

    HAL_UART_Transmit_IT(UART_SIM, (unsigned char *)mqtt, (uint16_t)len);

    osDelay(CMD_DELAY_VERYSHORT);

    HAL_UART_Transmit_IT(UART_SIM, (unsigned char *)temp, (uint16_t)(1));

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
    osDelay(CMD_DELAY_VERYSHORT);
#else
    HAL_Delay(CMD_DELAY_VERYSHORT);
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

#if FREERTOS == 1
        osDelay(CMD_DELAY_MEDIUM);
#else
        HAL_Delay(CMD_DELAY_MEDIUM);
#endif

        // HAL_UART_Transmit_IT(UART_SIM, (unsigned char *)"AT+CIPSEND\r\n", (uint16_t)strlen("AT+CIPSEND\r\n"));
        SIM_sendATCommand("AT+CIPSEND\r\n");

#if FREERTOS == 1
        osDelay(CMD_DELAY_VERYSHORT);
#else
        HAL_Delay(CMD_DELAY_VERYSHORT);
#endif

        if (strstr(rx_buffer, ">") != NULL)
        {
            state &= MQTT_SendMQTT((char*)buf, mqtt_len, "SEND OK");
			return state;
        }
#if FREERTOS == 1
        osDelay(CMD_DELAY_VERYSHORT);
#else
        HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
        SIM_clearRX();
        return false;
    }
    return false;
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
    
    int mqtt_len = MQTTSerialize_publish(buf, sizeof(buf), 0, 0, 0, 0, topicString, (unsigned char *)payload, (int)strlen(payload));

    SIM_sendATCommand("AT+CIPSEND\r\n");
#if FREERTOS == 1
    osDelay(CMD_DELAY_VERYSHORT);
#else
    HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
    if (strstr(rx_buffer, ">") != NULL)
    {
		osDelay(CMD_DELAY_VERYSHORT);
        state &= MQTT_SendMQTT((char*)buf, mqtt_len, "SEND OK");
        // osDelay(CMD_DELAY_VERYSHORT);
        SIM_clearRX();
        return state;
    }
    SIM_clearRX();
    return false;
}

/**
 * @brief 
 * 
 * @param topic 
 * @param payload 
 */
bool MQTT_PubUint8(char *topic, uint8_t payload)
{
    char str[32] = {0};
    sprintf(str, "%u", payload);
    return MQTT_Pub(topic, str);
}


void MQTT_PubUint16(char *topic, uint16_t payload);


void MQTT_PubUint32(char *topic, uint32_t payload);

/**
 * @brief 
 * 
 * @param topic 
 * @param payload 
 * @param digit 
 */
bool MQTT_PubFloat(char *topic, float payload, uint8_t digit)
{
    char str[32] = {0};
    ftoa(payload, str, digit);
    return MQTT_Pub(topic, str);
}

/**
 * @brief 
 * 
 * @param topic 
 * @param payload 
 * @param digit 
 */
bool MQTT_PubDouble(char *topic, double payload, uint8_t digit)
{
    char str[32] = {0};
    ftoa(payload, str, digit);
    return MQTT_Pub(topic, str);
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool MQTT_PingReq(void)
{
    unsigned char buf[16] = {0};
    bool state = true;

    int mqtt_len = MQTTSerialize_zero(buf, sizeof(buf), PINGREQ);
    SIM_sendATCommand("AT+CIPSEND\r\n");
#if FREERTOS == 1
    osDelay(CMD_DELAY_VERYSHORT);
#else
    HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
    if (strstr(rx_buffer, ">") != NULL)
    {
        state &= MQTT_SendMQTT((char*)buf, mqtt_len, "SEND OK");
        osDelay(100);
        SIM_clearRX();
        return state;
    }
    SIM_clearRX();
    return false;
}

/**
 * @brief 
 * 
 * @param topic 
 * @return true 
 * @return false 
 */
bool MQTT_Sub(char *topic)
{
    unsigned char buf[256] = {0};
    bool state = true;

    MQTTString topicString = MQTTString_initializer;
    topicString.cstring = topic;

    int mqtt_len = MQTTSerialize_subscribe(buf, sizeof(buf), 0, 1, 1, &topicString, 0);
    SIM_sendATCommand("AT+CIPSEND\r\n");
#if FREERTOS == 1
    osDelay(CMD_DELAY_VERYSHORT);
#else
    HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
    if (strstr(rx_buffer, ">") != NULL)
    {
        state &= MQTT_SendMQTT((char*)buf, mqtt_len, "SEND OK");
        osDelay(100);
        SIM_clearRX();
        return state;
    }
    SIM_clearRX();
	return false;
}

/**
 * @brief 
 * 
 * @param buf 
 */
void MQTT_Receive(unsigned char *buf)
{
    memset(MQTT.mqttReceive.topic, 0, sizeof(MQTT.mqttReceive.topic));
    memset(MQTT.mqttReceive.payload, 0, sizeof(MQTT.mqttReceive.payload));
    MQTTString receivedTopic;
    unsigned char *payload;
    MQTTDeserialize_publish(&MQTT.mqttReceive.dup, &MQTT.mqttReceive.qos, &MQTT.mqttReceive.retained,
                            &MQTT.mqttReceive.msgId,
                            &receivedTopic, &payload, &MQTT.mqttReceive.payloadLen, buf,
                            sizeof(buf));
    memcpy(MQTT.mqttReceive.topic, receivedTopic.lenstring.data, receivedTopic.lenstring.len);
    MQTT.mqttReceive.topicLen = receivedTopic.lenstring.len;
    memcpy(MQTT.mqttReceive.payload, payload, MQTT.mqttReceive.payloadLen);
    MQTT.mqttReceive.newEvent = 1;
}













