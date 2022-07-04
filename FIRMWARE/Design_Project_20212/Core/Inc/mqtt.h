/**
 * @file mqtt.h
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-06-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _MQTT_H_
#define _MQTT_H_

#include "main.h"
#include "sim.h"


typedef struct
{
    char *host;
    uint16_t port;
    uint8_t connect;
} mqttServer_t;

typedef struct
{
    char *username;
    char *pass;
    char *clientID;
    unsigned short keepAliveInterval;
} mqttClient_t;

typedef struct
{
    uint8_t newEvent;
    unsigned char dup;
    int qos;
    unsigned char retained;
    unsigned short msgId;
    unsigned char payload[64];
    int payloadLen;
    unsigned char topic[64];
    int topicLen;
} mqttReceive_t;

typedef struct 
{
    mqttServer_t mqttServer;
    mqttClient_t mqttClient;
    mqttReceive_t mqttReceive;
} MQTT_t;

typedef enum
{
    MQTT_OK,
    MQTT_NOK
} MQTT_ST_T;

typedef struct
{
    SIM_ST_t SIM_ST;
    MQTT_ST_T MQTT_ST;
} MQTT_ST_t;

//------------------------------------------------------------------------------------------------------------


bool MQTT_SendMQTT(char* mqtt, uint8_t len, char* response);

bool MQTT_Connect(void);

bool MQTT_Pub(char *topic, char *payload);

bool MQTT_PubUint8(char *topic, uint8_t payload);

void MQTT_PubUint16(char *topic, uint16_t payload);

void MQTT_PubUint32(char *topic, uint32_t payload);

bool MQTT_PubFloat(char *topic, float payload, uint8_t digit);

bool MQTT_PubDouble(char *topic, double payload, uint8_t digit);

bool MQTT_PingReq(void);

bool MQTT_Sub(char *topic);

void MQTT_Receive(unsigned char *buf);

//-------------------------------------------------


#endif /*_MQTT_H_*/