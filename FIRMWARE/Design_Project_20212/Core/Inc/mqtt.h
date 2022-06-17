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

enum connack_return_codes
{
    MQTT_CONNECTION_ACCEPTED = 0,
    MQTT_UNNACCEPTABLE_PROTOCOL = 1,
    MQTT_CLIENTID_REJECTED = 2,
    MQTT_SERVER_UNAVAILABLE = 3,
    MQTT_BAD_USERNAME_OR_PASSWORD = 4,
    MQTT_NOT_AUTHORIZED = 5,
};

typedef union
{
	unsigned char all;	/**< all connect flags */
	struct
	{
		unsigned int : 1;	     			/**< unused */
		unsigned int cleansession : 1;	    /**< cleansession flag */
		unsigned int will : 1;			    /**< will flag */
		unsigned int willQoS : 2;			/**< will QoS value */
		unsigned int willRetain : 1;		/**< will retain setting */
		unsigned int password : 1; 			/**< 3.1 password */
		unsigned int username : 1;			/**< 3.1 user name */
	} bits;
} MQTTConnectFlags;	/**< connect flags byte */

typedef struct
{
	/** The eyecatcher for this structure.  must be MQTW. */
	char struct_id[4];
	/** The version number of this structure.  Must be 0 */
	int struct_version;
	/** The LWT topic to which the LWT message will be published. */
	MQTTString topicName;
	/** The LWT payload. */
	MQTTString message;
	/**
      * The retained flag for the LWT message (see MQTTAsync_message.retained).
      */
	unsigned char retained;
	/**
      * The quality of service setting for the LWT message (see
      * MQTTAsync_message.qos and @ref qos).
      */
	char qos;
} MQTTPacket_willOptions;

enum errors
{
	MQTTPACKET_BUFFER_TOO_SHORT = -2,
	MQTTPACKET_READ_ERROR = -1,
	MQTTPACKET_READ_COMPLETE
};

enum msgTypes
{
	CONNECT = 1, CONNACK, PUBLISH, PUBACK, PUBREC, PUBREL,
	PUBCOMP, SUBSCRIBE, SUBACK, UNSUBSCRIBE, UNSUBACK,
	PINGREQ, PINGRESP, DISCONNECT
};

/**
 * Bitfields for the MQTT header byte.
 */
typedef union
{
	unsigned char byte;	                /**< the whole byte */
	struct
	{
		unsigned int retain : 1;		/**< retained flag bit */
		unsigned int qos : 2;				/**< QoS value, 0, 1 or 2 */
		unsigned int dup : 1;				/**< DUP flag bit */
		unsigned int type : 4;			/**< message type nibble */
	} bits;
} MQTTHeader;

typedef struct
{
	int len;
	char* data;
} MQTTLenString;

typedef struct
{
	char* cstring;
	MQTTLenString lenstring;
} MQTTString;

typedef struct
{
	/** The eyecatcher for this structure.  must be MQTC. */
	char struct_id[4];
	/** The version number of this structure.  Must be 0 */
	int struct_version;
	/** Version of MQTT to be used.  3 = 3.1 4 = 3.1.1
	  */
	unsigned char MQTTVersion;
	MQTTString clientID;
	unsigned short keepAliveInterval;
	unsigned char cleansession;
	unsigned char willFlag;
	MQTTPacket_willOptions will;
	MQTTString username;
	MQTTString password;
} MQTTPacket_connectData;

typedef union
{
	unsigned char all;	/**< all connack flags */
	struct
	{
		unsigned int sessionpresent : 1;    /**< session present flag */
    unsigned int reserved: 7;	     			/**< unused */
	} bits;
} MQTTConnackFlags;	/**< connack flags byte */

#define MQTTPacket_willOptions_initializer  { {'M', 'Q', 'T', 'W'}, 0, {NULL, {0, NULL}}, {NULL, {0, NULL}}, 0, 0 }
#define MQTTString_initializer              {NULL, {0, NULL}}
#define MQTTPacket_connectData_initializer  { {'M', 'Q', 'T', 'C'}, 0, 4, {NULL, {0, NULL}}, 60, 1, 0, MQTTPacket_willOptions_initializer, {NULL, {0, NULL}}, {NULL, {0, NULL}} }

//------------------------------------------------------------------------------------------------------------


bool MQTT_SendMQTT(char* mqtt, uint8_t len, char* response);

bool MQTT_Connect(void);

bool MQTT_Pub(char *topic, char *payload);

void MQTT_PubUint8(char *topic, uint8_t data);

void MQTT_PubUint16(char *topic, uint16_t data);

void MQTT_PubUint32(char *topic, uint32_t data);

void MQTT_PubFloat(char *topic, float payload, uint8_t digit);

void MQTT_PubDouble(char *topic, double data, uint8_t digit);

void MQTT_PingReq(void);

void MQTT_Sub(char *topic);

void MQTT_Receive(unsigned char *buf);

//-------------------------------------------------

int MQTTstrlen(MQTTString mqttstring);

int MQTTSerialize_connect(unsigned char* buf, int buflen, MQTTPacket_connectData* options);

int MQTTSerialize_publish(unsigned char* buf, int buflen, unsigned char dup, int qos, unsigned char retained, unsigned short packetid, MQTTString topicName, unsigned char* payload, int payloadlen);


#endif /*_MQTT_H_*/