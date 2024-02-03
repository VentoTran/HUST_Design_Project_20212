/**
 * @file topic.h
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief topic for MQTT
 * @version 1.0
 * @date 2022-08-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef _TOPIC_H_
#define _TOPIC_H_


//-------------------------------------------------- PUBLISH ----------------------------------------------
#define STATUS_TOPIC        "mandevices/running"
#define DATA_TOPIC          "mandevices/data"

//-------------------------------------------------- SUBCRIBE ---------------------------------------------
#define PING_TOPIC          "mandevices/pingtest"
#define TIME_TOPIC          "mandevices/time"



#endif  /*_TOPIC_H_*/