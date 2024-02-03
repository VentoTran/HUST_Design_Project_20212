/**
 * @file ds1307.h
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-06-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef __DS1307_H__
#define __DS1307_H__


#include "stm32f1xx_hal.h"
#include <string.h>

#define DS1307_I2C_ADDRESS      (0x68 << 1)

/* Register addresses */
#define DS1307_ADDR_SEC 		0x00
#define DS1307_ADDR_MIN 		0x01
#define DS1307_ADDR_HRS			0x02
#define DS1307_ADDR_DAY			0x03
#define DS1307_ADDR_DATE		0x04
#define DS1307_ADDR_MONTH		0x05
#define DS1307_ADDR_YEAR		0x06

// #define TIME_FORMAT_12HRS_AM 	0
// #define TIME_FORMAT_12HRS_PM 	1
#define TIME_FORMAT_24HRS 		2

#define SUNDAY  	1
#define MONDAY  	2
#define TUESDAY  	3
#define WEDNESDAY   4
#define THURSDAY  	5
#define FRIDAY  	6
#define SATURDAY  	7

typedef struct
{
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day;
}RTC_date_t;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
}RTC_time_t;

typedef struct
{
    RTC_date_t Date;
    RTC_time_t Time;
}   RTC_t;

void ds1307_set_current_time(RTC_time_t *rtc_time);
void ds1307_get_current_time(RTC_time_t *rtc_time);

void ds1307_set_current_date(RTC_date_t *rtc_date);
void ds1307_get_current_date(RTC_date_t *rtc_date);

char *get_day_of_week(uint8_t count);
void number_to_string(uint8_t num, char *buf);
char *time_to_string(RTC_time_t *rtc_time);
char *date_to_string(RTC_date_t *rtc_date);

#endif


