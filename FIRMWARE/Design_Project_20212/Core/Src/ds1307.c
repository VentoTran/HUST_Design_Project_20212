/**
 * @file ds1307.c
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-06-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ds1307.h"


extern I2C_HandleTypeDef hi2c1;

/**
 * @brief 
 * 
 * @param value 
 * @return uint8_t 
 */
static uint8_t bcd_to_binary(uint8_t value)
{
	uint8_t m, n;
	m = (uint8_t)((value >> 4) * 10);
	n = value & (uint8_t)0x0F;

	return (m + n);
}

/**
 * @brief 
 * 
 * @param value 
 * @return uint8_t 
 */
static uint8_t binary_to_bcd(uint8_t value)
{
	uint8_t m, n;
	uint8_t bcd;

	bcd = value;
	if(value >= 10)
	{
		m = value / 10;
		n = value % 10;
		bcd = (m << 4) | n ;
	}
	return bcd;
}

/**
 * @brief 
 * 
 * @param ds1307_addr 
 * @return uint8_t 
 */
static uint8_t ds1307_read(uint8_t ds1307_addr)
{
	uint8_t data;
    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DS1307_I2C_ADDRESS, &ds1307_addr, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DS1307_I2C_ADDRESS, &data, 1, 100);
    return data;
}

/**
 * @brief 
 * 
 * @param value 
 * @param ds1307_addr 
 */
static void ds1307_write(uint8_t value, uint8_t ds1307_addr)
{
	uint8_t tx[2];
	tx[0] = ds1307_addr;
	tx[1] = value;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DS1307_I2C_ADDRESS, tx, 2, 100);
}

/**
 * @brief 
 * 
 * @param rtc_date 
 */
void ds1307_get_current_date(RTC_date_t *rtc_date)
{
	rtc_date->day = bcd_to_binary(ds1307_read((uint8_t)DS1307_ADDR_DAY));
	rtc_date->date = bcd_to_binary(ds1307_read((uint8_t)DS1307_ADDR_DATE));
	rtc_date->month = bcd_to_binary(ds1307_read((uint8_t)DS1307_ADDR_MONTH));
	rtc_date->year = bcd_to_binary(ds1307_read((uint8_t)DS1307_ADDR_YEAR));
}

/**
 * @brief 
 * 
 * @param rtc_time 
 */
void ds1307_get_current_time(RTC_time_t *rtc_time)
{
	uint8_t seconds, hrs;
	seconds = ds1307_read(DS1307_ADDR_SEC);
	seconds &= ~(1 << 7);

	rtc_time->seconds = bcd_to_binary(seconds);

	rtc_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));
	hrs = ds1307_read(DS1307_ADDR_HRS);
	
	if(hrs & (1 << 6))
	{
		//12-hour format
		rtc_time->time_format = !((hrs & (1 << 5)) == 0);
		hrs &= ~(0x3 << 5); // Clear 6 and 5
	}
	else
	{
		//24-hour format
		rtc_time->time_format = TIME_FORMAT_24HRS;
	}
	rtc_time->hours = bcd_to_binary(hrs);
}

/**
 * @brief 
 * 
 * @param rtc_date 
 */
void ds1307_set_current_date(RTC_date_t *rtc_date)
{
	ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);
	ds1307_write(binary_to_bcd(rtc_date->month) ,DS1307_ADDR_MONTH);
	ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);
	ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);

}

/**
 * @brief 
 * 
 * @param rtc_time 
 */
void ds1307_set_current_time(RTC_time_t *rtc_time)
{
	uint8_t seconds, hrs;
	seconds = binary_to_bcd(rtc_time->seconds);
	seconds &= ~(1 << 7);
	ds1307_write(seconds, DS1307_ADDR_SEC);

	ds1307_write(binary_to_bcd(rtc_time->minutes), DS1307_ADDR_MIN);

	hrs = binary_to_bcd(rtc_time->hours);

	if(rtc_time->time_format == TIME_FORMAT_24HRS){
		hrs &= ~(1 << 6);
	}
	// else
	// {
	// 	hrs |= (1 << 6);
	// 	hrs = (rtc_time->time_format == TIME_FORMAT_12HRS_PM) ? hrs | (1 << 5) :  hrs & ~( 1 << 5);
	// }

	ds1307_write(hrs, DS1307_ADDR_HRS);
}

/**
 * @brief Get the day of week object
 * 
 * @param count 
 * @return char* 
 */
char *get_day_of_week(uint8_t count)
{
	char* days[] = { "Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
	return days[count - 1];
}

/**
 * @brief 
 * 
 * @param num 
 * @param buf 
 */
void number_to_string(uint8_t num, char *buf)
{

	if(num < 10)
	{
		buf[0] = '0';
		buf[1] = num + 48;
	}
	else if(num >= 10 && num < 99)
	{
		buf[0] = (num / 10) + 48;
		buf[1] = (num % 10) + 48;
	}
}

/**
 * @brief 
 * 	hh:mm:ss
 * @param rtc_time 
 * @return char* 
 */
char *time_to_string(RTC_time_t *rtc_time)
{
	static char buf[9];

	buf[2]= ':';
	buf[5]= ':';

	number_to_string(rtc_time->hours, buf);
	number_to_string(rtc_time->minutes, &buf[3]);
	number_to_string(rtc_time->seconds, &buf[6]);

	buf[8] = '\0';

	return buf;
}

/**
 * @brief 
 * 	dd/mm/yy
 * @param rtc_date 
 * @return char* 
 */
char *date_to_string(RTC_date_t *rtc_date)
{
	static char buf[9];

	buf[2]= '/';
	buf[5]= '/';

	number_to_string(rtc_date->date, buf);
	number_to_string(rtc_date->month, &buf[3]);
	number_to_string(rtc_date->year, &buf[6]);

	buf[8]= '\0';

	return buf;
}