/*
 * dht11.c
 *
 *  Created on: Jan 9, 2021
 *      Author: lindianfeng
 */

#include "dht11.h"
#include "delay.h"

/**
 * @brief DHT11 输出模式
 */
static inline void DHT11_Mode_OUT_PP(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = DHT11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief DHT11 输入模式
 */
static inline void DHT11_Mode_IN_NP(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = DHT11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}
//复位DHT11
static inline void DHT11_Rst(void) {
	DHT11_Mode_IN_NP();
	DWT_DelayMs(1);
	DHT11_Mode_OUT_PP();   //SET OUTPUT
	DHT11_OUT_0;  //拉低DQ
	DWT_DelayMs(20); //拉低至少18ms
	DHT11_OUT_1;  //DQ=1
	DWT_DelayUs(30);       //主机拉高20~40us
}

//等待DHT11的回应
//返回1:未检测到DHT11的存在
//返回0:存在
static inline uint8_t DHT11_Check(void) {
	uint8_t retry = 0;
	DHT11_Mode_IN_NP();
	while (DHT11_IN && retry < 100) {
		//DHT11会拉低40~80us
		retry++;
		DWT_DelayUs(1);
	}

	if (retry >= 100) {
		return 1;
	}

	retry = 0;

	while (!DHT11_IN && retry < 100) {
		//DHT11拉低后会再次拉高40~80us
		retry++;
		DWT_DelayUs(1);
	}

	if (retry >= 100) {
		return 1;
	}

	return 0;
}

//从DHT11读取一个位
//返回值：1/0
static inline uint8_t DHT11_ReadBit(void) {
	uint8_t retry = 0;

	while (DHT11_IN && retry < 100) {
		//等待变高电平
		retry++;
		DWT_DelayUs(1);
	}

	retry = 0;

	while (!DHT11_IN && retry < 100) {
		//等待变高电平
		retry++;
		DWT_DelayUs(1);
	}

	DWT_DelayUs(40);       //等待40us

	return DHT11_IN == GPIO_PIN_RESET ? 0 : 1;
}

//从DHT11读取一个字节
//返回值：读到的数据
static inline uint8_t DHT11_ReadByte(void) {
	uint8_t dat = 0;
	for (int i = 0; i < 8; i++) {
		dat <<= 1;
		dat |= DHT11_ReadBit();
	}
	return dat;
}

//从DHT11读取一次数据
//temp:温度值(范围:0~50°)
//humi:湿度值(范围:20%~90%)
//返回值：0,正常;1,读取失败
uint8_t DHT11_ReadData(uint8_t *temp_int, uint8_t *temp_deci, uint8_t *humi_int) {
	uint8_t buf[5] = { 0, 0, 0, 0, 0 };

	DHT11_Rst();

	if (DHT11_Check()) {
		return 1;
	}

	for (int i = 0; i < 5; i++) {
		//读取40位数据
		buf[i] = DHT11_ReadByte();
	}

	if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
		*humi_int = buf[0];
		*temp_int = buf[2];
		*temp_deci = buf[3];
	}

	return 0;
}

uint8_t DHT11_ReadDataFloat(float *temp, float *humi) {
	uint8_t buf[5] = { 0, 0, 0, 0, 0 };

	DHT11_Rst();

	if (DHT11_Check()) {
		return 1;
	}

	for (int i = 0; i < 5; i++) //读取40位数据
			{
		buf[i] = DHT11_ReadByte();
	}

	if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
			{
		*humi = ((buf[0] << 8) + buf[1]) / 10.0;
		*temp = ((buf[2] << 8) + buf[3]) / 10.0;
	}

	return 0;
}

uint8_t DHT11_Init(void) {
	DHT11_Mode_OUT_PP();
	DHT11_Rst();
	return DHT11_Check();
}
