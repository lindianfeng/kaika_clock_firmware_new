/*
 * dht11.h
 *
 *  Created on: Jan 9, 2021
 *      Author: lindianfeng
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "main.h"

#define DHT11_OUT_1       HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET)
#define DHT11_OUT_0       HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET)
#define DHT11_IN          HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)

uint8_t DHT11_Init(void);
uint8_t DHT11_ReadData(uint8_t *temp_int, uint8_t *temp_deci, uint8_t *humi_int);
uint8_t DHT11_ReadDataFloat(float *temp, float *humi);

#endif /* INC_DHT11_H_ */
