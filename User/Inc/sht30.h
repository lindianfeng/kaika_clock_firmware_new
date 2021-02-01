/*
 * sht30.h
 *
 *  Created on: 2021年2月1日
 *      Author: linhao
 */

#ifndef INC_SHT30_H_
#define INC_SHT30_H_

#include <stdbool.h>
#include <stdint.h>

#define SHT_ADDR (0x44)

uint8_t sht30_init();

uint8_t sht30_sample(float *t, float *h);

#endif /* INC_SHT30_H_ */
