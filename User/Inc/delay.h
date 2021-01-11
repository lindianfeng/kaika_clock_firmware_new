/*
 * delay.h
 *
 *  Created on: Dec 10, 2020
 *      Author: linhao
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f1xx.h"
#include <stdbool.h>
#include <stdint.h>

uint32_t DWT_Init(void);
void DWT_DelayUs(uint32_t us);
void DWT_DelayMs(uint32_t ms);

static inline uint32_t DWT_MaxSec(void) {
	return (UINT32_MAX / SystemCoreClock);
}

static inline uint32_t DWT_MaxMsec(void) {
	return (UINT32_MAX / (SystemCoreClock / 1000));
}

static inline uint32_t DWT_MaxUsec(void) {
	return (UINT32_MAX / (SystemCoreClock / 1000000));
}

static inline uint32_t DWT_GetCycles(void) {
	return (DWT->CYCCNT);
}

#endif /* DELAY_H_ */
