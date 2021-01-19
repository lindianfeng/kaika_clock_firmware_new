/*
 * at24cxx.h
 *
 *  Created on: 2021年1月19日
 *      Author: linhao
 */

#ifndef INC_AT24CXX_H_
#define INC_AT24CXX_H_

#include <stdint.h>

#define AT24CXX_DEV_ADDRESS     0xA0
#define AT24CXX_DELAY_MS        osDelay

void AT24CXX_Write(uint16_t reg_address, uint8_t data);
uint8_t AT24CXX_Read(uint16_t);

#endif /* INC_AT24CXX_H_ */
