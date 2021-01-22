/*
 * key.h
 *
 *  Created on: Dec 10, 2020
 *      Author: linhao
 */

#ifndef KEY_H_
#define KEY_H_

#include <stdint.h>
#include "stm32f1xx.h"


#define KEY_ON  0
#define KEY_OFF 1

uint8_t Key_Scan(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif /* KEY_H_ */
