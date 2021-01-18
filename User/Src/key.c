/*
 * key.c
 *
 *  Created on: Jan 18, 2021
 *      Author: linhao
 */

#include "key.h"

uint8_t Key_Scan(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == KEY_ON)
  {
    /*等待按键释放 */
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == KEY_ON)
    ;
    return KEY_ON;
  }
  else
  {
    return KEY_OFF;
  }
}
