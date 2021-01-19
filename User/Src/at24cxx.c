/*
 * at24cxx.c
 *
 *  Created on: 2021年1月19日
 *      Author: linhao
 */

#include "at24cxx.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

void AT24CXX_Write(uint16_t reg_address, uint8_t data)
{
  HAL_I2C_Mem_Write(&hi2c2, AT24CXX_DEV_ADDRESS, reg_address, I2C_MEMADD_SIZE_16BIT, &data, 1, 100);
  AT24CXX_DELAY_MS(5);    //AT24C64的最长写入时间是5ms
}

uint8_t AT24CXX_Read(uint16_t reg_address)
{
  uint8_t data = 0;
  HAL_I2C_Mem_Read(&hi2c2, AT24CXX_DEV_ADDRESS, reg_address, I2C_MEMADD_SIZE_16BIT, &data, 2, 100);
  return data;

}



