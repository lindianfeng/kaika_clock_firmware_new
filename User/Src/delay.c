/*
 * delay.c
 *
 *  Created on: Jan 11, 2021
 *      Author: linhao
 */

#include "delay.h"
#include "stm32f1xx_hal.h"

//void delay_us(uint32_t us) {
//	uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
//	while (delay--) {
//		;
//	}
//}

uint32_t DWT_Init(void)
{
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

  /* Enable  clock cycle counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  for (int i = 0; i < 5; ++i)
  {
    __NOP();
  }

  /* Check if clock cycle counter has started */
  return (DWT->CYCCNT) ? 0 : 1;
}

void DWT_DelayUs(volatile uint32_t us)
{
  uint32_t startTick = DWT->CYCCNT;
  uint32_t delayTicks = us * (SystemCoreClock / 1000000);

  while (DWT->CYCCNT - startTick < delayTicks)
    ;
}

void DWT_DelayMs(uint32_t ms)
{
  DWT_DelayUs(1000 * ms);
}

