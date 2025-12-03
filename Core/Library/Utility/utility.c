/*
 * utility.c
 *
 *  Created on: Jul 23, 2025
 *      Author: kaana
 */



#include "utiltity.h"

void TIM_Delay(uint32_t ms)
{
	TIM6->CR1 |= TIM_CR1_CEN;
	TIM6->CNT = 0;
	while((TIM6->CNT) < ms);
}
