#include "EOS.h"

// Init EOS x ms. Value x maximum: 2097
void EOS_Init(uint16_t x)
{
	SysTick->LOAD = 72000000/1000*x;
	SysTick->VAL = 0;
	SysTick->CTRL = 7;
}
