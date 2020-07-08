#ifndef __MSYSTICK_H
#define __MSYSTICK_H
#include "stm32f4xx.h"


void SysTick_Init(void);

int get_tick_count(unsigned long *count);

void TimeStamp_Increment(void);

#endif //__SYSTICK_H

