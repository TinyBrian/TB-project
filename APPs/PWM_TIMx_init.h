#ifndef __PWM_TIMx_INIT_H
#define __PWM_TIMx_INIT_H


#include "Delay.h"
#include "ADC_init.h"


#define STM32F407 0
#define STM32F103 1



void PWM_Timer_init(u16 pre , u16 psc);
void PWM_Generate_4(u16 PWM_Motor_1,u16 PWM_Motor_2,u16 PWM_Motor_3,u16 PWM_Motor_4) ;//(1ms->2ms)20ms




#endif

