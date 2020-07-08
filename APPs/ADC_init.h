#ifndef __ADC_init_H
#define __ADC_init_H

#include "stm32f4xx.h"
#include "Delay.h"
/*
DAC&ADC
Channel 0   --------  PA0
Channel 1   --------  PA1
Channel 2   --------  PA2
Channel 3   --------  PA3
Channel 4   --------  PA4
Channel 5   --------  PA5
Channel 6   --------  PA6
Channel 7   --------  PA7
Channel 8   --------  PB0
Channel 9   --------  PB1
Channel 10  --------  PC0 
Channel 11  --------  PC1 
Channel 12  --------  PC2 
Channel 13  --------  PC3 
Channel 14  --------  PC4 
Channel 15  --------  PC5 
Channel 16  --------  内部温湿度
Channel 17  --------  内部Vrefint
*/




void ADC_init(void );
int ADC_return(void);
int ADC_filter_return_lcd(void);
u16 ADC_filter_return(void);
float ADC_filter_return_battery(void);

#endif

