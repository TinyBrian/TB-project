#include "LED.h"
    
//LED d2 ->PA6  on = RESET
//    d3 ->PA7	on = RESET

void LED_Init(void)
{
 
	GPIO_InitTypeDef GPIO_Initstructre;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	

	GPIO_Initstructre.GPIO_Mode=GPIO_Mode_OUT;									    ///LED PA6 D2
	GPIO_Initstructre.GPIO_OType=GPIO_OType_PP;        			 			
	GPIO_Initstructre.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Initstructre.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7; 
		GPIO_Init(GPIOA,&GPIO_Initstructre); 
						    
	GPIO_ResetBits(GPIOA,GPIO_Pin_6|GPIO_Pin_7);
	Delay_ms(500);
	GPIO_SetBits(GPIOA,GPIO_Pin_6|GPIO_Pin_7);
	Delay_ms(500);

}
 
