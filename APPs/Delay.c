#include "Delay.h"

//计数器每计数一次的时间为1/SYSCLK
//PPL_M=8->SYSCLK=168M
//T=1/168M
//SysTick_CLK=168/8=21M
//SysTick->LOAD [23:0]
//SysTick->VAL [23:0]
int Delay_ms(u32 a)
{	
	while(SystemCoreClock/1000 > SysTick_LOAD_RELOAD_Msk);
	
	SysTick->LOAD=SystemCoreClock/1000;
	SysTick->VAL =0;
	SysTick->CTRL|= SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk;
	
	for(int i=0;i<a;i++)
	{
		while ( !((SysTick->CTRL)&(1<<16)) );
	}
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
	
	return 0;
}

int Delay_us(u32 a)
{	
	while(SystemCoreClock/1000000 > SysTick_LOAD_RELOAD_Msk);
	
	SysTick->LOAD=SystemCoreClock/1000000;
	SysTick->VAL =0;
	SysTick->CTRL|= SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk;

	for(int i=0;i<a;i++)
	{
		while ( !((SysTick->CTRL)&(1<<16)) );
	}
	
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
	
	return 0;

}

