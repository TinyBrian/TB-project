#include "MSysTick.h" 

volatile int  j=0;
volatile uint32_t g_ul_ms_ticks=0;
static volatile uint32_t TimingDelay=0;

void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
//	if (SysTick_Config(SystemFrequency / 100000))	// ST3.0.0库版本
	
/*********************************NVIC_CONFIG***********************************************/
	if (SysTick_Config(SystemCoreClock/1000))	//系统定时器此时设置的优先级在内核外设中是最低的
	{ 
		while (1);
	}
	
	// off 
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
	// on
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |SysTick_CTRL_TICKINT_Msk |SysTick_CTRL_ENABLE_Msk;
	
}


int get_tick_count(unsigned long *count)
{
    count[0] = g_ul_ms_ticks;
	return 0;
}

void TimeStamp_Increment(void)
{

	g_ul_ms_ticks++;
}

