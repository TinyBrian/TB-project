#include "MSysTick.h" 

volatile int  j=0;
volatile uint32_t g_ul_ms_ticks=0;
static volatile uint32_t TimingDelay=0;

void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	 * SystemFrequency / 1000000 1us�ж�һ��
	 */
//	if (SysTick_Config(SystemFrequency / 100000))	// ST3.0.0��汾
	
/*********************************NVIC_CONFIG***********************************************/
	if (SysTick_Config(SystemCoreClock/1000))	//ϵͳ��ʱ����ʱ���õ����ȼ����ں�����������͵�
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

