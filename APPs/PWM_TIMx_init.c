#include "PWM_TIMx_init.h"
//Tout=1/(prescaler+1) * (period+1)

void PWM_Timer_init(u16 per , u16 psc)  //located in "struct for Timer"
{		TIM_DeInit(TIM1);
		GPIO_InitTypeDef GPIO_Initstruct;
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitstruct;
		TIM_OCInitTypeDef  TIM_OCInitstruct;

//TIM1->APB2_TIMER =168M	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

//RCC enable+GPIOxAF enable
//TIMx_CH1->PA8\PE9	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);
	
		
//GPIO init---PE9--TIM_CH1
		GPIO_Initstruct.GPIO_Mode=GPIO_Mode_AF;
		GPIO_Initstruct.GPIO_OType=GPIO_OType_PP;
		GPIO_Initstruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 
		GPIO_Initstruct.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Initstruct.GPIO_Pin=GPIO_Pin_9;
			GPIO_Init(GPIOE,&GPIO_Initstruct);

//GPIO init---PE11--TIM_CH2		

		GPIO_Initstruct.GPIO_Pin=GPIO_Pin_11;
			GPIO_Init(GPIOE,&GPIO_Initstruct);

//GPIO init---PE13--TIM_CH3	

		GPIO_Initstruct.GPIO_Pin=GPIO_Pin_13;
			GPIO_Init(GPIOE,&GPIO_Initstruct);

//GPIO init---PE14--TIM_CH4		

		GPIO_Initstruct.GPIO_Pin=GPIO_Pin_14;
			GPIO_Init(GPIOE,&GPIO_Initstruct);
	

	//CLK=100kHz 
//struct for Timer(period->ARR,)
		TIM_TimeBaseInitstruct.TIM_Prescaler=psc;	//TIMx->PSC=Prescaler;
		TIM_TimeBaseInitstruct.TIM_Period=per;		//TIMx->ARR=Period ;
		TIM_TimeBaseInitstruct.TIM_CounterMode=TIM_CounterMode_Up;
		TIM_TimeBaseInitstruct.TIM_ClockDivision=TIM_CKD_DIV1;
		TIM_TimeBaseInitstruct.TIM_RepetitionCounter=0;	
			TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitstruct);	
			
		
			
//init for oc1-ch1	
		TIM_OCInitstruct.TIM_OCMode=TIM_OCMode_PWM1;	// 模式1先高电平达到比较值改变电平
		TIM_OCInitstruct.TIM_OCPolarity=TIM_OCPolarity_High;
		TIM_OCInitstruct.TIM_OCNPolarity=TIM_OCNPolarity_High;
		TIM_OCInitstruct.TIM_OCIdleState=TIM_OCIdleState_Reset;// 主输出在被禁止时为LOw电平
		TIM_OCInitstruct.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
		TIM_OCInitstruct.TIM_OutputState=TIM_OutputState_Enable;
		TIM_OCInitstruct.TIM_OutputNState=TIM_OutputNState_Disable;
		TIM_OCInitstruct.TIM_Pulse=100;	//初始占空比为1ms
			TIM_OC1Init(TIM1,& TIM_OCInitstruct);
			TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);

//init for oc1-ch2	
		TIM_OCInitstruct.TIM_OCMode=TIM_OCMode_PWM1;
		TIM_OCInitstruct.TIM_OCPolarity=TIM_OCPolarity_High;
		TIM_OCInitstruct.TIM_OutputState=TIM_OutputState_Enable;
			TIM_OC2Init(TIM1,& TIM_OCInitstruct);
			TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
		
//init for oc1-ch3		
		TIM_OCInitstruct.TIM_OCMode=TIM_OCMode_PWM1;
		TIM_OCInitstruct.TIM_OCPolarity=TIM_OCPolarity_High;
		TIM_OCInitstruct.TIM_OutputState=TIM_OutputState_Enable;
			TIM_OC3Init(TIM1,& TIM_OCInitstruct);
			TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);

//init for oc1-ch4		
		TIM_OCInitstruct.TIM_OCMode=TIM_OCMode_PWM1; 
		TIM_OCInitstruct.TIM_OCPolarity=TIM_OCPolarity_High;
		TIM_OCInitstruct.TIM_OutputState=TIM_OutputState_Enable;
			TIM_OC4Init(TIM1,& TIM_OCInitstruct);
			TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);

			
//interrupt
//tim2 interrupt enable
//	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
//	NVIC_InitTypeDef NVIC_Initsturcture;
//		NVIC_Initsturcture.NVIC_IRQChannel=TIM1_IRQn; //SR->UIF=1(Updata interrupt flag)
//		NVIC_Initsturcture.NVIC_IRQChannelPreemptionPriority=2;
//		NVIC_Initsturcture.NVIC_IRQChannelSubPriority=2;
//		NVIC_Initsturcture.NVIC_IRQChannelCmd=ENABLE;
//			NVIC_Init(&NVIC_Initsturcture);	
				
			
//preload register		
		TIM_ARRPreloadConfig(TIM1,ENABLE);

//start timer
		TIM_Cmd(TIM1,ENABLE);
//only for advanced TIMER (1,8) 		
		TIM_CtrlPWMOutputs(TIM1,ENABLE);

}

void PWM_Generate_4(u16 PWM_Motor_1,u16 PWM_Motor_2,u16 PWM_Motor_3,u16 PWM_Motor_4) //(1ms->2ms)20ms
{	
	/* -----------------------------------------------------------------------
	- Prescaler = (TIM1CLK / TIM1counter clock) - 1
    
		SystemCoreClock is set to 168 MHz 
	
    The TIM1 is set to 168 MHz 
	
    TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR)* 100 = (100)1ms
    TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR)* 100 = 
    TIM1 Channel3 duty cycle = (TIM1_CCR3/ TIM1_ARR)* 100 = 
    TIM1 Channel4 duty cycle = (TIM1_CCR4/ TIM1_ARR)* 100 = (200)2ms
	----------------------------------------------------------------------- */
	
		if(PWM_Motor_1>2000)TIM_SetCompare1(TIM1,2000);else if(PWM_Motor_1<1000)TIM_SetCompare1(TIM1,1000);else TIM_SetCompare1(TIM1,PWM_Motor_1); 
		if(PWM_Motor_2>2000)TIM_SetCompare2(TIM1,2000);else if(PWM_Motor_2<1000)TIM_SetCompare2(TIM1,1000);else TIM_SetCompare2(TIM1,PWM_Motor_2); 
		if(PWM_Motor_3>2000)TIM_SetCompare3(TIM1,2000);else if(PWM_Motor_3<1000)TIM_SetCompare3(TIM1,1000);else TIM_SetCompare3(TIM1,PWM_Motor_3); 
		if(PWM_Motor_4>2000)TIM_SetCompare4(TIM1,2000);else if(PWM_Motor_4<1000)TIM_SetCompare4(TIM1,1000);else TIM_SetCompare4(TIM1,PWM_Motor_4); 
	
}
