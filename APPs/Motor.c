#include "Motor.h"
//½×¶Î0-1-2-3
extern Float_UVAdata Remote_data;//remote signal

int stage=0;

int  Motor_Init()
{	  
		u16 PWM_Motor[4];
	
		PWM_Motor[0]=Remote_data.throttle;
	
while(stage==0)
{
	if(PWM_Motor[0]>=800&&PWM_Motor[0]<=1200)
	{
	  TIM_SetCompare1(TIM1,1000); 
		TIM_SetCompare2(TIM1,1000); 
		TIM_SetCompare3(TIM1,1000); 
		TIM_SetCompare4(TIM1,1000); 
		stage=1;
	}
	else
		return 1;
}	

while(stage==1)
{
	if(PWM_Motor[0]>=2000&&PWM_Motor[0]<=2200)
		{
	  TIM_SetCompare1(TIM1,2000); 
		TIM_SetCompare2(TIM1,2000); 
		TIM_SetCompare3(TIM1,2000); 
		TIM_SetCompare4(TIM1,2000); 
		stage=2;
		}
	else
		return 1;
}	

while(stage==2)
{
	if(PWM_Motor[0]>=800&&PWM_Motor[0]<=1200)
	{
	  TIM_SetCompare1(TIM1,1000); 
		TIM_SetCompare2(TIM1,1000); 
		TIM_SetCompare3(TIM1,1000); 
		TIM_SetCompare4(TIM1,1000); 
		stage=3;
	}
	else
		return 1;
}	

	if(stage==3)
	{		
			Delay_ms(2000);
			return 0;
	}

	return 1;
}
