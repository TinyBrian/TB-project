#include "ADC_init.h"



void ADC_init(void )
{	
		ADC_DeInit();
		ADC_InitTypeDef	 ADC_Initsturture;
		GPIO_InitTypeDef GPIO_Initstructre;
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);		 											//RCC
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);


	
	//GPIOConfig
	GPIO_Initstructre.GPIO_Mode=GPIO_Mode_AIN;							//Mode:       Analog Input
	GPIO_Initstructre.GPIO_Pin=GPIO_Pin_0;                 //CHANNEL:     IN0
	GPIO_Initstructre.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Initstructre.GPIO_Speed=GPIO_Speed_50MHz;			
			GPIO_Init(GPIOA,&GPIO_Initstructre);                       
	
		ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;
		ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent; 				 //独立模式
		ADC_CommonInitStructure.ADC_Prescaler=ADC_Prescaler_Div6;	 			// Fadc<=36M, fpclk2=84mhz
		ADC_CommonInitStructure.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_20Cycles;
			ADC_CommonInit(&ADC_CommonInitStructure);

//ADC-init
		ADC_Initsturture.ADC_Resolution=ADC_Resolution_12b;                
		ADC_Initsturture.ADC_ScanConvMode=DISABLE;                        		//多通道                           否
		ADC_Initsturture.ADC_ContinuousConvMode=DISABLE;                  	  //连续转                           否
		ADC_Initsturture.ADC_DataAlign=ADC_DataAlign_Right;                   //数据右对齐：                     0000 xxxx xxxx xxxx
		ADC_Initsturture.ADC_NbrOfConversion=1;
		ADC_Initsturture.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
		ADC_Initsturture.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_CC1;    //使用软件触发，外部触发不用配置，但千万别注释掉，这个代替以前配置成软件模式
			ADC_Init(ADC1,&ADC_Initsturture);
			


//使能中断NVIC

//		NVIC_InitTypeDef NVIC_Initstucture;
//		NVIC_Initstucture.NVIC_IRQChannel=ADC_IRQn;              /////////////////////////ADC1///////////////////////////
//		NVIC_Initstucture.NVIC_IRQChannelCmd=ENABLE;
//		NVIC_Initstucture.NVIC_IRQChannelPreemptionPriority=1;
//		NVIC_Initstucture.NVIC_IRQChannelSubPriority=1;
//			 NVIC_Init(&NVIC_Initstucture);
	
	//	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);                       //转换完成中断：End of conversion（EOC）

	

			
//规则，采样周期
		ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_480Cycles);

//ADC start up
		ADC_Cmd(ADC1,ENABLE);		
	
	ADC_SoftwareStartConv(ADC1);

}

		
/***ADC(0-3.3)->(0,4096)->(0,100)***/		
int ADC_filter_return_lcd()
{
	 u16 a=0,b;
	
	for(int i=0;i<=9;i++)											//滤波 
	{		
			ADC_SoftwareStartConv(ADC1);
			while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
			a+=ADC_GetConversionValue(ADC1);
	}
    b=a*10/4096;   

		return b;
	
	
}

/***ADC(0-3.3)->(0,4096)->(300,1300)**********/
u16 ADC_filter_return()
{		u8 j=0;
		u32 a=0;
		volatile float b=0;
		u16 fliter=0,pre=0;
	for(int i=0;i<=9;i++)											//滤波 
	{	
			ADC_SoftwareStartConv(ADC1);
			

		while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET)
			{
				j++;
				if(j==200)
				{	j=0;
					break;
				}
			}
			fliter=ADC_GetConversionValue(ADC1);
		
			if(i==0)
				pre=fliter;
			
			if((fliter-pre)<10&&(fliter-pre)>-10)
			{
				a=a+fliter;	
				pre=fliter;				
			}
				else
				a=a+pre;
	}
    a/=10;
		b=a*0.2441+300;   
	
	return b;
}

/***ADC(0-3.3)->(0,4096)->(0,13)**********/
float ADC_filter_return_battery()  
{
		u32 a=0;
		float b=0;
		u16 fliter=0,pre=0;
	for(int i=0;i<=999;i++)											//滤波 
	{	
			ADC_SoftwareStartConv(ADC1);
			

		while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET)
			{
					for(int j=0;j<=2000;j--)
						if(j==0)
						{
							break;
						}
			}
			
			fliter=ADC_GetConversionValue(ADC1);
		
			if(i==0)
				pre=fliter;
			
			if((fliter-pre)<10&&(fliter-pre)>-10)
			{
				a=a+fliter;	
				pre=fliter;				
			}
				else
				a=a+pre;
	}
    a/=1000;
		b=a*0.0031738;   
	
	return b;
}

	
/*void ADC1_2_IRQHandler()
{
		LCD_ADC_percent();
		Delay_ms(100);
}*/

