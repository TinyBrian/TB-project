#include "main.h"

#define Throttlereset 0

u8 UPPerdata[32]={0};
u8 Remotedata[32]={0};
u8 Throttle_reset=0;
u16 PWM_Motor[4]={1000};    

Raw_UVAdata Remote_data_raw;//remote raw data
Float_UVAdata Remote_data;  //coverted data
Float_UVAdata Measure_data;

int main()
{		
#if(Throttlereset)
	int i;
#endif   
		u8 status;
		Meausure_Pinit();
		RemoteControl_Pinit();
		PID_all_init_cl();
		RemoteControl_Raw_simul();
	
	//	USART_init(9600);     
		
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
			
		PWM_Timer_init(20000-1,168-1); //Timer_init(period,prescaler);CLK=100kHz, ARR=19999 ;T=20ms			
	
		IIC_init(0xAA,400000);		

		ADC_init();

		LED_Init();   
			
		NRF_SPI_init();
		NRF24L01_Self_checking(); //自检完成进接受等待UNLOCK信号		
		
		while(MPU_Self_checking()==ERROR);		
		MPU_DMP_Setup(); 	

		SysTick_Init(); //tick&interrupt on

		EXTI_init();

//		while(UVA_standby());//等待UNLOCK信号

	//NRF_Rx_mode();
	while(1) 
	{
		
#if(Throttlereset)
		if(Throttle_reset!=0)
		{
//#endif
			i=ADC_filter_return(); 
			PWM_Generate_4(i,i,i,i);	 
//#if(Throttlereset)	
//		}
#endif
//From Supervisor PC
		
	
		status=NRF_Rx_data(UPPerdata);	
		if(status==RX_DR)
		{	
			for(int i=1;i<=UPPerdata[0]+1;i++)
				Data_Receive_ANO_Prepare(UPPerdata[i]);
		}
	
//From Remote contoler		

//		if(NRF_Rx_data_remote(UPPerdata)==RX_DR)
//		{		
//			u16 sum=0;
//			
//			for(int i=1;i<UPPerdata[0];i++)
//				sum+=UPPerdata[i];
//			
//			if(sum==UPPerdata[9])
//			{		
//				Remote_data_raw.pitch=Remotedata[2]<<8|Remotedata[1];
//				Remote_data_raw.roll=Remotedata[4]<<8|Remotedata[3];
//				Remote_data_raw.yaw=Remotedata[6]<<8|Remotedata[5];
//				Remote_data_raw.throttle=Remotedata[8]<<8|Remotedata[7];
//			}
//	  }
		Remote_data_raw.throttle=ADC_filter_return(); 
		MPU_DMP_Loop();

			
	}  
}


void System_reset()
{
				__set_FAULTMASK(1);
				NVIC_SystemReset();
}

void RemoteControl_Pinit()
{
	Remote_data.Arminfo=0;
	Remote_data.pitch=0;
	Remote_data.roll=0;
	Remote_data.yaw=0;
	Remote_data.throttle=1500;
}

void RemoteControl_Raw_simul()
{
	Remote_data_raw.pitch=1500;
	Remote_data_raw.roll=1500;
	Remote_data_raw.yaw=1500;
	Remote_data_raw.throttle=1000;
}

void Meausure_Pinit()
{
	Measure_data.pitch=0;
	Measure_data.roll=0;
	Measure_data.yaw=0;
	Measure_data.throttle=0;
	Measure_data.Arminfo=0;
}


