#include "EXIT_init.h"
//K-UP		 PA0
//PE3 			K1
//PE4 			K0
extern u8 Throttle_reset;
extern void gyro_data_ready_cb(void);
extern Float_UVAdata Remote_data;//remote signal
extern Float_UVAdata Measure_data;//remote signal
extern u8 nrf_flag;
extern u8 UPPerdata[32];
 /**
  * @brief  ���� Ϊ���жϿڣ��������ж����ȼ�  
  * @param  ��
  * @retval ��
  */
void EXTI_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(EXTI_GPIO_CLK_MPU9250|EXTI_GPIO_CLK_Armed|EXTI_GPIO_CLK_Kx|EXTI_GPIO_CLK_NRF,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
												
	/* config the NVIC */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQ_MPU9250;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQ_Armed;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQ_NRF;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);
//	
	
#if( Throttlereset)		
	NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQ_Kx;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
#endif	
	
  GPIO_InitStructure.GPIO_Pin = EXTI_GPIO_PIN_MPU9250;       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_Init(EXTI_GPIO_PORT_MPU9250, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = EXTI_GPIO_PIN_Armed;       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_Init(EXTI_GPIO_PORT_Armed, &GPIO_InitStructure);
//		
//	GPIO_InitStructure.GPIO_Pin = EXTI_GPIO_PIN_NRF;       
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
//		GPIO_Init(EXTI_GPIO_PORT_NRF, &GPIO_InitStructure);
		
		
		
#if( Throttlereset)	
	GPIO_InitStructure.GPIO_Pin = EXTI_GPIO_PIN_Kx;       
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_Init(EXTI_GPIO_PORT_Armed, &GPIO_InitStructure);
#endif		
		
		
	/* EXTI line mode config */
  SYSCFG_EXTILineConfig(EXTI_SOURCE_PORT_MPU9250,EXTI_SOURCE_PIN_MPU9250);  
  EXTI_InitStructure.EXTI_Line = EXTI_GPIO_PIN_MPU9250;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure); 
		
	SYSCFG_EXTILineConfig(EXTI_SOURCE_PORT_Armed,EXTI_SOURCE_PIN_Armed);  
  EXTI_InitStructure.EXTI_Line = EXTI_GPIO_PIN_Armed;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure); 	
//		
//	SYSCFG_EXTILineConfig(EXTI_SOURCE_PORT_NRF,EXTI_SOURCE_PIN_NRF);  
//  EXTI_InitStructure.EXTI_Line = EXTI_GPIO_PIN_NRF;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//		EXTI_Init(&EXTI_InitStructure); 	
//		
		
#if( Throttlereset)		
	SYSCFG_EXTILineConfig(EXTI_SOURCE_PORT_Kx,EXTI_SOURCE_PIN_Kx);  
  EXTI_InitStructure.EXTI_Line = EXTI_GPIO_PIN_Kx;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure); 
#endif	
	
}
    

void EXTI_INT_HANDLER_MPU9250(void)
{
	if(EXTI_GetITStatus(EXTI_GPIO_PIN_MPU9250) != RESET) //ȷ���Ƿ������EXTI Line�ж�
	{
		  /* Handle new gyro*/
		gyro_data_ready_cb(); 
		
		EXTI_ClearITPendingBit(EXTI_GPIO_PIN_MPU9250);     //����жϱ�־λ
		
	}  
	

	
}

void EXTI_INT_HANDLER_Armed(void )  
{
	if(EXTI_GetITStatus(EXTI_GPIO_PIN_Armed) != RESET) //ȷ���Ƿ������EXTI Line�ж�
	{
		
	//	Delay_ms(2);	
	//	if(GPIO_ReadInputDataBit(EXTI_GPIO_PORT_Armed,EXTI_GPIO_PIN_Armed)==0)
			Measure_data.Arminfo=!Measure_data.Arminfo;
	//	Delay_ms(2);	
		
		EXTI_ClearITPendingBit(EXTI_GPIO_PIN_Armed);     //����жϱ�־λ
		
	}  
}

#if( Throttlereset)
void EXTI_INT_HANDLER_Kx(void )  
{
	if(EXTI_GetITStatus(EXTI_GPIO_PIN_Kx) != RESET) //ȷ���Ƿ������EXTI Line�ж�
	{
	//	Delay_ms(2);	
	//	if(GPIO_ReadInputDataBit(EXTI_GPIO_PORT_Kx,EXTI_GPIO_PIN_Kx)==0)
			Throttle_reset=!Throttle_reset;
	//	Delay_ms(2);
		
		EXTI_ClearITPendingBit(EXTI_GPIO_PIN_Kx);     //����жϱ�־λ	
	}  
} 
#endif



//void EXTI_INT_HANDLER_NRF(void)
//{
//	static u8 status;

//	if (EXTI_GetITStatus(EXTI_Line8)==SET)	//�ж��Ƿ���IRQ������ж�
//	{
//		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)==0)
//			{
//		   status=NRF_Read_Reg(STATUS);            // ��ȡ״̬�Ĵ������ж����ݽ���״��
//			 nrf_flag=1; 
//				
//		if(status&0x40)//bit8:���ݽ����ж�
//			{
//			NRF_Read_Buf(RD_RX_PLOAD,UPPerdata,RX_PLOAD_WIDTH);//��ȡ����
//			NRF_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ���
//			}

//		else if((status&0x10)>0)////�ﵽ����ʹ����ж� 
//			{
//			NRF_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
//			}
//		else if((status&0x20)>0)//TX��������ж�
//			{
//			NRF_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ���
//			}
//			NRF_Write_Reg(NRF_WRITE_REG+STATUS,status);//���״̬�Ĵ���
//		}
//		EXTI_ClearITPendingBit(EXTI_LINE_NRF); //�����־
//}
//}
