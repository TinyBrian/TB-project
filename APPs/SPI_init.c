#include "SPI_init.h"

//PA5---SCK
//PA6---MISO
//PA7---MOSIO

void SPI_init()
{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	
	SPI_InitTypeDef SPI_Initstructure;
	GPIO_InitTypeDef GPIO_Initstructure;
	

	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_7;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_Initstructure);
	
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_Initstructure);
	
		
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_4;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_Initstructure);
		GPIO_SetBits(GPIOA,GPIO_Pin_4);
	

	SPI_Initstructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_8; //pre4--18M---NRF2401L(0-10M)
	SPI_Initstructure.SPI_CPHA=SPI_CPHA_1Edge;
	SPI_Initstructure.SPI_CPOL=SPI_CPOL_Low;
	SPI_Initstructure.SPI_CRCPolynomial=0;
	SPI_Initstructure.SPI_DataSize=SPI_DataSize_8b;
	SPI_Initstructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	SPI_Initstructure.SPI_FirstBit=SPI_FirstBit_MSB;
	SPI_Initstructure.SPI_Mode=SPI_Mode_Master;
	SPI_Initstructure.SPI_NSS=SPI_NSS_Soft;
		SPI_Init(SPI1,&SPI_Initstructure);
	
	
	SPI_DataSizeConfig(SPI1,SPI_DataSize_8b);
	
	SPI_Cmd(SPI1,ENABLE);
	
	
}



ErrorStatus SPI_biduplex_bytes(u8 NSSx,u8* data_s,u8* data_r,u8 Numbertosent,u8 Numbertoread)
{
	assert_param(IS_NSS_SELECT(NSSx));
	
	if(NSSx==0)
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);

if(Numbertosent<=8)
	{
	while(Numbertosent)
				{	
						int i=0;
						u32 Tout;
					
						while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==0)
							{Tout=Timeout;if(Tout==0)return ERROR;}
						
						SPI_I2S_SendData(SPI1,*(data_s+i));
						
						if(Numbertoread>0)
						{
							while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==0)
								{Tout=Timeout;if(Tout==0)return ERROR;}
							
							*(data_r+i)=SPI_I2S_ReceiveData(SPI1);							
						}	
					
						i++;
						Numbertosent--;
						Numbertoread--;
				}
	}
	return SUCCESS;
}


void SPI_Timeout_Handle()
{
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	
	

}


void SPI_sent_command(u8 NSSx,u8 data_s,u8 Numbertoread,u8* data_r)
{
		assert_param(IS_NSS_SELECT(NSSx));
		
		int i=0;
		
		if(NSSx==0)
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);

		while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==0);
		SPI_I2S_SendData(SPI1,data_s);
		if(Numbertoread>0)
		{	
			while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==0);
			*(data_r+i)=SPI_I2S_ReceiveData(SPI1);	
			i++;
			Numbertoread--;
		}
}


u8 SPI_read_byte(u8 NSSx)
{
	u32 SPITimeout;
	
	assert_param(IS_NSS_SELECT(NSSx));
	
	if(NSSx==0)
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);

	SPITimeout=SPI_FLAG_TIMEOUT;
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==0)
		if((SPITimeout--) == 0) 
			break;
	
		SPI_I2S_SendData(SPI1,dummy);

	SPITimeout=SPI_FLAG_TIMEOUT;
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==0)
		if((SPITimeout--) == 0) 
			break;
	
		
		return SPI_I2S_ReceiveData(SPI1);

}


