#include "NRF_USART_MODULE.h"

void NRF_Datai_pc_ascll(u8 data,u8 Bytestoreceive)   
{
	if(Bytestoreceive>1&&Bytestoreceive<=31){ 

		char pdata[32];
		itoa((long)data,pdata,10);//最后一位 x有效位数
		NRF_Datac_pc_ascll(pdata,strlen(pdata)+2);

}}



void NRF_Dataf_pc_ascll(float data,u8 Bytestoreceive)   
{
	if(Bytestoreceive>1&&Bytestoreceive<=31){ 
		
		char* pdata;
		pdata=ftoa(data,pdata,3);//最后一位 x有效位数
		NRF_Datac_pc_ascll(pdata,strlen(pdata)+2);

}}



void NRF_Datac_pc_ascll(char* pdata,u8 Bytestoreceive)     
{		 
		int i;
	
		u8 USART_data[32]={0};
		u8 len;

		len=strlen(pdata);
		
		for(i=1;i<=len;i++)
			USART_data[i]=*pdata++;
		
		USART_data[0]=len+1;
		
		USART_data[i]='\n'; 
		
	  USART_data[i+1]='\0';   
			
		NRF_Tx_mode(); 
			
		NRF_Tx_data(USART_data);
		
		NRF_Rx_mode();  
			
}

//添加字头0，用户决定发几个字符，一般strlen（outdata）+1.
void NRF_USART_Send_char(u8* outdata,u8 len)
	{
		int i;
		u8 USART_data[32]={0};
	
		USART_data[0]=len;
		
		for(i=1;i<=len;i++)
			USART_data[i]=*outdata++;
		
		USART_data[len+1]='\n';
		 		
		NRF_Tx_mode();  
		
		NRF_Tx_data(USART_data);
		
		NRF_Rx_mode();  
	}
	
	
	void NRF_USART_Send_gyro(char* outdata,u8 length)
	{
		int i,len;
		u8 USART_data[32]={0};
		
		len=strlen(outdata);      
		
		USART_data[0]=len+1;
		
		for(i=1;i<=len;i++)
			USART_data[i]=*outdata++;
		
		USART_data[len+1]='\n';
		
		NRF_Tx_mode();  
		
		NRF_Tx_data(USART_data);
		
		NRF_Rx_mode();  
	}
	
/***********************************************************************************/	
void NRF_USART_Read_char(u8* indata)
	{
//		NRF_Rx_mode();
//		NRF_Rx_data(indata);				
	}


/**********************************************************************************/
void NRF_MPU_Gyrdata_c(int16_t *Gyro)     
{	
	char Gyrostr[32];

	sprintf(Gyrostr,"X:%.2f,Y:%.2f,Z:%.2f",Gyro[0]*Gyro2000,Gyro[1]*Gyro2000,Gyro[2]*Gyro2000); 
	
	NRF_USART_Send_gyro(Gyrostr,32); 
	
}





