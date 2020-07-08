#include "NRF24L01.h" 


u8 RX_BUF[RX_PLOAD_WIDTH];		//�������ݻ���
u8 TX_BUF[TX_PLOAD_WIDTH];		//�������ݻ���
u8 TX_ADDRESS[TX_ADR_WIDTH] = {0xA5,0xA5,0xA5,0xA5,0xA5};  // ����һ����̬���͵�ַ
u8 RX_ADDRESS[RX_ADR_WIDTH] = {0xA5,0xA5,0xA5,0xA5,0xA5};
u8 RX_ADDRESS_1[RX_ADR_WIDTH] = {0xB1,0xC2,0xD3,0xE4,0xF5};
u8 nrf_flag;
void NRF_SPI_init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /*������ӦIO�˿ڵ�ʱ��*/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB 
                         |NRF_CSN_GPIO_CLK
                         |NRF_CE_GPIO_CLK
                         |NRF_IRQ_GPIO_CLK,ENABLE);
  

 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

//  SPI1_SCK-------PA5/PB3
//	SPI1_MISO------PA6/PB4
//	SPI1_MOSI------PA7/PB5
	//���ù�����ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1);


//SCK		  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
		GPIO_Init(NRF_CSN_GPIO_PORT, &GPIO_InitStructure);	
//MOSI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		GPIO_Init(NRF_CSN_GPIO_PORT, &GPIO_InitStructure);
//MISO	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
		GPIO_Init(NRF_CSN_GPIO_PORT, &GPIO_InitStructure);

	
	
//���༸��	
  //CSN ����
  GPIO_InitStructure.GPIO_Pin = NRF_CSN_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//GPIO_OType_OD
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_Init(NRF_CSN_GPIO_PORT, &GPIO_InitStructure);
  
	//CE����
  GPIO_InitStructure.GPIO_Pin = NRF_CE_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_Init(NRF_CE_GPIO_PORT, &GPIO_InitStructure);

  //IRQ����
  GPIO_InitStructure.GPIO_Pin = NRF_IRQ_GPIO_CLK;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_Init(NRF_IRQ_GPIO_PORT, &GPIO_InitStructure); 

//SPI1<-APB2-PCLK=84M
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //˫��ȫ˫��
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	 					           //��ģʽ
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	 				         //���ݴ�С8λ
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		 			               //ʱ�Ӽ��ԣ�����ʱΪ��
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						           //��1��������Ч��������Ϊ����ʱ��
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		   					           //NSS�ź����������
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; //16��Ƶ��84/16MHz
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  				       //��λ��ǰ
  SPI_InitStructure.SPI_CRCPolynomial = 7;
		SPI_Init(SPI1, &SPI_InitStructure);


  SPI_Cmd(SPI1, ENABLE);
	Delay_ms(2);
	
	
	NRF_CE_LOW();
	NRF_CSN_HIGH(); 
}

/****************************************************************************************
  * @brief  ��Ҫ����NRF��MCU�Ƿ���������
  * @param  ��
  * @retval SUCCESS/ERROR ��������/����ʧ��
  */

ErrorStatus NRF_Check(void)
{
	u8 ADDR[5]={0xA5,0xA5,0xA5,0xA5,0xA5};
	u8 ADDR_c[5]={0};
	u8 i; 
	 
	/*д��5���ֽڵĵ�ַ.  */  
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,ADDR,5);
	
	/*����д��ĵ�ַ */
	NRF_Read_Buf(NRF_READ_REG+TX_ADDR,ADDR_c,5); 
	 
	/*�Ƚ�*/               
	for(i=0;i<5;i++)
	{
		if(ADDR_c[i]!=0xA5)
		break;
	} 
	       
	if(i==5)
		return SUCCESS ;        //MCU��NRF�ɹ����� 
	else
		return ERROR ;        //MCU��NRF����������
}

/*********************************************************************************************/
u8 SPI_NRF_RW_byte(u8 data)
{  int i=1000;
	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
			{
				i--;
				if(i==0)
					{i=1000;break;}
			}
			
  SPI_I2S_SendData(SPI1, data);		

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET)
			{
				i--;
				if(i==0)
					{i=1000;break;}
			}
			
	return SPI_I2S_ReceiveData(SPI1);
}

 

//������NRF�ļĴ�����д��һ������ 
//NRF������+�Ĵ�����ַ
/**************************************************************************************/
u8 NRF_Write_Buf(u8 reg ,u8 *pBuf,u8 Numbertosent)
{
	 u8 status;
	 
	NRF_CSN_LOW();
	 
	NRF_CE_LOW();
   	 Delay_ms(1);
	 /*���ͼĴ�����*/	
	 status =reg;                                                               
   status = SPI_NRF_RW_byte(reg); 
 	
  	  /*�򻺳���д������*/
	 for(int i=Numbertosent;i>0;i--)
	{
		 SPI_NRF_RW_byte(*pBuf++);	//д���ݵ������� 	 
	} 	   
	/*CSN���ߣ����*/
		NRF_CSN_HIGH();			
  
  	return status;	//����NRF24L01��״̬ 		
}


//������NRF�ļĴ�����д��һ������ 
//NRF������+�Ĵ�����ַ
/*********************************************************************************************/
u8 NRF_Read_Buf(u8 reg,u8 *pBuf,u8 Numbertoread)
{
 	u8 status;

  NRF_CE_LOW();
	/*�õ�CSN��ʹ��SPI����*/
	NRF_CSN_LOW();
		
	/*���ͼĴ�����*/
	status =reg;
	status = SPI_NRF_RW_byte(reg); 

 	/*��ȡ����������*/
	 	for(int i=0;i<Numbertoread;i++)	
{	
	   *pBuf++= SPI_NRF_RW_byte(NOP); //��NRF24L01��ȡ����  
}
	 /*CSN���ߣ����*/
	NRF_CSN_HIGH();	
		
 	return status;		//���ؼĴ���״ֵ̬
}



/*********************************************************************************************/
ErrorStatus NRF24L01_Self_checking(void)
{	
	char* Txcharbuf="NRF Connected!"; 
	u8 status;	

  status = NRF_Check();  //��� NRF ģ���� MCU ������
  
  if(status == SUCCESS)	 //�ж�����״̬   
		{	
			GPIO_ResetBits(GPIOA,GPIO_Pin_6);
			NRF_Tx_mode();
			NRF_Datac_pc_ascll(Txcharbuf,16);  
			NRF_CE_LOW(); //ת������ģʽ
			//NRF_Rx_mode();
		} 
  else	  
		{	 
			return ERROR;
		}

		return SUCCESS; 	
}


//***********************************************************************************************
u8 NRF_Write_Reg(u8 reg,u8 data)
{
 	u8 status;
	 NRF_CE_LOW();
	/*�õ�CSN��ʹ��SPI����*/
    NRF_CSN_LOW();
				
	/*��������Ĵ����� */
	status = SPI_NRF_RW_byte(reg);
		 
	 /*��Ĵ���д������*/
   SPI_NRF_RW_byte(data); 
	          
	/*CSN���ߣ����*/	   
  	NRF_CSN_HIGH();	
		
	/*����״̬�Ĵ�����ֵ*/
   	return(status);
}

/*********************************************************************************************/
u8 NRF_Read_Reg(u8 reg)
{
 	u8 reg_val;

	NRF_CE_LOW();
	/*�õ�CSN��ʹ��SPI����*/
 	NRF_CSN_LOW();
				
  	 /*���ͼĴ�����*/
	SPI_NRF_RW_byte(reg); 

	 /*��ȡ�Ĵ�����ֵ */
	reg_val = SPI_NRF_RW_byte(NOP);
	            
   	/*CSN���ߣ����*/
	NRF_CSN_HIGH();		
   	
	return reg_val;
}	

/********************************************************************************************************/
u8 NRF_Read_StringBuf(u8 reg,char *pBuf,u8 Numbertoread)
{
 	u8 status;

	  NRF_CE_LOW();
	/*�õ�CSN��ʹ��SPI����*/
	NRF_CSN_LOW();
		
	/*���ͼĴ�����*/		
	status = SPI_NRF_RW_byte(reg); 

 	/*��ȡ����������*/
	for(int i=0;i<Numbertoread;i++)	  
	   pBuf[i] = SPI_NRF_RW_byte(NOP); //��NRF24L01��ȡ����  

	 /*CSN���ߣ����*/
	NRF_CSN_HIGH();	
		
 	return status;		//���ؼĴ���״ֵ̬
}



/*********************************************************************************************************/

u8 NRF_Write_StringBuf(u8 reg ,char *pBuf,u8 Numbertosent)
{
	 u8 status;
	 NRF_CE_LOW();
   	 /*�õ�CSN��ʹ��SPI����*/
	 NRF_CSN_LOW();			

	 /*���ͼĴ�����*/	
  	 status = SPI_NRF_RW_byte(reg); 
 	
  	  /*�򻺳���д������*/
	  for(int i=Numbertosent;i>0;i--)
			SPI_NRF_RW_byte(*pBuf++);	//д���ݵ������� 	 
	  	   
	/*CSN���ߣ����*/
		NRF_CSN_HIGH();			
  
  	return (status);	//����NRF24L01��״̬ 		
}


 //Rx
/**********************************************************************************************/
void NRF_Rx_mode(void)
{
	NRF_CE_LOW();	

   NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ

   NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    

   NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ    

   NRF_Write_Reg(NRF_WRITE_REG+RF_CH,CHANAL);      //����RFͨ��Ƶ��    

   NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��      

   NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); //����TX�������,0db����,2Mbps,���������濪��   

   NRF_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);  //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 

/*CE���ߣ��������ģʽ*/	
  NRF_CE_HIGH();

}    

// Tx
/********************************************************************************* */
void NRF_Tx_mode(void)
{  
	NRF_CE_LOW();		

   NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);    //дTX�ڵ��ַ 

   NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK   

   NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    

   NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  

   NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��

   NRF_Write_Reg(NRF_WRITE_REG+RF_CH,CHANAL);       //����RFͨ��ΪCHANAL

   NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
	
   NRF_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�

/*CE���ߣ����뷢��ģʽ*/	
  NRF_CE_HIGH();
    Delay_ms(5); //CEҪ����һ��ʱ��Ž��뷢��ģʽ
}



/***************************************************************************************
  * @brief   ������NRF�ķ��ͻ�������д������
  * @param   
  *		@arg Txbuf���洢�˽�Ҫ���͵����ݵ����飬�ⲿ����	
  * @retval  ���ͽ�����ɹ�����TXDS,ʧ�ܷ���MAXRT��ERROR
  */
u8 NRF_Tx_data(u8 *Txbuf)
{
	u8 state;  

	 /*ceΪ�ͣ��������ģʽ1*/
	NRF_CE_LOW();

	/*д���ݵ�TX BUF ��� 32���ֽ�*/						
   NRF_Write_Buf(WR_TX_PLOAD,Txbuf,TX_PLOAD_WIDTH);

      /*CEΪ�ߣ�txbuf�ǿգ��������ݰ� */   
 	 NRF_CE_HIGH();
	  	
	  /*�ȴ���������ж� */                            
	while(NRF_Read_IRQ()!=0); 	
	
	/*��ȡ״̬�Ĵ�����ֵ */                              
	state = NRF_Read_Reg(STATUS);

	 /*���TX_DS��MAX_RT�жϱ�־*/                  
	NRF_Write_Reg(NRF_WRITE_REG+STATUS,state); 	

	NRF_Write_Reg(FLUSH_TX,NOP);    //���TX FIFO�Ĵ��� 

	 /*�ж��ж�����*/    
	if(state&MAX_RT)                     //�ﵽ����ط�����
			 return MAX_RT; 

	else if(state&TX_DS)                  //�������
		 	return TX_DS;
	 else						  
			return ERROR;                 //����ԭ����ʧ��
} 

/**************************************************************************************
  * @brief   ���ڴ�NRF�Ľ��ջ������ж�������
  * @param   
  *		@arg Rxbuf �����ڽ��ո����ݵ����飬�ⲿ����	
  * @retval 
  *		@arg ���ս��
  */
u8 NRF_Rx_data(u8 *Rxbuf)
{
	u8 state; 
	NRF_CE_HIGH();	 //�������״̬
	 /*�ȴ������ж�*/
	while(NRF_Read_IRQ()==0)
  {
    NRF_CE_LOW();  	 //�������״̬
   
		/*��ȡstatus�Ĵ�����ֵ  */               
		state=NRF_Read_Reg(STATUS);
		
		if((state>>1&0x07)!=0x00) 
			return ERROR;          //��P0���ݣ�����
  
		/* ����жϱ�־*/      
   NRF_Write_Reg(NRF_WRITE_REG+STATUS,state);

    /*�ж��Ƿ���յ�����*/
    if(state&RX_DR)                                 //���յ�����
    {
			 NRF_Read_Buf(RD_RX_PLOAD,Rxbuf,RX_PLOAD_WIDTH);//��ȡ����
			 NRF_Write_Reg(FLUSH_RX,NOP);          //���RX FIFO�Ĵ���
       return RX_DR; 
    }
    else    
			 return ERROR;                    //û�յ��κ�����
  }
  
  return ERROR;                    //û�յ��κ�����
}

u8 NRF_Rx_data_remote(u8 *Rxbuf)
{
	u8 state; 
	NRF_CE_HIGH();	 //�������״̬
	 /*�ȴ������ж�*/
	while(NRF_Read_IRQ()==0)
  {
    NRF_CE_LOW();  	 //�������״̬
   
		/*��ȡstatus�Ĵ�����ֵ  */               
		state=NRF_Read_Reg(STATUS);
		
  	if((state>>1&0x07)!=1) 
		return ERROR;					//��P1���ݣ�����
			
		/* ����жϱ�־*/      
   NRF_Write_Reg(NRF_WRITE_REG+STATUS,state);

    /*�ж��Ƿ���յ�����*/
    if(state&RX_DR)                                 //���յ�����
    {
			 NRF_Read_Buf(RD_RX_PLOAD,Rxbuf,RX_PLOAD_WIDTH);//��ȡ����
			 NRF_Write_Reg(FLUSH_RX,NOP);          //���RX FIFO�Ĵ���
       return RX_DR; 
    }
    else    
			 return ERROR;                    //û�յ��κ�����
  }
  
  return ERROR;                    //û�յ��κ�����
}
/***********************************************************************************/
u8 NRF_Rx_string(char *Rxcharbuf)
{
	u8 state; 
	NRF_CE_HIGH();	 //�������״̬

	while(NRF_Read_IRQ()==0)	 /*�ȴ������ж�*/
  {
    NRF_CE_LOW();  	 //�������״̬
   
	           
		state=NRF_Read_Reg(STATUS);	/*��ȡstatus�Ĵ�����ֵ  */    
  
		
   NRF_Write_Reg(NRF_WRITE_REG+STATUS,state);/* ����жϱ�־*/      


    if(state&RX_DR)//���յ�����
    {
			 NRF_Read_StringBuf(RD_RX_PLOAD,Rxcharbuf,RX_PLOAD_WIDTH);//��ȡ����
			 NRF_Write_Reg(FLUSH_RX,NOP);          //���RX FIFO�Ĵ���
       return RX_DR; 
    }
    else//û�յ��κ�����    
			 return ERROR;                    
  }
  
  return ERROR;                    //û�յ��κ�����
}

/************************************************************************************/
u8 NRF_Tx_string(char *Txcharbuf)
{
	u8 state;  
	
	NRF_CE_LOW(); /*ceΪ�ͣ��������ģʽ1*/
						
  NRF_Write_StringBuf(WR_TX_PLOAD,Txcharbuf,TX_PLOAD_WIDTH);/*д���ݵ�TX BUF ��� 32���ֽ�*/	
        
 	NRF_CE_HIGH();/*CEΪ�ߣ�Txbuf�ǿգ��������ݰ� */ 
	  	                          
	while(NRF_Read_IRQ()!=0); 		  /*�ȴ���������ж� */ 
                            
	state = NRF_Read_Reg(STATUS);	/*��ȡ״̬�Ĵ�����ֵ */ 
	           
	NRF_Write_Reg(NRF_WRITE_REG+STATUS,state);	 /*���TX_DS��MAX_RT�жϱ�־*/   	

	NRF_Write_Reg(FLUSH_TX,NOP);    //���TX FIFO�Ĵ��� 

	 /*�ж��ж�����*/    
	if(state&MAX_RT)                //�ﵽ����ط�����
			 return MAX_RT; 

	else if(state&TX_DS)            //�������
		 	return TX_DS;
	
			else						  
			return ERROR;               //����ԭ����ʧ��
} 

u8 NRF_Tx_complete() 
{ u8 status;
	u16 I2CTimeout=10000;
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==RESET)
	{    status=BUSY;
					if((I2CTimeout--)==0)
						return BUSY;
	}
	status=FREE;
	return status;
}
	
u8 NRF_Rx_complete() 
{ u8 status;
	u16 I2CTimeout=10000;
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==RESET)
	{    status=BUSY;
					if((I2CTimeout--)==0)
						return BUSY;
	}
	status=FREE;
	return status;
}
	

