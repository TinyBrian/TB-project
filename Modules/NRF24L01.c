#include "NRF24L01.h" 


u8 RX_BUF[RX_PLOAD_WIDTH];		//接收数据缓存
u8 TX_BUF[TX_PLOAD_WIDTH];		//发射数据缓存
u8 TX_ADDRESS[TX_ADR_WIDTH] = {0xA5,0xA5,0xA5,0xA5,0xA5};  // 定义一个静态发送地址
u8 RX_ADDRESS[RX_ADR_WIDTH] = {0xA5,0xA5,0xA5,0xA5,0xA5};
u8 RX_ADDRESS_1[RX_ADR_WIDTH] = {0xB1,0xC2,0xD3,0xE4,0xF5};
u8 nrf_flag;
void NRF_SPI_init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /*开启相应IO端口的时钟*/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB 
                         |NRF_CSN_GPIO_CLK
                         |NRF_CE_GPIO_CLK
                         |NRF_IRQ_GPIO_CLK,ENABLE);
  

 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

//  SPI1_SCK-------PA5/PB3
//	SPI1_MISO------PA6/PB4
//	SPI1_MOSI------PA7/PB5
	//复用功能重映射
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

	
	
//其余几个	
  //CSN 引脚
  GPIO_InitStructure.GPIO_Pin = NRF_CSN_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//GPIO_OType_OD
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_Init(NRF_CSN_GPIO_PORT, &GPIO_InitStructure);
  
	//CE引脚
  GPIO_InitStructure.GPIO_Pin = NRF_CE_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_Init(NRF_CE_GPIO_PORT, &GPIO_InitStructure);

  //IRQ引脚
  GPIO_InitStructure.GPIO_Pin = NRF_IRQ_GPIO_CLK;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_Init(NRF_IRQ_GPIO_PORT, &GPIO_InitStructure); 

//SPI1<-APB2-PCLK=84M
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //双线全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	 					           //主模式
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	 				         //数据大小8位
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		 			               //时钟极性，空闲时为低
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						           //第1个边沿有效，上升沿为采样时刻
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		   					           //NSS信号由软件产生
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; //16分频，84/16MHz
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  				       //高位在前
  SPI_InitStructure.SPI_CRCPolynomial = 7;
		SPI_Init(SPI1, &SPI_InitStructure);


  SPI_Cmd(SPI1, ENABLE);
	Delay_ms(2);
	
	
	NRF_CE_LOW();
	NRF_CSN_HIGH(); 
}

/****************************************************************************************
  * @brief  主要用于NRF与MCU是否正常连接
  * @param  无
  * @retval SUCCESS/ERROR 连接正常/连接失败
  */

ErrorStatus NRF_Check(void)
{
	u8 ADDR[5]={0xA5,0xA5,0xA5,0xA5,0xA5};
	u8 ADDR_c[5]={0};
	u8 i; 
	 
	/*写入5个字节的地址.  */  
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,ADDR,5);
	
	/*读出写入的地址 */
	NRF_Read_Buf(NRF_READ_REG+TX_ADDR,ADDR_c,5); 
	 
	/*比较*/               
	for(i=0;i<5;i++)
	{
		if(ADDR_c[i]!=0xA5)
		break;
	} 
	       
	if(i==5)
		return SUCCESS ;        //MCU与NRF成功连接 
	else
		return ERROR ;        //MCU与NRF不正常连接
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

 

//用于向NRF的寄存器中写入一串数据 
//NRF的命令+寄存器地址
/**************************************************************************************/
u8 NRF_Write_Buf(u8 reg ,u8 *pBuf,u8 Numbertosent)
{
	 u8 status;
	 
	NRF_CSN_LOW();
	 
	NRF_CE_LOW();
   	 Delay_ms(1);
	 /*发送寄存器号*/	
	 status =reg;                                                               
   status = SPI_NRF_RW_byte(reg); 
 	
  	  /*向缓冲区写入数据*/
	 for(int i=Numbertosent;i>0;i--)
	{
		 SPI_NRF_RW_byte(*pBuf++);	//写数据到缓冲区 	 
	} 	   
	/*CSN拉高，完成*/
		NRF_CSN_HIGH();			
  
  	return status;	//返回NRF24L01的状态 		
}


//用于向NRF的寄存器中写入一串数据 
//NRF的命令+寄存器地址
/*********************************************************************************************/
u8 NRF_Read_Buf(u8 reg,u8 *pBuf,u8 Numbertoread)
{
 	u8 status;

  NRF_CE_LOW();
	/*置低CSN，使能SPI传输*/
	NRF_CSN_LOW();
		
	/*发送寄存器号*/
	status =reg;
	status = SPI_NRF_RW_byte(reg); 

 	/*读取缓冲区数据*/
	 	for(int i=0;i<Numbertoread;i++)	
{	
	   *pBuf++= SPI_NRF_RW_byte(NOP); //从NRF24L01读取数据  
}
	 /*CSN拉高，完成*/
	NRF_CSN_HIGH();	
		
 	return status;		//返回寄存器状态值
}



/*********************************************************************************************/
ErrorStatus NRF24L01_Self_checking(void)
{	
	char* Txcharbuf="NRF Connected!"; 
	u8 status;	

  status = NRF_Check();  //检测 NRF 模块与 MCU 的连接
  
  if(status == SUCCESS)	 //判断连接状态   
		{	
			GPIO_ResetBits(GPIOA,GPIO_Pin_6);
			NRF_Tx_mode();
			NRF_Datac_pc_ascll(Txcharbuf,16);  
			NRF_CE_LOW(); //转到空闲模式
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
	/*置低CSN，使能SPI传输*/
    NRF_CSN_LOW();
				
	/*发送命令及寄存器号 */
	status = SPI_NRF_RW_byte(reg);
		 
	 /*向寄存器写入数据*/
   SPI_NRF_RW_byte(data); 
	          
	/*CSN拉高，完成*/	   
  	NRF_CSN_HIGH();	
		
	/*返回状态寄存器的值*/
   	return(status);
}

/*********************************************************************************************/
u8 NRF_Read_Reg(u8 reg)
{
 	u8 reg_val;

	NRF_CE_LOW();
	/*置低CSN，使能SPI传输*/
 	NRF_CSN_LOW();
				
  	 /*发送寄存器号*/
	SPI_NRF_RW_byte(reg); 

	 /*读取寄存器的值 */
	reg_val = SPI_NRF_RW_byte(NOP);
	            
   	/*CSN拉高，完成*/
	NRF_CSN_HIGH();		
   	
	return reg_val;
}	

/********************************************************************************************************/
u8 NRF_Read_StringBuf(u8 reg,char *pBuf,u8 Numbertoread)
{
 	u8 status;

	  NRF_CE_LOW();
	/*置低CSN，使能SPI传输*/
	NRF_CSN_LOW();
		
	/*发送寄存器号*/		
	status = SPI_NRF_RW_byte(reg); 

 	/*读取缓冲区数据*/
	for(int i=0;i<Numbertoread;i++)	  
	   pBuf[i] = SPI_NRF_RW_byte(NOP); //从NRF24L01读取数据  

	 /*CSN拉高，完成*/
	NRF_CSN_HIGH();	
		
 	return status;		//返回寄存器状态值
}



/*********************************************************************************************************/

u8 NRF_Write_StringBuf(u8 reg ,char *pBuf,u8 Numbertosent)
{
	 u8 status;
	 NRF_CE_LOW();
   	 /*置低CSN，使能SPI传输*/
	 NRF_CSN_LOW();			

	 /*发送寄存器号*/	
  	 status = SPI_NRF_RW_byte(reg); 
 	
  	  /*向缓冲区写入数据*/
	  for(int i=Numbertosent;i>0;i--)
			SPI_NRF_RW_byte(*pBuf++);	//写数据到缓冲区 	 
	  	   
	/*CSN拉高，完成*/
		NRF_CSN_HIGH();			
  
  	return (status);	//返回NRF24L01的状态 		
}


 //Rx
/**********************************************************************************************/
void NRF_Rx_mode(void)
{
	NRF_CE_LOW();	

   NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址

   NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    

   NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址    

   NRF_Write_Reg(NRF_WRITE_REG+RF_CH,CHANAL);      //设置RF通信频率    

   NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度      

   NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   

   NRF_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);  //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 

/*CE拉高，进入接收模式*/	
  NRF_CE_HIGH();

}    

// Tx
/********************************************************************************* */
void NRF_Tx_mode(void)
{  
	NRF_CE_LOW();		

   NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);    //写TX节点地址 

   NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK   

   NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    

   NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  

   NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次

   NRF_Write_Reg(NRF_WRITE_REG+RF_CH,CHANAL);       //设置RF通道为CHANAL

   NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
	
   NRF_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发射模式,开启所有中断

/*CE拉高，进入发送模式*/	
  NRF_CE_HIGH();
    Delay_ms(5); //CE要拉高一段时间才进入发送模式
}



/***************************************************************************************
  * @brief   用于向NRF的发送缓冲区中写入数据
  * @param   
  *		@arg Txbuf：存储了将要发送的数据的数组，外部定义	
  * @retval  发送结果，成功返回TXDS,失败返回MAXRT或ERROR
  */
u8 NRF_Tx_data(u8 *Txbuf)
{
	u8 state;  

	 /*ce为低，进入待机模式1*/
	NRF_CE_LOW();

	/*写数据到TX BUF 最大 32个字节*/						
   NRF_Write_Buf(WR_TX_PLOAD,Txbuf,TX_PLOAD_WIDTH);

      /*CE为高，txbuf非空，发送数据包 */   
 	 NRF_CE_HIGH();
	  	
	  /*等待发送完成中断 */                            
	while(NRF_Read_IRQ()!=0); 	
	
	/*读取状态寄存器的值 */                              
	state = NRF_Read_Reg(STATUS);

	 /*清除TX_DS或MAX_RT中断标志*/                  
	NRF_Write_Reg(NRF_WRITE_REG+STATUS,state); 	

	NRF_Write_Reg(FLUSH_TX,NOP);    //清除TX FIFO寄存器 

	 /*判断中断类型*/    
	if(state&MAX_RT)                     //达到最大重发次数
			 return MAX_RT; 

	else if(state&TX_DS)                  //发送完成
		 	return TX_DS;
	 else						  
			return ERROR;                 //其他原因发送失败
} 

/**************************************************************************************
  * @brief   用于从NRF的接收缓冲区中读出数据
  * @param   
  *		@arg Rxbuf ：用于接收该数据的数组，外部定义	
  * @retval 
  *		@arg 接收结果
  */
u8 NRF_Rx_data(u8 *Rxbuf)
{
	u8 state; 
	NRF_CE_HIGH();	 //进入接收状态
	 /*等待接收中断*/
	while(NRF_Read_IRQ()==0)
  {
    NRF_CE_LOW();  	 //进入待机状态
   
		/*读取status寄存器的值  */               
		state=NRF_Read_Reg(STATUS);
		
		if((state>>1&0x07)!=0x00) 
			return ERROR;          //非P0数据，返回
  
		/* 清除中断标志*/      
   NRF_Write_Reg(NRF_WRITE_REG+STATUS,state);

    /*判断是否接收到数据*/
    if(state&RX_DR)                                 //接收到数据
    {
			 NRF_Read_Buf(RD_RX_PLOAD,Rxbuf,RX_PLOAD_WIDTH);//读取数据
			 NRF_Write_Reg(FLUSH_RX,NOP);          //清除RX FIFO寄存器
       return RX_DR; 
    }
    else    
			 return ERROR;                    //没收到任何数据
  }
  
  return ERROR;                    //没收到任何数据
}

u8 NRF_Rx_data_remote(u8 *Rxbuf)
{
	u8 state; 
	NRF_CE_HIGH();	 //进入接收状态
	 /*等待接收中断*/
	while(NRF_Read_IRQ()==0)
  {
    NRF_CE_LOW();  	 //进入待机状态
   
		/*读取status寄存器的值  */               
		state=NRF_Read_Reg(STATUS);
		
  	if((state>>1&0x07)!=1) 
		return ERROR;					//非P1数据，返回
			
		/* 清除中断标志*/      
   NRF_Write_Reg(NRF_WRITE_REG+STATUS,state);

    /*判断是否接收到数据*/
    if(state&RX_DR)                                 //接收到数据
    {
			 NRF_Read_Buf(RD_RX_PLOAD,Rxbuf,RX_PLOAD_WIDTH);//读取数据
			 NRF_Write_Reg(FLUSH_RX,NOP);          //清除RX FIFO寄存器
       return RX_DR; 
    }
    else    
			 return ERROR;                    //没收到任何数据
  }
  
  return ERROR;                    //没收到任何数据
}
/***********************************************************************************/
u8 NRF_Rx_string(char *Rxcharbuf)
{
	u8 state; 
	NRF_CE_HIGH();	 //进入接收状态

	while(NRF_Read_IRQ()==0)	 /*等待接收中断*/
  {
    NRF_CE_LOW();  	 //进入待机状态
   
	           
		state=NRF_Read_Reg(STATUS);	/*读取status寄存器的值  */    
  
		
   NRF_Write_Reg(NRF_WRITE_REG+STATUS,state);/* 清除中断标志*/      


    if(state&RX_DR)//接收到数据
    {
			 NRF_Read_StringBuf(RD_RX_PLOAD,Rxcharbuf,RX_PLOAD_WIDTH);//读取数据
			 NRF_Write_Reg(FLUSH_RX,NOP);          //清除RX FIFO寄存器
       return RX_DR; 
    }
    else//没收到任何数据    
			 return ERROR;                    
  }
  
  return ERROR;                    //没收到任何数据
}

/************************************************************************************/
u8 NRF_Tx_string(char *Txcharbuf)
{
	u8 state;  
	
	NRF_CE_LOW(); /*ce为低，进入待机模式1*/
						
  NRF_Write_StringBuf(WR_TX_PLOAD,Txcharbuf,TX_PLOAD_WIDTH);/*写数据到TX BUF 最大 32个字节*/	
        
 	NRF_CE_HIGH();/*CE为高，Txbuf非空，发送数据包 */ 
	  	                          
	while(NRF_Read_IRQ()!=0); 		  /*等待发送完成中断 */ 
                            
	state = NRF_Read_Reg(STATUS);	/*读取状态寄存器的值 */ 
	           
	NRF_Write_Reg(NRF_WRITE_REG+STATUS,state);	 /*清除TX_DS或MAX_RT中断标志*/   	

	NRF_Write_Reg(FLUSH_TX,NOP);    //清除TX FIFO寄存器 

	 /*判断中断类型*/    
	if(state&MAX_RT)                //达到最大重发次数
			 return MAX_RT; 

	else if(state&TX_DS)            //发送完成
		 	return TX_DS;
	
			else						  
			return ERROR;               //其他原因发送失败
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
	

