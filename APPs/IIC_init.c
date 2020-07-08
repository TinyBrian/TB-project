#include "IIC_init.h" 


//stm32f10x
//PB6--SCL--TIM4_CH1
//PB7--SDA--TIM4_CH2


//PB10--SCL--USART_TX
//PB11--SDA--USART_RX

//Soft_I2C_SDA /I2C2-SDA		GPIOB_Pin_11
//Soft_I2C_SCL /I2C2-SCL		GPIOB_Pin_10


//手头带IIC的LCD1602 设备地址 0x3f (0x0111 111 )（出厂默认，只有3位能改）(100000Hz)
//                     写地址 0x7E (0x0111 1110)
//                     读地址 0x7F (0x0111 1111)


//手头带IIC的MPU6050 设备地址 0x68 (0x0111 111 )（A0=0）(400000Hz)
//                     写地址 0x7E (0x0111 1110)
//                     读地址 0x7F (0x0111 1111)


	#define mdelay Delay_ms
// 硬件IIC
#if(HardwareIIC)

static __IO uint32_t  I2CTimeout = I2CT_LONG_TIMEOUT; 
/*************************************************************************************/
//Master receive
u8 IIC_Read_byte(u8 address)
{		
		u32 data_r;
		
		I2C_GenerateSTART(I2Cx, ENABLE);
		
		EV5(I2CTimeout);

		I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Receiver);
		
		EV6_Reciever(I2CTimeout);
	 
		I2C_AcknowledgeConfig(I2Cx, DISABLE);
				
		I2C_GenerateSTOP(I2Cx, ENABLE);
			
		EV7(I2CTimeout);
			 
		I2C_AcknowledgeConfig(I2Cx, ENABLE);
		
		return data_r; //正常，返回1
}



ErrorStatus IIC_Read_bytes(u8 address,u8 * pdata_r,u8 Numbertoread)
{		
		

		I2C_GenerateSTART(I2Cx, ENABLE);
		
		EV5(I2CTimeout);
			
		I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Receiver);
		
  	EV6_Reciever(I2CTimeout);
		
		while(Numbertoread)  	                                                    
		{	
			if(Numbertoread ==1)	//NACK 
			{
				I2C_AcknowledgeConfig(I2Cx, DISABLE); 
				
				I2C_GenerateSTOP(I2Cx, ENABLE);
				
				EV7(I2CTimeout);
				
				*pdata_r = I2C_ReceiveData(I2Cx);		
				 Numbertoread--; 		
			}		
			else 
			{ 
				EV7(I2CTimeout);
				
				*pdata_r++=I2C_ReceiveData(I2Cx);
				
				Numbertoread--;        
			}   
		}
		
		I2C_GenerateSTOP(I2Cx, ENABLE);
		
		I2C_AcknowledgeConfig(I2Cx, ENABLE); 
		
		return SUCCESS; //正常，返回1
}


ErrorStatus IIC_Read_register_bytes(u8 address,u8 reg,u8 * pdata_r,u8 Numbertoread)
{		
		I2C_GenerateSTART(I2Cx, ENABLE);
	
		EV5(I2CTimeout);
  
  	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Transmitter);
			
		EV6_Transmiter(I2CTimeout);
	
		I2C_SendData(I2Cx, reg); 
		
		EV8(I2CTimeout);
		
		EV8_2(I2CTimeout);

		I2C_GenerateSTART(I2Cx, ENABLE);
		
		EV5(I2CTimeout);
			
		I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Receiver);

		EV6_Reciever(I2CTimeout);
			
		while(Numbertoread)  	                                                    
		{	
			if(Numbertoread ==1)	//NACK 
			{
				I2C_AcknowledgeConfig(I2Cx, DISABLE); 
				
				I2C_GenerateSTOP(I2Cx, ENABLE);
				
				EV7(I2CTimeout);
				
				*pdata_r = I2C_ReceiveData(I2Cx);		
				 Numbertoread--; 		
			}		
			else 
			{ 
				EV7(I2CTimeout);
				
				*pdata_r++=I2C_ReceiveData(I2Cx);
				
				Numbertoread--;        
			}   
		}
		
		I2C_GenerateSTOP(I2Cx, ENABLE);
		
		I2C_AcknowledgeConfig(I2Cx, ENABLE); 
		
		return SUCCESS; //正常，返回1

}
/**************************************************************************************/
//Master sent
ErrorStatus IIC_Send_register_byte(u8 address,u8 reg,u8 pdata)
{ 

	I2C_GenerateSTART(I2Cx, ENABLE);
	
	EV5(I2CTimeout);

	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Transmitter);

	EV6_Transmiter(I2CTimeout);

  I2C_SendData(I2Cx, reg); 
	
	EV8(I2CTimeout);
	
	I2C_SendData(I2Cx, pdata); 
	
	EV8_2(I2CTimeout);
 
  I2C_GenerateSTOP(I2Cx, ENABLE);
	
	return SUCCESS; //正常返回1
		
}

ErrorStatus IIC_Send_register_bytes(u8 address,u8 reg,const u8* pdata,u8 Numbertosent)
{ 


	I2C_GenerateSTART(I2Cx, ENABLE); 
	
	if(EV5(I2CTimeout)==ERROR)
		{IIC_Timeout_error_hanlder();return ERROR;}

	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Transmitter);

	if(EV6_Transmiter(I2CTimeout)==ERROR)
		{IIC_Timeout_error_hanlder();return ERROR;}
 
	I2C_SendData(I2Cx, reg); 
	
	if(EV8(I2CTimeout)==ERROR)
		{IIC_Timeout_error_hanlder();return ERROR;}
	
	while(Numbertosent)
	{
		I2C_SendData(I2Cx, *pdata++); 
		
		if(Numbertosent==1){
				if(EV8_2(I2CTimeout)==ERROR)
					{IIC_Timeout_error_hanlder();return ERROR;}}				
	
		else{	
				if(EV8(I2CTimeout)==ERROR)
					{IIC_Timeout_error_hanlder();return ERROR;}}
		
		Numbertosent--;
		
	}
	
  I2C_GenerateSTOP(I2Cx, ENABLE);
	 
	return SUCCESS; //正常返回1
		
}


ErrorStatus IIC_Send_bytes(u8 address,u8* pdata,u8 Numbertosent)
{	
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	EV5(I2CTimeout);
  
	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Transmitter);
	
	EV6_Transmiter(I2CTimeout);
	
	I2C_SendData(I2Cx, *pdata++); 

	while(Numbertosent)
	{		
		if(Numbertosent==1)
			EV8_2(I2CTimeout);
		else
	{
			EV8(I2CTimeout);
			I2C_SendData(I2Cx, *pdata++);
	}
	
		Numbertosent--;
	}	

	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	return SUCCESS; //正常返回1
}




void IIC_init(u8 ownaddr,u32 speed)
{
  I2C_DeInit(I2Cx);
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitStructure;

  RCC_APB1PeriphClockCmd(I2C_RCC_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(I2C_SCL_GPIO_CLK | I2C_SDA_GPIO_CLK, ENABLE);
	
	GPIO_PinAFConfig(I2C_SCL_GPIO_PORT,I2C_SCL_GPIO_PINSOURCE,I2C_AF);
	GPIO_PinAFConfig(I2C_SDA_GPIO_PORT,I2C_SDA_GPIO_PINSOURCE,I2C_AF);
	

//SCL
  GPIO_InitStructure.GPIO_Pin =  I2C_SCL_GPIO_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_Init(I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

//SDA
  GPIO_InitStructure.GPIO_Pin = I2C_SDA_GPIO_PIN; 
		GPIO_Init(I2C_SDA_GPIO_PORT, &GPIO_InitStructure);  
  

  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = ownaddr;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = speed;
    I2C_Init(I2Cx, &I2C_InitStructure);

  I2C_Cmd(I2Cx, ENABLE);  
   

//RESET LOCK BREAK	
		if(I2C_GetFlagStatus(I2Cx,I2C_FLAG_BUSY)==SET)
			IIC_Timeout_error_hanlder();   
}

	
#if(MASTER) //EVENT()//4-8

ErrorStatus EV5(u32 I2CTimeout)
{ 	
			I2CTimeout = I2CT_FLAG_TIMEOUT;/* EV5  */
			while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)==RESET)	
			{
					if((I2CTimeout--) == 0) 
					{
						return ERROR;
					}
			}
			 return SUCCESS;
}

ErrorStatus EV6_Reciever(u32 I2CTimeout)
{                                                                                                                                          
		I2CTimeout = I2CT_FLAG_TIMEOUT;	/* EV6  */
		while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)==RESET)			
		 {
					if((I2CTimeout--) == 0) 
					{
						return ERROR;
					}
		 }
		 return SUCCESS;
}

ErrorStatus EV6_Transmiter(u32 I2CTimeout)
{                                                                                                                                          
		I2CTimeout = I2CT_FLAG_TIMEOUT;	/* EV6  */
		while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==RESET)				
		 {
					if((I2CTimeout--) == 0) 
					{
						return ERROR;
					}
		 }
		 return SUCCESS;
}

ErrorStatus EV7(u32 I2CTimeout)
{ 	I2CTimeout = I2CT_FLAG_TIMEOUT;
		while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)==RESET) 
   	{			if((I2CTimeout--) == 0) 
					{
						return ERROR;
					}
		}return SUCCESS;
}

ErrorStatus EV8(u32 I2CTimeout)
{ 	I2CTimeout = I2CT_FLAG_TIMEOUT;
		while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING)==RESET)
			{
					if((I2CTimeout--) == 0) 
					{
						
						return ERROR;
					}
			}return SUCCESS;
}

ErrorStatus EV8_2(u32 I2CTimeout)
{ 	I2CTimeout = I2CT_FLAG_TIMEOUT;
		while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)==RESET)
			{
					if((I2CTimeout--) == 0) 
					{
						
						return ERROR;
					}
			}return SUCCESS;
}

BusyStatus Check_IIC_busy(u32 I2CTimeout)
{		
	
		while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) 
		{	
			I2C_Cmd(I2Cx, ENABLE);
			I2C_GenerateSTOP(I2Cx, ENABLE);
			return BUSY;
		}	
		 return FREE;
}

ACKStatus Check_IIC_NACK_Error()
{ 

	if(I2C_GetFlagStatus(I2Cx,I2C_FLAG_AF)==SET)//send address <-NACK
	{	
		 I2C_GenerateSTOP(I2Cx, ENABLE);
		return NACK;
	}
	
	return ACK;
	
} 
#endif //EVENT()4-8   MASTER




ErrorStatus IIC_Timeout_error_hanlder()
{		

	GPIO_InitTypeDef GPIO_Initstructure;

		if(I2C_GetFlagStatus(I2Cx,I2C_FLAG_BUSY)==SET)
	{
		GPIO_Initstructure.GPIO_Mode=GPIO_Mode_OUT;
		GPIO_Initstructure.GPIO_OType=GPIO_OType_PP;
		GPIO_Initstructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_Initstructure.GPIO_Pin=I2C_SCL_GPIO_PIN;
		GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
			GPIO_Init(I2C_SCL_GPIO_PORT,&GPIO_Initstructure);
		
		for(int i=0;i<10;i++)
		{	
			GPIO_WriteBit(I2C_SCL_GPIO_PORT,I2C_SCL_GPIO_PIN,Bit_RESET);
			Delay_us(100);	
			GPIO_WriteBit(I2C_SCL_GPIO_PORT,I2C_SCL_GPIO_PIN,Bit_SET);
			Delay_us(100);
		}
		
		GPIO_Initstructure.GPIO_Mode=GPIO_Mode_AF;
		GPIO_Initstructure.GPIO_OType=GPIO_OType_OD;
		GPIO_Initstructure.GPIO_PuPd=GPIO_PuPd_UP;
			GPIO_Init(I2C_SCL_GPIO_PORT,&GPIO_Initstructure);	
			GPIO_PinAFConfig(I2C_SCL_GPIO_PORT,I2C_SCL_GPIO_PINSOURCE,I2C_AF);
	} 
	
		return SUCCESS;
}







#endif //HardwareIIC





#if(1) //MPU6050_DMP
/********************************* Prototypes *********************************/
unsigned long ST_Hard_Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue);
unsigned long ST_Hard_Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);
/*******************************  Function ************************************/


#define ST_Sensors_I2C_WriteRegister  ST_Hard_Sensors_I2C_WriteRegister
#define ST_Sensors_I2C_ReadRegister ST_Hard_Sensors_I2C_ReadRegister


unsigned long ST_Hard_Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
	 if(IIC_Send_register_bytes(Address<<1,RegisterAddr,RegisterValue,RegisterLen)==ERROR)    
		return 1;
	return 0;
}
 

unsigned long ST_Hard_Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
	if(IIC_Read_register_bytes(Address<<1,RegisterAddr,RegisterValue,RegisterLen)==ERROR)
		return 1;
	return 0;
}


  
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len,
                                        const unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryWriteAgain:
  ret = 0;
  ret = ST_Sensors_I2C_WriteRegister( slave_addr, reg_addr, len, ( unsigned char *)data_ptr);

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;

    mdelay(retry_in_mlsec); 
    goto tryWriteAgain;
  }
  return ret;
}


/**
  * @brief  向IIC设备的寄存器连续读出数据,带超时重试设置，供mpu接口调用
  * @param  Address: IIC设备地址
  * @param  RegisterAddr: 寄存器地址
  * @param  RegisterLen: 要读取的数据长度
  * @param  RegisterValue: 指向存储读出数据的指针
  * @retval 0正常，非0异常
  */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len,
                                       unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryReadAgain:
  ret = 0;
  ret = ST_Sensors_I2C_ReadRegister( slave_addr, reg_addr, len, ( unsigned char *)data_ptr);

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;

    mdelay(retry_in_mlsec);
    goto tryReadAgain;
  }
  return ret;
}


static unsigned short RETRY_IN_MLSEC  = 55;

/**
  * @brief  设置iic重试时间
  * @param  ml_sec：重试的时间，单位毫秒
  * @retval 重试的时间，单位毫秒
  */

void Set_I2C_Retry(unsigned short ml_sec)
{
  RETRY_IN_MLSEC = ml_sec;
}

/**
  * @brief  获取设置的iic重试时间
  * @param  none
  * @retval none
  */
unsigned short Get_I2C_Retry(void)
{
  return RETRY_IN_MLSEC;
}

#endif //MPU6050_DMP




#if(SimuIIC)


#define Soft_I2C_DELAY 				Soft_I2C_Delay(200000)
#define Soft_I2C_NOP					Soft_I2C_Delay(100) //168M 20-250
//  
#define Soft_I2C_READY		0x00
#define Soft_I2C_BUS_BUSY	0x01	
#define Soft_I2C_BUS_ERROR	0x02
//
#define Soft_I2C_NACK	  0x00 
#define Soft_I2C_ACK		0x01
static u8 IIC_start(void);				//发送IIC开始信号
static void IIC_stop(void);	  			//发送IIC停止信号
static void IIC_Ack(void);					//IIC发送ACK信号
static void IIC_NAck(void);				//IIC不发送ACK信号
static u8 IIC_wait_Ack(void); 				//IIC等待ACK信号

static void Soft_I2C_Delay(uint32_t dly) 
{	
	while(--dly);	//dly=23: 8.75us; dly=100: 85.58 us (SYSCLK=168MHz)
}


//初始化IIC
void IIC_init(u8 period,u32 prescaler)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(	RCC_AHB1Periph_GPIOB, ENABLE );	//使能GPIOB时钟
	    
	GPIO_InitStructure.GPIO_Pin = Soft_I2C_SCL | Soft_I2C_SDA;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=	GPIO_OType_OD;																//开漏输出GPIO_Mode_Out_OD
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(Soft_I2C_PORT, &GPIO_InitStructure);
	
	
	Soft_I2C_SCL_1; 
	Soft_I2C_SDA_1;
	Soft_I2C_DELAY;
	
	
}


static u8 IIC_start(void)
{
	Soft_I2C_SDA_1; 
 	Soft_I2C_NOP;
  // 
 	Soft_I2C_SCL_1; 
 	Soft_I2C_NOP;    
	//
 	if(!Soft_I2C_SDA_STATE) return Soft_I2C_BUS_BUSY;
	//
 	Soft_I2C_SDA_0;
 	Soft_I2C_NOP;
  //
 	Soft_I2C_SCL_0;  
 	Soft_I2C_NOP; 
	//
 	if(Soft_I2C_SDA_STATE) return Soft_I2C_BUS_ERROR;
	
	return Soft_I2C_READY;
}	  

static void IIC_stop(void)
{
 	Soft_I2C_SDA_0; 
 	Soft_I2C_NOP;
  // 
 	Soft_I2C_SCL_1; 
 	Soft_I2C_NOP;    
	//
 	Soft_I2C_SDA_1;
 	Soft_I2C_NOP;
}

static u8 IIC_wait_Ack()
{
	u16 ucErrTime=0;

	Soft_I2C_SDA_1;
	Soft_I2C_NOP;	   
	Soft_I2C_SCL_1;
	Soft_I2C_NOP;	 
	
  while(Soft_I2C_SDA_STATE)                                                                                                                                                                       	while(Soft_I2C_SDA_STATE)
	{
		ucErrTime++;
		if(ucErrTime>575)
		{
			IIC_stop();
			return Soft_I2C_BUS_ERROR;
		}
	}
	Soft_I2C_SCL_0;  

	return 0;  
} 


static void IIC_Ack(void)
{
 	Soft_I2C_SDA_0;
 	Soft_I2C_NOP;
 	Soft_I2C_SCL_1;
 	Soft_I2C_NOP;
 	Soft_I2C_SCL_0; 
 	Soft_I2C_NOP;  
}
    
static void IIC_NAck(void)
{
	Soft_I2C_SDA_1;
	Soft_I2C_NOP;
	Soft_I2C_SCL_1;
	Soft_I2C_NOP;
	Soft_I2C_SCL_0; 
	Soft_I2C_NOP;  
}					 				     

u8 IIC_send_7bitaddress(u8 address,u8 direction)
{
		u8 i; 	
		assert_param(IS_Direction(DIR));

			if(direction==ADDR_W)  
			 address&=0xfe;
			 else
			 address|=0x01;
			 
				Soft_I2C_SCL_0;
				
			 for(i=0;i<8;i++)
				{  
						if(address&0x80) 
							Soft_I2C_SDA_1;
						else 
							Soft_I2C_SDA_0;
						//
						address<<=1;
						Soft_I2C_NOP;
						//
						Soft_I2C_SCL_1;
						Soft_I2C_NOP;
						Soft_I2C_SCL_0;
						Soft_I2C_NOP; 
				}
				
			if(IIC_wait_Ack())
				return 1;
				
	return 0;		
			
}


u8 IIC_Send_data(u8* pdata_s)
{ 
 	uint8_t i;
 	
	Soft_I2C_SCL_0;
 	for(i=0;i<8;i++)
 	{  
  		if(*pdata_s&0x80) 
				Soft_I2C_SDA_1;
   		else 
				Soft_I2C_SDA_0;
  		
			*pdata_s<<=1;
  		Soft_I2C_NOP;
  		Soft_I2C_SCL_1;
  		Soft_I2C_NOP;
  		Soft_I2C_SCL_0;
  		Soft_I2C_NOP; 
 	}
			if(IIC_wait_Ack())
				return 1;
	return 0;
} 
u8 IIC_Send_data_t(const u8* pdata_s)
{ 
		uint8_t i;
		volatile u8 ppdata=*pdata_s;
	
 	for(i=0;i<8;i++)
 	{  
  		if(ppdata&0x80) 
				Soft_I2C_SDA_1;
   		else 
				Soft_I2C_SDA_0;

  		ppdata<<=1;
  		Soft_I2C_NOP;
  		Soft_I2C_SCL_1;
  		Soft_I2C_NOP;
  		Soft_I2C_SCL_0;
  		Soft_I2C_NOP; 
 	}
	
				if(IIC_wait_Ack())
				return 1;
	return 0;
}


u8 IIC_Read_data()
{ 
	uint8_t i,data=0;
	
	//
 	Soft_I2C_SDA_1;
 	Soft_I2C_SCL_0; 
	//
	
 	for(i=0;i<8;i++)
 	{
  		Soft_I2C_SCL_1;
  		Soft_I2C_NOP; 
  	
			data<<=1;
		
  		if(Soft_I2C_SDA_STATE)	
				data|=0x01; 
  
  		Soft_I2C_SCL_0;  
  		Soft_I2C_NOP;         
 	}
	
 	return data;

} 	



ErrorStatus IIC_Send_bytes(u8 address,u8* pdata_s,u8 Numbertosent)
{ 
		IIC_start();
	
		IIC_send_7bitaddress(address,ADDR_W);
	
		while(Numbertosent)
			{
					IIC_Send_data(pdata_s++);
												
					Numbertosent--;
					
			}		
			
		IIC_stop();
		
			return SUCCESS;
				
} 	

//读1个字节
u8 IIC_Read_byte(u8 address)
{	
		IIC_start();
		
		IIC_send_7bitaddress(address,ADDR_R);
			
		u8 receive=IIC_Read_data();			 
    
    IIC_NAck();//发送nACK
 	
		IIC_stop();
		
		return receive;

}

ErrorStatus IIC_Read_bytes(u8 address,u8* data_r,u8 Numbertoread)
{		
		IIC_start();
	
		IIC_send_7bitaddress(address,ADDR_R);
	
		while(Numbertoread)
		{
				*data_r++=IIC_Read_data();
	
				if(Numbertoread==1)
				{
						IIC_NAck();
						Numbertoread--;
				}
					else
					{
						IIC_Ack();	
						Numbertoread--;
					}		
		}	
 	
		IIC_stop();
	

		return SUCCESS;
			
}

//IIC_Read_register_bytes(Address<<1,RegisterAddr,RegisterValue,RegisterLen)==ERROR
//IIC_Send_register_bytes(Address<<1,RegisterAddr,RegisterValue,RegisterLen)==ERROR  
//ErrorStatus IIC_Send_bytes(u8 address,u8* Pdata,u8 Numbertosent);
//ErrorStatus IIC_Read_bytes(u8 address,u8* pdata,u8 Numbertoread);



ErrorStatus IIC_Send_register_byte(u8 address,u8 reg,u8 pdata)
{
	IIC_start();
	
		IIC_send_7bitaddress(address,ADDR_W);
	
		IIC_Send_data(&reg);
				
		IIC_Send_data(&pdata); 
	
		IIC_stop();
	
	return SUCCESS;
}






ErrorStatus IIC_Send_register_bytes(u8 address,u8 reg,const u8* Pdata,u8 Numbertosent)
{
	
		IIC_start();
	
		IIC_send_7bitaddress(address,ADDR_W);
		
		IIC_Send_data(&reg);
				
		while(Numbertosent)
		{
				IIC_Send_data_t(Pdata++); 
		
				Numbertosent--;
		}		
 	
		IIC_stop();
	
	
	return SUCCESS;
}


ErrorStatus IIC_Read_register_bytes(u8 address,u8 reg,u8 * pdata_r,u8 Numbertoread)
{
		IIC_start();
	
		IIC_send_7bitaddress(address,ADDR_W);
		
		IIC_Send_data(&reg);			
		
		IIC_start();
	
		IIC_send_7bitaddress(address,ADDR_R);
							
		while(Numbertoread)
		{
				*pdata_r++=IIC_Read_data();
	
				if(Numbertoread==1)
				{
						IIC_NAck();
						Numbertoread--;
				}
					else
					{
						IIC_Ack();	
						Numbertoread--;
					}		
		}
		
		IIC_stop();
	
	
	return SUCCESS;
}



 

	
#if(0)   //orignal function
//void IIC_send_7bitaddress_t(u8 address)
//{
//			 
//				u8 t; 

//			IIC_SCL=0;//拉低时钟开始数据传输

//			for(t=0;t<8;t++)
//				{              				 
//						if((address&0x80)>>7)
//							IIC_SDA=1;
//						else
//							IIC_SDA=0;
//						
//						 	  
//						Delay_us(2); 
//						
//						IIC_SCL=1;
//						Delay_us(2); 
//						
//						IIC_SCL=0;	
//						Delay_us(2);
//				}
//			
//}
#endif //orignal function




#endif  //simulation





#if(0) //old function
//old FUNCTION AREA
/***************************************************************************************/

//			I2C_GenerateSTART(I2C1, ENABLE);
//			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
//			
//			I2C_AcknowledgeConfig(I2C1, DISABLE); //core !!!!!!!!
//		
//			I2C_Send7bitAddress  ( I2C1,address, I2C_Direction_Transmitter);
//			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//		
//			I2C_SendData  ( I2C1, reg);
//			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTING));
//		
//			I2C_GenerateSTART(I2C1, ENABLE);      //restart
//			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); //ev5
//			
//			I2C_Send7bitAddress  ( I2C1,address, I2C_Direction_Receiver);
//			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); //ev6
//			
//			I2C_GenerateSTOP(I2C1, ENABLE);
//			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); //ev7


ErrorStatus IIC_Read_register_bytes(u8 address,u8 reg,u8 * pdata_r,u8 Numbertoread)
{		
		
		
		I2C_GenerateSTART(I2Cx, ENABLE);
	
		EV5(I2CTimeout);
  
  	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Transmitter);
			
		EV6_Transmiter(I2CTimeout);
	
		I2C_SendData(I2Cx, reg); 
		
		EV8(I2CTimeout);
		
		EV8_2(I2CTimeout);

		I2C_GenerateSTART(I2Cx, ENABLE);
		
		EV5(I2CTimeout);
			
		I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Receiver);

		EV6_Reciever(I2CTimeout);
		
	  I2C_AcknowledgeConfig(I2Cx, DISABLE); //
	
		I2C_GenerateSTOP(I2C1, ENABLE);
		
		EV7(I2CTimeout);
	
		while(Numbertoread)  	                                                    
		{	
			if(Numbertoread ==1)	//NACK 
			{
				*pdata_r = I2C_ReceiveData(I2Cx);		
				 Numbertoread--; 		
			}		
			else 
			{  
				*pdata_r++=I2C_ReceiveData(I2Cx);
				Numbertoread--;        
			}   
		}
	
		I2C_GenerateSTOP(I2Cx, ENABLE);
		
		return SUCCESS; //正常，返回1

}

ErrorStatus IIC_Send_bytes(u8 address,u8* pdata,u8 Numbertosent)
{	
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	EV5(I2CTimeout);
  
	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Transmitter);
	
	EV6_Transmiter(I2CTimeout);
	
	while(Numbertosent)
	{	
		I2C_SendData(I2Cx, *pdata); 
		
		if(Numbertosent==1)
			EV8_2(I2CTimeout);
		else
			EV8(I2CTimeout);
			
		Numbertosent--;
		pdata++;
	}	

	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	return SUCCESS; //正常返回1
}*/




#endif //old FUNCTION AREA

