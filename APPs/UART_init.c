#include "UART_init.h"

//USART1---TXD --PA9                           
//USART1---RTD --PA10-----APB2

//USART2---TXD --PA2
//USART2---RTD --PA3------APB1

//USART3---TXD --PB10
//USART3---RXD --PB11-----APB1
	


void USART_init(u32 BAUDRATE)  
{	
//	NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
		
  RCC_AHB1PeriphClockCmd(DEBUG_USART_RX_GPIO_CLK|DEBUG_USART_TX_GPIO_CLK,ENABLE);
  RCC_APB1PeriphClockCmd(DEBUG_USART_CLK, ENABLE);
  
  /* GPIO初始化 */	
	GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT,DEBUG_USART_RX_SOURCE,DEBUG_USART_RX_AF);
  GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT,DEBUG_USART_TX_SOURCE,DEBUG_USART_TX_AF);
	
	
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(DEBUG_USART_RX_GPIO_PORT , &GPIO_InitStructure);  

  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
		GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);


//8+1+NO
  USART_InitStructure.USART_BaudRate = BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(DEBUG_USART, &USART_InitStructure); 
	
//  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);
	
//	USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);
	
  USART_Cmd(DEBUG_USART, ENABLE);
}
	

void USART_Send_byte(u8 ch)
{

	USART_SendData(DEBUG_USART,ch);
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
	
}


void USART_Send_bytes(u8* ch,u8 Numbertosent)
{
	do
	{
		USART_SendData(DEBUG_USART,*ch++);
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
		Numbertosent--;
	}while(Numbertosent==1);
}

void USART_SendString(char *str)
{
	unsigned int k=0;
  do 
  {
      USART_Send_byte(*(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  while(USART_GetFlagStatus(DEBUG_USART,USART_FLAG_TC)==RESET);
}


void USART_SendHalfWord(u16 ch)
{
	uint8_t temp_h, temp_l;
	
	temp_h = (ch&0XFF00)>>8;
	temp_l = ch&0XFF;
	
	USART_SendData(DEBUG_USART,temp_h);	
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
	
	USART_SendData(DEBUG_USART,temp_l);	
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);	
}

//重定向printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		USART_SendData(DEBUG_USART,(uint8_t) ch);
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);		
		return (ch);
}

//重定向scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) == RESET);
		return (int)USART_ReceiveData(DEBUG_USART);
}



