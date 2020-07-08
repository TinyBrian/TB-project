#ifndef __UART_init_H
#define __UART_init_H


#include "stdio.h"
#include "Delay.h"

//Òý½Å¶¨Òå
/*******************************************************/
#define DEBUG_USARTx                            USART2

/****************/
#define DEBUG_USART                             USART2
#define DEBUG_USART_CLK                         RCC_APB1Periph_USART2

#define DEBUG_USART_RX_GPIO_PORT                GPIOA                    
#define DEBUG_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define DEBUG_USART_RX_PIN                      GPIO_Pin_3
#define DEBUG_USART_RX_AF                       GPIO_AF_USART2
#define DEBUG_USART_RX_SOURCE                   GPIO_PinSource3

#define DEBUG_USART_TX_GPIO_PORT                GPIOA
#define DEBUG_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define DEBUG_USART_TX_PIN                      GPIO_Pin_2
#define DEBUG_USART_TX_AF                       GPIO_AF_USART2
#define DEBUG_USART_TX_SOURCE                   GPIO_PinSource2

#define DEBUG_USART_IRQHandler                  USART2_IRQHandler
#define DEBUG_USART_IRQ                 				USART2_IRQn
/************************************************************/


void USART_init(u32); 

void USART_Send_byte(u8 ch);
void USART_Send_bytes(u8* ch,u8 Numbertosent);

void USART_SendString(char *str);

void USART_SendHalfWord(u16 ch);

#endif

