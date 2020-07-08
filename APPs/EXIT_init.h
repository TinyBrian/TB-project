#ifndef __EXIT_INIT_H
#define __EXIT_INIT_H


//#include "LCD_init.h"
#include "Delay.h"
#include "IIC_init.h"
#include "UART_init.h"
#include "stdlib.h"
#include "stdio.h"
#include "ADC_init.h"
#include "PWM_TIMx_init.h"
#include "NRF24L01.h" 
#include "MPUx_init.h" 
#include "MPU_DMP.h"
#include "Data_Type_Conv.h"  
#include "string.h"  
#include "NRF_USART_MODULE.h"
#include "stm32f4xx_it.h"
#include "MSysTick.h"

#define             EXTI_GPIO_CLK_MPU9250                       (RCC_AHB1Periph_GPIOA )     
#define             EXTI_GPIO_PORT_MPU9250                       GPIOA   
#define             EXTI_GPIO_PIN_MPU9250                       GPIO_Pin_15
#define             EXTI_SOURCE_PORT_MPU9250                     EXTI_PortSourceGPIOA
#define             EXTI_SOURCE_PIN_MPU9250                      GPIO_PinSource15
#define             EXTI_LINE_MPU9250                            EXTI_Line15
#define             EXTI_IRQ_MPU9250                             EXTI15_10_IRQn
#define             EXTI_INT_HANDLER_MPU9250                   EXTI15_10_IRQHandler

#define             EXTI_GPIO_CLK_Armed                        (RCC_AHB1Periph_GPIOC )     
#define             EXTI_GPIO_PORT_Armed                       GPIOC   
#define             EXTI_GPIO_PIN_Armed                        GPIO_Pin_0
#define             EXTI_SOURCE_PORT_Armed                     EXTI_PortSourceGPIOC
#define             EXTI_SOURCE_PIN_Armed                   	 EXTI_PinSource0
#define             EXTI_LINE_Armed                            EXTI_Line0
#define             EXTI_IRQ_Armed                             EXTI0_IRQn
#define             EXTI_INT_HANDLER_Armed                    EXTI0_IRQHandler

#define             EXTI_GPIO_CLK_NRF                        (RCC_AHB1Periph_GPIOB )     
#define             EXTI_GPIO_PORT_NRF                        GPIOB   
#define             EXTI_GPIO_PIN_NRF                         GPIO_Pin_8
#define             EXTI_SOURCE_PORT_NRF                     EXTI_PortSourceGPIOB
#define             EXTI_SOURCE_PIN_NRF                   	 EXTI_PinSource8
#define             EXTI_LINE_NRF                           EXTI_Line8
#define             EXTI_IRQ_NRF                             EXTI9_5_IRQn
#define             EXTI_INT_HANDLER_NRF                     EXTI9_5_IRQHandler




#define             EXTI_GPIO_CLK_Kx                       (RCC_AHB1Periph_GPIOE )     
#define             EXTI_GPIO_PORT_Kx                       GPIOE  
#define             EXTI_GPIO_PIN_Kx                        GPIO_Pin_4
#define             EXTI_SOURCE_PORT_Kx                     EXTI_PortSourceGPIOE
#define             EXTI_SOURCE_PIN_Kx                  	 EXTI_PinSource4
#define             EXTI_LINE_Kx                          EXTI_Line4
#define             EXTI_IRQ_Kx                           EXTI4_IRQn
#define             EXTI_INT_HANDLER_Kx                   EXTI4_IRQHandler



#define ENABLE_INV_INTERRUPTS  EnableInvInterrupt()
#define DISABLE_INV_INTERRUPTS DisableInvInterrupt()


void EXTI_init(void);
void EnableInvInterrupt(void);
void DisableInvInterrupt(void);
void EnableInvInterrupt(void);
void DisableInvInterrupt(void);

#endif

