#ifndef __NRF_USART_MOUDULE_H
#define __NRF_USART_MOUDULE_H

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
#include "Data_Type_Conv.h"  
#include "string.h"  

void NRF_Datai_pc_ascll(u8 data,u8 Bytestoreceive);
void NRF_Datac_pc_ascll(char* pdata,u8 Bytestoreceive);
void NRF_Dataf_pc_ascll(float data,u8 Bytestoreceive);

void NRF_MPU_Gyrdata_c(int16_t *Gyro);  

void NRF_MPU6050_Tempdata_c(float TEMPdata);  
void NRF_MPU6050_euler(float roll ,float pitch,float yaw);   

void NRF_USART_Send_char(u8*,u8);
void NRF_USART_Send_gyro(char* outdata,u8 length);

void NRF_USART_Read_char(u8* indata);




#endif //__NRF_USART_MOUDULE_H


