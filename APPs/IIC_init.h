#ifndef __IIC_init_H
#define __IIC_init_H

#include "main.h"
#include "Delay.h"
#include "Bit_ADDR.h"
#include "MSysTick.h"
#include "Type_Define_self.h"


#define SimuIIC 1             
#define HardwareIIC 0  //换成硬件时要插播一次才正常不知道为什么

#define MASTER 1
#define SLAVE 0

typedef enum {ADDR_R = 1, ADDR_W = !ADDR_R} Direction;
typedef enum {ACK = 1, NACK = !ACK}ACKStatus;

#define IS_Direction(DIR) (((DIR) == ADDR_W) || ((ADDR_R) == ENABLE))
  
#if(HardwareIIC)  
//I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED :							  EV1 
//I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED :									EV1 
//I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED :					EV1 
//I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED : 						EV1 
//I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED : 								EV1

//I2C_EVENT_SLAVE_BYTE_RECEIVED : 														EV2 
//(I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_DUALF) :					EV2 
//(I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_GENCALL) : 				EV2

//I2C_EVENT_SLAVE_BYTE_TRANSMITTED :													EV3 
//(I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_DUALF) : 			EV3 
//(I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_GENCALL) :			EV3 
//I2C_EVENT_SLAVE_ACK_FAILURE : 															EV3_2 

//I2C_EVENT_SLAVE_STOP_DETECTED : 														EV4 

//I2C_EVENT_MASTER_MODE_SELECT : 															EV5

//I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED :							  EV6 
//I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED :										EV6 

//I2C_EVENT_MASTER_BYTE_RECEIVED : 														EV7 

//I2C_EVENT_MASTER_BYTE_TRANSMITTING : 												EV8 
//I2C_EVENT_MASTER_BYTE_TRANSMITTED :													EV8_2 

//I2C_EVENT_MASTER_MODE_ADDRESS10 : 													EV9 
#define I2Cx  										I2C2         
#define I2C_RCC_CLK               RCC_APB1Periph_I2C2
#define I2C_AF                    GPIO_AF_I2C2

#define I2C_SCL_GPIO_PORT         GPIOB
#define I2C_SCL_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define I2C_SCL_GPIO_PIN          GPIO_Pin_10
#define I2C_SCL_GPIO_PINSOURCE    GPIO_PinSource10
 
#define I2C_SDA_GPIO_PORT         GPIOB
#define I2C_SDA_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define I2C_SDA_GPIO_PIN          GPIO_Pin_11
#define I2C_SDA_GPIO_PINSOURCE    GPIO_PinSource11



			
//Pin
       


/*等待超时时间*/                       
#define I2CT_FLAG_TIMEOUT         ((uint32_t)0x168000) //1ms
#define I2CT_LONG_TIMEOUT         ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))
/**************************************************************************************/
#if(MASTER) //EVENT()//4-8

ErrorStatus EV5(u32 I2CTimeout);

ErrorStatus EV6_Reciever(u32 I2CTimeout);
ErrorStatus EV6_Transmiter(u32 I2CTimeout);

ErrorStatus EV7(u32 I2CTimeout);

ErrorStatus EV8(u32 I2CTimeout);

ErrorStatus EV8_2(u32 I2CTimeout);

BusyStatus Check_IIC_busy(u32 I2CTimeout);

ACKStatus Check_IIC_NACK_Error(void);
#endif
/*************************************************************************************************/



void IIC_init(u8,u32);              

u8 IIC_Read_byte(u8 address);

ErrorStatus IIC_Send_register_byte(u8 address,u8 reg,u8 pdata);

/****/
ErrorStatus IIC_Send_register_bytes(u8 address,u8 reg,const u8* Pdata,u8 Numbertosent);
ErrorStatus IIC_Read_register_bytes(u8 address,u8 reg,u8 * pdata_r,u8 Numbertoread);
/****/

ErrorStatus IIC_Send_bytes(u8 address,u8* Pdata,u8 Numbertosent);
ErrorStatus IIC_Read_bytes(u8 address,u8* pdata,u8 Numbertoread);

void Check_for_busy(u8);
ErrorStatus IIC_Timeout_error_hanlder(void );
#endif

#if(SimuIIC)  	

/**************************************************************/
//
#define Soft_I2C_SDA 		GPIO_Pin_11
#define Soft_I2C_SCL 		GPIO_Pin_10
#define Soft_I2C_PORT   GPIOB
//
#define Soft_I2C_SCL_0 		GPIO_ResetBits(Soft_I2C_PORT, Soft_I2C_SCL)
#define Soft_I2C_SCL_1 		GPIO_SetBits(Soft_I2C_PORT, Soft_I2C_SCL)
#define Soft_I2C_SDA_0 		GPIO_ResetBits(Soft_I2C_PORT, Soft_I2C_SDA)
#define Soft_I2C_SDA_1   	GPIO_SetBits(Soft_I2C_PORT, Soft_I2C_SDA)
#define Soft_I2C_SDA_STATE   	GPIO_ReadInputDataBit(Soft_I2C_PORT, Soft_I2C_SDA)


//IIC所有操作函数做兼容官方标准库处理
/***********************************************************/



void IIC_init(u8,u32);       //初始化IIC的IO口				 

u8 IIC_send_7bitaddress(u8 address,u8 direction);
u8 IIC_Send_data(u8* pdata_s);
u8 IIC_Send_data_t(const u8* pdata_s);
u8 IIC_Read_data(void);




/***********************************************************/


u8 IIC_Read_byte(u8 address);//IIC读取一个字节
ErrorStatus IIC_Send_register_byte(u8 address,u8 reg,u8 pdata);

ErrorStatus IIC_Send_bytes(u8 address,u8* Pdata,u8 Numbertosent);
ErrorStatus IIC_Read_bytes(u8 address,u8* pdata,u8 Numbertoread);

ErrorStatus IIC_Send_register_bytes(u8 address,u8 reg,const u8* Pdata,u8 Numbertosent);
ErrorStatus IIC_Read_register_bytes(u8 address,u8 reg,u8 * pdata_r,u8 Numbertoread);



/************************************************/
//test function area


/****************************************************************************/
//original function area
void IIC_send_7bitaddress_t(u8 address);





#endif //SimuIIC




void I2C_Bus_Init(void);

uint8_t I2C_ByteWrite(uint8_t pBuffer, uint8_t WriteAddr);

uint8_t I2C_BufferRead(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
																					
void Set_I2C_Retry(unsigned short ml_sec);

unsigned short Get_I2C_Retry(void);

int Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, 
                      unsigned short RegisterLen, unsigned char *RegisterValue);

int Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, 
                      unsigned short RegisterLen, const unsigned char *RegisterValue);
																					


#endif //__IIC_init_H
