#ifndef _MAIN_H
#define _MAIN_H


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
#include "EXIT_init.h"
#include "MSysTick.h"

#define TASK_ENABLE 0


/* Private typedef -----------------------------------------------------------*/
/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */                                                                                                                                                             
#define DEFAULT_MPU_HZ  (40) //10 20 40 50 100

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800) 

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)




/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* ---------------------------------------------------------------------------*/



u8 ADC_Data_process(u16 fliter,u16 pre);
void PWM_Generate(u16 d);
void System_reset(void);

#endif //_MAIN_H

