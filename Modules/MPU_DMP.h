#ifndef _MPU_DMP_H
#define _MPU_DMP_H



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
#include "NRF_USART_MODULE.h"
#include "stm32f4xx_it.h"
#include "EXIT_init.h"
#include "mltypes.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "log.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mpu.h"
#include "inv_mpu.h"
#include "log.h"
#include "packet.h"
#include "PID_Control.h"

#ifdef COMPASS_ENABLED
void send_status_compass(void);
#endif //COMPASS_ENABLED

struct platform_data_s {
    signed char orientation[9]; 
};

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};









static void read_from_mpl(void);
static void setup_gyro(void);
static inline void run_self_test(void);
static void handle_input(void);


void MPU_DMP_Setup(void);
void inline MPU_DMP_Loop(void);


void MPU9250_run_self_test(void);
void gyro_data_ready_cb(void);
void send_status_compass(void);
#endif  //_MPU_DMP_H 

