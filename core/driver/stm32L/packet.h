/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 * $Id: $
 *******************************************************************************/

/**
 *  @defgroup STM32L STM32L System Layer
 *  @brief  STM32L System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file   packet.h
 *      @brief  Defines needed for sending data/debug packets via USB.
 */

#ifndef __PACKET_H__
#define __PACKET_H__

#include "mltypes.h"
#include "stm32f4xx.h"
#include "Type_Define_self.h"  

typedef enum {
    PACKET_DATA_ACCEL = 0,
    PACKET_DATA_GYRO,
    PACKET_DATA_COMPASS,
    PACKET_DATA_QUAT,
    PACKET_DATA_EULER,
    PACKET_DATA_ROT,
    PACKET_DATA_HEADING,
    PACKET_DATA_LINEAR_ACCEL,
    NUM_DATA_PACKETS
} eMPL_packet_e;


//ANO_TC
/**********/

typedef struct 
{
		u8 send_version;
		u8 send_status;
		u8 send_sensor;
		u8 send_pid1;
		u8 send_pid2;
		u8 send_pid3;
		u8 send_pid4;
		u8 send_pid5;
		u8 send_pid6;
		u8 send_rcdata;
		u8 send_offset;
		u8 send_motopwm;
		u8 send_power;

}dt_flag_t;


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	


/**
 *  @brief      Send a quaternion packet via UART.
 *  The host is expected to use the data in this packet to graphically
 *  represent the device orientation. To send quaternion in the same manner
 *  as any other data packet, use eMPL_send_data.
 *  @param[in]  quat    Quaternion data.
 */
//void eMPL_send_quat(long *quat);

/**
 *  @brief      Send a data packet via UART
 *  @param[in]  type    Contents of packet (PACKET_DATA_ACCEL, etc).
 *  @param[in]  data    Data (length dependent on contents).
 */
//void eMPL_send_data(unsigned char type, long *data);





//ANO_TC
/**********/


void Send_Data(int16_t *Gyro,int16_t *Accel);
void Send_Data_t(int16_t *Gyro,int16_t *Accel,int16_t *Compass);

void Data_Send_ANO_Status(float Pitch,float Roll,float Yaw,u8 Armedinfo);
void Data_Send_ANO_Sensors(int16_t *Gyro,int16_t *Accel,int16_t *Compass);
void Data_Send_ANO_Battery_V(int16_t Voltage,int16_t Currency);
void Send_ANO_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);

void ANO_DT_Data_Exchange(void);
void Data_Receive_ANO_Prepare(u8 data);
void Data_Receive_ANO_Anlyze(u8 *data_buf,u8 num);



#endif /* __PACKET_H__ */

/**
 * @}
 */
