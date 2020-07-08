/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 * $Id: $
 *******************************************************************************/

/**
 *  @defgroup MSP430_System_Layer MSP430 System Layer
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file   log_msp430.c
 *      @brief  Logging facility for the TI MSP430.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "packet.h"
#include "log.h"

#include "UART_init.h"
#include "IIC_init.h"
#include "NRF_USART_MODULE.h"

#define BUF_SIZE        (256)
#define PACKET_LENGTH   (25)

#define PACKET_DEBUG    (1)
#define PACKET_QUAT     (2)
#define PACKET_DATA     (3)


static void Send_ANO_Check(u8 head, u8 check_sum) ;
extern PID_MPU pid_pitch_rate;
extern PID_MPU pid_roll_rate;
extern PID_MPU pid_yaw_rate;

extern PID_MPU pid_pitch_angle; 
extern PID_MPU pid_roll_angle;
extern PID_MPU pid_yaw_angle;
dt_flag_t f;//需要发送数据的标志
u8 data_to_send[32];//发送数据缓存

extern struct inv_sensor_cal_t sensors;  //陀螺仪或加速度计等获得数据都在这

extern Float_UVAdata Remote_data;
extern Float_UVAdata Measure_data;

extern u8 UPPerdata[32];	//上位机数据

int fputcc(int ch)
{
//		/* 发送一个字节数据到USART1 */
//		USART_SendData(DEBUG_USARTx, (uint8_t) ch);
//		
//		/* 等待发送完毕 */
//		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);		
//	
		return (ch);  
}

/**
 *  @brief      Prints a variable argument log message.
 *  USB output will be formatted as follows:\n
 *  packet[0]       = $\n
 *  packet[1]       = packet type (1: debug, 2: quat, 3: data)\n
 *  packet[2]       = \n for debug packets: log priority\n
 *                    for quaternion packets: unused\n
 *                    for data packets: packet content (accel, gyro, etc)\n
 *  packet[3-20]    = data\n
 *  packet[21]      = \\r\n
 *  packet[22]      = \\n
 *  @param[in]  priority    Log priority (based on Android).
 *  @param[in]  tag         File specific string.
 *  @param[in]  fmt         String of text with optional format tags.
 *
 *  @return     0 if successful.
 */
int _MLPrintLog (int priority, const char* tag, const char* fmt, ...)
{
    va_list args;
    int length, ii;
    char buf[BUF_SIZE], out[PACKET_LENGTH], this_length;

    /* This can be modified to exit for unsupported priorities. */
    switch (priority) {
    case MPL_LOG_UNKNOWN:
    case MPL_LOG_DEFAULT:
    case MPL_LOG_VERBOSE:
    case MPL_LOG_DEBUG:
    case MPL_LOG_INFO:
    case MPL_LOG_WARN:
    case MPL_LOG_ERROR:
    case MPL_LOG_SILENT:
        break;
    default:
        return 0;
    }

    va_start(args, fmt);

    length = vsprintf(buf, fmt, args);
    if (length <= 0) {
        va_end(args);
        return length;
    }

    memset(out, 0, PACKET_LENGTH);
    out[1] = '$';
    out[2] = PACKET_DEBUG;
    out[3] = priority;
    out[22] = '\r';
    out[23] = '\n';
		
		
    for (ii = 0; ii < length; ii += (PACKET_LENGTH-5)) {
#define min(a,b) ((a < b) ? a : b)
        this_length = min(length-ii, PACKET_LENGTH-5);
        memset(out+3, 0, 18);
        memcpy(out+3, buf+ii, this_length);
     
    }
		
    //NRF_Datac_pc_ascll(out,strlen(out)+1);
   
		va_end(args);

    return 0;
}


void eMPL_send_quat(long *quat)
{
    char out[PACKET_LENGTH];
    int i;
    if (!quat)
        return;
    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_QUAT;
    out[3] = (char)(quat[0] >> 24);
    out[4] = (char)(quat[0] >> 16);
    out[5] = (char)(quat[0] >> 8);
    out[6] = (char)quat[0];
    out[7] = (char)(quat[1] >> 24);
    out[8] = (char)(quat[1] >> 16);
    out[9] = (char)(quat[1] >> 8);
    out[10] = (char)quat[1];
    out[11] = (char)(quat[2] >> 24);
    out[12] = (char)(quat[2] >> 16);
    out[13] = (char)(quat[2] >> 8);
    out[14] = (char)quat[2];
    out[15] = (char)(quat[3] >> 24);
    out[16] = (char)(quat[3] >> 16);
    out[17] = (char)(quat[3] >> 8);
    out[18] = (char)quat[3];
    out[21] = '\r';
    out[22] = '\n';
		
    for (i=0; i<PACKET_LENGTH; i++) {
      fputcc(out[i]);
    }
}

void eMPL_send_data(unsigned char type, long *data)
{
    char out[PACKET_LENGTH];
    int i;
    if (!data)
        return;
    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_DATA;
    out[2] = type;
    out[21] = '\r';
    out[22] = '\n';
    switch (type) {
    /* Two bytes per-element. */
    case PACKET_DATA_ROT:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[1] >> 24);
        out[6] = (char)(data[1] >> 16);
        out[7] = (char)(data[2] >> 24);
        out[8] = (char)(data[2] >> 16);
        out[9] = (char)(data[3] >> 24);
        out[10] = (char)(data[3] >> 16);
        out[11] = (char)(data[4] >> 24);
        out[12] = (char)(data[4] >> 16);
        out[13] = (char)(data[5] >> 24);
        out[14] = (char)(data[5] >> 16);
        out[15] = (char)(data[6] >> 24);
        out[16] = (char)(data[6] >> 16);
        out[17] = (char)(data[7] >> 24);
        out[18] = (char)(data[7] >> 16);
        out[19] = (char)(data[8] >> 24);
        out[20] = (char)(data[8] >> 16);
        break;
    /* Four bytes per-element. */
    /* Four elements. */
    case PACKET_DATA_QUAT:
        out[15] = (char)(data[3] >> 24);
        out[16] = (char)(data[3] >> 16);
        out[17] = (char)(data[3] >> 8);
        out[18] = (char)data[3];
    /* Three elements. */
    case PACKET_DATA_ACCEL:
    case PACKET_DATA_GYRO:
    case PACKET_DATA_COMPASS:
    case PACKET_DATA_EULER:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[0] >> 8);
        out[6] = (char)data[0];
        out[7] = (char)(data[1] >> 24);
        out[8] = (char)(data[1] >> 16);
        out[9] = (char)(data[1] >> 8);
        out[10] = (char)data[1];
        out[11] = (char)(data[2] >> 24);
        out[12] = (char)(data[2] >> 16);
        out[13] = (char)(data[2] >> 8);
        out[14] = (char)data[2];
        break;
    case PACKET_DATA_HEADING:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[0] >> 8);
        out[6] = (char)data[0];
        break;
    default:
        return;
    }
    for (i=0; i<PACKET_LENGTH; i++) {
      fputcc(out[i]);
    }
}



//ANO_TC
/*************************************************************************************/
void Data_Send_ANO_Status(float Pitch,float Roll,float Yaw, u8 Armedinfo) 
{ 
	unsigned char i=0;   
	unsigned char _cnt=0,sum = 0;
	unsigned int _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(Roll*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Pitch*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0;//海拔
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++] =0;//fly_model
	
	data_to_send[_cnt++]=Armedinfo; 
	
	data_to_send[3] = _cnt-4;
	//和校验
	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
		
	data_to_send[_cnt++]=sum;
	
	//串口发送数据

	NRF_USART_Send_char(data_to_send,_cnt); 
 
}


void Data_Send_ANO_Sensors(int16_t *Gyro,int16_t *Accel,int16_t *Compass)
{
	unsigned char i=0; 
	unsigned char _cnt=0,sum = 0;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	

	data_to_send[_cnt++]=BYTE1(Accel[0]);
	data_to_send[_cnt++]=BYTE0(Accel[0]);
	data_to_send[_cnt++]=BYTE1(Accel[1]);
	data_to_send[_cnt++]=BYTE0(Accel[1]);
	data_to_send[_cnt++]=BYTE1(Accel[2]);
	data_to_send[_cnt++]=BYTE0(Accel[2]);
	
	data_to_send[_cnt++]=BYTE1(Gyro[0]);
	data_to_send[_cnt++]=BYTE0(Gyro[0]);
	data_to_send[_cnt++]=BYTE1(Gyro[1]);
	data_to_send[_cnt++]=BYTE0(Gyro[1]);
	data_to_send[_cnt++]=BYTE1(Gyro[2]);
	data_to_send[_cnt++]=BYTE0(Gyro[2]);
	
	data_to_send[_cnt++]=BYTE1(Compass[0]);
	data_to_send[_cnt++]=BYTE0(Compass[0]);
	data_to_send[_cnt++]=BYTE1(Compass[1]);
	data_to_send[_cnt++]=BYTE0(Compass[1]);
	data_to_send[_cnt++]=BYTE1(Compass[2]);
	data_to_send[_cnt++]=BYTE0(Compass[2]);
	
	data_to_send[3] = 18;
	//和校验
	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	//串口发送数据
	NRF_USART_Send_char(data_to_send,_cnt); 

}


void Data_Send_ANO_Battery_V(int16_t Voltage,int16_t Currency)
{
	unsigned char i=0; 
	unsigned char _cnt=0,sum = 0;
	unsigned int _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(Voltage*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =(int)(Currency*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	//和校验
	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
		
	data_to_send[_cnt++]=sum;
	
	//串口发送数据

	NRF_USART_Send_char(data_to_send,_cnt);  

}



/************************************************************************/
extern u8 UPPerdata[32];	//发送数据缓存

void Data_Receive_ANO_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		Data_Receive_ANO_Anlyze(RxBuffer,_data_cnt+5); 
	}
	else
		state = 0;
}


/***************************************************/
void ANO_DT_Data_Exchange(void)
{
	static u16 cnt = 0;
	static u16 sensor_cnt 	= 5;
	static u16 status_cnt 	= 3;
//	static u8 rcdata_cnt 	= 20;
//	static u8 motopwm_cnt	= 20;   
	static u16 power_cnt		=	1000;
	
	if((cnt % sensor_cnt) == (sensor_cnt-1))
		f.send_sensor = 1;	
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
	
//	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
//		f.send_rcdata = 1;	
//	
//	if((cnt % motopwm_cnt) == (motopwm_cnt-1))
//		f.send_motopwm = 0;	
	
	if((cnt % power_cnt) == (power_cnt-1))
		f.send_power = 0;		
	
	cnt++;

/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_status)
	{
		f.send_status = 0;
		Data_Send_ANO_Status(Measure_data.pitch,Measure_data.roll,-Measure_data.yaw,Measure_data.Arminfo);   
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_sensor)
	{
		f.send_sensor = 0;
		Data_Send_ANO_Sensors((int16_t *)&sensors.gyro.raw,(int16_t *)&sensors.accel.raw,(int16_t *)&sensors.compass.raw);	//原始数据	
	}	
/////////////////////////////////////////////////////////////////////////////////////
//	else if(f.send_rcdata)
//	{
//		f.send_rcdata = 0;
//		ANO_DT_Send_RCData(Rc_Pwm_In[0],Rc_Pwm_In[1],Rc_Pwm_In[2],Rc_Pwm_In[3],Rc_Pwm_In[4],Rc_Pwm_In[5],Rc_Pwm_In[6],Rc_Pwm_In[7],0,0);
//	}	
/////////////////////////////////////////////////////////////////////////////////////	
//	else if(f.send_motopwm)
//	{
//		f.send_motopwm = 0;
//		ANO_DT_Send_MotoPWM(1,2,3,4,5,6,7,8);
//	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_power)
	{
		f.send_power = 0;
		Data_Send_ANO_Battery_V(Measure_data.Battery,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid1)
	{
		f.send_pid1 = 0;
		Send_ANO_PID(1,pid_roll_rate.p,pid_roll_rate.i,pid_roll_rate.d,
											pid_pitch_rate.p,pid_pitch_rate.i,pid_pitch_rate.d,
											pid_yaw_rate.p,pid_yaw_rate.i,pid_yaw_rate.d);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid2)
	{
		f.send_pid2 = 0;
		Send_ANO_PID(2,pid_roll_angle.p,pid_roll_angle.i,pid_roll_angle.d,
											pid_roll_angle.p,pid_roll_angle.i,pid_roll_angle.d,
											pid_roll_angle.p,pid_roll_angle.i,pid_roll_angle.d);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid3)
	{
		f.send_pid3 = 0;
		Send_ANO_PID(3,0,0,0,0,0,0,0,0,0);
	}

}


void Data_Receive_ANO_Anlyze(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
//	if(*(data_buf+2)==0X01)
//	{
//		if(*(data_buf+4)==0X01)
//			mpu6050.Acc_CALIBRATE = 1;
//		if(*(data_buf+4)==0X02)
//			mpu6050.Gyro_CALIBRATE = 1;
//		if(*(data_buf+4)==0X03)
//		{
//			mpu6050.Acc_CALIBRATE = 1;		
//			mpu6050.Gyro_CALIBRATE = 1;			
//		}
//	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
 			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
//			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
//			Para_ResetToFactorySetup();
		}
	}

    if(*(data_buf+2)==0X10)								//PID1
    {
        pid_roll_rate.p 	= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        pid_roll_rate.i 	= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
				pid_roll_rate.d 	= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        pid_pitch_rate.p 	= 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        pid_pitch_rate.i  = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        pid_pitch_rate.d 	= 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        pid_yaw_rate.p	  = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        pid_yaw_rate.i	  = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        pid_yaw_rate.d	  = 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        Send_ANO_Check(*(data_buf+2),sum);
    }
		
			if(*(data_buf+2)==0X11)								//PID2
    {
        pid_roll_angle.p 			= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        pid_roll_angle.i 			= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
				pid_roll_angle.d 			= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        pid_pitch_angle.p 		= 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        pid_pitch_angle.i 		= 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        pid_pitch_angle.d 		= 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        pid_yaw_angle.p	 		= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        pid_yaw_angle.i	 		= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        pid_yaw_angle.d	 		= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        Send_ANO_Check(*(data_buf+2),sum);
    }
		
	if(*(data_buf+2)==0X12)								//PID3
    {	
        Send_ANO_Check(*(data_buf+2),sum); 
	
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
		Send_ANO_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		Send_ANO_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		Send_ANO_Check(*(data_buf+2),sum);
	}
		
}

static void Send_ANO_Check(u8 head, u8 check_sum) 
{
	UPPerdata[0]=0xAA;
	UPPerdata[1]=0xAA;
	UPPerdata[2]=0xEF;
	UPPerdata[3]=2;
	UPPerdata[4]=head;
	UPPerdata[5]=check_sum;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += UPPerdata[i];
	UPPerdata[6]=sum;
	NRF_USART_Send_char(UPPerdata, 7);
}

void Send_ANO_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	
	UPPerdata[_cnt++]=0xAA;
	UPPerdata[_cnt++]=0xAA;
	UPPerdata[_cnt++]=0x10+group-1;
	UPPerdata[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	UPPerdata[_cnt++]=BYTE1(_temp);
	UPPerdata[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	UPPerdata[_cnt++]=BYTE1(_temp);
	UPPerdata[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	UPPerdata[_cnt++]=BYTE1(_temp);
	UPPerdata[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	UPPerdata[_cnt++]=BYTE1(_temp);
	UPPerdata[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	UPPerdata[_cnt++]=BYTE1(_temp);
	UPPerdata[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	UPPerdata[_cnt++]=BYTE1(_temp);
	UPPerdata[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	UPPerdata[_cnt++]=BYTE1(_temp);
	UPPerdata[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	UPPerdata[_cnt++]=BYTE1(_temp);
	UPPerdata[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	UPPerdata[_cnt++]=BYTE1(_temp);
	UPPerdata[_cnt++]=BYTE0(_temp);
	
	UPPerdata[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += UPPerdata[i];
	
	UPPerdata[_cnt++]=sum;

	NRF_USART_Send_char(UPPerdata, _cnt);
}
/**
 * @}
**/
/************************************************************************/

//void Send_Data_t(int16_t *Gyro,int16_t *Accel,int16_t *Compass)
//{
//	unsigned char i=0;
//	unsigned char _cnt=0,sum = 0;
//	u8 data_to_send[50];

//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x02;
//	data_to_send[_cnt++]=0;
//	

//	data_to_send[_cnt++]=BYTE1(Accel[0]);
//	data_to_send[_cnt++]=BYTE0(Accel[0]);
//	data_to_send[_cnt++]=BYTE1(Accel[1]);
//	data_to_send[_cnt++]=BYTE0(Accel[1]);
//	data_to_send[_cnt++]=BYTE1(Accel[2]);
//	data_to_send[_cnt++]=BYTE0(Accel[2]);
//	
//	data_to_send[_cnt++]=BYTE1(Gyro[0]);
//	data_to_send[_cnt++]=BYTE0(Gyro[0]);
//	data_to_send[_cnt++]=BYTE1(Gyro[1]);
//	data_to_send[_cnt++]=BYTE0(Gyro[1]);
//	data_to_send[_cnt++]=BYTE1(Gyro[2]);
//	data_to_send[_cnt++]=BYTE0(Gyro[2]);
//	
//	data_to_send[_cnt++]=BYTE1(Compass[0]);
//	data_to_send[_cnt++]=BYTE0(Compass[0]);
//	data_to_send[_cnt++]=BYTE1(Compass[1]);
//	data_to_send[_cnt++]=BYTE0(Compass[1]);
//	data_to_send[_cnt++]=BYTE1(Compass[2]);
//	data_to_send[_cnt++]=BYTE0(Compass[2]);
//	
//	data_to_send[3] = 18;
//	//和校验
//	for(i=0;i<_cnt;i++)
//		sum+= data_to_send[i];
//	data_to_send[_cnt++]=sum;
//	
//	//串口发送数据

//	//	USART_Send_char(data_to_send[i]);
//	//	NRF_Datai_pc_ascll(data_to_send[i],8);
//		NRF_USART_Send_char(data_to_send,_cnt); 
//		//NRF_USART_Send_char(data_to_send+8); 
//}


