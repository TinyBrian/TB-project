#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

#include "stm32f4xx.h" 
#include "Type_Define_self.h"  

#define Gyro2000 0.0610351563
#define Accl2g   0.0000305176

typedef struct PID
{
	float y; //积分分离门限
	float p;
	float i;
	float d;
	float Error;
	float PreError;
	float intergral;
	float Umax;
	float Umin;
	float Pout;
	float Iout;
	float Dout;
	float OutPut;
	
}PID_MPU;

typedef struct 
{
	float pitch; 	// -90.0° <---> +90.0°
	float roll;		//-180.0°<---> +180.0°
	float yaw;		//-180.0°<---> +180.0°
	float Battery;
	u16 throttle;
	u8   Arminfo;
	
}Float_UVAdata;

typedef struct 
{
	u16 pitch;  //1000-2000
	u16 roll;		//1000-2000
	u16 yaw;		//1000-2000
	u16 throttle;//1000-2000
	u8   Arminfo;
}Raw_UVAdata;

typedef struct 
{
	float pitch;
	float roll;
	float yaw;
	
}Float_Angle;

typedef struct 
{
	float X;
	float Y;
	float Z;
	
}Float_XYZ;



ArmStatus UVA_standby(void);   

void PID_Controler(PID_MPU *pid,float references,float measurements);

void UVA_Controler(Raw_UVAdata Remote_Reference, Float_UVAdata measuer_euler, int16_t* measuer_Gyro_rate);
   
void PID_Pitch_rate(float P,float I,float D);
void PID_Yaw_rate(float P,float I,float D);
void PID_Roll_rate(float P,float I,float D);

void PID_Pitch_angle(float P,float I,float D);
void PID_Yaw_angle(float P,float I,float D);
void PID_Roll_angle(float P,float I,float D);


void PID_all_init_cl(void);
void PID_all_update_cl(void);


#endif //__PID_CONTROL_H


