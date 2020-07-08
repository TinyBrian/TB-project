#include "PID_Control.h"
#include "mymath.h" 
#include "PWM_TIMx_init.h"
#include "NRF24L01.h" 


extern u16 PWM_Motor[4];

extern Float_UVAdata Remote_data;
extern Float_UVAdata Measure_data;

//			 Float_Angle Measure_Euler;		 
//			 Float_XYZ Measure_Gyro_rate;

PID_MPU pid_pitch_rate;
PID_MPU pid_roll_rate;
PID_MPU pid_yaw_rate;

PID_MPU pid_pitch_angle; 
PID_MPU pid_roll_angle;
PID_MPU pid_yaw_angle;


void PID_Controler(PID_MPU *pid,float references,float measurements)
{ 
	float	Ke=0;
	float Kp=pid->p;
	float Ki=pid->i;
	float Kd=pid->d;
	
		pid->Error=references-measurements;
	
		pid->Pout=Kp*pid->Error;
	
	if(pid->intergral > pid->Umax && pid->Error >=0)
	{			

				pid->intergral= pid->Umax;
	}
	else if(pid->intergral < pid->Umin && pid->Error <=0)
	{

				pid->intergral= pid->Umin;
	}
	else
	{
		if( pid->Error > pid->y || pid->Error < - pid->y )
		Ke=0;
		else
		Ke=1;
		pid->intergral+=Ke*pid->Error;
	}	
	
	
		pid->Iout=Ki*pid->intergral;
	
		pid->Dout=Kd*(pid->PreError-pid->Error);
		
		pid->OutPut=pid->Pout+pid->Iout+pid->Dout;
	
		pid->PreError=pid->Error;

}


void PID_Pitch_rate(float P,float I,float D)
{

	pid_pitch_rate.p=P;
	pid_pitch_rate.i=I;
	pid_pitch_rate.d=D;
	pid_pitch_rate.Umax=2000;
	pid_pitch_rate.Umin=-2000;
	pid_pitch_rate.y=30;
	
}

void PID_Pitch_angle(float P,float I,float D)
{

	pid_pitch_angle.p=P;
	pid_pitch_angle.i=I;
	pid_pitch_angle.d=D;
	pid_pitch_angle.Umax=2000;
	pid_pitch_angle.Umin=-2000;
	pid_pitch_angle.y=30;
	

}
void PID_Yaw_rate(float P,float I,float D)
{

	pid_yaw_rate.p=P;
	pid_yaw_rate.i=I;
	pid_yaw_rate.d=D;
	pid_yaw_rate.Umax=2000;
	pid_yaw_rate.Umin=-2000;
	pid_yaw_rate.y=30;
	
}
void PID_Yaw_angle(float P,float I,float D)
{
	pid_yaw_angle.p=P;
	pid_yaw_angle.i=I;
	pid_yaw_angle.d=D;
	pid_yaw_angle.Umax=2000;
	pid_yaw_angle.Umin=-2000;
	pid_yaw_angle.y=30;
	
}
void PID_Roll_rate(float P,float I,float D)
{

	pid_roll_rate.p=P;
	pid_roll_rate.i=I;
	pid_roll_rate.d=D;
	pid_roll_rate.Umax=2000;
	pid_roll_rate.Umin=-2000;
	pid_roll_rate.y=30;

}
void PID_Roll_angle(float P,float I,float D)
{
	pid_roll_angle.p=P;
	pid_roll_angle.i=I;
	pid_roll_angle.d=D;
	pid_roll_angle.Umax=2000;
	pid_roll_angle.Umin=-2000;
	pid_roll_angle.y=30;
	
}                         

void UVA_Controler(Raw_UVAdata Remote_Raw_in, Float_UVAdata Measuer_Raw_euler_in, int16_t* measuer_Gyro_rate)
{  
	Float_Angle Measure_Euler;		 
	Float_XYZ Measure_Gyro_rate;
	
/**********///依据遥控器遥感值来做线性变换
	
	Remote_data.throttle=Remote_Raw_in.throttle; 
	Remote_data.pitch=(float)((Remote_Raw_in.pitch-1500)/50.0f);
	Remote_data.roll=(float)((Remote_Raw_in.roll-1500)/50.0f);
	Remote_data.yaw=(float)((Remote_Raw_in.yaw-1500)/50.0f);
	
/***********/	
	Measure_Euler.pitch=Measuer_Raw_euler_in.pitch; 
	Measure_Euler.roll=Measuer_Raw_euler_in.roll;
	Measure_Euler.yaw=Measuer_Raw_euler_in.yaw;
	
/***********/		
	Measure_Gyro_rate.X=measuer_Gyro_rate[0]*Gyro2000;//raw->rad
	Measure_Gyro_rate.Y=measuer_Gyro_rate[1]*Gyro2000;//raw->rad
	Measure_Gyro_rate.Z=measuer_Gyro_rate[2]*Gyro2000;//raw->rad
	
/***********/	

//外环角度 
		PID_Controler(&pid_roll_angle,Remote_data.roll,Measure_Euler.roll);
		PID_Controler(&pid_pitch_angle,Remote_data.pitch,Measure_Euler.pitch);
//内环角速度
		PID_Controler(&pid_roll_rate,pid_roll_angle.OutPut,Measure_Gyro_rate.Y*radtodeg);
		PID_Controler(&pid_pitch_rate,pid_pitch_angle.OutPut,Measure_Gyro_rate.X*radtodeg);
		PID_Controler(&pid_yaw_rate,Remote_data.yaw*pid_yaw_angle.p,Measure_Gyro_rate.Z*radtodeg);
//PWM		
		PWM_Motor[0]=Remote_data.throttle-pid_roll_rate.OutPut-pid_pitch_rate.OutPut+pid_yaw_rate.OutPut;
		PWM_Motor[1]=Remote_data.throttle+pid_roll_rate.OutPut-pid_pitch_rate.OutPut-pid_yaw_rate.OutPut;
		PWM_Motor[2]=Remote_data.throttle+pid_roll_rate.OutPut+pid_pitch_rate.OutPut+pid_yaw_rate.OutPut;
		PWM_Motor[3]=Remote_data.throttle-pid_roll_rate.OutPut+pid_pitch_rate.OutPut-pid_yaw_rate.OutPut;
	
		PWM_Generate_4(PWM_Motor[0],PWM_Motor[1],PWM_Motor[2],PWM_Motor[3]);

//上锁停车		
		while(Measure_data.Arminfo==0)
		{	
				if(Measure_data.Arminfo==0)
			{
				TIM_SetCompare1(TIM1,1000); 
				TIM_SetCompare2(TIM1,1000); 
				TIM_SetCompare3(TIM1,1000); 
				TIM_SetCompare4(TIM1,1000); 
			}
			else break;
		}

}
  
ArmStatus UVA_standby()
{ 	
	ArmStatus f;
//	NRF_Rx_mode();
		
	f=(ArmStatus)Remote_data.Arminfo; 
		
			if(f==1)
			{
				f= ARMED;		
			}
			else 
				f= NARMED;
			
			
	if(f==ARMED)
	{
//			NRF_Tx_mode();
//			NRF_Tx_data(ARMinfo);
	}
	
	return f;
}

void PID_all_init_cl()
{
 PID_Pitch_rate(0,0,0);
 PID_Yaw_rate(0,0,0);
 PID_Roll_rate(0,0,0);

 PID_Pitch_angle(0,0,0);
 PID_Yaw_angle(0,0,0);
 PID_Roll_angle(0,0,0);
}	

void PID_all_update_cl()
{
 PID_Pitch_rate(1,pid_pitch_rate.i,pid_pitch_rate.d);
 PID_Yaw_rate(pid_yaw_rate.p,pid_yaw_rate.i,pid_yaw_rate.d);
 PID_Roll_rate(pid_roll_rate.p,pid_roll_rate.i,pid_roll_rate.d);

 PID_Pitch_angle(pid_pitch_angle.p,pid_pitch_angle.i,pid_pitch_angle.d);
 PID_Yaw_angle(pid_yaw_angle.p,pid_yaw_angle.i,pid_yaw_angle.d);
 PID_Roll_angle(pid_roll_angle.p,pid_roll_angle.i,pid_roll_angle.d);
}	
	

