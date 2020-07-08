#include "MPUx_init.h"

#if(MPU6050_SLAVE) 

ErrorStatus MPU_Check()
{
	u8 i=0;
#ifdef MPU6050
	IIC_Read_register_bytes(MPU6050_SALVE_ADDR,MPU6050_WHO_AM_I,&i,1);
	
	if(i==MPU6050_WHO_AM_I_value)
		return SUCCESS ;        //MCU��MPU6050�ɹ����� 
	else
		return ERROR ;        //MCU��MPU6050����������
#elif MPU9250
	IIC_Read_register_bytes(MPU9250_SALVE_ADDR,MPU9250_WHO_AM_I,&i,1); 
	if(i==MPU9250_WHO_AM_I_value)
		return SUCCESS ;        //MCU��MPU6050�ɹ����� 
	else
		return ERROR ;        //MCU��MPU6050����������
#endif
}



ErrorStatus MPU_Self_checking()
{
	char* Txcharbuf;
	u8 status;	
                                                                    
  status = MPU_Check();  //��� MPU6050 ģ���� MCU ������
  
  if(status == SUCCESS)	 //�ж�����״̬   
		{	 
			GPIO_ResetBits(GPIOA,GPIO_Pin_7);
			Txcharbuf="MPU Connected!\n"; 
			NRF_Datac_pc_ascll(Txcharbuf,strlen(Txcharbuf)); 
			Delay_ms(200);
		} 
  else	  
		{		 
			Txcharbuf="MPU ERROR\n";
			NRF_Datac_pc_ascll(Txcharbuf,strlen(Txcharbuf)); 
			#if(HardwareIIC)  
			IIC_Timeout_error_hanlder();
			#endif
			return ERROR;
		}

		return SUCCESS; 
	
}



void MPU6050_IIC_init()
{ 	
		if((*(unsigned int*)(I2C1_BASE+I2C_Register_CR1)&0x01)==0) 
		
		IIC_init(0xAA,100000);		 

	Delay_ms(100);  //�ڳ�ʼ��֮ǰҪ��ʱһ��ʱ�䣬��û����ʱ����ϵ�����ϵ����ݿ��ܻ����
	
	MPU6050_Write_register(MPU6050_RA_PWR_MGMT_1, 0x00);	     //�������״̬
	MPU6050_Write_register(MPU6050_RA_SMPLRT_DIV , 0x07);	    //�����ǲ�����
	MPU6050_Write_register(MPU6050_RA_CONFIG , 0x06);	
	MPU6050_Write_register(MPU6050_RA_ACCEL_CONFIG , 0x01);	  //���ü��ٶȴ�����������2Gģʽ
	MPU6050_Write_register(MPU6050_RA_GYRO_CONFIG, 0x18);     //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)

}


void MPU6050_Write_register(u8 regaddr,u8 regcommand)
{
	IIC_Send_register_byte(MPU6050_SALVE_ADDR,regaddr,regcommand);
}

void MPU6050_Return_regbytes(u8 reg, u8* pdata_r, u8 Numbertoread)	
{
	IIC_Read_register_bytes(MPU6050_SALVE_ADDR,reg,pdata_r,Numbertoread);
}


void MPU6050_Return_gyroscope(short *GYRdata)	
{
	  u8 buf[6];
		MPU6050_Return_regbytes(MPU6050_GYRO_OUT, buf, 6);
    GYRdata[0] = (buf[0] << 8) | buf[1];
    GYRdata[1] = (buf[2] << 8) | buf[3];
    GYRdata[2] = (buf[4] << 8) | buf[5];
	
}

void MPU6050_Return_acclerometer(short *ACCdata)	
{
	  u8 buf[6];
    MPU6050_Return_regbytes(MPU6050_ACC_OUT, buf, 6);
    ACCdata[0] = (buf[0] << 8) | buf[1];
    ACCdata[1] = (buf[2] << 8) | buf[3];
    ACCdata[2] = (buf[4] << 8) | buf[5];
	
}

float MPU6050_Return_temp()	
{	

	short temp3=0;
	u8 buf[2];
	MPU6050_Return_regbytes(MPU6050_RA_TEMP_OUT_H,buf,2);     //��ȡ�¶�ֵ
  temp3= (buf[0] << 8) | buf[1];	
	return ((double) temp3/340.0)+36.53;//from datasheet
	
}


void DMPQuat_To_Euler(long* Qx,float* Eulerangle)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	float pith,roll,yaw;
	
	q0=Qx[0]/ ((double)(1L << 30));
	q1=Qx[1]/ ((double)(1L << 30));
	q2=Qx[2]/ ((double)(1L << 30));
	q3=Qx[3]/ ((double)(1L << 30));
	
	pith=asin(-2*q1*q3+2*q0*q2)*57.3;
	roll=atan2(2*q2*q3+2*q0*q1,-2*q1*q1-2*q2*q2+1)*57.3;
	yaw=atan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3;
	
	Eulerangle[0]=pith;
	Eulerangle[1]=roll;
	Eulerangle[2]=yaw;
	
}









#endif //MPU6050_SLAVE


#if(MPU6050_MASTER)

#endif //MPU6050_MASTER

