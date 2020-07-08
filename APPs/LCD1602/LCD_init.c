#include "LCD_init.h"


//手头带IIC的LCD1602 设备地址 0x3f (0x0111 111 )（出厂默认，只有3位能改）
//                     写地址 0x7E (0x0111 1110)
//                     读地址 0x7F (0x0111 1111)





// P7   P6   P5   P4  P3  P2  P1   P0
//DB7  DB6  DB5  DB4  X   E   R/W  RS
 
//库函数传的地址是只要移一位就行了，宏定义Raddr


#if(LCD_SALVE)


#if(LCD4bit)
void LCD_init()
{	
		Delay_ms(50);  
		LCD_Receive_command(0x30);  
 		Delay_ms(5);  
		LCD_Receive_command(0x30);
		Delay_ms(1);  
		LCD_Receive_command(0x30);
		Delay_ms(10);  
		LCD_Receive_command(0x20); //4-bit mode
		Delay_ms(10);  


		LCD_Receive_command(0x28);Delay_ms(1);	//Function set
		LCD_Receive_command(0x08);Delay_ms(1); //Display on/off control
		LCD_Receive_command(0x01);Delay_ms(2);	//clear display
		LCD_Receive_command(0x06);Delay_ms(1); //Entry mode set
		LCD_Receive_command(0x0c);Delay_ms(1); //Display on/off control
	
		LCD_clear();
		Delay_ms(10);
	
}


// P7   P6   P5   P4  P3  P2  P1   P0
//DB7  DB6  DB5  DB4  X   E   R/W  RS

void LCD_Receive_command(u8 a)
{   char data_h,data_l;
		u8 data_t[4];
	
		data_h=a&0xf0;
		data_l=(a<<4)&0xf0;
	
		data_t[0]=data_h|0x0c; //E=1,Rs=0
		data_t[1]=data_h|0x08;//E=0,Rs=0
		data_t[2]=data_l|0x0c;//E=1,Rs=0
		data_t[3]=data_l|0x08;//E=0,Rs=0
	
		IIC_Send_bytes(LCD1602_SALVE_ADDR,data_t,4);  

	
}

void LCD_Return_data(char b)
{   char data_h,data_l;
		u8 data_t[4];
	
		data_h=b&0xf0;
		data_l=(b<<4)&0xf0;
	
		data_t[0]=data_h|0x0d;//E=1,Rs=1
		data_t[1]=data_h|0x09;//E=0,Rs=1
		data_t[2]=data_l|0x0d;//E=1,Rs=1
		data_t[3]=data_l|0x09;//E=0,Rs=1
	
		IIC_Send_bytes(LCD1602_SALVE_ADDR,data_t,4);

	
}



#endif



//普通并口LCD1602
#if(LCD8bit)

void LCD_init(void )
{
		Delay_ms(15);  
		sent_command(0x38);
		Delay_ms(5);  
		sent_command(0x38);
		Delay_ms(5);  
		sent_command(0x38);
		Delay_ms(5);  

		sent_command(0x38);
		sent_command(0x08);
		sent_command(0x01);
		sent_command(0x06);
		sent_command(0x0c);
}

void LCD_Receive_command(u8 a)
{
		
}


void LCD_Return_data(char b)                                                           
{

	
			
}

#endif




void LCD_Startpage()
{
//		LCD_clear();
//		Delay_ms(1000);
//		lcd_send_string ("Initialize done");
//		Delay_ms(500);
//		LCD_setCursor(1,0);
//		lcd_send_string ("STAND BY");
//		Delay_ms(1000);

//		LCD_clear();
//		Delay_ms(5);
//		lcd_send_string ("HELLO TINYBRIAN");
//		Delay_ms(500);
		LCD_setCursor(1,0);
		lcd_send_string ("PWM");
		LCD_setCursor(1,4);
		lcd_send_string ("DUTY:");
		LCD_setCursor(1,14);
		lcd_send_string ("%");
		LCD_setCursor(0,13);
		lcd_send_char((char)223);
		LCD_setCursor(0,14);
		lcd_send_string ("C");

}

/*********************************************************************************/
//ADC(0-4096)->(0,100)
void LCD_ADC_percent()
	{			
				char p[10]={'0','1','2','3','4','5','6','7','8','9'};
				int b=ADC_filter_return_lcd();  
			
				char ADCchar[3]={p[b/100%10],p[b/10%10],p[b%10]};
				
				
				if(b==100)
				{	
					LCD_setCursor(1,11);
					lcd_send_string_t(ADCchar,3);

				}
				else
					if(b>=10)
					{ LCD_setCursor(1,11);
						lcd_send_char(' ');
						LCD_setCursor(1,12);
						lcd_send_string_t(ADCchar+1,2);

					}
					else
					{	LCD_setCursor(1,11);
						lcd_send_char(' ');
						LCD_setCursor(1,12);
						lcd_send_char(' ');
						LCD_setCursor(1,13);
						lcd_send_string_t(ADCchar+2,1);

					}
					

		
	}

	
/*********************************************************************************/
//ADC(0-4096)->(0,100)
void LCD_MPU6050_u8(u8* pdata,u8 length)
	{			
				char p[10]={'0','1','2','3','4','5','6','7','8','9'};
				
			while(length>0)
			{				
				int b=*pdata;  
			
				char Datachar[3]={p[b/100%10],p[b/10%10],p[b%10]};
				
				
				if(b==100)
				{	
					LCD_setCursor(0,11);
					lcd_send_string_t(Datachar,3);

				}
				else
					if(b>=10)
					{ LCD_setCursor(0,11);
						lcd_send_char(' ');
						LCD_setCursor(0,12);
						lcd_send_string_t(Datachar+1,2);

					}
					else
					{	LCD_setCursor(0,11);
						lcd_send_char(' ');
						LCD_setCursor(0,12);
						lcd_send_char(' ');
						LCD_setCursor(0,13);
						lcd_send_string_t(Datachar+2,1);

					}
					
					pdata++;
					length--;
	
				}
	}
	
	void	LCD_setCursor(u8 row,u8 col)                                                              
{
	    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    LCD_Receive_command(col);
														
}



void LCD_clear() 
{
		LCD_Receive_command(0x01);  // 1.clear display Command
		
	/*	lcd_send_cmd (0x80);     //2.write '' to clear
			for (int i=0; i<70; i++)
			{
					lcd_send_data (' ');
			}*/																								
}


void lcd_send_string (char *str)
{
	while (*str) LCD_Return_data(*str++);
}


void lcd_send_string_t(char *str,int i)
{
	while (i>=1)
		{ 
		LCD_Return_data(*str++);
		i--;
		}
}

void lcd_send_char(char str)
{
		LCD_Return_data(str);
}

void LCD_backlight(status a)
{
		if(a==on)
		{
		 LCD_Receive_command(0x0c);
		}
		if(a==off)
		{
		 LCD_Receive_command(0x08);
		}
		
}

int LCD_Check_busy()
{
	int state/*=((PCF_ReceiveData(I2C1)>>7)&1)*/;          //DB7=BF(busy flag)
	
	return state;
	
}
	
	
#endif //slave



#if(LCD_MASTER)
#endif //master


