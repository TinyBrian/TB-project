#include "Data_Type_Conv.h"

/*****************************************************************************************/
//int to char ()

char* itoa(int num,char* str,int radix)
{
	char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";//索引表
	unsigned unum;//存放要转换的整数的绝对值,转换的整数可能是负数
	int i=0,j,k;//i用来指示设置字符串相应位，转换之后i其实就是字符串的长度；转换后顺序是逆序的，有正负的情况，k用来指示调整顺序的开始位置;j用来指示调整顺序时的交换。
 
	//获取要转换的整数的绝对值
	if(radix==10&&num<0)//要转换成十进制数并且是负数
	{
		unum=(unsigned)-num;//将num的绝对值赋给unum
		str[i++]='-';//在字符串最前面设置为'-'号，并且索引加1
	}
	else unum=(unsigned)num;//若是num为正，直接赋值给unum
 
	//转换部分，注意转换后是逆序的
	do
	{
		str[i++]=index[unum%((unsigned)radix)];//取unum的最后一位，并设置为str对应位，指示索引加1
		unum/=radix;//unum去掉最后一位
 
	}while(unum);//直至unum为0退出循环
 
	str[i]='\0';//在字符串最后添加'\0'字符，c语言字符串以'\0'结束。
 
	//将顺序调整过来
	if(str[0]=='-') k=1;//如果是负数，符号不用调整，从符号后面开始调整
	else k=0;//不是负数，全部都要调整
 
	char temp;//临时变量，交换两个值时用到
	for(j=k;j<=(i-1)/2;j++)//头尾一一对称交换，i其实就是字符串的长度，索引最大值比长度少1
	{
		temp=str[j];//头部赋值给临时变量
		str[j]=str[i-1+k-j];//尾部赋值给头部
		str[i-1+k-j]=temp;//将临时变量的值(其实就是之前的头部值)赋给尾部
	}
 
	return str;//返回转换后的字符串
}

/*******************************************************************************/  
//char to int /long int()  

long  atol(const char *nptr)
{
        int c;              /* 当前要转换的字符(一个一个字符转换成数字) */
        long total;         /* 当前转换结果 */
        int sign;           /* 标志转换结果是否带负号*/
 
        /*跳过空格，空格不进行转换*/
        while ( isdigit((int)(unsigned char)*nptr) )
            ++nptr;
 
        c = (int)(unsigned char)*nptr++;//获取一个字符准备转换 
        sign = c;           /*保存符号标示*/
        if (c == '-' || c == '+')
            c = (int)(unsigned char)*nptr++;    /*跳过'+'、'-'号，不进行转换*/
 
        total = 0;//设置转换结果为0 
 
        while (isdigit(c)) {//如果字符是数字 
            total = 10 * total + (c - '0');     /* 根据ASCII码将字符转换为对应的数字，并且乘10累积到结果 */
            c = (int)(unsigned char)*nptr++;    /* 取下一个字符 */
        }
 
 		//根据符号指示返回是否带负号的结果 
        if (sign == '-')
            return -total;
        else
            return total;  
}


int  atoi(const char *nptr)
{
        return (int)atol(nptr);
}


/**********************************************************************************/
//	char to float();










/**********************************************************************************/
// float	to char ();

char* ftoa(float number,char *str,int ndigit)
{
    long int_part;//
	double float_part;
	
	char str_int[32];
	char str_float[32];
	
	memset(str_int,0,32);
	memset(str_float,0,32);
	
	int_part = (long)number;
	
	float_part = number - int_part;
	// 整数部分处理
	itoa(int_part,str_int,10);
	// 小数部分处理
	if(ndigit>0)
	{
		float_part =fabs( pow(10,ndigit)*float_part);
		itoa((long)float_part,str_float,10);
	}
	
	int i = strlen(str_int);
	str_int[i] = '.';
	strcat(str_int,str_float);
	
	str=str_int;
	
	return str;
}








