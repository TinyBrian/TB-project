#include "Data_Type_Conv.h"

/*****************************************************************************************/
//int to char ()

char* itoa(int num,char* str,int radix)
{
	char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";//������
	unsigned unum;//���Ҫת���������ľ���ֵ,ת�������������Ǹ���
	int i=0,j,k;//i����ָʾ�����ַ�����Ӧλ��ת��֮��i��ʵ�����ַ����ĳ��ȣ�ת����˳��������ģ��������������k����ָʾ����˳��Ŀ�ʼλ��;j����ָʾ����˳��ʱ�Ľ�����
 
	//��ȡҪת���������ľ���ֵ
	if(radix==10&&num<0)//Ҫת����ʮ�����������Ǹ���
	{
		unum=(unsigned)-num;//��num�ľ���ֵ����unum
		str[i++]='-';//���ַ�����ǰ������Ϊ'-'�ţ�����������1
	}
	else unum=(unsigned)num;//����numΪ����ֱ�Ӹ�ֵ��unum
 
	//ת�����֣�ע��ת�����������
	do
	{
		str[i++]=index[unum%((unsigned)radix)];//ȡunum�����һλ��������Ϊstr��Ӧλ��ָʾ������1
		unum/=radix;//unumȥ�����һλ
 
	}while(unum);//ֱ��unumΪ0�˳�ѭ��
 
	str[i]='\0';//���ַ���������'\0'�ַ���c�����ַ�����'\0'������
 
	//��˳���������
	if(str[0]=='-') k=1;//����Ǹ��������Ų��õ������ӷ��ź��濪ʼ����
	else k=0;//���Ǹ�����ȫ����Ҫ����
 
	char temp;//��ʱ��������������ֵʱ�õ�
	for(j=k;j<=(i-1)/2;j++)//ͷβһһ�Գƽ�����i��ʵ�����ַ����ĳ��ȣ��������ֵ�ȳ�����1
	{
		temp=str[j];//ͷ����ֵ����ʱ����
		str[j]=str[i-1+k-j];//β����ֵ��ͷ��
		str[i-1+k-j]=temp;//����ʱ������ֵ(��ʵ����֮ǰ��ͷ��ֵ)����β��
	}
 
	return str;//����ת������ַ���
}

/*******************************************************************************/  
//char to int /long int()  

long  atol(const char *nptr)
{
        int c;              /* ��ǰҪת�����ַ�(һ��һ���ַ�ת��������) */
        long total;         /* ��ǰת����� */
        int sign;           /* ��־ת������Ƿ������*/
 
        /*�����ո񣬿ո񲻽���ת��*/
        while ( isdigit((int)(unsigned char)*nptr) )
            ++nptr;
 
        c = (int)(unsigned char)*nptr++;//��ȡһ���ַ�׼��ת�� 
        sign = c;           /*������ű�ʾ*/
        if (c == '-' || c == '+')
            c = (int)(unsigned char)*nptr++;    /*����'+'��'-'�ţ�������ת��*/
 
        total = 0;//����ת�����Ϊ0 
 
        while (isdigit(c)) {//����ַ������� 
            total = 10 * total + (c - '0');     /* ����ASCII�뽫�ַ�ת��Ϊ��Ӧ�����֣����ҳ�10�ۻ������ */
            c = (int)(unsigned char)*nptr++;    /* ȡ��һ���ַ� */
        }
 
 		//���ݷ���ָʾ�����Ƿ�����ŵĽ�� 
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
	// �������ִ���
	itoa(int_part,str_int,10);
	// С�����ִ���
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








