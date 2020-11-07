

#include "PCF8563.h"
#include "System_Init.h"


#define SEC         0x02 //��Ĵ���
#define MIN         0x03 //�ּĴ���
#define HOUR        0x04 //ʱ�Ĵ���
#define DAY         0x05 //��Ĵ��� 
#define WEEK        0x06 //�ܼĴ���
#define MONTH       0x07 //�¼Ĵ��� 
#define YEAR        0x08 //��Ĵ��� 
#define Read_Addr   0xA3 //�����ݵ�ַ 
#define Write_Addr  0xA2 //д�����ݵ�ַ

unsigned char ucYear = 0;
unsigned char ucSet_Year = 0;

void delay()         //��ʱԼ5us
{
unsigned int i = 36;
while(i--);
}

void RTC_IIC_Start()   
{
  RTC_SDA_High();
  RTC_SCL_High();
  delay();
  RTC_SDA_Low();
  delay();
  RTC_SCL_Low();
}

void RTC_IIC_Stop()  
{
RTC_SDA_Low();
RTC_SCL_High();
delay();
RTC_SDA_High();
delay();
RTC_SCL_Low();
}

void RTC_IIC_ACk()  
{
unsigned char i=0;
unsigned char RTC_SDA_Status = 0;
RTC_SCL_High();
delay();
RTC_SDA_High();
RTC_SDA_Status =	RTC_SDA_Status();	 //IO������Ϊ��©���ʱ���Զ�ȡ���״̬ (ʹ��GPIO_ReadInputDataBit��ȡ) ��ȡǰ��IO���ø� (��©���ģʽ��IO���õ������  �ø�Ϊ����״̬ �����״̬����) Ӳ������Ҫ����         
while((RTC_SDA_Status)&&(i<255))
{
RTC_SDA_High();	
RTC_SDA_Status = RTC_SDA_Status();	
i++;
}
RTC_SCL_Low();
delay();
}






void RTC_IIC_Send_Byte(unsigned char byte)  
{
unsigned char i;
for(i=0;i<=7;i++)
{
	if(byte&0x80)
	{
	RTC_SCL_Low();
	RTC_SDA_High();
  delay();
	RTC_SCL_High();
	delay();
	}
	else
  {
	RTC_SCL_Low();
	RTC_SDA_Low();
  delay();
	RTC_SCL_High();
	}
  byte=byte<<1;
}
RTC_SCL_Low();
RTC_SDA_High();   //�ͷ�����
}

unsigned char RTC_IIC_Receive_Data()  
{
  unsigned char temp,i;
	unsigned char RTC_SDA_Status = 0;
  RTC_SDA_High();
	RTC_SDA_Status =	RTC_SDA_Status();	 //IO������Ϊ��©���ʱ���Զ�ȡ���״̬ (ʹ��GPIO_ReadInputDataBit��ȡ) ��ȡǰ��IO���ø� (��©���ģʽ��IO���õ������  �ø�Ϊ����״̬ �����״̬����) Ӳ������Ҫ���� 
  for(i=0;i<=7;i++)
  {
	 RTC_SCL_Low();
   delay();
   RTC_SCL_High();
   delay();
	 RTC_SDA_High();
	 RTC_SDA_Status =	RTC_SDA_Status();
   temp=(temp<<1)|RTC_SDA_Status;
  }
	RTC_SCL_Low();
  delay();
  return temp;
}


void RTC_IIC_Send_Addr_byte(unsigned char Addr,unsigned char Byte) 
{
RTC_IIC_Start();
RTC_IIC_Send_Byte(Write_Addr);
RTC_IIC_ACk();
RTC_IIC_Send_Byte(Addr);
RTC_IIC_ACk();
RTC_IIC_Send_Byte(Byte);
RTC_IIC_ACk();
RTC_IIC_Stop();
}

unsigned char RTC_IIC_Receive_Addr_Byte(unsigned char Addr) 
{
   unsigned char temp;
   RTC_IIC_Start();
   RTC_IIC_Send_Byte(Write_Addr);
   RTC_IIC_ACk();
   RTC_IIC_Send_Byte(Addr);
   RTC_IIC_ACk();
   RTC_IIC_Start();
   RTC_IIC_Send_Byte(Read_Addr);
   RTC_IIC_ACk();
   temp=RTC_IIC_Receive_Data();
   RTC_IIC_Stop();
   return temp;
}

/*******************************************************************************
 ����������: void BCD_To_Hex(void)
 ����:       ��BCD��תΪHex
 �������:      		
 �������:       		 		
 ����˵��:     
*******************************************************************************/
unsigned char BCD_To_Hex(unsigned char uc_BCD)
{
	unsigned char uc_Hex = 0;
  uc_Hex = (uc_BCD >> 4)&0x0F;
	uc_Hex = uc_Hex *10 +(uc_BCD&0x0F);
	return uc_Hex;

}
/*******************************************************************************
 ����������: void Hex_To_BCD(void)
 ����:       ��BCD��תΪHex
 �������:      		
 �������:       		 		
 ����˵��:     
*******************************************************************************/
unsigned char Hex_To_BCD(unsigned char uc_Hex)
{
  unsigned char uc_BCD = 0;
  uc_BCD = uc_Hex / 10;
	uc_Hex = uc_Hex % 10;
	uc_BCD = (uc_BCD << 4) +(uc_Hex&0x0F);
	return uc_BCD;

}
/*******************************************************************************
 ����������: void IIC_PCF8563(void)
 ����:       ��PCF8563���ж�ȡ��д��
 �������:      		
 �������:       		 		
 ����˵��:     
*******************************************************************************/
void IIC_PCF8563(void)
{
ucSet_Year = 	Hex_To_BCD(ucSet_Year);
RTC_IIC_Send_Addr_byte(YEAR,ucSet_Year);
ucYear = RTC_IIC_Receive_Addr_Byte(YEAR);
ucYear = BCD_To_Hex(ucYear);

}
