

#include "PCF8563.h"
#include "System_Init.h"


#define SEC         0x02 //秒寄存器
#define MIN         0x03 //分寄存器
#define HOUR        0x04 //时寄存器
#define DAY         0x05 //天寄存器 
#define WEEK        0x06 //周寄存器
#define MONTH       0x07 //月寄存器 
#define YEAR        0x08 //年寄存器 
#define Read_Addr   0xA3 //读数据地址 
#define Write_Addr  0xA2 //写入数据地址

unsigned char ucYear = 0;
unsigned char ucSet_Year = 0;

void delay()         //延时约5us
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
RTC_SDA_Status =	RTC_SDA_Status();	 //IO口设置为开漏输出时可以读取外界状态 (使用GPIO_ReadInputDataBit读取) 读取前将IO口置高 (开漏输出模式下IO口置低输出低  置高为悬空状态 由外界状态决定) 硬件上需要上拉         
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
RTC_SDA_High();   //释放总线
}

unsigned char RTC_IIC_Receive_Data()  
{
  unsigned char temp,i;
	unsigned char RTC_SDA_Status = 0;
  RTC_SDA_High();
	RTC_SDA_Status =	RTC_SDA_Status();	 //IO口设置为开漏输出时可以读取外界状态 (使用GPIO_ReadInputDataBit读取) 读取前将IO口置高 (开漏输出模式下IO口置低输出低  置高为悬空状态 由外界状态决定) 硬件上需要上拉 
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
 函数块名称: void BCD_To_Hex(void)
 功能:       将BCD码转为Hex
 输入参数:      		
 输出参数:       		 		
 其他说明:     
*******************************************************************************/
unsigned char BCD_To_Hex(unsigned char uc_BCD)
{
	unsigned char uc_Hex = 0;
  uc_Hex = (uc_BCD >> 4)&0x0F;
	uc_Hex = uc_Hex *10 +(uc_BCD&0x0F);
	return uc_Hex;

}
/*******************************************************************************
 函数块名称: void Hex_To_BCD(void)
 功能:       将BCD码转为Hex
 输入参数:      		
 输出参数:       		 		
 其他说明:     
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
 函数块名称: void IIC_PCF8563(void)
 功能:       对PCF8563进行读取和写入
 输入参数:      		
 输出参数:       		 		
 其他说明:     
*******************************************************************************/
void IIC_PCF8563(void)
{
ucSet_Year = 	Hex_To_BCD(ucSet_Year);
RTC_IIC_Send_Addr_byte(YEAR,ucSet_Year);
ucYear = RTC_IIC_Receive_Addr_Byte(YEAR);
ucYear = BCD_To_Hex(ucYear);

}
