#include "IIC.h"
#include "System_Init.h"

void delay_us(unsigned int n)   
{
	unsigned char i;
while(n--)
 {
 i = 7;
 while(i--);
 };

}
void delay_ms(unsigned char n)
{
	unsigned int i;
while(n--)
 {
 i = 12000;
 while(i--);
 };

}
//产生IIC起始信号
void IIC_Start(void)
{
	IIC_24C0XX_SDA_High();	  //对位进行赋值
	IIC_24C0XX_SCL_High();
	delay_us(4);
 	IIC_24C0XX_SDA_Low();//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_24C0XX_SCL_Low();//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	IIC_24C0XX_SCL_Low();
	IIC_24C0XX_SDA_Low();//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_24C0XX_SCL_High(); 
	IIC_24C0XX_SDA_High();//发送I2C总线结束信号
	delay_us(4);				
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 i;
	u8 ucErrTime=0;
	
	IIC_24C0XX_SDA_High();delay_us(1);	   
	IIC_24C0XX_SCL_High();delay_us(1);
	IIC_24C0XX_SDA_High();
	i = IIC_24C0XX_SDA_Status();
	while(i)//READ_SDA
	{
		IIC_24C0XX_SDA_High();
		i = IIC_24C0XX_SDA_Status();
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_24C0XX_SCL_Low();//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_24C0XX_SCL_Low();
	IIC_24C0XX_SDA_Low();
	delay_us(2);
	IIC_24C0XX_SCL_High();
	delay_us(2);
	IIC_24C0XX_SCL_Low();
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_24C0XX_SCL_Low();
	IIC_24C0XX_SDA_High();
	delay_us(2);
	IIC_24C0XX_SCL_High();
	delay_us(2);
	IIC_24C0XX_SCL_Low();
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;    	    
    IIC_24C0XX_SCL_Low();//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7)
			IIC_24C0XX_SDA_High();
		else
			IIC_24C0XX_SDA_Low();
		txd<<=1; 	  
		delay_us(2);         //对TEA5767这三个延时都是必须的
		IIC_24C0XX_SCL_High();
		delay_us(2); 
		IIC_24C0XX_SCL_Low();	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	u8 a;

	IIC_24C0XX_SDA_High();
    for(i=0;i<8;i++ )
	  {
//			IIC_24C0XX_SDA_High();               //读取数据放在这里会造成将地址信息误当成数据读出   （改例程出现问题 备注做说明）
//		 a = IIC_24C0XX_SDA_Status();
     IIC_24C0XX_SCL_Low(); 
     delay_us(2);
	   IIC_24C0XX_SCL_High();
     receive<<=1;           
		 IIC_24C0XX_SDA_High();                 //需放在这里
		 a = IIC_24C0XX_SDA_Status();
     if(a)receive++;   
		delay_us(1); 
    }					
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}





















