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
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	IIC_24C0XX_SDA_High();	  //��λ���и�ֵ
	IIC_24C0XX_SCL_High();
	delay_us(4);
 	IIC_24C0XX_SDA_Low();//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_24C0XX_SCL_Low();//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	IIC_24C0XX_SCL_Low();
	IIC_24C0XX_SDA_Low();//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_24C0XX_SCL_High(); 
	IIC_24C0XX_SDA_High();//����I2C���߽����ź�
	delay_us(4);				
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
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
	IIC_24C0XX_SCL_Low();//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_24C0XX_SCL_Low();
	IIC_24C0XX_SDA_Low();
	delay_us(2);
	IIC_24C0XX_SCL_High();
	delay_us(2);
	IIC_24C0XX_SCL_Low();
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_24C0XX_SCL_Low();
	IIC_24C0XX_SDA_High();
	delay_us(2);
	IIC_24C0XX_SCL_High();
	delay_us(2);
	IIC_24C0XX_SCL_Low();
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;    	    
    IIC_24C0XX_SCL_Low();//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7)
			IIC_24C0XX_SDA_High();
		else
			IIC_24C0XX_SDA_Low();
		txd<<=1; 	  
		delay_us(2);         //��TEA5767��������ʱ���Ǳ����
		IIC_24C0XX_SCL_High();
		delay_us(2); 
		IIC_24C0XX_SCL_Low();	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	u8 a;

	IIC_24C0XX_SDA_High();
    for(i=0;i<8;i++ )
	  {
//			IIC_24C0XX_SDA_High();               //��ȡ���ݷ����������ɽ���ַ��Ϣ�󵱳����ݶ���   �������̳������� ��ע��˵����
//		 a = IIC_24C0XX_SDA_Status();
     IIC_24C0XX_SCL_Low(); 
     delay_us(2);
	   IIC_24C0XX_SCL_High();
     receive<<=1;           
		 IIC_24C0XX_SDA_High();                 //���������
		 a = IIC_24C0XX_SDA_Status();
     if(a)receive++;   
		delay_us(1); 
    }					
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}





















