
#include "DWIN_LCD.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "usart.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>
#include "Charger_Control.h"

unsigned char usart_cmd_status = 0;
unsigned char usart_reset_cnt = 0;
unsigned char usart_data_received_len;
unsigned char usart_crc_temp;

unsigned char LCD_usart_cmd_status = 0;
unsigned char LCD_usart_data_received_len;

unsigned char LCD_Rx_Data[110] = {0};  //LCD串口接收数据数组

u16 LCD_Page_Msg_Data = 0;   //LCD当前页面数据
u8 LCD_Page_MSg_Flg = 0;       //接收到LCD发送的页面信息标志

u16 LCD_Manual_Data = 0;   //LCD手动设置输出按钮信息
u8 LCD_Manual_Flg = 0;       //LCD手动设置输出按钮信息信息标志
unsigned int uiLCD_Msg_Addr = 0;

u16 LCD_Manual_V_OutputData = 0;   //LCD手动设置电压输出参数信息
u16 LCD_Manual_I_OutputData = 0;   //LCD手动设置电流输出参数信息

u16 LCD_Manual_ReturnData = 0;   //LCD手动设置返回主页面参数
u16 LCD_Manual_CrcData = 0;     //LCD进入手动设置校验参数

u16 LCD_Manual_SetData = 0;     //LCD进入手动设置按钮返回参数
u8 LCD_Year = 0;                    //LCD时间参数
u8 LCD_Month = 0;
u8 LCD_Date = 0;
u8 LCD_Hour = 0;
u8 LCD_Minute = 0;
u8 LCD_Seconds = 0;

u16 LCD_Set_Year = 0;                    //LCD设置时间参数
u16 LCD_Set_Month = 0;
u16 LCD_Set_Date = 0;
u16 LCD_Set_Hour = 0;
u16 LCD_Set_Minute = 0;
u16 LCD_Set_Seconds = 0;


u8  Usart2_TxBuffer[110];
u8  Usart2_RxBuffer[110];
/*
  USART2配置
*/
void USART2_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 打开GPIOA时钟、AFIO时钟，USART2时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	/* USART2 TX PA2 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* USART2 RX PA3 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
											
	/* USART 配置 */
	USART_DeInit(USART2);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	/* 使能USART2收中断 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  

	/* 使能USART2 */
	USART_Cmd(USART2, ENABLE);
	   
  
  /* Enable the USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			 //串口中断设置
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void USART3_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 打开GPIOA时钟、AFIO时钟，USART2时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	/* USART3 TX PC10 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* USART3 RX PC11 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
											
	/* USART3 配置 */
	USART_DeInit(USART3);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	/* 使能USART3收发中断 */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	   

	/* 使能USART2 */
	USART_Cmd(USART3, ENABLE);
	/* 清除发送完成标志 */
	USART_ClearFlag(USART3, USART_FLAG_TC);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		   
  
    /* Enable the USART2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;			 //串口中断设置
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
}


void USART5_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 打开GPIOC GPIOD时钟、AFIO时钟，UART5时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	/* USART5 TX PC12 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* USART5 RX PD2 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
											
	/* USART 配置 */
	USART_DeInit(UART5);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure);
	/* 使能USART2收发中断 */
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	   

	/* 使能USART2 */
	USART_Cmd(UART5, ENABLE);
	/* 清除发送完成标志 */
	USART_ClearFlag(UART5, USART_FLAG_TC);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		   
  
    /* Enable the USART2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;			 //串口中断设置
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
  串口发送函数
*/
void USART_Send(USART_TypeDef* USARTx, uint8_t *Dat,uint16_t len)
{
  uint16_t i;
  for(i=0;i<len;i++)
  {
  	USART_SendData(USARTx, Dat[i]);
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);     
  }
}

/*
  串口发送字符串
*/
void USART_STR(USART_TypeDef* USARTx,char *str)
{
    uint8_t len,i;
	len=strlen(str);
	for(i=0;i<len;i++)
	{
	  	USART_SendData(USARTx, str[i]);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET); 	
	}
}

/*******************************************************************************
 函数块名称:  void LCD_Usart_Deal(unsigned char LCD_Usart_dat)
 功能:        接收LCD反馈的数据
 输入参数:      		
 输出参数:       		 		
 其他说明:     LCD反馈数据格式  5A + A5 +反馈数据长度（几个字节） +0x82（命令） +地址 （两字节）+  反馈数据长度（几个字）+  反馈数据
*******************************************************************************/

void LCD_Usart_Deal(unsigned char LCD_Usart_dat)
{
	
	//识别串口命令格式
	switch(LCD_usart_cmd_status)
	{
	case LCD_USART_HEAD1:
	{
		if(LCD_Usart_dat == 0x5A)//命令起始0x5A      
		{
			LCD_Rx_Data[0] = LCD_Usart_dat;
			LCD_usart_cmd_status = LCD_USART_HEAD2;
		}
		break;
	}
	case LCD_USART_HEAD2:
	{
		if(LCD_Usart_dat == 0xA5)//命令第二帧0xA5    桢头判断  5A A5 
		{
			LCD_Rx_Data[1] = LCD_Usart_dat;
			LCD_usart_cmd_status = LCD_USART_LEN;
		}	
		break;
	}
	case LCD_USART_LEN:                    //判断接收到到的数据长度
	{  
		LCD_Rx_Data[2] = LCD_Usart_dat;
		if(LCD_Rx_Data[2] <= 4)              //滤除 发送数据后LCD反馈的  5A A5 03 82 4F 4B的接收反馈帧
		{
		LCD_usart_cmd_status = LCD_USART_END;
		}
		else
		{
		LCD_usart_cmd_status = LCD_USART_CMD;
		}
		break;
	}
	case LCD_USART_CMD:                    //反馈命令判断
	{
		LCD_Rx_Data[3] = LCD_Usart_dat;
	  LCD_usart_cmd_status = LCD_USART_Data;
		LCD_usart_data_received_len = 1;
		break;
	}
	case LCD_USART_Data:                  //反馈数据
	{
		LCD_Rx_Data[LCD_usart_data_received_len + 3] = LCD_Usart_dat;
		LCD_usart_data_received_len++;
		if(LCD_usart_data_received_len >= LCD_Rx_Data[2])   //以反馈数据字节数判断接收完成依据
		{
	  LCD_usart_cmd_status = LCD_USART_END;
		}
		break;
	}

  case LCD_USART_END:               
	{
		uiLCD_Msg_Addr = LCD_Rx_Data[4];
		uiLCD_Msg_Addr = (uiLCD_Msg_Addr << 8) + LCD_Rx_Data[5];
		if(uiLCD_Msg_Addr == Page_Msg_Addr)                              //区分反馈的数据信息  0x0014 为页面信息
		{
		LCD_Page_MSg_Flg = 1;
		LCD_Page_Msg_Data = LCD_Rx_Data[7];
		LCD_Page_Msg_Data = (LCD_Page_Msg_Data << 8) + LCD_Rx_Data[8];
		}
		if(uiLCD_Msg_Addr == Manual_Start_Stop_Addr)                    //区分反馈的数据信息  0x1110 为手动控制输出按钮
		{
		LCD_Manual_Flg = 1;
		LCD_Manual_Data = LCD_Rx_Data[7];
		LCD_Manual_Data = (LCD_Manual_Data << 8) +LCD_Rx_Data[8];
		} 
		if(uiLCD_Msg_Addr == Manual_Set_V_Output_Addr)            //区分反馈的数据信息  0x10F0 为手动控制输出电压参数
		{ 
		LCD_Manual_V_OutputData = LCD_Rx_Data[7];
		LCD_Manual_V_OutputData = (LCD_Manual_V_OutputData << 8) + LCD_Rx_Data[8];
		}
		if(uiLCD_Msg_Addr == Manual_Set_I_Output_Addr)           //区分反馈的数据信息  0x1100 为手动控制输出电流参数
		{ 
		LCD_Manual_I_OutputData = LCD_Rx_Data[7];
		LCD_Manual_I_OutputData = (LCD_Manual_I_OutputData << 8) + LCD_Rx_Data[8];
		}
		if(uiLCD_Msg_Addr == Manual_Set_Reurn_Addr)                //区分反馈的数据信息  0x1120 为手动控制返回主页面参数
		{ 
		LCD_Manual_ReturnData = LCD_Rx_Data[7];
		LCD_Manual_ReturnData = (LCD_Manual_ReturnData << 8) + LCD_Rx_Data[8];
		}
		if(uiLCD_Msg_Addr == Manual_Set_Password_Addr)       //区分反馈的数据信息  0x1190 为进入手动控制校验参数
		{ 
		LCD_Manual_CrcData = LCD_Rx_Data[7];
		LCD_Manual_CrcData = (LCD_Manual_CrcData << 8) + LCD_Rx_Data[8];
		}
		if(uiLCD_Msg_Addr == Manual_Set_Parameter_Addr)      //区分反馈的数据信息  0x11A0 为进入参数设置按钮返回值
		{ 
		LCD_Manual_SetData = LCD_Rx_Data[7];
		LCD_Manual_SetData = (LCD_Manual_SetData << 8) + LCD_Rx_Data[8];
		}
		if(uiLCD_Msg_Addr == Date_Read_Addr)                            //区分反馈的数据信息  0x0010 为时间参数
		{ 
		LCD_Year = LCD_Rx_Data[7];
		LCD_Month = LCD_Rx_Data[8];
		LCD_Date = LCD_Rx_Data[9];
		LCD_Hour = LCD_Rx_Data[11];
		LCD_Minute = LCD_Rx_Data[12];
		LCD_Seconds = LCD_Rx_Data[13];
		}
		
		if(uiLCD_Msg_Addr == LCD_Set_Year_Addr)                            //区分反馈的数据信息  0x1130 为时间设置参数
		{ 
		LCD_Set_Year = LCD_Rx_Data[7];
		LCD_Set_Year = (LCD_Set_Year << 8) + LCD_Rx_Data[8];
		}
		if(uiLCD_Msg_Addr == LCD_Set_Month_Addr)                            //区分反馈的数据信息  0x1140 为时间设置参数
		{ 
		LCD_Set_Month = LCD_Rx_Data[7];
		LCD_Set_Month = (LCD_Set_Month << 8) + LCD_Rx_Data[8];
		}
		if(uiLCD_Msg_Addr == LCD_Set_Date_Addr)                            //区分反馈的数据信息  0x1150 为时间设置参数
		{ 
		LCD_Set_Date = LCD_Rx_Data[7];
		LCD_Set_Date = (LCD_Set_Date << 8) + LCD_Rx_Data[8];
		}
		if(uiLCD_Msg_Addr == LCD_Set_Hour_Addr)                            //区分反馈的数据信息  0x1160 为时间设置参数
		{ 
		LCD_Set_Hour = LCD_Rx_Data[7];
		LCD_Set_Hour = (LCD_Set_Hour << 8) + LCD_Rx_Data[8];
		}
		if(uiLCD_Msg_Addr == LCD_Set_Minute_Addr)                            //区分反馈的数据信息  0x1170 为时间设置参数
		{ 
		LCD_Set_Minute = LCD_Rx_Data[7];
		LCD_Set_Minute = (LCD_Set_Minute << 8) + LCD_Rx_Data[8];
		}
		if(uiLCD_Msg_Addr == LCD_Set_Seconnds_Addr)                            //区分反馈的数据信息  0x1180 为时间设置参数
		{ 
		LCD_Set_Seconds = LCD_Rx_Data[7];
		LCD_Set_Seconds = (LCD_Set_Seconds << 8) + LCD_Rx_Data[8];
		}
		
		LCD_usart_cmd_status = LCD_USART_HEAD1;
		break;
	}
	}
}




unsigned char Usart_Deal(unsigned char usart_dat)
{
	unsigned return_value = 0;//返回值定义
	
	//连续收到5个0xFF串口接收状态复位
	if(usart_dat == 0xFF)
	{
		usart_reset_cnt++;
		if(usart_reset_cnt >= 3)
		{
			usart_cmd_status = USART_HEAD;
			usart_reset_cnt = 0;
		}
	}
	else
	{
		usart_reset_cnt = 0;
	}
	
	//识别串口命令格式
	switch(usart_cmd_status)
	{
	case USART_HEAD:
	{
		if(usart_dat == 0xAA)//命令起始0xAA
		{
			Usart2_RxBuffer[0] = usart_dat;
			usart_crc_temp = 0;
			usart_cmd_status = USART_CMD;
		}
		break;
	}
	case USART_CMD:
	{
		Usart2_RxBuffer[1] = usart_dat;
		usart_cmd_status = USART_LEN;		
		break;
	}
	case USART_LEN:
	{
		Usart2_RxBuffer[2] = usart_dat;
		usart_crc_temp += Usart2_RxBuffer[2];
		
		if(Usart2_RxBuffer[2] <= 2)
		{
			usart_cmd_status = USART_CRC;
		}
		else
		{
			usart_cmd_status = USART_DATA;
			usart_data_received_len = 0;
		}
		break;
	}
	case USART_DATA:
	{
		Usart2_RxBuffer[usart_data_received_len + 3] = usart_dat;//存储数据
		usart_crc_temp += Usart2_RxBuffer[usart_data_received_len + 3];//计算和校验
		usart_data_received_len++;//已接收数据递增
		if(usart_data_received_len >= Usart2_RxBuffer[2] - 2)//数据长度等于命令长度减校验、结束两字节
		{
			usart_cmd_status = USART_CRC;//数据接收完成，进入接收校验字节
		}
		break;
	}
	case USART_CRC:
	{
		Usart2_RxBuffer[usart_data_received_len + 3] = usart_dat;
		if(Usart2_RxBuffer[usart_data_received_len + 3] == usart_crc_temp)
		{
			usart_cmd_status = USART_END;
		}
		else
		{
			usart_cmd_status = USART_HEAD;
		}
		break;
	}
	case USART_END:
	{
		if(usart_dat == 0x55) //命令结束0x55
		{
			return_value = 1;
		}
		usart_cmd_status = USART_HEAD;
		break;
	}
	}
	return return_value;
}



//void usart_cmd_process(unsigned char *receive_dat)
//{
//	float currentTemp,voltageTemp;//输出电压电流值
//	unsigned int iAdcCurrentTemp,iAdcVoltageTemp;//读取电流电压值，精确到小数点后两位
//	
//	unsigned char i;//循环变量
//	unsigned char processedCharLen;
//	
//	switch(receive_dat[1])
//	{
//	case SET_OUTPUT://设置电流电压输出
//	{
//		voltageTemp = (float)((receive_dat[3] - '0') * 10 + (receive_dat[4] - '0'));
//		currentTemp = (float)((receive_dat[5] - '0') * 10 + (receive_dat[6] - '0'));
//        //vSetOutCurrentAsTure(currentTemp);
//		SetOutCurrentAsTrue(currentTemp);
//        SetOutVoltageAsTrue(voltageTemp);
//        C_EnFanRelay;
//        C_EnChargerOutPutRelay;
//		vDelay5Ms(100);
//		C_EnL25600Work;
//		
//		coefStatus = TRUE;
//		//发送命令反馈
//		TxBuffer1[0] = 0xAA;
//		TxBuffer1[1] = SET_OUTPUT;
//		TxBuffer1[2] = 0x02;
//		TxBuffer1[3] = 0x02;
//		TxBuffer1[4] = 0x55;
//		UART1SendData(TxBuffer1, 5);
//		break;
//	}
//	case SET_OUTPUT_COEF://设置输出系数
//	{
//		processedCharLen = 0;
//		for(i=0;i<RxBuffer1[2];i++)
//		{
//			if(RxBuffer1[i+3] == 0x3B)//两个浮点数以“;”隔开
//			{
//				DacCurrentValueAsTure[0] = stringToFloat(RxBuffer1 + 3, i);
//				processedCharLen = i + 1;
//				break;
//			}
//		}
//		for(i=processedCharLen;i<RxBuffer1[2];i++)
//		{
//			if(RxBuffer1[i+3] == 0x3B)//两个浮点数以“;”隔开
//			{
//				DacCurrentValueOffset[0] = stringToFloat(RxBuffer1 + processedCharLen + 3, i - processedCharLen);
//				processedCharLen = i + 1;
//				break;
//			}
//		}
//		for(i=processedCharLen;i<RxBuffer1[2];i++)
//		{
//			if(RxBuffer1[i+3] == 0x3B)//两个浮点数以“;”隔开
//			{
//				DacCurrentValueAsTure[1] = stringToFloat(RxBuffer1 + processedCharLen + 3, i - processedCharLen);
//				processedCharLen = i + 1;
//				break;
//			}
//		}
//		for(i=processedCharLen;i<RxBuffer1[2];i++)
//		{
//			if(RxBuffer1[i+3] == 0x3B)//两个浮点数以“;”隔开
//			{
//				DacCurrentValueOffset[1] = stringToFloat(RxBuffer1 + processedCharLen + 3, i - processedCharLen);
//				processedCharLen = i + 1;
//				break;
//			}
//		}
//		for(i=processedCharLen;i<RxBuffer1[2];i++)
//		{
//			if(RxBuffer1[i+3] == 0x3B)//两个浮点数以“;”隔开
//			{
//				DacCurrentValueAsTure[2] = stringToFloat(RxBuffer1 + processedCharLen + 3, i - processedCharLen);
//				processedCharLen = i + 1;
//				break;
//			}
//		}
//		for(i=processedCharLen;i<RxBuffer1[2];i++)
//		{
//			if(RxBuffer1[i+3] == 0x3B)//两个浮点数以“;”隔开
//			{
//				DacCurrentValueOffset[2] = stringToFloat(RxBuffer1 + processedCharLen + 3, i - processedCharLen);
//				processedCharLen = i + 1;
//				break;
//			}
//		}
//		for(i=processedCharLen;i<RxBuffer1[2];i++)
//		{
//			if(RxBuffer1[i+3] == 0x3B)//两个浮点数以“;”隔开
//			{
//				DacVoltageValueAsTure = stringToFloat(RxBuffer1 + processedCharLen + 3, i - processedCharLen);
//				processedCharLen = i + 1;
//				break;
//			}
//		}
//		DacVoltageValueOffset = stringToFloat(RxBuffer1 + processedCharLen + 3, RxBuffer1[2] - processedCharLen - 2);
//		
//		//读FLASH存的数据	
//		Readflash_stm(0X000, (u32 *)flashbuff, 100);
//		flashbuff[0] = DacCurrentValueAsTure[0];
//		flashbuff[1] = DacCurrentValueOffset[0];
//		flashbuff[8] = DacCurrentValueAsTure[1];
//		flashbuff[9] = DacCurrentValueOffset[1];
//		flashbuff[10] = DacCurrentValueAsTure[2];
//		flashbuff[11] = DacCurrentValueOffset[2];
//		flashbuff[2] = DacVoltageValueAsTure;
//		flashbuff[3] = DacVoltageValueOffset;
//		//电压电流计算系数写入FLASH
//		Writeflash_stm(0x000, (u32 *)flashbuff, 100);
//		
//		//发送命令反馈
//		TxBuffer1[0] = 0xAA;
//		TxBuffer1[1] = SET_OUTPUT_COEF;
//		TxBuffer1[2] = 0x02;
//		TxBuffer1[3] = 0x02;
//		TxBuffer1[4] = 0x55;
//		UART1SendData(TxBuffer1, 5);
//		break;
//	}
//	case SET_ADC_COEF://设置ADC采样系数
//	{
//		processedCharLen = 0;
//		for(i=0;i<RxBuffer1[2];i++)
//		{
//			if(RxBuffer1[i+3] == 0x3B)//两个浮点数以“;”隔开
//			{
//				AdcCurrentValueAsTure = stringToFloat(RxBuffer1 + 3, i);
//				processedCharLen = i + 1;
//				break;
//			}
//		}
//		for(i=processedCharLen;i<RxBuffer1[2];i++)
//		{
//			if(RxBuffer1[i+3] == 0x3B)//两个浮点数以“;”隔开
//			{
//				AdcCurrentValueOffset = stringToFloat(RxBuffer1 + processedCharLen + 3, i - processedCharLen);
//				processedCharLen = i + 1;
//				break;
//			}
//		}
//		for(i=processedCharLen;i<RxBuffer1[2];i++)
//		{
//			if(RxBuffer1[i+3] == 0x3B)//两个浮点数以“;”隔开
//			{
//				AdcVoltageValueAsTure = stringToFloat(RxBuffer1 + processedCharLen + 3, i - processedCharLen);
//				processedCharLen = i + 1;
//				break;
//			}
//		}
//		AdcVoltageValueOffset = stringToFloat(RxBuffer1 + processedCharLen + 3, RxBuffer1[2] - processedCharLen - 2);
//		
//		//读FLASH存的数据	
//		Readflash_stm(0X000, (u32 *)flashbuff, 100);
//		flashbuff[4] = AdcCurrentValueAsTure;
//		flashbuff[5] = AdcCurrentValueOffset;
//		flashbuff[6] = AdcVoltageValueAsTure;
//		flashbuff[7] = AdcVoltageValueOffset;
//		//电压电流计算系数写入FLASH
//		Writeflash_stm(0x000, (u32 *)flashbuff, 100);
//		
//		//发送命令反馈
//		TxBuffer1[0] = 0xAA;
//		TxBuffer1[1] = SET_ADC_COEF;
//		TxBuffer1[2] = 0x02;
//		TxBuffer1[3] = 0x02;
//		TxBuffer1[4] = 0x55;
//		UART1SendData(TxBuffer1, 5);
//		break;
//	}
//	case SET_CONFIG:		//设置型号参数
//	{
//		Readflash_stm(0X000, flashBuffConfig, 100);

//		flashBuffConfig[SetDefaultPreChargeStep1LimitTimeIndex] = (RxBuffer1[3] << 24) + (RxBuffer1[4] << 16) + (RxBuffer1[5] << 8) + RxBuffer1[6];
//		flashBuffConfig[SetDefaultPreChargeStep1ChargeChangeVoltageIndex] = (RxBuffer1[7] << 8) + RxBuffer1[8];
//		flashBuffConfig[SetDefaultPreChargeStep1ChargeChangeCurrentIndex] = (RxBuffer1[9] << 8) + RxBuffer1[10];
//		flashBuffConfig[SetDefaultPreChargeStep1ChargeConstantVoltageIndex] = (RxBuffer1[11] << 8) + RxBuffer1[12];
//		flashBuffConfig[SetDefaultPreChargeStep1ChargeConstantCurrentIndex] = (RxBuffer1[13] << 8) + RxBuffer1[14];
//			
//		flashBuffConfig[SetDefaultChangeStep2LimitTimeIndex] = (RxBuffer1[15] << 24) + (RxBuffer1[16] << 16) + (RxBuffer1[17] << 8) + RxBuffer1[18];
//		flashBuffConfig[SetDefaultChangeStep2ChargeChangeVoltageIndex] = (RxBuffer1[19] << 8) + RxBuffer1[20];
//		flashBuffConfig[SetDefaultChangeStep2ChargeChangeCurrentIndex] = (RxBuffer1[21] << 8) + RxBuffer1[22];
//		flashBuffConfig[SetDefaultChangeStep2ChargeConstantVoltageIndex] = (RxBuffer1[23] << 8) + RxBuffer1[24];
//		flashBuffConfig[SetDefaultChangeStep2ChargeConstantCurrentIndex] = (RxBuffer1[25] << 8) + RxBuffer1[26];
//	
//		flashBuffConfig[SetDefaultChangeStep3LimitTimeIndex] = (RxBuffer1[27] << 24) + (RxBuffer1[28] << 16) + (RxBuffer1[29] << 8) + RxBuffer1[30];
//		flashBuffConfig[SetDefaultChangeStep3ChargeChangeVoltageIndex] = (RxBuffer1[31] << 8) + RxBuffer1[32];
//		flashBuffConfig[SetDefaultChangeStep3ChargeChangeCurrentIndex] = (RxBuffer1[33] << 8) + RxBuffer1[34];
//		flashBuffConfig[SetDefaultChangeStep3ChargeConstantVoltageIndex] = (RxBuffer1[35] << 8) + RxBuffer1[36];
//		flashBuffConfig[SetDefaultChangeStep3ChargeConstantCurrentIndex] = (RxBuffer1[37] << 8) + RxBuffer1[38];
//			
//		flashBuffConfig[SetDefaultChangeStep4LimitTimeIndex] = (RxBuffer1[39] << 24) + (RxBuffer1[40] << 16) + (RxBuffer1[41] << 8) + RxBuffer1[42];
//		flashBuffConfig[SetDefaultChangeStep4ChargeChangeVoltageIndex] = (RxBuffer1[43] << 8) + RxBuffer1[44];
//		flashBuffConfig[SetDefaultChangeStep4ChargeChangeCurrentIndex] = (RxBuffer1[45] << 8) + RxBuffer1[46];
//		flashBuffConfig[SetDefaultChangeStep4ChargeConstantVoltageIndex] = (RxBuffer1[47] << 8) + RxBuffer1[48];
//		flashBuffConfig[SetDefaultChangeStep4ChargeConstantCurrentIndex] = (RxBuffer1[49] << 8) + RxBuffer1[50];
//		
//		flashBuffConfig[SetDefaultChangeStep5LimitTimeIndex] = (RxBuffer1[51] << 24) + (RxBuffer1[52] << 16) + (RxBuffer1[53] << 8) + RxBuffer1[54];
//		flashBuffConfig[SetDefaultChangeStep5ChargeChangeVoltageIndex] = (RxBuffer1[55] << 8) + RxBuffer1[56];
//		flashBuffConfig[SetDefaultChangeStep5ChargeChangeCurrentIndex] = (RxBuffer1[57] << 8) + RxBuffer1[58];
//		flashBuffConfig[SetDefaultChangeStep5ChargeConstantVoltageIndex] = (RxBuffer1[59] << 8) + RxBuffer1[60];
//		flashBuffConfig[SetDefaultChangeStep5ChargeConstantCurrentIndex] = (RxBuffer1[61] << 8) + RxBuffer1[62];
//		
//		flashBuffConfig[SetDefaultChangeStep6LimitTimeIndex] = (RxBuffer1[63] << 24) + (RxBuffer1[64] << 16) + (RxBuffer1[65] << 8) + RxBuffer1[66];
//		flashBuffConfig[SetDefaultChangeStep6ChargeChangeVoltageIndex] = (RxBuffer1[67] << 8) + RxBuffer1[68];
//		flashBuffConfig[SetDefaultChangeStep6ChargeChangeCurrentIndex] = (RxBuffer1[69] << 8) + RxBuffer1[70];
//		flashBuffConfig[SetDefaultChangeStep6ChargeConstantVoltageIndex] = (RxBuffer1[71] << 8) + RxBuffer1[72];
//		flashBuffConfig[SetDefaultChangeStep6ChargeConstantCurrentIndex] = (RxBuffer1[73] << 8) + RxBuffer1[74];
//		
//		/********************************************************************************************************/
//		//2015/12/02 增加预充电2阶段参数配置
//		flashBuffConfig[SetDefaultPreChargeStep2LimitTimeIndex] = (RxBuffer1[75] << 24) + (RxBuffer1[76] << 16) + (RxBuffer1[77] << 8) + RxBuffer1[78];
//		flashBuffConfig[SetDefaultPreChargeStep2ChargeChangeVoltageIndex] = (RxBuffer1[79] << 8) + RxBuffer1[80];
//		flashBuffConfig[SetDefaultPreChargeStep2ChargeChangeCurrentIndex] = (RxBuffer1[81] << 8) + RxBuffer1[82];
//		flashBuffConfig[SetDefaultPreChargeStep2ChargeConstantVoltageIndex] = (RxBuffer1[83] << 8) + RxBuffer1[84];
//		flashBuffConfig[SetDefaultPreChargeStep2ChargeConstantCurrentIndex] = (RxBuffer1[85] << 8) + RxBuffer1[86];
//		/********************************************************************************************************/
//		
//		/********************************************************************************************************/
//		//2015/12/02增加型号配置说明
//		flashBuffConfig[TYPE1_INDEX] = RxBuffer1[87];
//		flashBuffConfig[TYPE2_INDEX] = RxBuffer1[88];
//		flashBuffConfig[TYPE3_INDEX] = RxBuffer1[89];
//		flashBuffConfig[TYPE4_INDEX] = RxBuffer1[90];
//		flashBuffConfig[TYPE5_INDEX] = RxBuffer1[91];
//		/********************************************************************************************************/

//		/********************************************************************************************************/
//		//2012/12/12增加温度补偿配置
//		//温度补偿起始点设置
//		flashBuffConfig[START_TEMP_INDEX] = RxBuffer1[92];
//		//温度补偿系数 毫伏/度
//		flashBuffConfig[TEMP1_INDEX] = RxBuffer1[93];				//第一段预充电阶段1
//		flashBuffConfig[TEMP2_INDEX] = RxBuffer1[94];				//第二段
//		flashBuffConfig[TEMP3_INDEX] = RxBuffer1[95];				//第三段
//		flashBuffConfig[TEMP4_INDEX] = RxBuffer1[96];				//第四段
//		flashBuffConfig[TEMP5_INDEX] = RxBuffer1[97];				//第五段
//		flashBuffConfig[TEMP6_INDEX] = RxBuffer1[98];				//第六段
//		flashBuffConfig[TEMP7_INDEX] = RxBuffer1[99];				//第一段预充电阶段2
//		/********************************************************************************************************/
//        
//        /********************************************************************************************************/
//        flashBuffConfig[LED_CONFIG_INDEX] = RxBuffer1[100];         //LED配置
//        //保护点电压电流配置
//        flashBuffConfig[PROTECT_VOLTAGE_INDEX] = (RxBuffer1[101]<<8) + RxBuffer1[102];
//        flashBuffConfig[PROTECT_CURRENT_INDEX] = (RxBuffer1[103]<<8) + RxBuffer1[104];
// 
//        flashBuffConfig[IN_PUT_VOLTAGE_LOW_INDEX] = (RxBuffer1[105]<<8) + RxBuffer1[106];
//        flashBuffConfig[IN_PUT_VOLTAGE_HIGH_INDEX] = (RxBuffer1[107]<<8) + RxBuffer1[108];
//        /********************************************************************************************************/ 
//        /********************************************************************************************************/
//		Writeflash_stm(0x000, flashBuffConfig, 100);
//		
//		//发送命令反馈
//		TxBuffer1[0] = 0xAA;
//		TxBuffer1[1] = SET_CONFIG;
//		TxBuffer1[2] = 0x02;
//		TxBuffer1[3] = 0x02;
//		TxBuffer1[4] = 0x55;
//		UART1SendData(TxBuffer1, 5);
//		break;
//	}
//	case GET_OUTPUT_COEF://读取输出系数
//	{
//		TxBuffer1[0] = 0xAA;
//		TxBuffer1[1] = GET_OUTPUT_COEF;
//		TxBuffer1[2] = 0x22;
//		
//		Readflash_stm(0X000, (u32 *)flashbuff, 100);
//		
//		for(i=0;i<16;i++)
//		{
//			TxBuffer1[i+3] = *(((unsigned char *)flashbuff) + i);
//		}
//		for(i=16;i<32;i++)
//		{
//			TxBuffer1[i+3] = *(((unsigned char *)flashbuff) + i + 16);
//		}	
//		TxBuffer1[35] = 0;
//		
//		for(i=2;i<35;i++)
//		{
//			TxBuffer1[35] += TxBuffer1[i];
//		}
//		TxBuffer1[36] = 0x55;
//		
//		UART1SendData(TxBuffer1, 37);
//		
//		break;
//	}

//	case GET_ADC_COEF://读取ADC采样系数
//	{
//		TxBuffer1[0] = 0xAA;
//		TxBuffer1[1] = GET_ADC_COEF;
//		TxBuffer1[2] = 0x12;
//		
//		Readflash_stm(0X000, (u32 *)flashbuff, 100);
//		
//		for(i=0;i<16;i++)
//		{
//			TxBuffer1[i+3] = *(((unsigned char *)flashbuff) + i + 16);
//		}
//	
//		TxBuffer1[19] = 0;
//		
//		for(i=2;i<19;i++)
//		{
//			TxBuffer1[19] += TxBuffer1[i];
//		}
//		TxBuffer1[20] = 0x55;
//		
//		UART1SendData(TxBuffer1, 21);
//		break;
//	}
//	case GET_CONFIG://读取型号配置参数
//	{
//		//发送命令反馈
//		TxBuffer1[0] = 0xAA;
//		TxBuffer1[1] = GET_CONFIG;
//		TxBuffer1[2] = 104;
//		
//		Readflash_stm(0X000, flashBuffConfig, 100);
//		//第一段参数
//		//超时时间
//		TxBuffer1[3] = (u8)(flashBuffConfig[SetDefaultPreChargeStep1LimitTimeIndex] >> 24);
//		TxBuffer1[4] = (u8)(flashBuffConfig[SetDefaultPreChargeStep1LimitTimeIndex] >> 16);
//		TxBuffer1[5] = (u8)(flashBuffConfig[SetDefaultPreChargeStep1LimitTimeIndex] >> 8);
//		TxBuffer1[6] = (u8)flashBuffConfig[SetDefaultPreChargeStep1LimitTimeIndex];
//		//转换点电压
//		TxBuffer1[7] = (u8)(flashBuffConfig[SetDefaultPreChargeStep1ChargeChangeVoltageIndex] >> 8);
//		TxBuffer1[8] = (u8)flashBuffConfig[SetDefaultPreChargeStep1ChargeChangeVoltageIndex];
//		//转换点电流
//		TxBuffer1[9] = (u8)(flashBuffConfig[SetDefaultPreChargeStep1ChargeChangeCurrentIndex] >> 8);
//		TxBuffer1[10] = (u8)flashBuffConfig[SetDefaultPreChargeStep1ChargeChangeCurrentIndex];
//		//恒压充电电压
//		TxBuffer1[11] = (u8)(flashBuffConfig[SetDefaultPreChargeStep1ChargeConstantVoltageIndex] >> 8);
//		TxBuffer1[12] = (u8)flashBuffConfig[SetDefaultPreChargeStep1ChargeConstantVoltageIndex];
//		//恒流充电电流
//		TxBuffer1[13] = (u8)(flashBuffConfig[SetDefaultPreChargeStep1ChargeConstantCurrentIndex] >> 8);
//		TxBuffer1[14] = (u8)flashBuffConfig[SetDefaultPreChargeStep1ChargeConstantCurrentIndex];
//		
//		//第一段参数
//		//超时时间
//		TxBuffer1[15] = (u8)(flashBuffConfig[SetDefaultChangeStep2LimitTimeIndex] >> 24);
//		TxBuffer1[16] = (u8)(flashBuffConfig[SetDefaultChangeStep2LimitTimeIndex] >> 16);
//		TxBuffer1[17] = (u8)(flashBuffConfig[SetDefaultChangeStep2LimitTimeIndex] >> 8);
//		TxBuffer1[18] = (u8)flashBuffConfig[SetDefaultChangeStep2LimitTimeIndex];
//		//转换点电压
//		TxBuffer1[19] = (u8)(flashBuffConfig[SetDefaultChangeStep2ChargeChangeVoltageIndex] >> 8);
//		TxBuffer1[20] = (u8)flashBuffConfig[SetDefaultChangeStep2ChargeChangeVoltageIndex];
//		//转换点电流
//		TxBuffer1[21] = (u8)(flashBuffConfig[SetDefaultChangeStep2ChargeChangeCurrentIndex] >> 8);
//		TxBuffer1[22] = (u8)flashBuffConfig[SetDefaultChangeStep2ChargeChangeCurrentIndex];
//		//恒压充电电压
//		TxBuffer1[23] = (u8)(flashBuffConfig[SetDefaultChangeStep2ChargeConstantVoltageIndex] >> 8);
//		TxBuffer1[24] = (u8)flashBuffConfig[SetDefaultChangeStep2ChargeConstantVoltageIndex];
//		//恒流充电电流
//		TxBuffer1[25] = (u8)(flashBuffConfig[SetDefaultChangeStep2ChargeConstantCurrentIndex] >> 8);
//		TxBuffer1[26] = (u8)flashBuffConfig[SetDefaultChangeStep2ChargeConstantCurrentIndex];
//		
//		
//		//第一段参数
//		//超时时间
//		TxBuffer1[27] = (u8)(flashBuffConfig[SetDefaultChangeStep3LimitTimeIndex] >> 24);
//		TxBuffer1[28] = (u8)(flashBuffConfig[SetDefaultChangeStep3LimitTimeIndex] >> 16);
//		TxBuffer1[29] = (u8)(flashBuffConfig[SetDefaultChangeStep3LimitTimeIndex] >> 8);
//		TxBuffer1[30] = (u8)flashBuffConfig[SetDefaultChangeStep3LimitTimeIndex];
//		//转换点电压
//		TxBuffer1[31] = (u8)(flashBuffConfig[SetDefaultChangeStep3ChargeChangeVoltageIndex] >> 8);
//		TxBuffer1[32] = (u8)flashBuffConfig[SetDefaultChangeStep3ChargeChangeVoltageIndex];
//		//转换点电流
//		TxBuffer1[33] = (u8)(flashBuffConfig[SetDefaultChangeStep3ChargeChangeCurrentIndex] >> 8);
//		TxBuffer1[34] = (u8)flashBuffConfig[SetDefaultChangeStep3ChargeChangeCurrentIndex];
//		//恒压充电电压
//		TxBuffer1[35] = (u8)(flashBuffConfig[SetDefaultChangeStep3ChargeConstantVoltageIndex] >> 8);
//		TxBuffer1[36] = (u8)flashBuffConfig[SetDefaultChangeStep3ChargeConstantVoltageIndex];
//		//恒流充电电流
//		TxBuffer1[37] = (u8)(flashBuffConfig[SetDefaultChangeStep3ChargeConstantCurrentIndex] >> 8);
//		TxBuffer1[38] = (u8)flashBuffConfig[SetDefaultChangeStep3ChargeConstantCurrentIndex];
//		
//		//第一段参数
//		//超时时间
//		TxBuffer1[39] = (u8)(flashBuffConfig[SetDefaultChangeStep4LimitTimeIndex] >> 24);
//		TxBuffer1[40] = (u8)(flashBuffConfig[SetDefaultChangeStep4LimitTimeIndex] >> 16);
//		TxBuffer1[41] = (u8)(flashBuffConfig[SetDefaultChangeStep4LimitTimeIndex] >> 8);
//		TxBuffer1[42] = (u8)flashBuffConfig[SetDefaultChangeStep4LimitTimeIndex];
//		//转换点电压
//		TxBuffer1[43] = (u8)(flashBuffConfig[SetDefaultChangeStep4ChargeChangeVoltageIndex] >> 8);
//		TxBuffer1[44] = (u8)flashBuffConfig[SetDefaultChangeStep4ChargeChangeVoltageIndex];
//		//转换点电流
//		TxBuffer1[45] = (u8)(flashBuffConfig[SetDefaultChangeStep4ChargeChangeCurrentIndex] >> 8);
//		TxBuffer1[46] = (u8)flashBuffConfig[SetDefaultChangeStep4ChargeChangeCurrentIndex];
//		//恒压充电电压
//		TxBuffer1[47] = (u8)(flashBuffConfig[SetDefaultChangeStep4ChargeConstantVoltageIndex] >> 8);
//		TxBuffer1[48] = (u8)flashBuffConfig[SetDefaultChangeStep4ChargeConstantVoltageIndex];
//		//恒流充电电流
//		TxBuffer1[49] = (u8)(flashBuffConfig[SetDefaultChangeStep4ChargeConstantCurrentIndex] >> 8);
//		TxBuffer1[50] = (u8)flashBuffConfig[SetDefaultChangeStep4ChargeConstantCurrentIndex];
//		
//		//第一段参数
//		//超时时间
//		TxBuffer1[51] = (u8)(flashBuffConfig[SetDefaultChangeStep5LimitTimeIndex] >> 24);
//		TxBuffer1[52] = (u8)(flashBuffConfig[SetDefaultChangeStep5LimitTimeIndex] >> 16);
//		TxBuffer1[53] = (u8)(flashBuffConfig[SetDefaultChangeStep5LimitTimeIndex] >> 8);
//		TxBuffer1[54] = (u8)flashBuffConfig[SetDefaultChangeStep5LimitTimeIndex];
//		//转换点电压
//		TxBuffer1[55] = (u8)(flashBuffConfig[SetDefaultChangeStep5ChargeChangeVoltageIndex] >> 8);
//		TxBuffer1[56] = (u8)flashBuffConfig[SetDefaultChangeStep5ChargeChangeVoltageIndex];
//		//转换点电流
//		TxBuffer1[57] = (u8)(flashBuffConfig[SetDefaultChangeStep5ChargeChangeCurrentIndex] >> 8);
//		TxBuffer1[58] = (u8)flashBuffConfig[SetDefaultChangeStep5ChargeChangeCurrentIndex];
//		//恒压充电电压
//		TxBuffer1[59] = (u8)(flashBuffConfig[SetDefaultChangeStep5ChargeConstantVoltageIndex] >> 8);
//		TxBuffer1[60] = (u8)flashBuffConfig[SetDefaultChangeStep5ChargeConstantVoltageIndex];
//		//恒流充电电流
//		TxBuffer1[61] = (u8)(flashBuffConfig[SetDefaultChangeStep5ChargeConstantCurrentIndex] >> 8);
//		TxBuffer1[62] = (u8)flashBuffConfig[SetDefaultChangeStep5ChargeConstantCurrentIndex];
//		
//		//第一段参数
//		//超时时间
//		TxBuffer1[63] = (u8)(flashBuffConfig[SetDefaultChangeStep6LimitTimeIndex] >> 24);
//		TxBuffer1[64] = (u8)(flashBuffConfig[SetDefaultChangeStep6LimitTimeIndex] >> 16);
//		TxBuffer1[65] = (u8)(flashBuffConfig[SetDefaultChangeStep6LimitTimeIndex] >> 8);
//		TxBuffer1[66] = (u8)flashBuffConfig[SetDefaultChangeStep6LimitTimeIndex];
//		//转换点电压
//		TxBuffer1[67] = (u8)(flashBuffConfig[SetDefaultChangeStep6ChargeChangeVoltageIndex] >> 8);
//		TxBuffer1[68] = (u8)flashBuffConfig[SetDefaultChangeStep6ChargeChangeVoltageIndex];
//		//转换点电流
//		TxBuffer1[69] = (u8)(flashBuffConfig[SetDefaultChangeStep6ChargeChangeCurrentIndex] >> 8);
//		TxBuffer1[70] = (u8)flashBuffConfig[SetDefaultChangeStep6ChargeChangeCurrentIndex];
//		//恒压充电电压
//		TxBuffer1[71] = (u8)(flashBuffConfig[SetDefaultChangeStep6ChargeConstantVoltageIndex] >> 8);
//		TxBuffer1[72] = (u8)flashBuffConfig[SetDefaultChangeStep6ChargeConstantVoltageIndex];
//		//恒流充电电流
//		TxBuffer1[73] = (u8)(flashBuffConfig[SetDefaultChangeStep6ChargeConstantCurrentIndex] >> 8);
//		TxBuffer1[74] = (u8)flashBuffConfig[SetDefaultChangeStep6ChargeConstantCurrentIndex];
//		
//		/********************************************************************************************************/
//		//2015/12/02增加预充电2阶段参数读取
//		//第一段参数2
//		//超时时间
//		TxBuffer1[75] = (u8)(flashBuffConfig[SetDefaultPreChargeStep2LimitTimeIndex] >> 24);
//		TxBuffer1[76] = (u8)(flashBuffConfig[SetDefaultPreChargeStep2LimitTimeIndex] >> 16);
//		TxBuffer1[77] = (u8)(flashBuffConfig[SetDefaultPreChargeStep2LimitTimeIndex] >> 8);
//		TxBuffer1[78] = (u8)flashBuffConfig[SetDefaultPreChargeStep2LimitTimeIndex];
//		//转换点电压
//		TxBuffer1[79] = (u8)(flashBuffConfig[SetDefaultPreChargeStep2ChargeChangeVoltageIndex] >> 8);
//		TxBuffer1[80] = (u8)flashBuffConfig[SetDefaultPreChargeStep2ChargeChangeVoltageIndex];
//		//转换点电流
//		TxBuffer1[81] = (u8)(flashBuffConfig[SetDefaultPreChargeStep2ChargeChangeCurrentIndex] >> 8);
//		TxBuffer1[82] = (u8)flashBuffConfig[SetDefaultPreChargeStep2ChargeChangeCurrentIndex];
//		//恒压充电电压
//		TxBuffer1[83] = (u8)(flashBuffConfig[SetDefaultPreChargeStep2ChargeConstantVoltageIndex] >> 8);
//		TxBuffer1[84] = (u8)flashBuffConfig[SetDefaultPreChargeStep2ChargeConstantVoltageIndex];
//		//恒流充电电流
//		TxBuffer1[85] = (u8)(flashBuffConfig[SetDefaultPreChargeStep2ChargeConstantCurrentIndex] >> 8);
//		TxBuffer1[86] = (u8)flashBuffConfig[SetDefaultPreChargeStep2ChargeConstantCurrentIndex];
//		/********************************************************************************************************/
//		
//		/********************************************************************************************************/
//		//增加型号配置说明
//		TxBuffer1[87] = flashBuffConfig[TYPE1_INDEX];
//		TxBuffer1[88] = flashBuffConfig[TYPE2_INDEX];
//		TxBuffer1[89] = flashBuffConfig[TYPE3_INDEX];
//		TxBuffer1[90] = flashBuffConfig[TYPE4_INDEX];
//		TxBuffer1[91] = flashBuffConfig[TYPE5_INDEX];
//		/********************************************************************************************************/
//		
//		/********************************************************************************************************/
//		//2015/12/12增加温度补偿配置读取
//		//温度补偿起始点设置读取
//		TxBuffer1[92] = flashBuffConfig[START_TEMP_INDEX];
//		//温度补偿系数毫伏/度读取
//		TxBuffer1[93] = flashBuffConfig[TEMP1_INDEX];			//第一段预充阶段1
//		TxBuffer1[94] = flashBuffConfig[TEMP2_INDEX];			//第二段
//		TxBuffer1[95] = flashBuffConfig[TEMP3_INDEX];			//第三段
//		TxBuffer1[96] = flashBuffConfig[TEMP4_INDEX];			//第四段
//		TxBuffer1[97] = flashBuffConfig[TEMP5_INDEX];			//第五段
//		TxBuffer1[98] = flashBuffConfig[TEMP6_INDEX];			//第六段
//		TxBuffer1[99] = flashBuffConfig[TEMP7_INDEX];			//第一段预充阶段2
//		/********************************************************************************************************/
//        
//		/********************************************************************************************************/
//        TxBuffer1[100] = flashBuffConfig[LED_CONFIG_INDEX];
//        
//        TxBuffer1[101] = (u8)(flashBuffConfig[PROTECT_VOLTAGE_INDEX] >> 8);
//        TxBuffer1[102] = (u8)(flashBuffConfig[PROTECT_VOLTAGE_INDEX]);
//        TxBuffer1[103] = (u8)(flashBuffConfig[PROTECT_CURRENT_INDEX] >> 8);
//        TxBuffer1[104] = (u8)(flashBuffConfig[PROTECT_CURRENT_INDEX]);
//        TxBuffer1[105] = (u8)(flashBuffConfig[IN_PUT_VOLTAGE_LOW_INDEX] >> 8);
//        TxBuffer1[106] = (u8)(flashBuffConfig[IN_PUT_VOLTAGE_LOW_INDEX]);
//        TxBuffer1[107] = (u8)(flashBuffConfig[IN_PUT_VOLTAGE_HIGH_INDEX] >> 8);
//        TxBuffer1[108] = (u8)(flashBuffConfig[IN_PUT_VOLTAGE_HIGH_INDEX]);
//        /********************************************************************************************************/
//		for(i=2;i<109;i++)
//		{
//			TxBuffer1[109] += TxBuffer1[i];
//		}		
//		TxBuffer1[110] = 0x55;
//		UART1SendData(TxBuffer1, 111);
//	}
//	case GET_ADC_DATA://读取ADC采样数据
//	{
//		TxBuffer1[0] = 0xAA;
//		TxBuffer1[1] = GET_ADC_DATA;
//		TxBuffer1[2] = 0x0E;
//		fGetBattVoltage = fGetOutVoltageAsTrue();
//		fSampleCurrent = fGetOutCurrentAsTrue();
//		
//		iAdcVoltageTemp = (unsigned int)(fGetBattVoltage * 100);
//		iAdcCurrentTemp = (unsigned int)(fSampleCurrent * 100);
//		
//		TxBuffer1[3] = iAdcVoltageTemp / 1000 + '0';
//		iAdcVoltageTemp = iAdcVoltageTemp % 1000;
//		TxBuffer1[4] = iAdcVoltageTemp / 100 + '0';
//		iAdcVoltageTemp = iAdcVoltageTemp % 100;
//		TxBuffer1[5] = iAdcVoltageTemp / 10 + '0';
//		TxBuffer1[6] = iAdcVoltageTemp % 10 + '0';
//		TxBuffer1[7] = 0x0D;
//		TxBuffer1[8] = 0x0A;		

//		TxBuffer1[9] = iAdcCurrentTemp / 1000 + '0';
//		iAdcCurrentTemp  = iAdcCurrentTemp % 1000;
//		TxBuffer1[10] = iAdcCurrentTemp / 100 + '0';
//		iAdcCurrentTemp  = iAdcCurrentTemp % 100;
//		TxBuffer1[11] = iAdcCurrentTemp / 10 + '0';
//		TxBuffer1[12] = iAdcCurrentTemp % 10 + '0';
//		TxBuffer1[13] = 0x0D;
//		TxBuffer1[14] = 0x0A;
//		
//		TxBuffer1[15] = 0x00;
//		
//        for(i=2;i<15;i++)
//		{
//			TxBuffer1[15] += TxBuffer1[i];
//		}
//		TxBuffer1[16] = 0x55;
//		
//		UART1SendData(TxBuffer1, 17);
//		break;
//	}
//	case END_ADJ://退出标定状态
//	{
//        C_DisEnFanRelay;
//        C_DisEnChargerOutPutRelay;
//        vSetOutCurrentAsTure(0);
//        SetOutVoltageAsTrue(0);
//		coefStatus = FALSE;
//		//发送命令反馈
//		TxBuffer1[0] = 0xAA;
//		TxBuffer1[1] = END_ADJ;
//		TxBuffer1[2] = 0x02;
//		TxBuffer1[3] = 0x02;
//		TxBuffer1[4] = 0x55;
//		UART1SendData(TxBuffer1, 5);
//		break;
//	}
//    case SET_INFO:
//    {
//        break;
//    }
//    case GET_INFO:
//    {
//        sprintf(cTxData, "午阳充电机,机型:WYMF2C,固件版本号:V0.4005,通用固件");
//        vSendStr(cTxData);        
//        break;
//    }
//    case ERROR_REPORT:
//    {
//        TxBuffer1[0] = 0xAA;
//        TxBuffer1[1] = ERROR_REPORT;
//        TxBuffer1[2] = 52;
//        readErrorReport(TxBuffer1 + 3);
//        TxBuffer1[53] = 0;
//        for(i=2;i<53;i++)
//        {
//            TxBuffer1[53] += TxBuffer1[i];
//        }
//        TxBuffer1[54] = 0x55;
//        UART1SendData(TxBuffer1, 55);
//    }
//    #ifdef RS_DATA
//    case CHARGER_CMD:
//    {
//        for(i=0;i<8;i++)
//		{
//			RxMessage.Data[i] = RxBuffer1[i+3];
//		}
//        canCmd = TRUE;
//        ucTagTimeOut = FALSE;
//        break;
//    }
//    #endif
//	}
//}


///*******************************************************************************
//*字符串转化为浮点数
//*
//*参数：unsigned char *buffer指向一个数据去的指针，内存放需要转化的字符串。
//*      unsigned char strLen字符串长度
//*
//*返回：转化完成的浮点型数据
//*******************************************************************************/
//float stringToFloat(unsigned char *buffer,unsigned char strLen)
//{
//	unsigned char i,j,k;
//	float floatTemp, signedTemp, mathTemp = 0;
//	
//	floatTemp = 0;
//	if(buffer[0] == '-')
//	{
//		buffer++;
//		strLen--;
//		signedTemp = -1;
//	}
//	else
//	{
//		signedTemp = 1;
//	}
//	
//	for(i=0;i<strLen;i++)
//	{
//		if(buffer[i] == 0x2E)
//		{
//			for(j=0;j<i;j++)
//			{
//				mathTemp = 1;
//				for(k=1;k<i-j;k++)
//				{
//					mathTemp = mathTemp * 10;
//				}
//				floatTemp += (buffer[j]-0x30) * mathTemp;
//			}
//			break;
//		}
//	}
//	for(j=1;j<strLen-i;j++)
//	{
//		mathTemp = 1;
//		for(k=0;k<j;k++)
//		{
//			mathTemp = mathTemp *0.1;
//		}
//		floatTemp += (buffer[i+j] - 0x30) * mathTemp;
//	}
//	floatTemp *= signedTemp;
//	
//	return floatTemp;
//	
//}



