
#include "System_Init.h"
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_tim.h"
#include "stdio.h"
#include "usart.h"
#include "CAN.h"
#include "misc.h"
#include "stm32f10x_iwdg.h"
#include "Charger_Control.h"
#include "stm32f10x_spi.h"
#include "sd.h"	
unsigned char ucOBC_Work_Mode = 0;

float flashbuff[100] = {0x12345609, 0x12345601, 0x12345602, 0x12345603, 0x12345604};
u32 flashBuffConfig[100];


float DacVoltageValueAsTure;
float DacCurrentValueAsTure[3];
float DacVoltageValueOffset;
float DacCurrentValueOffset[3];

void System_Init(void)
{
  /* System Clocks Configuration */
  RCC_Configuration(); 
  // 定时器2配置
	Timer2_Init();
	// 串口2配置														
  USART2_Configuration();
  // CAN1 配置
  CAN1_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);  			   			  		  				                         			  
  // CAN2 配置
  CAN2_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);
	// IO口 配置
	Gpio_Init();
	//看门狗配置
	IWDG_Configuration();
	//SD卡SPI模式读取初始化
  SPI3_Init();		//初始化SPI,并配置SPI参数
  SD_SPI_SpeedLow();	//设置到低速模式(281.25KHz)
  //充电机输出模式控制
  ucOBC_Work_Mode = OBC_Work_In_CAN_Mode;
	//杭叉通信协议初始化参数
		CAN_Msg_StepState = CAN_Msg_In_Handshake;
		OBC_Err_type_Msg2.all = 0xF0;
		OBC_Err_type_Msg3.all = 0xF0;
		CAN_RX_OverTime_Msg1.all = 0xfc;
		CAN_RX_OverTime_Msg2.all = 0xf0;
		CAN_RX_OverTime_Msg3.all = 0xC0;
		CAN_RX_OverTime_Msg4.all = 0xfc;
}

void RCC_Configuration(void)
{   
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();						  			 
}

void Timer2_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);//
    /* PWM信号电平跳变值 */
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 36000;     //当定时器从0计数到36000，即为36000/72000=0.5毫秒/次，为一个定时周期
    TIM_TimeBaseStructure.TIM_Prescaler = 1;	   //设置预分频：2预分频，即为0.5*2=1毫秒
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//设置时钟分频系数：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);		/* 清除溢出中断标志 */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(TIM2, ENABLE);			 // 使能TIM4重载寄存器ARR
    TIM_Cmd(TIM2, ENABLE);                   //使能定时器1
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		 
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);//定时器溢出中断
}



void Gpio_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = RTC_SCL_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(RTC_SCL_Port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = RTC_SDAPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(RTC_SDA_Port, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = MCU_CTL_Led1_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(MCU_CTL_Led1_Port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = MCU_CTL_Led2_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(MCU_CTL_Led2_Port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = MCU_CTL_Led3_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(MCU_CTL_Led3_Port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = MCU_CTL_Led4_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(MCU_CTL_Led4_Port, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = MCU_CTL_Led5_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(MCU_CTL_Led5_Port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = MCU_CTL_Rly1_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(MCU_CTL_Rly1_Port, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = MCU_CTL_Rly2_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(MCU_CTL_Rly2_Port, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = IIC_24C0XX_SCL_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IIC_24C0XX_SCL_Port, &GPIO_InitStructure);

	
	GPIO_InitStructure.GPIO_Pin = IIC_24C0XX_SDAPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(IIC_24C0XX_SDA_Port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = IIC_24C0XX_Read_EN_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(IIC_24C0XX_Read_EN_Port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SD_CS_EN_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(SD_CS_EN_Port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB3/5复用推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB3/5复用推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	

	
}
/*******************************************************************************
 函数块名称:  void IWDG_Configuration(void)
 功能:       看门狗初始化
 输入参数:      		
 输出参数:       		 		
 其他说明:   看门狗定时时间计算  4*2^预分频系数*重装载值/40(ms)  看门狗时钟的频率为40K
             256*300/40=1920ms   (256分频 对应的预分频系数为6  6/7都是256分频)
*******************************************************************************/
void IWDG_Configuration(void)
{
	/* 写入0x5555,用于允许狗狗寄存器写入功能 */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	
	/* 狗狗时钟分频,40K/256=156HZ(6.4ms)*/
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	
	/* 喂狗时间 6.4MS*300=1920ms .注意不能大于0xfff*/
	IWDG_SetReload(300);
	
	/* 喂狗*/
	IWDG_ReloadCounter();
	
	/* 使能狗狗*/
	IWDG_Enable();
}

/*******************************************************************************
 函数块名称:  void reloadWDG(void)
 功能:       重新将预装载值赋给计数值
 输入参数:      		
 输出参数:       		 		
 其他说明:   定时器中断中每100ms赋值一次
*******************************************************************************/
void reloadWDG(void)
{
	IWDG_ReloadCounter();
}


void SPI3_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,  ENABLE);//SPI3时钟使能 	
  
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI3, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI3, ENABLE); //使能SPI外设
	
	SPI3_ReadWriteByte(0xff);//启动传输		 
 
  SD_CS_DISABLE();		//禁止片选
}   

//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频   
//SPI_BaudRatePrescaler_8   8分频   
//SPI_BaudRatePrescaler_16  16分频  
//SPI_BaudRatePrescaler_256 256分频 
  
void SPI3_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI3->CR1&=0XFFC7;
	SPI3->CR1|=SPI_BaudRatePrescaler;	//设置SPI3速度 
	SPI_Cmd(SPI3,ENABLE); 

} 

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI3_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI3, TxData); //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI3); //返回通过SPIx最近接收的数据					    
}


//void Parameter_Init(void)
//{
//  //读FLASH存的数据	
//	Readflash_stm(0X000, (u32 *)flashbuff, 100);
//	//是否为初次上电
//	if(*(u32 *)(&flashbuff[0]) == 0 || *(u32 *)(&flashbuff[0]) == 0xFFFFFFFF)
//	{ //初次上电，输出电流限定值计算系数、偏移量赋初值
//		flashbuff[0] = 0.0427;                  //输出电流限定值计算系数1
//		flashbuff[1] = 0;                       //输出电流限定值计算偏移量1
//		DacCurrentValueAsTure[0] = flashbuff[0];
//		DacCurrentValueOffset[0] = flashbuff[1];
//		flashbuff[8] = 0.0427;                  //输出电流限定值计算系数2
//		flashbuff[9] = 0;                       //输出电流限定值计算偏移量2
//		DacCurrentValueAsTure[1] = flashbuff[8];
//		DacCurrentValueOffset[1] = flashbuff[9];
//		flashbuff[10] = 0.0427;                 //输出电流限定值计算系数3
//		flashbuff[11] = 0;                      //输出电流限定值计算偏移量3
//		DacCurrentValueAsTure[2] = flashbuff[10];
//		DacCurrentValueOffset[2] = flashbuff[11];
//        //初次上电，输出电压限定值系数、偏移量赋初值
//		flashbuff[2] = 0.112;                   //输出电压限定值计算系数
//		flashbuff[3] = 0;                       //输出电压限定值计算偏移量
//		DacVoltageValueAsTure = flashbuff[2];
//		DacVoltageValueOffset = flashbuff[3];
//        //初次上电，ADC采样计算电流真实值参数赋初值
//		flashbuff[4] = 0.03072;                 //ADC采样电流真实值计算系数
//		flashbuff[5] = 0.1268;                  //ADC采样电流真实值计算偏移量
//		AdcCurrentValueAsTure = flashbuff[4];
//		AdcCurrentValueOffset = flashbuff[5];
//        //初次上电，ADC采样电压真实值计算系数、偏移量赋初值
//		flashbuff[6] = 0.026855;                //ADC采样电压真实值计算系数
//		flashbuff[7] = 0;                       //ADC采样电压真实值计算偏移量
//		AdcVoltageValueAsTure = flashbuff[6];
//		AdcVoltageValueOffset = flashbuff[7];        
//		Writeflash_stm(0x000, (u32 *)flashbuff, 100);
//	}
//	else
//	{ //不是初次上电，读取FLASH中数据到RAM
//		DacCurrentValueAsTure[0] = flashbuff[0];
//		DacCurrentValueOffset[0] = flashbuff[1];
//		DacCurrentValueAsTure[1] = flashbuff[8];
//		DacCurrentValueOffset[1] = flashbuff[9];
//		DacCurrentValueAsTure[2] = flashbuff[10];
//		DacCurrentValueOffset[2] = flashbuff[11];
//        //不是初次上电，读取FLASH中数据到RAM
//		DacVoltageValueAsTure = flashbuff[2];
//		DacVoltageValueOffset = flashbuff[3];
//        //不是初次上电，读取FLASH中数据到RAM
//		AdcCurrentValueAsTure= flashbuff[4];
//		AdcCurrentValueOffset = flashbuff[5];
//        //不是初次上电，读取FLASH中数据到RAM
//		AdcVoltageValueAsTure = flashbuff[6];
//		AdcVoltageValueOffset = flashbuff[7];
//	}
//    
//    Readflash_stm(0X000, flashBuffConfig, 100);
//    //未配置型号参数，初始配置为6015参数
//    if(flashBuffConfig[SetDefaultPreChargeStep1LimitTimeIndex] == 0xFFFFFFFF)
//    {
//        flashBuffConfig[SetDefaultPreChargeStep1LimitTimeIndex] = SetDefaultPreChargeStep1LimitTime;
//        flashBuffConfig[SetDefaultPreChargeStep1ChargeChangeVoltageIndex] = SetDefaultPreChargeStep1ChargeChangeVoltage;
//        flashBuffConfig[SetDefaultPreChargeStep1ChargeChangeCurrentIndex] = SetDefaultPreChargeStep1ChargeChangeCurrent;
//        flashBuffConfig[SetDefaultPreChargeStep1ChargeConstantVoltageIndex] = SetDefaultPreChargeStep1ChargeConstantVoltage;
//        flashBuffConfig[SetDefaultPreChargeStep1ChargeConstantCurrentIndex] = SetDefaultPreChargeStep1ChargeConstantCurrent;

//        /******************************************************************************************************/
//        //2015/12/02 增加预充电2阶段参数存储
//        flashBuffConfig[SetDefaultPreChargeStep2LimitTimeIndex] = SetDefaultPreChargeStep2LimitTime;
//        flashBuffConfig[SetDefaultPreChargeStep2ChargeChangeVoltageIndex] = SetDefaultPreChargeStep2ChargeChangeVoltage;
//        flashBuffConfig[SetDefaultPreChargeStep2ChargeChangeCurrentIndex] = SetDefaultPreChargeStep2ChargeChangeCurrent;
//        flashBuffConfig[SetDefaultPreChargeStep2ChargeConstantVoltageIndex] = SetDefaultPreChargeStep2ChargeConstantVoltage;
//        flashBuffConfig[SetDefaultPreChargeStep2ChargeConstantCurrentIndex] = SetDefaultPreChargeStep2ChargeConstantCurrent;
//        /******************************************************************************************************/
//    
//        flashBuffConfig[SetDefaultChangeStep2LimitTimeIndex] = SetDefaultChangeStep2LimitTime;
//        flashBuffConfig[SetDefaultChangeStep2ChargeChangeVoltageIndex] = SetDefaultChangeStep2ChargeChangeVoltage;
//        flashBuffConfig[SetDefaultChangeStep2ChargeChangeCurrentIndex] = SetDefaultChangeStep2ChargeChangeCurrent;
//        flashBuffConfig[SetDefaultChangeStep2ChargeConstantVoltageIndex] = SetDefaultChangeStep2ChargeConstantVoltage;
//        flashBuffConfig[SetDefaultChangeStep2ChargeConstantCurrentIndex] = SetDefaultChangeStep2ChargeConstantCurrent;
//    
//        flashBuffConfig[SetDefaultChangeStep3LimitTimeIndex] = SetDefaultChangeStep3LimitTime;
//        flashBuffConfig[SetDefaultChangeStep3ChargeChangeVoltageIndex] = SetDefaultChangeStep3ChargeChangeVoltage;
//        flashBuffConfig[SetDefaultChangeStep3ChargeChangeCurrentIndex] = SetDefaultChangeStep3ChargeChangeCurrent;
//        flashBuffConfig[SetDefaultChangeStep3ChargeConstantVoltageIndex] = SetDefaultChangeStep3ChargeConstantVoltage;
//        flashBuffConfig[SetDefaultChangeStep3ChargeConstantCurrentIndex] = SetDefaultChangeStep3ChargeConstantCurrent;
//    
//        flashBuffConfig[SetDefaultChangeStep4LimitTimeIndex] = SetDefaultChangeStep4LimitTime;
//        flashBuffConfig[SetDefaultChangeStep4ChargeChangeVoltageIndex] = SetDefaultChangeStep4ChargeChangeVoltage;
//        flashBuffConfig[SetDefaultChangeStep4ChargeChangeCurrentIndex] = SetDefaultChangeStep4ChargeChangeCurrent;
//        flashBuffConfig[SetDefaultChangeStep4ChargeConstantVoltageIndex] = SetDefaultChangeStep4ChargeConstantVoltage;
//        flashBuffConfig[SetDefaultChangeStep4ChargeConstantCurrentIndex] = SetDefaultChangeStep4ChargeConstantCurrent;
//        
//        flashBuffConfig[SetDefaultChangeStep5LimitTimeIndex] = SetDefaultChangeStep5LimitTime;
//        flashBuffConfig[SetDefaultChangeStep5ChargeChangeVoltageIndex] = SetDefaultChangeStep5ChargeChangeVoltage;
//        flashBuffConfig[SetDefaultChangeStep5ChargeChangeCurrentIndex] = SetDefaultChangeStep5ChargeChangeCurrent;
//        flashBuffConfig[SetDefaultChangeStep5ChargeConstantVoltageIndex] = SetDefaultChangeStep5ChargeConstantVoltage;
//        flashBuffConfig[SetDefaultChangeStep5ChargeConstantCurrentIndex] = SetDefaultChangeStep5ChargeConstantCurrent;
//        
//        flashBuffConfig[SetDefaultChangeStep6LimitTimeIndex] = SetDefaultChangeStep6LimitTime;
//        flashBuffConfig[SetDefaultChangeStep6ChargeChangeVoltageIndex] = SetDefaultChangeStep6ChargeChangeVoltage;
//        flashBuffConfig[SetDefaultChangeStep6ChargeChangeCurrentIndex] = SetDefaultChangeStep6ChargeChangeCurrent;
//        flashBuffConfig[SetDefaultChangeStep6ChargeConstantVoltageIndex] = SetDefaultChangeStep6ChargeConstantVoltage;
//        flashBuffConfig[SetDefaultChangeStep6ChargeConstantCurrentIndex] = SetDefaultChangeStep6ChargeConstantCurrent;
//        
//        /****************************************************************************************************/
//        //2015/12/12增加温度补偿配置
//        //温度补偿基准温度
//        flashBuffConfig[START_TEMP_INDEX] = 20;
//        //温度补偿系数 毫伏/度
//        flashBuffConfig[TEMP1_INDEX] = 0;
//        flashBuffConfig[TEMP2_INDEX] = 18;
//        flashBuffConfig[TEMP3_INDEX] = 18;
//        flashBuffConfig[TEMP4_INDEX] = 18;
//        flashBuffConfig[TEMP5_INDEX] = 18;
//        flashBuffConfig[TEMP6_INDEX] = 18;
//        flashBuffConfig[TEMP7_INDEX] = 0;
//        /****************************************************************************************************/
//        
//        /****************************************************************************************************/
//        flashBuffConfig[PROTECT_VOLTAGE_INDEX] = 9000;              //保护点电压值
//        flashBuffConfig[PROTECT_CURRENT_INDEX] = 3000;              //保护点电流值
//        /****************************************************************************************************/
//        
//        /****************************************************************************************************/
//        flashBuffConfig[IN_PUT_VOLTAGE_LOW_INDEX] = 0;
//        flashBuffConfig[IN_PUT_VOLTAGE_HIGH_INDEX] = 15000;
//        /****************************************************************************************************/
//        Writeflash_stm(0x000, flashBuffConfig, 100);

//    }
//	


//}

