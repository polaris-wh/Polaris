
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
  // ��ʱ��2����
	Timer2_Init();
	// ����2����														
  USART2_Configuration();
  // CAN1 ����
  CAN1_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);  			   			  		  				                         			  
  // CAN2 ����
  CAN2_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);
	// IO�� ����
	Gpio_Init();
	//���Ź�����
	IWDG_Configuration();
	//SD��SPIģʽ��ȡ��ʼ��
  SPI3_Init();		//��ʼ��SPI,������SPI����
  SD_SPI_SpeedLow();	//���õ�����ģʽ(281.25KHz)
  //�������ģʽ����
  ucOBC_Work_Mode = OBC_Work_In_CAN_Mode;
	//����ͨ��Э���ʼ������
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
    /* PWM�źŵ�ƽ����ֵ */
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 36000;     //����ʱ����0������36000����Ϊ36000/72000=0.5����/�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = 1;	   //����Ԥ��Ƶ��2Ԥ��Ƶ����Ϊ0.5*2=1����
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//����ʱ�ӷ�Ƶϵ��������Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);		/* �������жϱ�־ */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(TIM2, ENABLE);			 // ʹ��TIM4���ؼĴ���ARR
    TIM_Cmd(TIM2, ENABLE);                   //ʹ�ܶ�ʱ��1
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		 
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);//��ʱ������ж�
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB3/5����������� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB3/5����������� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	

	
}
/*******************************************************************************
 ����������:  void IWDG_Configuration(void)
 ����:       ���Ź���ʼ��
 �������:      		
 �������:       		 		
 ����˵��:   ���Ź���ʱʱ�����  4*2^Ԥ��Ƶϵ��*��װ��ֵ/40(ms)  ���Ź�ʱ�ӵ�Ƶ��Ϊ40K
             256*300/40=1920ms   (256��Ƶ ��Ӧ��Ԥ��Ƶϵ��Ϊ6  6/7����256��Ƶ)
*******************************************************************************/
void IWDG_Configuration(void)
{
	/* д��0x5555,�����������Ĵ���д�빦�� */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	
	/* ����ʱ�ӷ�Ƶ,40K/256=156HZ(6.4ms)*/
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	
	/* ι��ʱ�� 6.4MS*300=1920ms .ע�ⲻ�ܴ���0xfff*/
	IWDG_SetReload(300);
	
	/* ι��*/
	IWDG_ReloadCounter();
	
	/* ʹ�ܹ���*/
	IWDG_Enable();
}

/*******************************************************************************
 ����������:  void reloadWDG(void)
 ����:       ���½�Ԥװ��ֵ��������ֵ
 �������:      		
 �������:       		 		
 ����˵��:   ��ʱ���ж���ÿ100ms��ֵһ��
*******************************************************************************/
void reloadWDG(void)
{
	IWDG_ReloadCounter();
}


void SPI3_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,  ENABLE);//SPI3ʱ��ʹ�� 	
  
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI3, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI3, ENABLE); //ʹ��SPI����
	
	SPI3_ReadWriteByte(0xff);//��������		 
 
  SD_CS_DISABLE();		//��ֹƬѡ
}   

//SPI �ٶ����ú���
//SpeedSet:
//SPI_BaudRatePrescaler_2   2��Ƶ   
//SPI_BaudRatePrescaler_8   8��Ƶ   
//SPI_BaudRatePrescaler_16  16��Ƶ  
//SPI_BaudRatePrescaler_256 256��Ƶ 
  
void SPI3_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI3->CR1&=0XFFC7;
	SPI3->CR1|=SPI_BaudRatePrescaler;	//����SPI3�ٶ� 
	SPI_Cmd(SPI3,ENABLE); 

} 

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI3_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI3, TxData); //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI3); //����ͨ��SPIx������յ�����					    
}


//void Parameter_Init(void)
//{
//  //��FLASH�������	
//	Readflash_stm(0X000, (u32 *)flashbuff, 100);
//	//�Ƿ�Ϊ�����ϵ�
//	if(*(u32 *)(&flashbuff[0]) == 0 || *(u32 *)(&flashbuff[0]) == 0xFFFFFFFF)
//	{ //�����ϵ磬��������޶�ֵ����ϵ����ƫ��������ֵ
//		flashbuff[0] = 0.0427;                  //��������޶�ֵ����ϵ��1
//		flashbuff[1] = 0;                       //��������޶�ֵ����ƫ����1
//		DacCurrentValueAsTure[0] = flashbuff[0];
//		DacCurrentValueOffset[0] = flashbuff[1];
//		flashbuff[8] = 0.0427;                  //��������޶�ֵ����ϵ��2
//		flashbuff[9] = 0;                       //��������޶�ֵ����ƫ����2
//		DacCurrentValueAsTure[1] = flashbuff[8];
//		DacCurrentValueOffset[1] = flashbuff[9];
//		flashbuff[10] = 0.0427;                 //��������޶�ֵ����ϵ��3
//		flashbuff[11] = 0;                      //��������޶�ֵ����ƫ����3
//		DacCurrentValueAsTure[2] = flashbuff[10];
//		DacCurrentValueOffset[2] = flashbuff[11];
//        //�����ϵ磬�����ѹ�޶�ֵϵ����ƫ��������ֵ
//		flashbuff[2] = 0.112;                   //�����ѹ�޶�ֵ����ϵ��
//		flashbuff[3] = 0;                       //�����ѹ�޶�ֵ����ƫ����
//		DacVoltageValueAsTure = flashbuff[2];
//		DacVoltageValueOffset = flashbuff[3];
//        //�����ϵ磬ADC�������������ʵֵ��������ֵ
//		flashbuff[4] = 0.03072;                 //ADC����������ʵֵ����ϵ��
//		flashbuff[5] = 0.1268;                  //ADC����������ʵֵ����ƫ����
//		AdcCurrentValueAsTure = flashbuff[4];
//		AdcCurrentValueOffset = flashbuff[5];
//        //�����ϵ磬ADC������ѹ��ʵֵ����ϵ����ƫ��������ֵ
//		flashbuff[6] = 0.026855;                //ADC������ѹ��ʵֵ����ϵ��
//		flashbuff[7] = 0;                       //ADC������ѹ��ʵֵ����ƫ����
//		AdcVoltageValueAsTure = flashbuff[6];
//		AdcVoltageValueOffset = flashbuff[7];        
//		Writeflash_stm(0x000, (u32 *)flashbuff, 100);
//	}
//	else
//	{ //���ǳ����ϵ磬��ȡFLASH�����ݵ�RAM
//		DacCurrentValueAsTure[0] = flashbuff[0];
//		DacCurrentValueOffset[0] = flashbuff[1];
//		DacCurrentValueAsTure[1] = flashbuff[8];
//		DacCurrentValueOffset[1] = flashbuff[9];
//		DacCurrentValueAsTure[2] = flashbuff[10];
//		DacCurrentValueOffset[2] = flashbuff[11];
//        //���ǳ����ϵ磬��ȡFLASH�����ݵ�RAM
//		DacVoltageValueAsTure = flashbuff[2];
//		DacVoltageValueOffset = flashbuff[3];
//        //���ǳ����ϵ磬��ȡFLASH�����ݵ�RAM
//		AdcCurrentValueAsTure= flashbuff[4];
//		AdcCurrentValueOffset = flashbuff[5];
//        //���ǳ����ϵ磬��ȡFLASH�����ݵ�RAM
//		AdcVoltageValueAsTure = flashbuff[6];
//		AdcVoltageValueOffset = flashbuff[7];
//	}
//    
//    Readflash_stm(0X000, flashBuffConfig, 100);
//    //δ�����ͺŲ�������ʼ����Ϊ6015����
//    if(flashBuffConfig[SetDefaultPreChargeStep1LimitTimeIndex] == 0xFFFFFFFF)
//    {
//        flashBuffConfig[SetDefaultPreChargeStep1LimitTimeIndex] = SetDefaultPreChargeStep1LimitTime;
//        flashBuffConfig[SetDefaultPreChargeStep1ChargeChangeVoltageIndex] = SetDefaultPreChargeStep1ChargeChangeVoltage;
//        flashBuffConfig[SetDefaultPreChargeStep1ChargeChangeCurrentIndex] = SetDefaultPreChargeStep1ChargeChangeCurrent;
//        flashBuffConfig[SetDefaultPreChargeStep1ChargeConstantVoltageIndex] = SetDefaultPreChargeStep1ChargeConstantVoltage;
//        flashBuffConfig[SetDefaultPreChargeStep1ChargeConstantCurrentIndex] = SetDefaultPreChargeStep1ChargeConstantCurrent;

//        /******************************************************************************************************/
//        //2015/12/02 ����Ԥ���2�׶β����洢
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
//        //2015/12/12�����¶Ȳ�������
//        //�¶Ȳ�����׼�¶�
//        flashBuffConfig[START_TEMP_INDEX] = 20;
//        //�¶Ȳ���ϵ�� ����/��
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
//        flashBuffConfig[PROTECT_VOLTAGE_INDEX] = 9000;              //�������ѹֵ
//        flashBuffConfig[PROTECT_CURRENT_INDEX] = 3000;              //���������ֵ
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

