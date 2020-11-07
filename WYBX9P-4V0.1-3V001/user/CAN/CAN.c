
/*****************************************
  CAN1 Remap
*****************************************/

#include "stm32f10x_can.h"
#include "CAN.h"
#include "usart.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>

void CAN1_Config(uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t pres)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;

    /* 打开GPIO时钟、AFIO时钟，CAN时钟 */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);


	  /* CAN1 RX PB8 */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		/* CAN1 TX PB9 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
							
							  	
    /* CAN  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler */
	  CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);   

    CAN_InitStructure.CAN_TTCM=DISABLE;
    CAN_InitStructure.CAN_ABOM=DISABLE;
    CAN_InitStructure.CAN_AWUM=DISABLE;
    CAN_InitStructure.CAN_NART=DISABLE;
    CAN_InitStructure.CAN_RFLM=DISABLE;
    CAN_InitStructure.CAN_TXFP=DISABLE;
    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
    CAN_InitStructure.CAN_SJW=sjw;
    CAN_InitStructure.CAN_BS1=bs1;  
    CAN_InitStructure.CAN_BS2=bs2;	
    CAN_InitStructure.CAN_Prescaler=pres;
    

    CAN_Init(CAN1,&CAN_InitStructure);	// CAN1											

    CAN_FilterInitStructure.CAN_FilterNumber=0;	 
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	 // 标识符屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   // 32位过滤器
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;			// 过滤器标识符
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;				
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;		// 过滤器屏蔽标识符
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	 // FIFO0指向过滤器
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);  // CAN1
		
		/* CAN1 Enabling interrupt */									  
    NVIC_InitStructure.NVIC_IRQChannel=CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);		

}

/*****************************************
  CAN2 Config
  FIFO_1	  
  返回：
*****************************************/
void CAN2_Config(uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t pres)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;

    /* 打开GPIO时钟、AFIO时钟，CAN时钟 */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

	
		/* CAN2 RX PB12 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		/* CAN2 TX PB13 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);		
	
							  	

    /* CAN  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler */
	  CAN_DeInit(CAN2);
    CAN_StructInit(&CAN_InitStructure);   

    CAN_InitStructure.CAN_TTCM=DISABLE;
    CAN_InitStructure.CAN_ABOM=DISABLE;
    CAN_InitStructure.CAN_AWUM=DISABLE;
    CAN_InitStructure.CAN_NART=DISABLE;
    CAN_InitStructure.CAN_RFLM=DISABLE;
    CAN_InitStructure.CAN_TXFP=DISABLE;
    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
    CAN_InitStructure.CAN_SJW=sjw;
    CAN_InitStructure.CAN_BS1=bs1;  
    CAN_InitStructure.CAN_BS2=bs2;	
    CAN_InitStructure.CAN_Prescaler=pres;

    CAN_Init(CAN2,&CAN_InitStructure);   // CAN2													

    CAN_FilterInitStructure.CAN_FilterNumber=14;	// 
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	 // 标识符屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   // 32位过滤器
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;			// 过滤器标识符
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;				
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;		// 过滤器屏蔽标识符
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO1;	 // FIFO1指向过滤器
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

	  CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);  // CAN2
	
		/* CAN2 Enabling interrupt */								 	  
    NVIC_InitStructure.NVIC_IRQChannel=CAN2_RX1_IRQn;	// FIFO_1
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
}


void Can_Send_LLC_Msg(u8* LLC_Msg)
{  
    u8 mbox2;
    u16 i2=0;
	  uint8_t retrys2=0;
    CanTxMsg TxMessage2;           
    TxMessage2.ExtId=0x1806E5F4;     
    TxMessage2.IDE=CAN_Id_Extended;  
    TxMessage2.RTR=CAN_RTR_Data;     
    TxMessage2.DLC=8;              
    for(i2=0;i2<8;i2++)
        TxMessage2.Data[i2]=LLC_Msg[i2];                     
	  do
	   {
	    mbox2=CAN_Transmit(CAN2, &TxMessage2);
		  retrys2++;
	   }
	while((mbox2==CAN_TxStatus_NoMailBox)&&(retrys2<0xFE));
	retrys2=0;	
    
}


void Can_Send_BMS_Msg(u8* BMS_Msg ,u8 Len , u32 CAN_ID)
{   
 u8 mbox;
    u16 i=0;
	  uint8_t retrys=0;
    CanTxMsg TxMessage;          
    TxMessage.ExtId=CAN_ID;     
    TxMessage.IDE=CAN_Id_Extended;  
    TxMessage.RTR=CAN_RTR_Data;     
    TxMessage.DLC=8;              
    for(i=0;i<Len;i++)
        TxMessage.Data[i]=BMS_Msg[i];                     
    do
	   {
	    mbox=CAN_Transmit(CAN1, &TxMessage);
		  retrys++;
	   }
	  while((mbox==CAN_TxStatus_NoMailBox)&&(retrys<0xFE));
	  retrys=0;
	  
  
}


