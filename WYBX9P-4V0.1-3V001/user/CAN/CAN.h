
#ifndef __CAN_H__
#define __CAN_H__

#include "stm32f10x.h"


// CAN波特率  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler 
#define    SET_CAN_SJW   CAN_SJW_1tq
#define    SET_CAN_BS1   CAN_BS1_7tq
#define    SET_CAN_BS2   CAN_BS2_8tq	
#define    SET_CAN_PRES  9				// 波特率分频器 9-250K 18-125K 




void CAN1_Config(uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t pres);
void CAN2_Config(uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t pres);

void Can_Send_LLC_Msg(u8* LLC_Msg);
void Can_Send_BMS_Msg(u8* BMS_Msg ,u8 Len , u32 CAN_ID);

extern uint8_t CAN_1_RX_Data[8];
extern uint8_t CAN_2_RX_Data[8];
#endif
