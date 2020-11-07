/*
* File:   main.c
* Author: WH
*Description :  LCDģ����ơ����ͨ����Ϣ��������ģ����Ϣ���ϼ���������
* Created on OCt 8, 2020
* History:
        < author >       < time >       < version >
	         WH            202010            V1.0
*/


#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_flash.h"
#include "stdio.h"
#include "misc.h"
#include "System_Init.h"
#include "Charger_Control.h"
#include "CAN.h"
#include "DWIN_LCD.h"
#include "PCF8563.h"
#include "Flash.h"
#include "24C0X.h"
#include "SD.h"
unsigned char uc_Protocol_HC_Flg = 0;
unsigned char uc_Protocol_WY_Flg = 1;

int main(void)
{
    System_Init(); //ϵͳ��ʼ��

    while(1)
    {
        if(ucTag10ms)
        {
            ucTag10ms = 0;
            LCD_Display();			                    //LCD��ʾ��Ϣ�ϴ�
            LCD_Page_Manage();                      //LCDҳ�������Ϣ��ȡ����
            if(uc_Protocol_HC_Flg)                  //����ͨ��Э�����ڽ϶� ���Ϊ10ms
            {
                CAN_Msg_Handle();
            }
        }
        if(ucTag100ms)
        {
            ucTag100ms = 0;
            OBC_Status_Manage();                    //OBC ״̬��⼰����
            if(uc_Protocol_WY_Flg)                  //����ͨ��Э�����ڶ�Ϊ100ms
            {
                CAN_Msg_Handle();
            }
            if((ucOBC_Work_Mode == OBC_Work_In_CAN_Mode)&&(LCD_Manual_Data != 0x01))   //CANͨѶ����ģ�鹤��
            {
                V_BMS_Set =CAN_BMS_RX_Data[0] * 256 + CAN_BMS_RX_Data[1];	            //BMS���������ѹ������Ϣת��
                I_BMS_Set =CAN_BMS_RX_Data[2] * 256 + CAN_BMS_RX_Data[3];
                Set_Output(V_BMS_Set,I_BMS_Set);                                      //�����ģ�鴮�ڷ������������ѹ��������
            }
            else if (ucOBC_Work_Mode == OBC_Work_In_MC_Mode)                        //ä��ģʽ���ƹ���
            {
                ChargeProcess();
            }

        }


    }



}





/****END OF FILE****/
