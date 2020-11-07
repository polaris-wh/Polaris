/*
* File:   main.c
* Author: WH
*Description :  LCD模块控制、外界通信信息处理、主控模块信息整合及参数传递
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
    System_Init(); //系统初始化

    while(1)
    {
        if(ucTag10ms)
        {
            ucTag10ms = 0;
            LCD_Display();			                    //LCD显示信息上传
            LCD_Page_Manage();                      //LCD页面管理及信息读取处理
            if(uc_Protocol_HC_Flg)                  //杭叉通信协议周期较多 最快为10ms
            {
                CAN_Msg_Handle();
            }
        }
        if(ucTag100ms)
        {
            ucTag100ms = 0;
            OBC_Status_Manage();                    //OBC 状态监测及处理
            if(uc_Protocol_WY_Flg)                  //午阳通信协议周期定为100ms
            {
                CAN_Msg_Handle();
            }
            if((ucOBC_Work_Mode == OBC_Work_In_CAN_Mode)&&(LCD_Manual_Data != 0x01))   //CAN通讯控制模块工作
            {
                V_BMS_Set =CAN_BMS_RX_Data[0] * 256 + CAN_BMS_RX_Data[1];	            //BMS请求输出电压电流信息转换
                I_BMS_Set =CAN_BMS_RX_Data[2] * 256 + CAN_BMS_RX_Data[3];
                Set_Output(V_BMS_Set,I_BMS_Set);                                      //向输出模块串口发送设置输出电压电流参数
            }
            else if (ucOBC_Work_Mode == OBC_Work_In_MC_Mode)                        //盲充模式控制工作
            {
                ChargeProcess();
            }

        }


    }



}





/****END OF FILE****/
