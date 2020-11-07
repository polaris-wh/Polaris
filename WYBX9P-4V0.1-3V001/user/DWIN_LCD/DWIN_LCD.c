
#include "Charger_Control.h"
#include "DWIN_LCD.h"
#include "System_Init.h"
#include "usart.h"
#include "stm32f10x_usart.h"
#include "CAN.h"
#include "PCF8563.h"

unsigned char ucReadLCD_MsgStep = 0;   //每次发送一帧 读取一个地址数据  否则串口接收异常
unsigned char ucDateStep = 0;          //LCD日期参数分次读取
unsigned char uc_Incident_Open_OBC_FLg = 1; //充电机显示开机事件标志

unsigned int uiPFC_Temp1 = 0;          //OBC5个内部温度AD值
unsigned int uiPFC_Temp2 = 0;
unsigned int uiPFC_Temp3 = 0;
unsigned int uiLLC_Temp1 = 0;
unsigned int uiLLC_Temp2 = 0;

unsigned int uiPFC_Temp1_AsTrue = 0;  //OBC内部温度换算为真是温度参数
unsigned int uiPFC_Temp2_AsTrue = 0;
unsigned int uiPFC_Temp3_AsTrue = 0;
unsigned int uiLLC_Temp1_AsTrue = 0;
unsigned int uiLLC_Temp2_AsTrue = 0;

unsigned int uiV_Output = 0;    //充电机输出电压参数  100mv单位
unsigned int uiI_Output = 0;    //充电机输出电电流参数 100mA单位
unsigned int uiV_Input = 0;
unsigned int uiI_Input = 0;
unsigned int uiV_Bus = 0;



void WriteLCDMsg_Float(unsigned  short usDataAddr , float *data)          //发送单个浮点型数据
{
    unsigned char ucSendCnt = 0;
    char ucSnedBuff[100] = {0};
    unsigned int uiTemp = 0;
    unsigned int uicDataTemp = 0;
    ucSnedBuff[ucSendCnt++] = SendHeard1;
    ucSnedBuff[ucSendCnt++] = SendHeard2;
    ucSnedBuff[ucSendCnt++] = 0;
    ucSnedBuff[ucSendCnt++] = WriteLCDDataOrder;
    ucSnedBuff[ucSendCnt++] = usDataAddr >> 8;
    ucSnedBuff[ucSendCnt++] = usDataAddr & 0xFF;

    for(uicDataTemp = 0; uicDataTemp < 4; uicDataTemp++)                    //浮点型数据转换为4字节整形  发送需倒着发
    {
        ucSnedBuff[ucSendCnt++] = *((unsigned char *)data +3 - uicDataTemp);
    }

    ucSnedBuff[2] = ucSendCnt - 3;
    for(uiTemp = 0; uiTemp < ucSendCnt; uiTemp++)
    {
        USART_SendData(USART2, ucSnedBuff[uiTemp]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
    }
}


void WriteLCDMsg(unsigned  short usDataAddr, unsigned char *Data)         //发送多个数据  数组传递
{
    unsigned char ucSendCnt = 0;
    unsigned char ucSnedBuff[100] = {0};
    unsigned int uiTemp = 0;
    unsigned int uicDataTemp = 0;
    ucSnedBuff[ucSendCnt++] = SendHeard1;
    ucSnedBuff[ucSendCnt++] = SendHeard2;
    ucSnedBuff[ucSendCnt++] = 0;
    ucSnedBuff[ucSendCnt++] = WriteLCDDataOrder;
    ucSnedBuff[ucSendCnt++] = usDataAddr >> 8;
    ucSnedBuff[ucSendCnt++] = usDataAddr & 0xFF;

    uicDataTemp = Data[99];
    for(uiTemp = 0; uiTemp < uicDataTemp; uiTemp++)
    {
        ucSnedBuff[ucSendCnt++] = *Data++;
    }

    ucSnedBuff[2] = ucSendCnt - 3;
    for(uiTemp = 0; uiTemp < ucSendCnt; uiTemp++)
    {
        USART_SendData(USART2, ucSnedBuff[uiTemp]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
    }
}

void WriteLCDOfString(unsigned short usStringAddr,char *cData)
{
    unsigned char ucSendStringCnt = 0;
    unsigned char ucSnedStringBuff[250] = {0};
    unsigned int uiStringTemp = 0;
    ucSnedStringBuff[ucSendStringCnt++] = SendHeard1;
    ucSnedStringBuff[ucSendStringCnt++] = SendHeard2;
    ucSnedStringBuff[ucSendStringCnt++] = 0;
    ucSnedStringBuff[ucSendStringCnt++] = WriteLCDDataOrder;
    ucSnedStringBuff[ucSendStringCnt++] = usStringAddr >> 8;
    ucSnedStringBuff[ucSendStringCnt++] = usStringAddr & 0xFF;

    while(*cData)
    {
        ucSnedStringBuff[ucSendStringCnt++] = *cData++;
    }
    ucSnedStringBuff[2] = ucSendStringCnt - 3;
    for(uiStringTemp = 0; uiStringTemp < ucSendStringCnt; uiStringTemp++)
    {
        USART_SendData(USART2, ucSnedStringBuff[uiStringTemp]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
    }

}
/*******************************************************************************
 函数块名称:
 功能:        读取指定LCD指定地址数据
 输入参数:
 输出参数:
 其他说明:
*******************************************************************************/
void ReadLCDMsg(unsigned  short usDataAddr, unsigned char *Data)
{
    unsigned char ucSendCnt = 0;
    unsigned char ucSnedBuff[100] = {0};
    unsigned int uiTemp = 0;
    unsigned int uicDataTemp = 0;
    ucSnedBuff[ucSendCnt++] = SendHeard1;
    ucSnedBuff[ucSendCnt++] = SendHeard2;
    ucSnedBuff[ucSendCnt++] = 0;
    ucSnedBuff[ucSendCnt++] = ReadLCDDataOrder;
    ucSnedBuff[ucSendCnt++] = usDataAddr >> 8;
    ucSnedBuff[ucSendCnt++] = usDataAddr & 0xFF;

    uicDataTemp = Data[99];
    for(uiTemp = 0; uiTemp < uicDataTemp; uiTemp++)
    {
        ucSnedBuff[ucSendCnt++] = *Data++;
    }

    ucSnedBuff[2] = ucSendCnt - 3;
    for(uiTemp = 0; uiTemp < ucSendCnt; uiTemp++)
    {
        USART_SendData(USART2, ucSnedBuff[uiTemp]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
    }
}
/*******************************************************************************
 函数块名称:
 功能:
 输入参数:
 输出参数:
 其他说明:
*******************************************************************************/
void LCD_Page_Change(unsigned  short usPage)
{
    unsigned char ucSnedBuff[10] = {0};
    unsigned int uiTemp = 0;
    ucSnedBuff[0] = SendHeard1;
    ucSnedBuff[1] = SendHeard2;
    ucSnedBuff[2] = 0x07;
    ucSnedBuff[3] = WriteLCDDataOrder;
    ucSnedBuff[4] = 0x00;
    ucSnedBuff[5] = 0x84;
    ucSnedBuff[6] = ChangeLCDPageOrder >> 8;
    ucSnedBuff[7] = ChangeLCDPageOrder & 0xFF;
    ucSnedBuff[8] = usPage >> 8;
    ucSnedBuff[9] = usPage;

    for(uiTemp = 0; uiTemp < 10; uiTemp++)
    {
        USART_SendData(USART2, ucSnedBuff[uiTemp]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
    }
}







/*******************************************************************************
 函数块名称:  void LCD_Page_Manage(void)
 功能:        控制LCD页面及参数处理
 输入参数:
 输出参数:
 其他说明:
*******************************************************************************/
void LCD_Page_Manage(void)
{
    unsigned int uiV_Manual_Set = 0;
    unsigned int uiI_Manual_Set = 0;

    unsigned char ucPageMsgData[100] = {0};
    unsigned char ucDateData[100] = {0};


//////////判断上电界面位置，上电5秒后若无人操作，转为主页面显示/////////
    if(ucTag5s&&(LCD_Page_Msg_Data == 0x00))   //仅在第一次上电进行判断处理
    {
        ucPageMsgData[99] = 1;
        ucPageMsgData[0] = 0x01;
        ReadLCDMsg(Page_Msg_Addr,ucPageMsgData);

        if(LCD_Page_MSg_Flg)
        {
            LCD_Page_MSg_Flg = 0;
            if(LCD_Page_Msg_Data == 0x00)
            {
                LCD_Page_Change(0x0001);
            }
        }
    }
//读取LCD当前所在页面信息
    if(ucReadLCD_MsgStep == 0)
    {
        ucReadLCD_MsgStep = 1;
        ucPageMsgData[99] = 1;
        ucPageMsgData[0] = 0x01;
        ReadLCDMsg(Page_Msg_Addr,ucPageMsgData);
    }
//////////检测手动控制输出按钮状态,进行状态改变////////
    else if(ucReadLCD_MsgStep == 1)
    {
        ucReadLCD_MsgStep = 2;
        ucPageMsgData[99] = 1;
        ucPageMsgData[0] = 0x01;
        ReadLCDMsg(Manual_Set_Password_Addr,ucPageMsgData);

        if(LCD_Manual_CrcData == 1234)  //当检测到进入手动参数设置界面时 清零返回标志
        {
            ucPageMsgData[99] = 2;        //清零密码参数
            ucPageMsgData[0] = 0x00;
            ucPageMsgData[1] = 0x00;
            WriteLCDMsg(Manual_Set_Password_Addr,ucPageMsgData);

            LCD_Page_Change(0x0B);       //进入手动已启动页面
        }

    }
///////	手动启动按钮显示切换///////////
    else if(ucReadLCD_MsgStep == 2)
    {
        ucReadLCD_MsgStep = 3;
        ucPageMsgData[99] = 1;
        ucPageMsgData[0] = 0x01;
        ReadLCDMsg(Manual_Start_Stop_Addr,ucPageMsgData);
        if(LCD_Manual_Flg&&((LCD_Manual_ReturnData&0x0001) == 0)&&LCD_Manual_SetData)
        {
            LCD_Manual_Flg = 0;
            if(LCD_Manual_Data == 0x01)
            {
                LCD_Page_Change(0x000A);
            }
            else
            {
                LCD_Page_Change(0x0009);
            }

        }
    }
////////////手动控制输出参数设置//////////
    else if(ucReadLCD_MsgStep == 3)
    {
        ucReadLCD_MsgStep = 4;
        if(LCD_Manual_Data == 0x01)
        {
            ucPageMsgData[99] = 1;
            ucPageMsgData[0] = 0x01;
            ReadLCDMsg(Manual_Set_V_Output_Addr,ucPageMsgData);
        }
    }
    else if (ucReadLCD_MsgStep == 4)
    {
        ucReadLCD_MsgStep = 5;
        if(LCD_Manual_Data == 0x01)
        {
            ucPageMsgData[99] = 1;
            ucPageMsgData[0] = 0x01;
            ReadLCDMsg(Manual_Set_I_Output_Addr,ucPageMsgData);

            if((LCD_Manual_V_OutputData != 0)&&(LCD_Manual_I_OutputData != 0))      //电压电流设置不为0时设置输出电压电流
            {
                uiV_Manual_Set = LCD_Manual_V_OutputData * 10;
                uiI_Manual_Set = LCD_Manual_I_OutputData * 10;
                Set_Output(uiV_Manual_Set,uiI_Manual_Set);          //此函数中有输出限幅设置
            }
        }
    }
//手动页面返回主页面参数判断  参数为0判断是否返回返回主页面  参数为1 时返回主页面
    else if (ucReadLCD_MsgStep == 5)
    {
        ucReadLCD_MsgStep = 6;
        ucPageMsgData[99] = 1;
        ucPageMsgData[0] = 0x01;
        ReadLCDMsg(Manual_Set_Reurn_Addr,ucPageMsgData);
        if(LCD_Manual_ReturnData&0x0001)  //当返回参数等于1时  当前页面在第九或者第十页时 返回首页
        {
            ucPageMsgData[99] = 2;        //清零手动设置按钮参数
            ucPageMsgData[0] = 0x00;
            ucPageMsgData[1] = 0x00;
            WriteLCDMsg(Manual_Set_Reurn_Addr,ucPageMsgData);
            if(LCD_Manual_SetData)      //当进入手动设置后清除返回值按钮值
            {
                ucPageMsgData[99] = 2;        //清零返回参数
                ucPageMsgData[0] = 0x00;
                ucPageMsgData[1] = 0x00;
                WriteLCDMsg(Manual_Set_Parameter_Addr,ucPageMsgData);
            }
            LCD_Page_Change(0x01);
        }
    }
//读取时间参数并更新显示器上时间参数
    else if (ucReadLCD_MsgStep == 6)
    {
        ucReadLCD_MsgStep = 7;
        if(ucDateStep == 0)                          //分次读取显示屏设置时间参数
        {
            ucDateStep = 1;
            ucPageMsgData[99] = 1;
            ucPageMsgData[0] = 0x04;
            ReadLCDMsg(Date_Read_Addr,ucPageMsgData);
        }
        else if(ucDateStep == 1)
        {
            ucDateStep = 2;
            ucPageMsgData[99] = 1;
            ucPageMsgData[0] = 0x01;
            ReadLCDMsg(LCD_Set_Year_Addr,ucPageMsgData);
        }
        else if(ucDateStep == 2)
        {
            ucDateStep = 3;
            ucPageMsgData[99] = 1;
            ucPageMsgData[0] = 0x01;
            ReadLCDMsg(LCD_Set_Month_Addr,ucPageMsgData);
        }
        else if(ucDateStep == 3)
        {
            ucDateStep = 4;
            ucPageMsgData[99] = 1;
            ucPageMsgData[0] = 0x01;
            ReadLCDMsg(LCD_Set_Date_Addr,ucPageMsgData);
        }
        else if(ucDateStep == 4)
        {
            ucDateStep = 5;
            ucPageMsgData[99] = 1;
            ucPageMsgData[0] = 0x01;
            ReadLCDMsg(LCD_Set_Hour_Addr,ucPageMsgData);
        }
        else if(ucDateStep == 5)
        {
            ucDateStep = 6;
            ucPageMsgData[99] = 1;
            ucPageMsgData[0] = 0x01;
            ReadLCDMsg(LCD_Set_Minute_Addr,ucPageMsgData);
        }
        else if(ucDateStep == 6)
        {
            ucDateStep = 7;
            ucPageMsgData[99] = 1;
            ucPageMsgData[0] = 0x01;
            ReadLCDMsg(LCD_Set_Seconnds_Addr,ucPageMsgData);
        }
        LCD_Set_Year = LCD_Set_Year % 100;
        if(LCD_Set_Month > 12)                          //时间设置限制
        {
            LCD_Set_Month = 12;
        }
        if(LCD_Set_Date > 31)
        {
            LCD_Set_Date = 31;
        }
        if(LCD_Set_Hour > 24)
        {
            LCD_Set_Hour = 24;
        }
        if(LCD_Set_Minute > 59)
        {
            LCD_Set_Minute = 59;
        }
        if(LCD_Set_Seconds > 59)
        {
            LCD_Set_Seconds = 59;
        }
        ucDateData[99] = 8;
        ucDateData[0] = 0x5A;
        ucDateData[1] = 0xA5;
        if(ucDateStep == 7)  //当读取7个数据后进行一次处理  判断当前参数和设置参数是否一致  一致不进行更新
        {
            ucDateStep = 0;
            if((LCD_Set_Year != LCD_Year)||(LCD_Set_Month != LCD_Month)||(LCD_Set_Date != LCD_Date)||(LCD_Set_Hour != LCD_Hour)||(LCD_Set_Minute != LCD_Minute)||(LCD_Set_Seconds != LCD_Seconds))
            {
                if(LCD_Page_Msg_Data == 0x0C)
                {
                    if((LCD_Set_Year == 0)||(LCD_Set_Year == LCD_Year))
                    {
                        ucDateData[2] = LCD_Year;
                    }
                    else
                    {
                        ucDateData[2] = LCD_Set_Year;
                    }
                    if((LCD_Set_Month == 0)||(LCD_Set_Month == LCD_Month))
                    {
                        ucDateData[3] = LCD_Month;
                    }
                    else
                    {
                        ucDateData[3] = LCD_Set_Month;
                    }
                    if((LCD_Set_Date == 0)||(LCD_Set_Date == LCD_Date))
                    {
                        ucDateData[4] = LCD_Date;
                    }
                    else
                    {
                        ucDateData[4] = LCD_Set_Date;
                    }
                    if((LCD_Set_Hour == 0)||(LCD_Set_Hour == LCD_Hour))
                    {
                        ucDateData[5] = LCD_Hour;
                    }
                    else
                    {
                        ucDateData[5] = LCD_Set_Hour;
                    }
                    if((LCD_Set_Minute == 0)||(LCD_Set_Minute == LCD_Minute))
                    {
                        ucDateData[6] = LCD_Minute;
                    }
                    else
                    {
                        ucDateData[6] = LCD_Set_Minute;
                    }
                    if((LCD_Set_Seconds == 0)||(LCD_Set_Seconds == LCD_Seconds))
                    {
                        ucDateData[7] = LCD_Seconds;
                    }
                    else
                    {
                        ucDateData[7] = LCD_Set_Seconds;
                    }
                    WriteLCDMsg(Date_Set_Addr,ucDateData);
                }
            }
        }
    }
    else if (ucReadLCD_MsgStep == 7)                 //读取参数设置按钮状态
    {
        ucReadLCD_MsgStep = 0;
        ucPageMsgData[99] = 1;
        ucPageMsgData[0] = 0x01;
        ReadLCDMsg(Manual_Set_Parameter_Addr,ucPageMsgData);

    }

}


//////////////////////////////////变量显示方式////////////////////
void LCD_Display(void)
{
    char cBuff[100];
    float ftemp;
    unsigned int uiTemp = 0;
    unsigned char ucTemp = 0;

    uiV_Output = CAN_LLC_RX_Data[0] * 256 + CAN_LLC_RX_Data[1];  //输出电压计算  100mV单位
    ftemp = uiV_Output;
    ftemp = ftemp / 10;                              //转换为1V单位
    WriteLCDMsg_Float(OutputVoltageAddr,&ftemp);     //串口发送到LCD


    uiI_Output = CAN_LLC_RX_Data[2] * 256 + CAN_LLC_RX_Data[3];  //输出电压计算  100mA单位
    ftemp = uiI_Output;
    ftemp = ftemp / 10;                             //转换为1A单位
    WriteLCDMsg_Float(OutputCurrentAddr,&ftemp);


    uiV_Input = CAN_PFC_RX_Data[7] * 256 + CAN_PFC_RX_Data[6];  //输入电压计算  1V单位
    ftemp = uiV_Input;
    ftemp =ftemp * 0.04665161;
    WriteLCDMsg_Float(V_Input_Addr,&ftemp);

    uiI_Input = CAN_LLC_RX_Data2[4] * 256 + CAN_LLC_RX_Data2[5];  //输入电流计算  100mA单位
    ftemp = uiI_Input;
    ftemp = uiV_Output;
    ftemp = ftemp * uiI_Output /(uiV_Input * 0.04665161)/141.4;
    if(uiV_Input == 0)
    {
        ftemp = 0;
    }
    WriteLCDMsg_Float(I_Input_Addr,&ftemp);

    uiV_Bus = CAN_LLC_RX_Data2[6] * 256 + CAN_LLC_RX_Data2[7];  //输入电流计算  1V单位
    ftemp = uiV_Bus;
    ftemp = ftemp*0.0242;
    WriteLCDMsg_Float(V_Bus_Addr,&ftemp);

    uiPFC_Temp3 = CAN_PFC_RX_Data[5] * 256 + CAN_PFC_RX_Data[4];   //LLC模块以及BUCK模块温度显示
    uiPFC_Temp1 = CAN_PFC_RX_Data[3] * 256 + CAN_PFC_RX_Data[2];
    uiPFC_Temp2 = CAN_PFC_RX_Data[1] * 256 + CAN_PFC_RX_Data[0];
    uiLLC_Temp1 = CAN_LLC_RX_Data[4] * 256 + CAN_LLC_RX_Data[5];
    uiLLC_Temp2 = CAN_LLC_RX_Data[6] * 256 + CAN_LLC_RX_Data[7];

    ftemp = uiPFC_Temp3;
    ftemp = ftemp /20480 *4.096;
    ftemp = ftemp*3.6/(5-ftemp);
    ftemp = ftemp * 1000;                                       //AD采样转换为电阻值
    for(uiTemp = 0; uiTemp < 135; uiTemp++)                     //不同阻值对应温度值转换
    {
        if((ftemp <= g_uiNtcTable_External[uiTemp]) && (ftemp > g_uiNtcTable_External[uiTemp + 1]))ucTemp = 0x66;    //查询到对应阻值对应得温度后退出
        if(0x66 == ucTemp) break;
    }
    if(0x66 == ucTemp)
    {
        ftemp = uiTemp;
        ftemp = (ftemp  - 40.00); // 从-40度算起
    }
    if(uiPFC_Temp3 == 0)      //机器通信异常或者为通信成功时默认温度为0度
    {
        ftemp = 0;
    }
    uiPFC_Temp3_AsTrue = 	ftemp;
    WriteLCDMsg_Float(T_PFC3Addr,&ftemp);

    ucTemp = 0;
    ftemp = uiPFC_Temp2;
    ftemp = ftemp /20480 *4.096;
    ftemp = ftemp*3.6/(5-ftemp);
    ftemp = ftemp * 1000;                                    //AD采样转换为电阻值
    for(uiTemp = 0; uiTemp < 135; uiTemp++)                  //不同阻值对应温度值转换
    {
        if((ftemp <= g_uiNtcTable_External[uiTemp]) && (ftemp > g_uiNtcTable_External[uiTemp + 1]))ucTemp = 0x66;
        if(0x66 == ucTemp) break;
    }
    if(0x66 == ucTemp)
    {
        ftemp = uiTemp;
        ftemp = (ftemp  - 40.00); // 从-40度算起
    }
    if(uiPFC_Temp2 == 0)      //机器通信异常或者为通信成功时默认温度为0度
    {
        ftemp = 0;
    }
    uiPFC_Temp2_AsTrue = 	ftemp;
    WriteLCDMsg_Float(T_PFC1Addr,&ftemp);

    ucTemp = 0;
    ftemp = uiPFC_Temp1;
    ftemp = ftemp /20480 *4.096;
    ftemp = ftemp*3.6/(5-ftemp);
    ftemp = ftemp * 1000;                                      //AD采样转换为电阻值    PFC为采样电阻和3.6K分压 ，采样电阻电压为检测电压
    for(uiTemp = 0; uiTemp < 135; uiTemp++)                    //不同阻值对应温度值转换
    {
        if((ftemp <= g_uiNtcTable_External[uiTemp]) && (ftemp > g_uiNtcTable_External[uiTemp + 1]))ucTemp = 0x66;
        if(0x66 == ucTemp) break;
    }
    if(0x66 == ucTemp)
    {
        ftemp = uiTemp;
        ftemp = (ftemp  - 40.00); // 从-40度算起
    }
    if(uiPFC_Temp1 == 0)      //机器通信异常或者为通信成功时默认温度为0度
    {
        ftemp = 0;
    }
    uiPFC_Temp1_AsTrue = 	ftemp;
    WriteLCDMsg_Float(T_PFC2Addr,&ftemp);

    ucTemp = 0;
    ftemp = uiLLC_Temp1;
    ftemp = ftemp /4096 *3.3;
    ftemp = (2.35-0.47*ftemp) /ftemp;                    //AD采样转换为电阻值    LLC为采样电阻和470R分压 ，470R电压为检测电压
    ftemp = ftemp * 1000;																 //不同阻值对应温度值转换
    for(uiTemp = 0; uiTemp < 135; uiTemp++)
    {
        if((ftemp <= g_uiNtcTable_External[uiTemp]) && (ftemp > g_uiNtcTable_External[uiTemp + 1]))ucTemp = 0x66;
        if(0x66 == ucTemp) break;
    }
    if(0x66 == ucTemp)
    {
        ftemp = uiTemp;
        ftemp = (ftemp  - 40.00); // 从-40度算起
    }
    if(uiLLC_Temp1 == 0)      //机器通信异常或者为通信成功时默认温度为0度
    {
        ftemp = 0;
    }
    uiLLC_Temp1_AsTrue = 	ftemp;
    WriteLCDMsg_Float(T_LLC1Addr,&ftemp);

    ucTemp = 0;
    ftemp = uiLLC_Temp2;
    ftemp = ftemp /4096 *3.3;
    ftemp = (2.35-0.47*ftemp) /ftemp;
    ftemp = ftemp * 1000;                              //AD采样转换为电阻值    LLC为采样电阻和470R分压 ，470R电压为检测电压
    for(uiTemp = 0; uiTemp < 135; uiTemp++)            //不同阻值对应温度值转换
    {
        if((ftemp <= g_uiNtcTable_External[uiTemp]) && (ftemp > g_uiNtcTable_External[uiTemp + 1]))ucTemp = 0x66;
        if(0x66 == ucTemp) break;
    }
    if(0x66 == ucTemp)
    {
        ftemp = uiTemp;
        ftemp = (ftemp  - 40.00); // 从-40度算起
    }
    if(uiLLC_Temp2 == 0)      //机器通信异常或者为通信成功时默认温度为0度
    {
        ftemp = 0;
    }
    uiLLC_Temp2_AsTrue = 	ftemp;
    WriteLCDMsg_Float(T_LLC2Addr,&ftemp);

    if(uiV_Output > 420 &&uiI_Output > 30)  //充电机工作状态判断
    {
        WriteLCDOfString(ChageStatusAddr,"充电中");
    }
    else if(uiV_Output > 42)
    {
        WriteLCDOfString(ChageStatusAddr,"已连接");
    }
    else
    {
        WriteLCDOfString(ChageStatusAddr,"未连接");
    }

    WriteLCDOfString(BattaryTypeAddr,"锂电");                      //电池类型显示

    if(Flg.bit.Hardware_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"硬件故障");                      //故障显示
    }
    else if(Flg.bit.OTP_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"过温故障");
    }
    else if(Flg.bit.Input_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"输入电压故障");
    }
    else if(Flg.bit.Batarry_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"电池未连接");
    }
    else if(Flg.bit.CAN_BMS_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"接收BMS报文超时");
    }
    else if(Flg.bit.CAN_LLC_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"接收LLC报文超时");
    }
    else if(Flg.bit.Usart_LLC_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"LLC串口接收超时");
    }
    else if(Flg.bit.Usart_PFC_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"PFC串口接收超时");
    }
    else if(Flg.bit.V_Output_OVP_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"输出过压");
    }
    else if(Flg.bit.I_Output_OCP_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"输出过流");
    }
    else if(Flg.bit.V_Output_UVP_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"输出欠压");
    }
    else if(Flg.bit.V_Bus_390VErr)
    {
        WriteLCDOfString(Err_MSg_Addr,"母线电压异常");
    }
		else
		{
		    WriteLCDOfString(Err_MSg_Addr,"工作状态正常");
		}

    if(uc_Output_En_Flg&&uc_Incident_Open_OBC_FLg)
    {
        uc_Incident_Open_OBC_FLg = 0;
        sprintf(cBuff, "%d%d-%d%d %d%d:%d%d:%d%d",(Hex_To_BCD(LCD_Month)>>4)&0x0F,Hex_To_BCD(LCD_Month)&0x0F,(Hex_To_BCD(LCD_Date)>>4)&0x0F,Hex_To_BCD(LCD_Date)&0x0F,(Hex_To_BCD(LCD_Hour)>>4)&0x0F,\
                Hex_To_BCD(LCD_Hour)&0x0F,(Hex_To_BCD(LCD_Minute)>>4)&0x0F,Hex_To_BCD(LCD_Minute)&0x0F,(Hex_To_BCD(LCD_Seconds)>>4)&0x0F, Hex_To_BCD(LCD_Seconds)&0x0F);
        WriteLCDOfString(Incident_Time_Start_Addr,cBuff);
        sprintf(cBuff, "开机");
        WriteLCDOfString(Incident_Content_Start_Addr0x,cBuff);
    }

}

















