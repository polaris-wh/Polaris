
#include "Charger_Control.h"
#include "DWIN_LCD.h"
#include "System_Init.h"
#include "usart.h"
#include "stm32f10x_usart.h"
#include "CAN.h"
#include "PCF8563.h"

unsigned char ucReadLCD_MsgStep = 0;   //ÿ�η���һ֡ ��ȡһ����ַ����  ���򴮿ڽ����쳣
unsigned char ucDateStep = 0;          //LCD���ڲ����ִζ�ȡ
unsigned char uc_Incident_Open_OBC_FLg = 1; //������ʾ�����¼���־

unsigned int uiPFC_Temp1 = 0;          //OBC5���ڲ��¶�ADֵ
unsigned int uiPFC_Temp2 = 0;
unsigned int uiPFC_Temp3 = 0;
unsigned int uiLLC_Temp1 = 0;
unsigned int uiLLC_Temp2 = 0;

unsigned int uiPFC_Temp1_AsTrue = 0;  //OBC�ڲ��¶Ȼ���Ϊ�����¶Ȳ���
unsigned int uiPFC_Temp2_AsTrue = 0;
unsigned int uiPFC_Temp3_AsTrue = 0;
unsigned int uiLLC_Temp1_AsTrue = 0;
unsigned int uiLLC_Temp2_AsTrue = 0;

unsigned int uiV_Output = 0;    //���������ѹ����  100mv��λ
unsigned int uiI_Output = 0;    //���������������� 100mA��λ
unsigned int uiV_Input = 0;
unsigned int uiI_Input = 0;
unsigned int uiV_Bus = 0;



void WriteLCDMsg_Float(unsigned  short usDataAddr , float *data)          //���͵�������������
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

    for(uicDataTemp = 0; uicDataTemp < 4; uicDataTemp++)                    //����������ת��Ϊ4�ֽ�����  �����赹�ŷ�
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


void WriteLCDMsg(unsigned  short usDataAddr, unsigned char *Data)         //���Ͷ������  ���鴫��
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
 ����������:
 ����:        ��ȡָ��LCDָ����ַ����
 �������:
 �������:
 ����˵��:
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
 ����������:
 ����:
 �������:
 �������:
 ����˵��:
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
 ����������:  void LCD_Page_Manage(void)
 ����:        ����LCDҳ�漰��������
 �������:
 �������:
 ����˵��:
*******************************************************************************/
void LCD_Page_Manage(void)
{
    unsigned int uiV_Manual_Set = 0;
    unsigned int uiI_Manual_Set = 0;

    unsigned char ucPageMsgData[100] = {0};
    unsigned char ucDateData[100] = {0};


//////////�ж��ϵ����λ�ã��ϵ�5��������˲�����תΪ��ҳ����ʾ/////////
    if(ucTag5s&&(LCD_Page_Msg_Data == 0x00))   //���ڵ�һ���ϵ�����жϴ���
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
//��ȡLCD��ǰ����ҳ����Ϣ
    if(ucReadLCD_MsgStep == 0)
    {
        ucReadLCD_MsgStep = 1;
        ucPageMsgData[99] = 1;
        ucPageMsgData[0] = 0x01;
        ReadLCDMsg(Page_Msg_Addr,ucPageMsgData);
    }
//////////����ֶ����������ť״̬,����״̬�ı�////////
    else if(ucReadLCD_MsgStep == 1)
    {
        ucReadLCD_MsgStep = 2;
        ucPageMsgData[99] = 1;
        ucPageMsgData[0] = 0x01;
        ReadLCDMsg(Manual_Set_Password_Addr,ucPageMsgData);

        if(LCD_Manual_CrcData == 1234)  //����⵽�����ֶ��������ý���ʱ ���㷵�ر�־
        {
            ucPageMsgData[99] = 2;        //�����������
            ucPageMsgData[0] = 0x00;
            ucPageMsgData[1] = 0x00;
            WriteLCDMsg(Manual_Set_Password_Addr,ucPageMsgData);

            LCD_Page_Change(0x0B);       //�����ֶ�������ҳ��
        }

    }
///////	�ֶ�������ť��ʾ�л�///////////
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
////////////�ֶ����������������//////////
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

            if((LCD_Manual_V_OutputData != 0)&&(LCD_Manual_I_OutputData != 0))      //��ѹ�������ò�Ϊ0ʱ���������ѹ����
            {
                uiV_Manual_Set = LCD_Manual_V_OutputData * 10;
                uiI_Manual_Set = LCD_Manual_I_OutputData * 10;
                Set_Output(uiV_Manual_Set,uiI_Manual_Set);          //�˺�����������޷�����
            }
        }
    }
//�ֶ�ҳ�淵����ҳ������ж�  ����Ϊ0�ж��Ƿ񷵻ط�����ҳ��  ����Ϊ1 ʱ������ҳ��
    else if (ucReadLCD_MsgStep == 5)
    {
        ucReadLCD_MsgStep = 6;
        ucPageMsgData[99] = 1;
        ucPageMsgData[0] = 0x01;
        ReadLCDMsg(Manual_Set_Reurn_Addr,ucPageMsgData);
        if(LCD_Manual_ReturnData&0x0001)  //�����ز�������1ʱ  ��ǰҳ���ڵھŻ��ߵ�ʮҳʱ ������ҳ
        {
            ucPageMsgData[99] = 2;        //�����ֶ����ð�ť����
            ucPageMsgData[0] = 0x00;
            ucPageMsgData[1] = 0x00;
            WriteLCDMsg(Manual_Set_Reurn_Addr,ucPageMsgData);
            if(LCD_Manual_SetData)      //�������ֶ����ú��������ֵ��ťֵ
            {
                ucPageMsgData[99] = 2;        //���㷵�ز���
                ucPageMsgData[0] = 0x00;
                ucPageMsgData[1] = 0x00;
                WriteLCDMsg(Manual_Set_Parameter_Addr,ucPageMsgData);
            }
            LCD_Page_Change(0x01);
        }
    }
//��ȡʱ�������������ʾ����ʱ�����
    else if (ucReadLCD_MsgStep == 6)
    {
        ucReadLCD_MsgStep = 7;
        if(ucDateStep == 0)                          //�ִζ�ȡ��ʾ������ʱ�����
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
        if(LCD_Set_Month > 12)                          //ʱ����������
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
        if(ucDateStep == 7)  //����ȡ7�����ݺ����һ�δ���  �жϵ�ǰ���������ò����Ƿ�һ��  һ�²����и���
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
    else if (ucReadLCD_MsgStep == 7)                 //��ȡ�������ð�ť״̬
    {
        ucReadLCD_MsgStep = 0;
        ucPageMsgData[99] = 1;
        ucPageMsgData[0] = 0x01;
        ReadLCDMsg(Manual_Set_Parameter_Addr,ucPageMsgData);

    }

}


//////////////////////////////////������ʾ��ʽ////////////////////
void LCD_Display(void)
{
    char cBuff[100];
    float ftemp;
    unsigned int uiTemp = 0;
    unsigned char ucTemp = 0;

    uiV_Output = CAN_LLC_RX_Data[0] * 256 + CAN_LLC_RX_Data[1];  //�����ѹ����  100mV��λ
    ftemp = uiV_Output;
    ftemp = ftemp / 10;                              //ת��Ϊ1V��λ
    WriteLCDMsg_Float(OutputVoltageAddr,&ftemp);     //���ڷ��͵�LCD


    uiI_Output = CAN_LLC_RX_Data[2] * 256 + CAN_LLC_RX_Data[3];  //�����ѹ����  100mA��λ
    ftemp = uiI_Output;
    ftemp = ftemp / 10;                             //ת��Ϊ1A��λ
    WriteLCDMsg_Float(OutputCurrentAddr,&ftemp);


    uiV_Input = CAN_PFC_RX_Data[7] * 256 + CAN_PFC_RX_Data[6];  //�����ѹ����  1V��λ
    ftemp = uiV_Input;
    ftemp =ftemp * 0.04665161;
    WriteLCDMsg_Float(V_Input_Addr,&ftemp);

    uiI_Input = CAN_LLC_RX_Data2[4] * 256 + CAN_LLC_RX_Data2[5];  //�����������  100mA��λ
    ftemp = uiI_Input;
    ftemp = uiV_Output;
    ftemp = ftemp * uiI_Output /(uiV_Input * 0.04665161)/141.4;
    if(uiV_Input == 0)
    {
        ftemp = 0;
    }
    WriteLCDMsg_Float(I_Input_Addr,&ftemp);

    uiV_Bus = CAN_LLC_RX_Data2[6] * 256 + CAN_LLC_RX_Data2[7];  //�����������  1V��λ
    ftemp = uiV_Bus;
    ftemp = ftemp*0.0242;
    WriteLCDMsg_Float(V_Bus_Addr,&ftemp);

    uiPFC_Temp3 = CAN_PFC_RX_Data[5] * 256 + CAN_PFC_RX_Data[4];   //LLCģ���Լ�BUCKģ���¶���ʾ
    uiPFC_Temp1 = CAN_PFC_RX_Data[3] * 256 + CAN_PFC_RX_Data[2];
    uiPFC_Temp2 = CAN_PFC_RX_Data[1] * 256 + CAN_PFC_RX_Data[0];
    uiLLC_Temp1 = CAN_LLC_RX_Data[4] * 256 + CAN_LLC_RX_Data[5];
    uiLLC_Temp2 = CAN_LLC_RX_Data[6] * 256 + CAN_LLC_RX_Data[7];

    ftemp = uiPFC_Temp3;
    ftemp = ftemp /20480 *4.096;
    ftemp = ftemp*3.6/(5-ftemp);
    ftemp = ftemp * 1000;                                       //AD����ת��Ϊ����ֵ
    for(uiTemp = 0; uiTemp < 135; uiTemp++)                     //��ͬ��ֵ��Ӧ�¶�ֵת��
    {
        if((ftemp <= g_uiNtcTable_External[uiTemp]) && (ftemp > g_uiNtcTable_External[uiTemp + 1]))ucTemp = 0x66;    //��ѯ����Ӧ��ֵ��Ӧ���¶Ⱥ��˳�
        if(0x66 == ucTemp) break;
    }
    if(0x66 == ucTemp)
    {
        ftemp = uiTemp;
        ftemp = (ftemp  - 40.00); // ��-40������
    }
    if(uiPFC_Temp3 == 0)      //����ͨ���쳣����Ϊͨ�ųɹ�ʱĬ���¶�Ϊ0��
    {
        ftemp = 0;
    }
    uiPFC_Temp3_AsTrue = 	ftemp;
    WriteLCDMsg_Float(T_PFC3Addr,&ftemp);

    ucTemp = 0;
    ftemp = uiPFC_Temp2;
    ftemp = ftemp /20480 *4.096;
    ftemp = ftemp*3.6/(5-ftemp);
    ftemp = ftemp * 1000;                                    //AD����ת��Ϊ����ֵ
    for(uiTemp = 0; uiTemp < 135; uiTemp++)                  //��ͬ��ֵ��Ӧ�¶�ֵת��
    {
        if((ftemp <= g_uiNtcTable_External[uiTemp]) && (ftemp > g_uiNtcTable_External[uiTemp + 1]))ucTemp = 0x66;
        if(0x66 == ucTemp) break;
    }
    if(0x66 == ucTemp)
    {
        ftemp = uiTemp;
        ftemp = (ftemp  - 40.00); // ��-40������
    }
    if(uiPFC_Temp2 == 0)      //����ͨ���쳣����Ϊͨ�ųɹ�ʱĬ���¶�Ϊ0��
    {
        ftemp = 0;
    }
    uiPFC_Temp2_AsTrue = 	ftemp;
    WriteLCDMsg_Float(T_PFC1Addr,&ftemp);

    ucTemp = 0;
    ftemp = uiPFC_Temp1;
    ftemp = ftemp /20480 *4.096;
    ftemp = ftemp*3.6/(5-ftemp);
    ftemp = ftemp * 1000;                                      //AD����ת��Ϊ����ֵ    PFCΪ���������3.6K��ѹ �����������ѹΪ����ѹ
    for(uiTemp = 0; uiTemp < 135; uiTemp++)                    //��ͬ��ֵ��Ӧ�¶�ֵת��
    {
        if((ftemp <= g_uiNtcTable_External[uiTemp]) && (ftemp > g_uiNtcTable_External[uiTemp + 1]))ucTemp = 0x66;
        if(0x66 == ucTemp) break;
    }
    if(0x66 == ucTemp)
    {
        ftemp = uiTemp;
        ftemp = (ftemp  - 40.00); // ��-40������
    }
    if(uiPFC_Temp1 == 0)      //����ͨ���쳣����Ϊͨ�ųɹ�ʱĬ���¶�Ϊ0��
    {
        ftemp = 0;
    }
    uiPFC_Temp1_AsTrue = 	ftemp;
    WriteLCDMsg_Float(T_PFC2Addr,&ftemp);

    ucTemp = 0;
    ftemp = uiLLC_Temp1;
    ftemp = ftemp /4096 *3.3;
    ftemp = (2.35-0.47*ftemp) /ftemp;                    //AD����ת��Ϊ����ֵ    LLCΪ���������470R��ѹ ��470R��ѹΪ����ѹ
    ftemp = ftemp * 1000;																 //��ͬ��ֵ��Ӧ�¶�ֵת��
    for(uiTemp = 0; uiTemp < 135; uiTemp++)
    {
        if((ftemp <= g_uiNtcTable_External[uiTemp]) && (ftemp > g_uiNtcTable_External[uiTemp + 1]))ucTemp = 0x66;
        if(0x66 == ucTemp) break;
    }
    if(0x66 == ucTemp)
    {
        ftemp = uiTemp;
        ftemp = (ftemp  - 40.00); // ��-40������
    }
    if(uiLLC_Temp1 == 0)      //����ͨ���쳣����Ϊͨ�ųɹ�ʱĬ���¶�Ϊ0��
    {
        ftemp = 0;
    }
    uiLLC_Temp1_AsTrue = 	ftemp;
    WriteLCDMsg_Float(T_LLC1Addr,&ftemp);

    ucTemp = 0;
    ftemp = uiLLC_Temp2;
    ftemp = ftemp /4096 *3.3;
    ftemp = (2.35-0.47*ftemp) /ftemp;
    ftemp = ftemp * 1000;                              //AD����ת��Ϊ����ֵ    LLCΪ���������470R��ѹ ��470R��ѹΪ����ѹ
    for(uiTemp = 0; uiTemp < 135; uiTemp++)            //��ͬ��ֵ��Ӧ�¶�ֵת��
    {
        if((ftemp <= g_uiNtcTable_External[uiTemp]) && (ftemp > g_uiNtcTable_External[uiTemp + 1]))ucTemp = 0x66;
        if(0x66 == ucTemp) break;
    }
    if(0x66 == ucTemp)
    {
        ftemp = uiTemp;
        ftemp = (ftemp  - 40.00); // ��-40������
    }
    if(uiLLC_Temp2 == 0)      //����ͨ���쳣����Ϊͨ�ųɹ�ʱĬ���¶�Ϊ0��
    {
        ftemp = 0;
    }
    uiLLC_Temp2_AsTrue = 	ftemp;
    WriteLCDMsg_Float(T_LLC2Addr,&ftemp);

    if(uiV_Output > 420 &&uiI_Output > 30)  //��������״̬�ж�
    {
        WriteLCDOfString(ChageStatusAddr,"�����");
    }
    else if(uiV_Output > 42)
    {
        WriteLCDOfString(ChageStatusAddr,"������");
    }
    else
    {
        WriteLCDOfString(ChageStatusAddr,"δ����");
    }

    WriteLCDOfString(BattaryTypeAddr,"﮵�");                      //���������ʾ

    if(Flg.bit.Hardware_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"Ӳ������");                      //������ʾ
    }
    else if(Flg.bit.OTP_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"���¹���");
    }
    else if(Flg.bit.Input_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"�����ѹ����");
    }
    else if(Flg.bit.Batarry_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"���δ����");
    }
    else if(Flg.bit.CAN_BMS_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"����BMS���ĳ�ʱ");
    }
    else if(Flg.bit.CAN_LLC_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"����LLC���ĳ�ʱ");
    }
    else if(Flg.bit.Usart_LLC_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"LLC���ڽ��ճ�ʱ");
    }
    else if(Flg.bit.Usart_PFC_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"PFC���ڽ��ճ�ʱ");
    }
    else if(Flg.bit.V_Output_OVP_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"�����ѹ");
    }
    else if(Flg.bit.I_Output_OCP_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"�������");
    }
    else if(Flg.bit.V_Output_UVP_Err)
    {
        WriteLCDOfString(Err_MSg_Addr,"���Ƿѹ");
    }
    else if(Flg.bit.V_Bus_390VErr)
    {
        WriteLCDOfString(Err_MSg_Addr,"ĸ�ߵ�ѹ�쳣");
    }
		else
		{
		    WriteLCDOfString(Err_MSg_Addr,"����״̬����");
		}

    if(uc_Output_En_Flg&&uc_Incident_Open_OBC_FLg)
    {
        uc_Incident_Open_OBC_FLg = 0;
        sprintf(cBuff, "%d%d-%d%d %d%d:%d%d:%d%d",(Hex_To_BCD(LCD_Month)>>4)&0x0F,Hex_To_BCD(LCD_Month)&0x0F,(Hex_To_BCD(LCD_Date)>>4)&0x0F,Hex_To_BCD(LCD_Date)&0x0F,(Hex_To_BCD(LCD_Hour)>>4)&0x0F,\
                Hex_To_BCD(LCD_Hour)&0x0F,(Hex_To_BCD(LCD_Minute)>>4)&0x0F,Hex_To_BCD(LCD_Minute)&0x0F,(Hex_To_BCD(LCD_Seconds)>>4)&0x0F, Hex_To_BCD(LCD_Seconds)&0x0F);
        WriteLCDOfString(Incident_Time_Start_Addr,cBuff);
        sprintf(cBuff, "����");
        WriteLCDOfString(Incident_Content_Start_Addr0x,cBuff);
    }

}

















