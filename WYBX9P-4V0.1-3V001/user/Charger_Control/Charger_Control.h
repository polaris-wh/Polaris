#ifndef __Charger_Control_H__
#define __Charger_Control_H__
#include "stm32f10x.h"

void ShowChargeStatusLed(void);
void ChargeProcess(void);
void ChargeStepManage(unsigned int uiChangeVoltage, unsigned int uiChangeCurrent, unsigned int uiConstantVoltage, unsigned int uiConstantCurrent, unsigned int uiChargeLimitTime );
void WriteLCDMsg(unsigned  short usDataAddr, unsigned char *Data);
void WriteLCDMsg_Float(unsigned  short usDataAddr , float *data);
void WriteLCDOfString(unsigned short usStringAddr,char *cData);
void Set_Output(unsigned int Voltage , unsigned int Currennt);
void OBC_Status_Manage(void);
void CAN_Msg_Handle(void);

extern unsigned int uiPFC_Temp1_AsTrue;         //OBC�ڲ��¶Ȼ���Ϊ�����¶Ȳ���
extern unsigned int uiPFC_Temp2_AsTrue;
extern unsigned int uiPFC_Temp3_AsTrue;
extern unsigned int uiLLC_Temp1_AsTrue;
extern unsigned int uiLLC_Temp2_AsTrue;

extern unsigned int uiV_Output;  //���������ѹ����  100mv��λ
extern unsigned int uiI_Output;  //���������������� 100mA��λ

extern unsigned int V_BMS_Set;  //BSM���������ѹ
extern unsigned int I_BMS_Set;  //BMS�����������

extern uint8_t CAN_LLC_RX_Data[8];
extern uint8_t CAN_LLC_RX_Data2[8];
extern uint8_t CAN_PFC_RX_Data[8];
extern uint8_t CAN_BMS_RX_Data[8];
extern uint8_t CAN_BMS_TX_Data[8];
extern uint8_t CAN_TX_LLC_Data[8];
extern unsigned long g_uiNtcTable_External[135];    //�¶ȵ���ֵ��Ӧ��

extern unsigned char LCD_Rx_Data[110];  //LCD���ڽ�����������
extern u8 LCD_Page_MSg_Flg;    //���յ�LCD���͵�ҳ����Ϣ��־
extern u8 LCD_Manual_Flg;       //LCD�ֶ����������ť��Ϣ��Ϣ��־
extern u16 LCD_Page_Msg_Data;   //LCD��ǰҳ������
extern u16 LCD_Manual_Data;   //LCD�ֶ����������ť��Ϣ
extern u16 LCD_Manual_V_OutputData;   //LCD�ֶ����õ�ѹ���������Ϣ
extern u16 LCD_Manual_I_OutputData;   //LCD�ֶ����õ������������Ϣ
extern u16 LCD_Manual_ReturnData;   //LCD�ֶ����÷�����ҳ�����
extern u16 LCD_Manual_CrcData;     //LCD�����ֶ�����У�����

extern u16 LCD_Manual_SetData;     //LCD�����ֶ����ð�ť���ز���
extern u8 LCD_Year;                //LCDʱ�����
extern u8 LCD_Month;
extern u8 LCD_Date;
extern u8 LCD_Hour;
extern u8 LCD_Minute;
extern u8 LCD_Seconds;



extern u16 LCD_Set_Year;                    //LCD����ʱ�����
extern u16 LCD_Set_Month;
extern u16 LCD_Set_Date;
extern u16 LCD_Set_Hour;
extern u16 LCD_Set_Minute;
extern u16 LCD_Set_Seconds;

extern unsigned char ucCAN_LLC_Rx_ErrFlg;
extern unsigned char ucCAN_LLC_Rx_ErrCnt;   //��LLCģ��ͨ�ų�ʱ����
extern unsigned char ucCAN_BMS_Rx_ErrFlg;
extern unsigned char ucCAN_BMS_Rx_ErrCnt;   //��BMSģ��ͨ�ų�ʱ����



union BITS {
    struct {
        unsigned int Hardware_Err : 1;
        unsigned int OTP_Err : 1;
        unsigned int Input_Err : 1;
        unsigned int Batarry_Err : 1;

        unsigned int CAN_BMS_Err : 1;
        unsigned int CAN_LLC_Err : 1;
        unsigned int Usart_LLC_Err : 1;
        unsigned int Usart_PFC_Err : 1;

        unsigned int V_Output_OVP_Err : 1;
        unsigned int V_Output_UVP_Err : 1;
        unsigned int I_Output_OCP_Err : 1;
        unsigned int V_Bus_390VErr :1;

        unsigned int NC1 : 2;
        unsigned int NC2 : 2;
    } bit;
    unsigned int all;
};
extern union BITS Flg;



struct strPerStepChargeInfo
{
    unsigned int uiChargeLimitTime;         //�˽׶γ��������ʱ��  1s  max 65535s = 18h
    unsigned int uiChargeChangeVoltage;     //�ڴ˽׶�ת������һ���׶ε�ѹֵ  10mV  ����˲�����Ϊ0 �Ժ�ѹ��ʽ���
    unsigned int uiChargeChangeCurrent;     //�ڴ˽׶�ת������һ���׶ε���ֵ 10mA  ����˲�����Ϊ0 �Ժ�����ʽ���
    unsigned int uiChargeConstantVoltage;   //�ڴ˽׶�Ҫ���ѹֵ����ʱ��ת����ĵ�ѹֵΪ0  10mV
    unsigned int uiChargeConstantCurrent;   //�ڴ˽׶�Ҫ�����ֵ����ʱ��ת����ĵ���ֵΪ0  10mA
};//ÿ�����״̬���׶Ρ���ص���������

struct strSysTemInfo
{
    unsigned char ucSystemState;                    //ϵͳ����״̬
    unsigned char ucSystemChargeStepState;          //ϵͳ���״̬����
    unsigned char ucSystemLedError;                 //ϵͳָʾ�ƴ���ָʾ״̬
    unsigned char ucCmdStatus;                      //�Ƿ���չ�������״̬
    unsigned char ucOutputMode;                     //���ģʽѡ��1����ģʽ��0���ģʽ
    unsigned char ucChargeHardWareEnFlag;           //���̵����������Ƿ�����־
    unsigned char ucBattStatus;                     //���״̬
    unsigned char ucPFCStatus;                      //PFC״̬
    u32 uiExternBatTemp;                            //�ⲿ����¶Ȳ���
    u32 uiInternAirTemp;                            //�ڲ�����¶Ȳ���
    u32 uiBattleVoltage;                            //��ص�ѹ����
    u32 uiChargeVoltage;                            //����ѹ����  ��ѹ�������ѹ
    u32 uiChargeCurrent;                            //����������
    u32 uiPowerIn;                                  //�����ѹ����
    u32 uiSetChargeVolValue100mVUnit;               //�����趨����ѹ��ֵȫ�ֱ���100mV
    u32 uiSetChargeCurValue100mAUnit;               //�����趨��������ֵȫ�ֱ���100mA
    u32 uiMaxChargeVoltage100mVUnit;                //�������ѹ
    u32 uiMinChargeVoltage100mVUnit;                //��С�����ѹ
    u32 uiMaxChargeCurrent100mAUnit;                //��������

    struct strPerStepChargeInfo strPreChangeStep1;  //��һ�׶�
    struct strPerStepChargeInfo strPreChangeStep2;
    struct strPerStepChargeInfo strChangeStep2;     //�׶β�һ���Ǻ������Ǻ�ѹ
    struct strPerStepChargeInfo strChangeStep3;
    struct strPerStepChargeInfo strChangeStep4;
    struct strPerStepChargeInfo strChangeStep5;
    struct strPerStepChargeInfo strChangeStep6;
};

#define SystemNoBatt                0x00//ϵͳ���޵��״̬
#define SystemInPreChargeStep1      0x01//ϵͳ��Ԥ���1״̬
#define SystemInPreChargeStep2      0x02//ϵͳ��Ԥ���2״̬
#define SystemInChargeStep2         0x03//ϵͳ�ڳ��״̬2
#define SystemInChargeStep3         0x04//ϵͳ�ڳ��״̬3
#define SystemInChargeStep4         0x05//ϵͳ�ڳ��״̬4
#define SystemInChargeStep5         0x06//ϵͳ�ڳ��״̬5
#define SystemInChargeStep6         0x07//ϵͳ�ڳ��״̬6
#define SystemInChargeFinish        0x08//ϵͳ������


#define ChargeModeInConstantVoltage   0x01//�ں�ѹģʽ�Ķ���
#define ChargeModeInConstantCurrent   0x02//�ں�ѹģʽ�Ķ���

//////////////////�������ǳ���ͨ��Э�����/////////////////
union  OBC_Err_type
{
    struct
    {
        unsigned char Bit1_2 :2;
        unsigned char Bit3_4 :2;
        unsigned char Bit5_6 :2;
        unsigned char Bit7_8 :2;
    } Bits;
    unsigned char all;
};

extern union  OBC_Err_type     OBC_Err_type_Msg1;
extern union  OBC_Err_type     OBC_Err_type_Msg2;
extern union  OBC_Err_type     OBC_Err_type_Msg3;

union  CAN_RX_OverTime
{
    struct
    {
        unsigned char Bit1_2 :2;
        unsigned char Bit3_4 :2;
        unsigned char Bit5_6 :2;
        unsigned char Bit7_8 :2;
    } Bits;
    unsigned char all;
};
extern union  CAN_RX_OverTime  CAN_RX_OverTime_Msg1;
extern union  CAN_RX_OverTime  CAN_RX_OverTime_Msg2;
extern union  CAN_RX_OverTime  CAN_RX_OverTime_Msg3;
extern union  CAN_RX_OverTime  CAN_RX_OverTime_Msg4;


#define CAN_Msg_In_Handshake                  0x01
#define CAN_Msg_In_Recongnition               0x02
#define CAN_Msg_In_ParameterConfiguration     0x03
#define CAN_Msg_In_Charge                     0x04
#define CAN_Msg_In_ChargeEnd                  0x05

////////////////////���Э�鴫��׶� ��ͬ�Ķ������ʹ�ܱ�־///////////
extern unsigned char CAN_Recongnition_1C02_MultiData_request_Flg;
extern unsigned char CAN_Recongnition_1C06_MultiData_request_Flg;
extern unsigned char CAN_Charge_1C11_MultiData_request_Flg;


extern unsigned char CAN_Msg_StepState;                   //CANͨѶ�׶���Ϣ
extern unsigned char CAN_HandShake_1827_Flg;              //���յ�BMS���ֱ��ı�־  0x182756F4
extern unsigned char CAN_Recongnition_1C02_Flg;           //���յ�BMS��ʶ���ı�־  0x1C0256F4
extern unsigned char CAN_Recongnition_1C06_Flg;           //���յ�BMS���ز������ı�־  0x1C0256F4    (����Э�鹦��  �������)
extern unsigned char CAN_ParameterConfiguration_1009_Flg; //���յ�BMS׼���������ı�־  0x100956F4
extern unsigned char CAN_Charge_EN_1810_Flg;              //���յ�BMS�������������ñ��ı�־  0x181056F4
extern unsigned char CAN_Charge_EN_1813_Flg;              //���յ�BMS���״̬���ı�־  0x181356F4
extern unsigned char CAN_Charge_EN_1019_Flg;              //���յ�BMS��ֹ��籨�ı�־  0x101956F4
extern unsigned char CAN_ChargeEnd_181C_Flg;              //���յ�BMS������ͳ�Ʊ��ı�־  0x181C56F4
extern unsigned char CAN_MultiData_request_Flg;           //������������־
extern unsigned char CAN_MultiData_RX_OK_Flg;             //������ͽ���������־


extern unsigned char CAN_HandShake_1827_Data[2];                //OBC�������ֱ�������
extern unsigned char CAN_Recongnition_1C02_Data[56];            //OBC���ձ�ʶ��������
extern unsigned char CAN_Recongnition_1C06_Data[16];            //OBC�������ز�����������

extern unsigned char CAN_ParameterConfiguration_1009_Data[1];   //OBC�ӵ��׼��״̬��������
extern unsigned char CAN_Charge_1810_Data[5];                   //OBC����BMS��������������
extern unsigned char CAN_Charge_1C11_Data[16];                  //OBC����BMS�����״̬����
extern unsigned char CAN_Charge_1813_Data[7];                   //OBC����BMS����״̬����
extern unsigned char CAN_Charge_1019_Data[4];                   //OBC����BMS��ֹ�������
extern unsigned char CAN_ChargeEnd_181C_Data[7];                //OBC����BMSͳ�Ʋ�������
extern unsigned char CAN_ChargeEnd_081E_Data[4];                //OBC����BMS����ͨ�Ź��ϣ�����

extern unsigned char CAN_ParameterConfiguration_TX1808_Data[8]; //OBC����������������������
extern unsigned char CAN_ParameterConfiguration_TX100A_Data[1]; //OBC����׼��״̬��������
extern unsigned char CAN_Rx_MultiData[8];                       //OBC���ն����������
extern unsigned char CAN_MultiData_request[8];                  //OBC���ն��������������



extern unsigned int ui_CAN1C02_Rx_Err_Cnt;     //OBC���ղ�ͬ֡ID��ʱ����
extern unsigned int ui_CAN1C06_Rx_Err_Cnt;
extern unsigned int ui_CAN1009_Rx_Err_Cnt;
extern unsigned int ui_CAN1810_Rx_Err_Cnt;
extern unsigned int ui_CAN1C11_Rx_Err_Cnt;
extern unsigned int ui_CAN1019_Rx_Err_Cnt;
extern unsigned int ui_CAN181C_Rx_Err_Cnt;


extern unsigned char OBC_Charge_Status;                   //����״̬��Ϣ   BMS�跴���Ƿ�����
extern unsigned int OBC_Charge_Time;                      //�����������ʱ�����
extern unsigned char uc_Output_En_Flg;   //���״̬��־λ

extern float OBC_Energy_Cal;        //OBC�������ͳ��  ���ü��1����ȡ���������ƽ���ķ�ʽ
extern float f_V_Output1;               //��ѹ����1
extern float f_V_Output2;               //��ѹ����2
extern float f_I_Output1;               //��������1
extern float f_I_Output2;               //��������2
extern unsigned int uiTimer60S_Cnt; //60s����
//////////////////�������ǳ���ͨ��Э�����/////////////////





#endif