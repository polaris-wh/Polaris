
#include "Charger_Control.h"
#include "System_Init.h"
#include "usart.h"
#include "stm32f10x_usart.h"
#include "CAN.h"
#include "PCF8563.h"

///////////////������������////////////////
unsigned char ucBat_Status = 0;



///////////////LEDָʾ��ģ���������///////////
unsigned int uiShowLedCnt=0;
unsigned char ucLedInitalStatusStep = 0;



///////////////ä��ģ���������///////////

#define  SampleChangeVoltageTime        10 //�ȴ�һ���ȶ�ʱ��1000ms
volatile struct strSysTemInfo g_strSysInfo = {0};//ϵͳһЩ״̬�ṹ��Ķ���

unsigned int uiChargeVoltageSet = 0;
unsigned int uiChargeCurrentSet = 0;
unsigned char ucChangeStepCnt = 0;

///////////////����������ñ�������///////////
float f_I_OutputCoef = 1;		         //��������ʲ���
unsigned int V_Set = 0;              //BMS����ֵת��ΪDSP�������ֵ
unsigned int I_Set = 0;              //BMS����ֵת��ΪDSP�������ֵ
unsigned int V_BMS_Set = 0;  //BSM���������ѹ
unsigned int I_BMS_Set = 0;  //BMS�����������


/////////////CANͨѶ���鶨��/////////////////
uint8_t CAN_LLC_RX_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_LLC_RX_Data2[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_PFC_RX_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_BMS_RX_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_BMS_TX_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_TX_LLC_Data[8] = {0,0,0,0,0,0,0,0};


//////////////////////�������ϼ��ģ�����/////////////

unsigned char ucOTPCnt = 0;          //���±�������
unsigned char ucOTPGrade1Cnt = 0;    //���½����ʽ׶��жϼ���  �����ʷ�3��
unsigned char ucOTPGrade2Cnt = 0;
unsigned char ucOTPRecoverCnt = 0;

unsigned char ucOTP_Flg = 0;          //���±�����־λ
unsigned char uc_Output_En_Flg = 0;   //���״̬��־λ


unsigned char uiV_Output_OVP_Err_Cnt = 0;   //�����ѹ���ϼ���
unsigned char uiV_Output_UVP_Err_Cnt = 0;   //���Ƿѹ���ϼ���
unsigned char uiV_Output_UVP_Delay_Cnt = 0; //���Ƿѹ���ϼ����ʱ����
unsigned char uiI_Output_Err_Cnt = 0;        //����������ϼ���

unsigned char ucCAN_LLC_Rx_ErrCnt = 0;     //��LLCģ��ͨ�ų�ʱ����
unsigned char ucCAN_BMS_Rx_ErrCnt = 0;     //��BMSģ��ͨ�ų�ʱ����

unsigned int uiLCD_Display_V_Set = 0;      //LCD��ؽ�����ʾ���õ�ѹ����   �������ֶ����ò�����BMS���ò�����
unsigned int uiLCD_Display_I_Set = 0;      //LCD��ؽ�����ʾ���õ�ѹ����
////////////���ϱ�־λ�����嶨��////////
union BITS Flg;


///////////////////�����������������///////////////
union
{
    char V_OutPut_Coef[3];
    float f_V_OutPut_Coef;
} V_D_Coef;                         //�����ѹ����ϵ��
union
{
    char V_OutPut_Offset[3];
    float f_V_OutPut_Offset;
} V_D_Offset;                       //�����ѹ����ƫ����
union
{
    char V_ADC_Coef[3];
    float f_V_ADC_Coef;
} V_A_Coef;                         //�����ѹ����ϵ��
union
{
    char V_ADC_Offset[3];
    float f_V_ADC_Offset;
} V_A_Offset;                       //�����ѹ����ƫ����    (������������ԶȲ��� ���Ƿֶα궨  Ԥ������ϵ�� ��ʱʹ��һ��)
union
{
    char I_OutPut_Coef[3];
    float f_I_OutPut_Coef;
} I_D_Coef;                         //�����������ϵ��
union
{
    char I_OutPut_Offset[3];
    float f_I_OutPut_Offset;
} I_D_Offset;                       //�����������ƫ����
union
{
    char I_OutPut_Coef1[3];
    float f_I_OutPut_Coef1;
} I_D_Coef1;                        //�����������ϵ��2
union
{
    char I_OutPut_Offset1[3];
    float f_I_OutPut_Offset1;
} I_D_Offset1;                     //�����������ƫ����2
union
{
    char I_OutPut_Coef2[3];
    float f_I_OutPut_Coef2;
} I_D_Coef2;                       //�����������ϵ��3
union
{
    char I_OutPut_Offset2[3];
    float f_I_OutPut_Offset2;
} I_D_Offset2;                     //�����������ƫ����3
union
{
    char I_ADC_Coef[3];
    float f_I_ADC_Coef;
} I_A_Coef;                        //�����������ϵ��
union
{
    char I_ADC_Offset[3];
    float f_I_ADC_Offset;
} I_A_Offset;                       //�����������ƫ����

///////////////////�����������������///////////////


//////////////////�������ǳ���ͨ��Э�����/////////////////

//����Э�鹦��   ID 0x1CEC56F4  BMS����������  0x1CECf456  OBC�ظ�����     0x1CEB56F4  BMS������Ϣ

unsigned char CAN_Msg_StepState = 0;                   //CANͨѶ�׶���Ϣ
unsigned char CAN_HandShake_1827_Flg = 0;              //���յ�BMS���ֱ��ı�־  0x182756F4
unsigned char CAN_Recongnition_1C02_Flg = 0;           //���յ�BMS��ʶ���ı�־  0x1C0256F4
unsigned char CAN_Recongnition_1C06_Flg = 0;           //���յ�BMS���ز������ı�־  0x1C0256F4    (����Э�鹦��  �������)
unsigned char CAN_ParameterConfiguration_1009_Flg = 0; //���յ�BMS׼���������ı�־  0x100956F4
unsigned char CAN_Charge_EN_1810_Flg = 0;              //���յ�BMS�������������ñ��ı�־  0x181056F4
unsigned char CAN_Charge_EN_1813_Flg = 0;              //���յ�BMS���״̬���ı�־  0x181356F4
unsigned char CAN_Charge_EN_1019_Flg = 0;              //���յ�BMS��ֹ��籨�ı�־  0x101956F4
unsigned char CAN_ChargeEnd_181C_Flg = 0;              //���յ�BMS������ͳ�Ʊ��ı�־  0x181C56F4
unsigned char CAN_MultiData_request_Flg = 0;           //������������־
unsigned char CAN_MultiData_RX_OK_Flg = 0;             //������ͽ���������־

unsigned char uc_CAN101A_Tx_Flg = 0;                   //OBC����������ֹ���ͱ��ı�־

unsigned char OBC_Charge_Status = 0;                   //����״̬��Ϣ   BMS�跴���Ƿ�����
unsigned int OBC_Charge_Time = 0;                      //�����������ʱ�����

float OBC_Energy_Cal = 0;        //OBC�������ͳ��  ���ü��1����ȡ���������ƽ���ķ�ʽ
float f_V_Output1;               //��ѹ����1
float f_V_Output2;               //��ѹ����2
float f_I_Output1;               //��������1
float f_I_Output2;               //��������2
unsigned int uiTimer60S_Cnt = 0; //60s����

unsigned int uiBMS_Set_Max_V_Output = 0;               //�����ߵ�ѹ����

unsigned char CAN_HandShake_1827_Data[2] = {0};                //OBC�������ֱ�������
unsigned char CAN_Recongnition_1C02_Data[56] = {0};            //OBC���ձ�ʶ��������
unsigned char CAN_Recongnition_1C06_Data[16] = {0};            //OBC�������ز�����������
unsigned char CAN_ParameterConfiguration_1009_Data[1] = {0};   //OBC�ӵ��׼��״̬��������
unsigned char CAN_Charge_1810_Data[5] = {0};                   //OBC����BMS��������������
unsigned char CAN_Charge_1C11_Data[16] = {0};                  //OBC����BMS�����״̬����
unsigned char CAN_Charge_1813_Data[7] = {0};                   //OBC����BMS����״̬����
unsigned char CAN_Charge_1019_Data[4] = {0};                   //OBC����BMS��ֹ�������
unsigned char CAN_ChargeEnd_181C_Data[7] = {0};                //OBC����BMSͳ�Ʋ�������
unsigned char CAN_ChargeEnd_081E_Data[4] = {0};                //OBC����BMS����ͨ�Ź��ϣ�����


unsigned char CAN_Rx_MultiData[8] = {0};                       //OBC���ն����������
unsigned char CAN_MultiData_request[8] = {0};                  //OBC���ն��������������
unsigned char CAN_MultiData_Answer[8] = {0};                   //OBC����������������������


////////////////////���Э�鴫��׶� ��ͬ�Ķ������ʹ�ܱ�־///////////
unsigned char CAN_Recongnition_1C02_MultiData_request_Flg = 0;
unsigned char CAN_Recongnition_1C06_MultiData_request_Flg = 0;
unsigned char CAN_Charge_1C11_MultiData_request_Flg = 0;

unsigned char CAN_HandShake_Tx1826_Data[3] = {0};              //OBC�������ֱ�������
unsigned char CAN_Recongnition_TX1801_Data[8] = {0};           //OBC���ͱ�ʶ��������
unsigned char CAN_ParameterConfiguration_TX1807_Data[8] = {0}; //ʱ��ͬ����Ϣ����
unsigned char CAN_ParameterConfiguration_TX1808_Data[8] = {0}; //OBC����������������������
unsigned char CAN_ParameterConfiguration_TX100A_Data[1] = {0}; //OBC����׼��״̬��������
unsigned char CAN_Charge_TX1812_Data[8] = {0};                 //OBC���ͳ��״̬��������
unsigned char CAN_Charge_TX101A_Data[4] = {0};                 //OBC������ֹ��籨������
unsigned char CAN_ChargeEnd_TX181D_Data[8] = {0};              //OBC����ͳ�Ʋ�����������
unsigned char CAN_ChargeEnd_TX081F_Data[4] = {0};              //OBC���ʹ���ͨ�Ŵ��󣩱�������

unsigned int BMS_Set_I_Output = 0;                            //BMS���������������

unsigned char OBC_Charge_Stop_Reason_BMS_Stop = 0;            //OBC��������ֹͣԭ�����

union  OBC_Err_type     OBC_Err_type_Msg1;                    //OBC���Ϸ�������������
union  OBC_Err_type     OBC_Err_type_Msg2;
union  OBC_Err_type     OBC_Err_type_Msg3;

union  CAN_RX_OverTime  CAN_RX_OverTime_Msg1;                 //OBCͨ�ų�ʱ����������
union  CAN_RX_OverTime  CAN_RX_OverTime_Msg2;
union  CAN_RX_OverTime  CAN_RX_OverTime_Msg3;
union  CAN_RX_OverTime  CAN_RX_OverTime_Msg4;

unsigned int ui_CAN1C02_Rx_Err_Cnt = 0;                      //OBC���ղ�ͬ֡ID��ʱ����
unsigned int ui_CAN1C06_Rx_Err_Cnt = 0;
unsigned int ui_CAN1009_Rx_Err_Cnt = 0;
unsigned int ui_CAN1810_Rx_Err_Cnt = 0;
unsigned int ui_CAN1C11_Rx_Err_Cnt = 0;
unsigned int ui_CAN1019_Rx_Err_Cnt = 0;
unsigned int ui_CAN181C_Rx_Err_Cnt = 0;



//////////////////�������ǳ���ͨ��Э�����/////////////////


//////////////////ä������̹�����///////////////////////////
void ChargeProcess(void)
{
    if (g_strSysInfo.ucSystemChargeStepState != SystemInChargeFinish)//������û����� һֱ�ڳ�������ѭ��
    {
        switch(g_strSysInfo.ucSystemChargeStepState)
        {
        case SystemInPreChargeStep1 :
        {
            while(SystemInPreChargeStep1 == g_strSysInfo.ucSystemChargeStepState) //�����Ԥ��������ѹ����
            {
                //�����Ԥ���  Ҫʵʱ��ص�ѹ�Ƿ��ڱ仯�����������ҵ���
                ChargeStepManage(g_strSysInfo.strPreChangeStep1.uiChargeChangeVoltage,
                                 g_strSysInfo.strPreChangeStep1.uiChargeChangeCurrent, g_strSysInfo.strPreChangeStep1.uiChargeConstantVoltage,
                                 g_strSysInfo.strPreChangeStep1.uiChargeConstantCurrent, g_strSysInfo.strPreChangeStep1.uiChargeLimitTime);
            }
            break;
        }
        case SystemInPreChargeStep2 :
        {
            while(SystemInPreChargeStep2 == g_strSysInfo.ucSystemChargeStepState) //�����Ԥ��������ѹ����
            {
                //�����Ԥ���  Ҫʵʱ��ص�ѹ�Ƿ��ڱ仯�����������ҵ���
                ChargeStepManage(g_strSysInfo.strPreChangeStep2.uiChargeChangeVoltage,
                                 g_strSysInfo.strPreChangeStep2.uiChargeChangeCurrent, g_strSysInfo.strPreChangeStep2.uiChargeConstantVoltage,
                                 g_strSysInfo.strPreChangeStep2.uiChargeConstantCurrent, g_strSysInfo.strPreChangeStep2.uiChargeLimitTime);
            }
            break;
        }
        /****************************************************************************************************/
        case SystemInChargeStep2 :
        {
            while(SystemInChargeStep2 == g_strSysInfo.ucSystemChargeStepState)
            {
                ChargeStepManage(g_strSysInfo.strChangeStep2.uiChargeChangeVoltage,
                                 g_strSysInfo.strChangeStep2.uiChargeChangeCurrent, g_strSysInfo.strChangeStep2.uiChargeConstantVoltage,
                                 g_strSysInfo.strChangeStep2.uiChargeConstantCurrent, g_strSysInfo.strChangeStep2.uiChargeLimitTime);
            }
            break;
        }
        case SystemInChargeStep3 :
        {
            ChargeStepManage(g_strSysInfo.strChangeStep3.uiChargeChangeVoltage,
                             g_strSysInfo.strChangeStep3.uiChargeChangeCurrent, g_strSysInfo.strChangeStep3.uiChargeConstantVoltage,
                             g_strSysInfo.strChangeStep3.uiChargeConstantCurrent, g_strSysInfo.strChangeStep3.uiChargeLimitTime);
            break;
        }
        case SystemInChargeStep4 :
        {
            while(SystemInChargeStep4 == g_strSysInfo.ucSystemChargeStepState)
            {
                ChargeStepManage(g_strSysInfo.strChangeStep4.uiChargeChangeVoltage,
                                 g_strSysInfo.strChangeStep4.uiChargeChangeCurrent, g_strSysInfo.strChangeStep4.uiChargeConstantVoltage,
                                 g_strSysInfo.strChangeStep4.uiChargeConstantCurrent, g_strSysInfo.strChangeStep4.uiChargeLimitTime);
            }
            break;
        }
        case SystemInChargeStep5 :
        {
            while(SystemInChargeStep5 == g_strSysInfo.ucSystemChargeStepState)
            {
                ChargeStepManage(g_strSysInfo.strChangeStep5.uiChargeChangeVoltage,
                                 g_strSysInfo.strChangeStep5.uiChargeChangeCurrent, g_strSysInfo.strChangeStep5.uiChargeConstantVoltage,
                                 g_strSysInfo.strChangeStep5.uiChargeConstantCurrent, g_strSysInfo.strChangeStep5.uiChargeLimitTime);
            }
            break;
        }
        case SystemInChargeStep6 :
        {
            while(SystemInChargeStep6 == g_strSysInfo.ucSystemChargeStepState)
            {
                ChargeStepManage(g_strSysInfo.strChangeStep6.uiChargeChangeVoltage,
                                 g_strSysInfo.strChangeStep6.uiChargeChangeCurrent, g_strSysInfo.strChangeStep6.uiChargeConstantVoltage,
                                 g_strSysInfo.strChangeStep6.uiChargeConstantCurrent, g_strSysInfo.strChangeStep6.uiChargeLimitTime);
            }
            break;
        }
        case SystemInChargeFinish://������
        {

            break;
        }
        default :
        {
            break;
        }
        }
    }

}


void ChargeStepManage(unsigned int uiChangeVoltage, unsigned int uiChangeCurrent, unsigned int uiConstantVoltage, unsigned int uiConstantCurrent, unsigned int uiChargeLimitTime )
{
    unsigned char ucChargeModeJudge = 0;//���ģʽ�жϱ���

    if((0 == uiChangeVoltage) && (0 == uiChangeCurrent) && (0 == uiConstantVoltage) && (0 == uiConstantCurrent) && (0 == uiChargeLimitTime))
    {
        g_strSysInfo.ucSystemChargeStepState += 1;
        return;//����öεĳ�����Ϊ0˵����Ҫ�˶εĳ�����ߣ�ת�뵽��һ��״̬�����Ҳ�����紦��
    }
    if(uiChangeVoltage && uiConstantCurrent)ucChargeModeJudge = ChargeModeInConstantCurrent;
    else if(uiChangeCurrent && uiConstantVoltage)ucChargeModeJudge = ChargeModeInConstantVoltage;
    else return;//���ں�ѹ������ģʽ���Ǵ����ģʽ

    if(ChargeModeInConstantCurrent == ucChargeModeJudge)//������ں���ģʽ
    {
        uiChargeVoltageSet = uiChangeVoltage + 40;
        uiChargeCurrentSet = 	uiConstantCurrent;

        if(uiV_Output >= uiChangeVoltage)
        {
            ucChangeStepCnt++;
        }
        else
        {
            ucChangeStepCnt = 0;
        }
        if(ucChangeStepCnt >=SampleChangeVoltageTime)
        {
            g_strSysInfo.ucSystemChargeStepState += 1;
            ucChangeStepCnt = 0;
        }
    }
    else if(ChargeModeInConstantVoltage == ucChargeModeJudge)//������ں�ѹģʽ
    {
        uiChargeVoltageSet = uiConstantVoltage;
        if(uiI_Output <= uiChangeCurrent)
        {
            ucChangeStepCnt++;
        }
        else
        {
            ucChangeStepCnt = 0;
        }

        if(ucChangeStepCnt >=SampleChangeVoltageTime)
        {
            if(SystemInChargeStep6 != g_strSysInfo.ucSystemChargeStepState)
            {
                g_strSysInfo.ucSystemChargeStepState += 1;
                ucChangeStepCnt = 0;
            }
            else
            {
                g_strSysInfo.ucSystemChargeStepState = SystemInChargeFinish;
            }
        }
    }
    if(ui100msCnt > uiChargeLimitTime)
    {
        if(SystemInChargeStep6 != g_strSysInfo.ucSystemChargeStepState)
            g_strSysInfo.ucSystemChargeStepState += 1;
        else
            g_strSysInfo.ucSystemChargeStepState = SystemInChargeFinish;
    }
    Set_Output(uiChargeVoltageSet , uiChargeCurrentSet);

}


void Set_Output(unsigned int Voltage , unsigned int Currennt)
{
    float f_OutputTemp = 0;
    V_D_Coef.f_V_OutPut_Coef = 21.9;                    //�����ѹ����ϵ����ƫ��������
    V_D_Offset.f_V_OutPut_Offset = 175;
    I_D_Coef.f_I_OutPut_Coef = 17.18;
    I_D_Offset.f_I_OutPut_Offset = -343;
    V_Set = Voltage;
    I_Set = Currennt;
    uiLCD_Display_V_Set = Voltage;
    uiLCD_Display_I_Set = Currennt;
    if(V_Set > 1100)     //BMS���������ѹ�����޷�
    {
        V_Set = 1100;
    }
    if(V_Set < 500)
    {
        V_Set = 500;
    }
    if(I_Set > 1100)
    {
        I_Set = 1100;
    }
    if(I_Set < 50)
    {
        I_Set = 50;
    }

    I_Set = I_Set *f_I_OutputCoef;   //����������   f_I_OutputCoefΪ������ϵ��

    f_OutputTemp = V_Set;
    f_OutputTemp = f_OutputTemp * V_D_Coef.f_V_OutPut_Coef + V_D_Offset.f_V_OutPut_Offset;  //���õ�ѹֵת��ΪLLC��Ƭ����·�����Ĳ���
    V_Set = f_OutputTemp;
    f_OutputTemp = I_Set;
    f_OutputTemp = f_OutputTemp * I_D_Coef.f_I_OutPut_Coef + I_D_Offset.f_I_OutPut_Offset;

    I_Set = f_OutputTemp;

    CAN_TX_LLC_Data[0] = V_Set >> 8;     //CANͨѶ���͵�ѹ�������õ�����
    CAN_TX_LLC_Data[1] = V_Set;
    CAN_TX_LLC_Data[2] = I_Set >> 8;
    CAN_TX_LLC_Data[3] = I_Set;
    if(LCD_Manual_Data == 0x01)
    {
        if(!(Flg.all&0x0FE7))
        {
            CAN_TX_LLC_Data[4] = 0x00;
            uc_Output_En_Flg = 1;
        }
        else
        {
            CAN_TX_LLC_Data[4] = 0x01;
            uc_Output_En_Flg = 0;
        }
    }
    else
    {
        if((!(Flg.all&0x0FFF))&&uiV_Output>420&&CAN_BMS_Rx_Flg&&(CC1_Control == 0)&&(CAN_BMS_RX_Data[4] == 0x00))    //��������״̬����Ᵽ��  ����ֹͣ�������
        {
            CAN_TX_LLC_Data[4] = 0x00;
            uc_Output_En_Flg = 1;
        }
        else
        {
            CAN_TX_LLC_Data[4] = 0x01;
            uc_Output_En_Flg = 0;
        }
    }
    Can_Send_LLC_Msg(CAN_TX_LLC_Data);      //CANͨѶ��������
}


/*******************************************************************************
 ����������:  void OBC_Status_Manage(void)
 ����:        �¶ȱ����������Ƿѹ�������ѹ��������Ƿѹ��ͨ�Ź��ϱ���
 �������:    ��
 �������:    ��
 ����˵��:
*******************************************************************************/
void OBC_Status_Manage(void)
{
/////////////////////////////�¶ȱ���//////////////////////////////////////
// LLCģ�����85�ȣ�BUCKģ�����90�ȹ��±���
// LLCģ�����70�ȣ�BUCKģ�����75�Ȼָ�������
//	LLCģ�����80�ȣ�BUCKģ�����85�Ƚ�������90%
    if(uiLLC_Temp2_AsTrue >= 85 ||uiLLC_Temp1_AsTrue >= 85||uiPFC_Temp1_AsTrue >= 90||uiPFC_Temp2_AsTrue >= 90||uiPFC_Temp3_AsTrue >= 90)
    {
        ucOTPCnt++;
        if(ucOTPCnt >= 100)
        {
            ucOTPCnt = 100;
            Flg.bit.OTP_Err = 1;
        }
    }
    else
    {
        ucOTPCnt = 0;
    }
    if(uiLLC_Temp2_AsTrue <= 70 &&uiLLC_Temp1_AsTrue <= 70&&uiPFC_Temp1_AsTrue <= 75&&uiPFC_Temp2_AsTrue <= 75&&uiPFC_Temp3_AsTrue <= 75)
    {
        ucOTPRecoverCnt++;
        ucOTPGrade2Cnt = 0;
        ucOTPGrade1Cnt = 0;
        if(ucOTPRecoverCnt>=100)
        {
            ucOTPRecoverCnt = 100;
            f_I_OutputCoef = 1;
            Flg.bit.OTP_Err = 0;
        }

    }
    else if(uiLLC_Temp2_AsTrue >= 80 ||uiLLC_Temp1_AsTrue >= 80||uiPFC_Temp1_AsTrue >= 85||uiPFC_Temp2_AsTrue >= 85||uiPFC_Temp3_AsTrue >= 85)
    {
        ucOTPGrade2Cnt++;
        ucOTPRecoverCnt = 0;
        ucOTPGrade1Cnt = 0;
        if(ucOTPGrade2Cnt>=100)
        {
            ucOTPGrade2Cnt = 100;
            f_I_OutputCoef = 0.9;
        }
    }
//    else if(uiLLC_Temp2_AsTrue >= 50 ||uiLLC_Temp1_AsTrue >= 50||uiPFC_Temp1_AsTrue >= 50||uiPFC_Temp2_AsTrue >= 50||uiPFC_Temp3_AsTrue >= 50)
//    {
//        ucOTPGrade1Cnt++;
//        ucOTPRecoverCnt = 0;
//        ucOTPGrade2Cnt = 0;
//        if(ucOTPGrade1Cnt>=100)
//        {
//            ucOTPGrade1Cnt = 100;
//            f_I_OutputCoef = 0.8;
//        }

//    }

/////////////////////////////�¶ȱ���///////////////////////////////////////

/////////////////////////////���״̬����//////////////////////////////////////

    if(uiI_Output > 1100)     //�����ѹ����  �������5S
    {
        uiI_Output_Err_Cnt++;
        if(uiI_Output_Err_Cnt >= 50)
        {
            uiI_Output_Err_Cnt = 50;
            Flg.bit.I_Output_OCP_Err = 1;
        }
    }
    else
    {
        uiI_Output_Err_Cnt = 0;
    }

    if(uiV_Output > 1100)      //�����������  �������5S
    {
        uiV_Output_OVP_Err_Cnt++;
        if(uiV_Output_OVP_Err_Cnt >= 50)
        {
            uiV_Output_OVP_Err_Cnt = 50;
            Flg.bit.V_Output_OVP_Err = 1;
        }
    }
    else
    {
        uiV_Output_OVP_Err_Cnt = 0;
    }

    if(uc_Output_En_Flg)    //Ƿѹ��Ᵽ����ʱ10S  �������ʱ�ȴ�������������Ƿ����Ƿѹ
    {
        uiV_Output_UVP_Delay_Cnt++;
        if(uiV_Output_UVP_Delay_Cnt > 100)
        {
            uiV_Output_UVP_Delay_Cnt = 100;
        }
    }
    else
    {
        uiV_Output_UVP_Delay_Cnt = 0;
    }
    if((uiV_Output < 420)&&(uiV_Output_UVP_Delay_Cnt >= 100))//Ƿѹ��Ᵽ��
    {
        uiV_Output_UVP_Err_Cnt++;
        if(uiV_Output_UVP_Err_Cnt >= 100)
        {
            uiV_Output_UVP_Err_Cnt = 100;
            Flg.bit.V_Output_UVP_Err = 1;
        }
    }
    else
    {
        uiV_Output_UVP_Err_Cnt = 0;
    }

    if(uiV_Output <= 420)         //��⵽��ص�ѹ
    {
        ucBat_Status = 0;
        Flg.bit.Batarry_Err = 1;
    }
    else
    {
        ucBat_Status = 1;
        Flg.bit.Batarry_Err = 0;
    }
    /////////////////////////////���״̬����//////////////////////////////////////

    /////////////////////////////ͨ�ų�ʱ����//////////////////////////////////////
    ucCAN_LLC_Rx_ErrCnt++;             //��LLCģ��CANͨѶ��ʱ���  ����5S
    if(ucCAN_LLC_Rx_ErrCnt >= 50)
    {
        ucCAN_LLC_Rx_ErrCnt = 50;
        Flg.bit.CAN_LLC_Err = 1;
    }
    else
    {
        Flg.bit.CAN_LLC_Err = 0;
    }
    ucCAN_BMS_Rx_ErrCnt++;
    if(ucCAN_BMS_Rx_ErrCnt >= 50)     //��NMSģ��CANͨѶ��ʱ���  ����5S
    {
        ucCAN_BMS_Rx_ErrCnt = 50;
        Flg.bit.CAN_BMS_Err = 1;
    }
    else
    {
        Flg.bit.CAN_BMS_Err = 0;
    }
    /////////////////////////////ͨ�ų�ʱ����//////////////////////////////////////



}

void ShowChargeStatusLed(void)
{
    uiShowLedCnt++;
    uiShowLedCnt %=100000;
    /*****************************�ϵ��ʼ״̬  ���ָʾ��ѭ��*******************/
    if(ucLedInitalStatusStep < 6)
    {
        if(uiShowLedCnt % 2)
        {
            if(ucLedInitalStatusStep == 0)
            {
                ucLedInitalStatusStep++;
                C_EnOutStateLed1();
            }
            else if(ucLedInitalStatusStep == 1)
            {
                ucLedInitalStatusStep++;
                C_EnOutStateLed2();
            }
            else if(ucLedInitalStatusStep == 2)
            {
                ucLedInitalStatusStep++;
                C_EnOutStateLed3();
            }
            else if(ucLedInitalStatusStep == 3)
            {
                ucLedInitalStatusStep++;
                C_EnOutStateLed4();
            }
            else if(ucLedInitalStatusStep == 4)
            {
                ucLedInitalStatusStep++;
                C_EnOutStateLed5();
            }
        }
        else
        {
            if(ucLedInitalStatusStep == 5)
            {
                ucLedInitalStatusStep++;
                ucLedInitalStatusStep = 0;
            }
            C_DisEnOutStateLed1();
            C_DisEnOutStateLed2();
            C_DisEnOutStateLed3();
            C_DisEnOutStateLed4();
            C_DisEnOutStateLed5();

        }
    }
    /*****************************�ϵ��ʼ״̬  ���ָʾ��ѭ��*******************/


    /*****************************��������״̬                *******************/
    else
    {
        if(CAN_BMS_Rx_Flg)                          //�����ͨ��ָʾ�ƿ��� ���յ�BMSͨѶʱ Ĭ��ͨ��״̬  ʹ��ͨѶ��
        {
            C_EnOutStateLed1();
        }
        else
        {
            C_DisEnOutStateLed1();
        }

        if(CAN_BMS_RX_Data[4] == 0x01)  //���յ��ػ��ź�  ��������ָʾ��
        {
            C_EnOutStateLed2();
            C_DisEnOutStateLed3();
            C_DisEnOutStateLed4();
            C_DisEnOutStateLed5();
        }
        else
        {
            /*****************************����״̬                *******************/
            if(Flg.all & 0x0FF7)           //���ֹ���ʱ �رպ�ѹ ����ָʾ�� �������ϵ�
            {
                C_DisEnOutStateLed3();
                C_DisEnOutStateLed4();
                C_EnOutStateLed5();
            }
            /*****************************����ָʾ״̬               *******************/
            else
            {
                C_DisEnOutStateLed5();
                if(uc_Output_En_Flg == 0&&uiI_Output > 30)    //���ʹ���������������3AΪ����״̬
                {
                    C_EnOutStateLed4();
                }
                else
                {
                    C_DisEnOutStateLed4();
                }
            }

        }


    }


}

///////////////////////ͨ��Э��//////////////////////////////////
void CAN_Msg_Handle(void)
{
//��������״̬��BMS
    CAN_BMS_TX_Data[0] = CAN_LLC_RX_Data[0];   //ǰ��λΪ�����ѹ
    CAN_BMS_TX_Data[1] = CAN_LLC_RX_Data[1];
    CAN_BMS_TX_Data[2] = CAN_LLC_RX_Data[2];	 //����λΪ�������
    CAN_BMS_TX_Data[3] = CAN_LLC_RX_Data[3];
    CAN_BMS_TX_Data[4] = (Flg.all&0xFF1F);     //������λΪ���ϴ���
    CAN_BMS_TX_Data[5] = (Flg.all&0xFFE0) >> 5;
    CAN_BMS_TX_Data[6] = CAN_PFC_RX_Data[7] ;  //Ԥ��λ
    CAN_BMS_TX_Data[7] = CAN_PFC_RX_Data[6] ;

    Can_Send_BMS_Msg(CAN_BMS_TX_Data,8,0x18FF50E6);      //����������Ϣ��BMS
}



////////////////////����CANЭ��////////////////////////////////

//void CAN_Msg_Handle(void)
//{

//	///////////////////////////�����ڸ��׶ν���BMS���ĳ�ʱ�ж�//////////////////////
////	if((CAN_Msg_StepState == CAN_Msg_In_Recongnition)&&(CAN_Recongnition_1C06_Flg == 0))       //��ʶ�׶�  �жϽ���BMS������ʶ���ĳ�ʱ 5s
////	  {
////		  ui_CAN1C02_Rx_Err_Cnt++;
////		  if(ui_CAN1C02_Rx_Err_Cnt >= 500)
////		    {
////		     CAN_RX_OverTime_Msg1.Bits.Bit1_2 = 0x01;
////		    }
////			else
////			{
////			CAN_RX_OverTime_Msg1.Bits.Bit1_2 = 0x00;
////			}
////		}
////	if(CAN_Msg_StepState == CAN_Msg_In_ParameterConfiguration)                  //�������ý׶�  �
////	  {
////		if(CAN_Recongnition_1C06_Flg == 0)                                         //���ղ������ñ���(5S)��ʱ
////			{
////	    ui_CAN1C06_Rx_Err_Cnt++;
////		  if(ui_CAN1C06_Rx_Err_Cnt >= 500)
////		    {
////		     CAN_RX_OverTime_Msg2.Bits.Bit1_2 = 0x01;
////		    }
////			else
////			  {
////			  CAN_RX_OverTime_Msg2.Bits.Bit1_2 = 0x00;
////			  }
////		  }
////		else
////		  {
////		  ui_CAN1009_Rx_Err_Cnt++;                                               //���յ��׼��������60S�����ĳ��
////		  if(ui_CAN1009_Rx_Err_Cnt >= 6000)
////		    {
////		     CAN_RX_OverTime_Msg2.Bits.Bit3_4 = 0x01;
////		    }
////			else
////			  {
////			  CAN_RX_OverTime_Msg2.Bits.Bit3_4 = 0x00;
////			  }
////		  }
////	  }
////
////	if(CAN_Msg_StepState == CAN_Msg_In_Charge)            //���׶�
////	  {
////	    ui_CAN1810_Rx_Err_Cnt++;
////		  if(ui_CAN1810_Rx_Err_Cnt >= 100)                 //���յ�س�������ĳ�ʱ
////		    {
////		     CAN_RX_OverTime_Msg3.Bits.Bit3_4 = 0x01;
////		    }
////			else
////			  {
////			  CAN_RX_OverTime_Msg3.Bits.Bit3_4 = 0x00;
////			  }
////			 ui_CAN1C11_Rx_Err_Cnt++;
////		  if(ui_CAN1C11_Rx_Err_Cnt >= 500)                 //���յ����״̬���ĳ�ʱ
////		    {
////		     CAN_RX_OverTime_Msg3.Bits.Bit1_2 = 0x01;
////		    }
////			else
////			  {
////			  CAN_RX_OverTime_Msg3.Bits.Bit1_2 = 0x00;
////			  }
////
////		if(uc_CAN101A_Tx_Flg)                             //����BMS�����ֹ������ʱ
////		   {
////		   ui_CAN1019_Rx_Err_Cnt++;
////		   if(ui_CAN1019_Rx_Err_Cnt >= 500)
////		     {
////		     CAN_RX_OverTime_Msg3.Bits.Bit5_6 = 0x01;
////		     }
////			 else
////			   {
////			   CAN_RX_OverTime_Msg3.Bits.Bit5_6 = 0x00;
////			   }
////		   }
////	  }
////
////if(CAN_Msg_StepState == CAN_Msg_In_ChargeEnd)         //�������׶�  ����BMSͳ�Ʊ��ĳ�ʱ
////	  {
////		  ui_CAN181C_Rx_Err_Cnt++;
////		  if(ui_CAN181C_Rx_Err_Cnt >= 500)
////		    {
////		     CAN_RX_OverTime_Msg4.Bits.Bit1_2 = 0x01;
////		    }
////			else
////			{
////			CAN_RX_OverTime_Msg4.Bits.Bit1_2 = 0x00;
////			}
////		}
//
/////////////////////////////�����ڸ��׶ν���BMS���ĳ�ʱ�ж�//////////////////////
//
//
/////////////////////////////�����ڸ��׶η�����Ϣ����Ϣ����//////////////////////
//	uiV_Output= CAN_LLC_RX_Data[0] * 256 + CAN_LLC_RX_Data[1];
//	uiI_Output = CAN_LLC_RX_Data[2] * 256 + CAN_LLC_RX_Data[3];
//	OBC_Charge_Time = ui100msCnt /600;
/////////////////////////////���������������//////////////////////
//	if((CAN_Charge_1813_Data[6] &0x10) == 0)
//	{
//	OBC_Charge_Stop_Reason_BMS_Stop |= 0x40;
//	}
//	if(1)
//	{
//	OBC_Charge_Status = 0x01;
//	uiTimer60S_Cnt++;
//	if(uiTimer60S_Cnt == 12000)
//	{
//	uiTimer60S_Cnt = 0;
//	f_V_Output2 = CAN_LLC_RX_Data[0] * 256 + CAN_LLC_RX_Data[1];
//	f_I_Output2	= CAN_LLC_RX_Data[2] * 256 + CAN_LLC_RX_Data[3];
//
//	OBC_Energy_Cal = OBC_Energy_Cal + (f_V_Output1 + f_V_Output2)*(f_V_Output1 + f_V_Output2)/4/60;
//	}
//	if(uiTimer60S_Cnt == 6000)
//	{
//	f_V_Output1 = CAN_LLC_RX_Data[0] * 256 + CAN_LLC_RX_Data[1];
//	f_I_Output1	= CAN_LLC_RX_Data[2] * 256 + CAN_LLC_RX_Data[3];
//	}
//	}
//	else
//	{
//	OBC_Charge_Status = 0x00;
//	}
/////////////////////////////���������������//////////////////////
//if (ucTag250ms)
//{
//ucTag250ms = 0;
//if(CAN_Msg_StepState == CAN_Msg_In_Handshake)
//  {
//	    CAN_HandShake_Tx1826_Data[0] = 0x01;
//		  CAN_HandShake_Tx1826_Data[1] = 0x01;
//			CAN_HandShake_Tx1826_Data[2] = 0x00;
//	    Can_Send_BMS_Msg(CAN_HandShake_Tx1826_Data, 3, 0x1826F456);
//
//	if(CAN_HandShake_1827_Flg)
//	  {
//	  CAN_Msg_StepState = CAN_Msg_In_Recongnition;
//	  uiBMS_Set_Max_V_Output = CAN_HandShake_1827_Data[0] * 256 + CAN_HandShake_1827_Data[1];
//	  }
//  }

//if(CAN_Msg_StepState == CAN_Msg_In_Recongnition)
//  {
//
//		if(CAN_Recongnition_1C02_Flg)
//    {
//		CAN_Msg_StepState = CAN_Msg_In_ParameterConfiguration;
//    CAN_MultiData_request_Flg = 0;
//		}
//
//		if(CAN_MultiData_request_Flg  == 1)
//		{
//		CAN_MultiData_request_Flg = 0;
//		CAN_MultiData_Answer[0] = 0x11;
//		CAN_MultiData_Answer[1] = 0x07;
//		CAN_MultiData_Answer[2] = 0x01;
//		CAN_MultiData_Answer[3] = 0xFF;
//		CAN_MultiData_Answer[4] = 0xFF;
//		CAN_MultiData_Answer[5] = 0x00;
//		CAN_MultiData_Answer[6] = 0x02;
//		CAN_MultiData_Answer[7] = 0x00;
//		Can_Send_BMS_Msg(CAN_MultiData_Answer, 8,0x1CECF456);
//		}
//		else if(CAN_MultiData_RX_OK_Flg)
//		{
//		CAN_MultiData_RX_OK_Flg = 0;
//		CAN_MultiData_Answer[0] = 0x13;
//		CAN_MultiData_Answer[1] = 0x31;
//		CAN_MultiData_Answer[2] = 0x00;
//		CAN_MultiData_Answer[3] = 0x07;
//		CAN_MultiData_Answer[4] = 0xFF;
//		CAN_MultiData_Answer[5] = 0x00;
//		CAN_MultiData_Answer[6] = 0x02;
//		CAN_MultiData_Answer[7] = 0x00;
//		Can_Send_BMS_Msg(CAN_MultiData_Answer, 8,0x1CECF456);
//		}
//		else
//		{
//		 if(CAN_Recongnition_1C02_Flg)
//		  {
//		   CAN_Recongnition_TX1801_Data[0] = 0xAA;
//		  }
//		 else
//		  {
//		  CAN_Recongnition_TX1801_Data[0] = 0;
//		  }
//		  CAN_Recongnition_TX1801_Data[1] = 0;
//	    CAN_Recongnition_TX1801_Data[2] = 0;
//		  CAN_Recongnition_TX1801_Data[3] = 0;
//		  CAN_Recongnition_TX1801_Data[4] = 0;
//	  	CAN_Recongnition_TX1801_Data[5] = 0xFF;
//	  	CAN_Recongnition_TX1801_Data[6] = 0xFF;
//	    CAN_Recongnition_TX1801_Data[7] = 0xFF;
//		  Can_Send_BMS_Msg(CAN_Recongnition_TX1801_Data, 8,0x1801F456);
//  	}
//	}
//
//if(CAN_Msg_StepState == CAN_Msg_In_ParameterConfiguration)
// {
//if(CAN_Recongnition_1C06_Flg == 0)
//	{
//  if(CAN_MultiData_request_Flg  == 1)
//		{
//		CAN_MultiData_request_Flg = 0;
//		CAN_MultiData_Answer[0] = 0x11;
//		CAN_MultiData_Answer[1] = 0x02;
//		CAN_MultiData_Answer[2] = 0x01;
//		CAN_MultiData_Answer[3] = 0xFF;
//		CAN_MultiData_Answer[4] = 0xFF;
//		CAN_MultiData_Answer[5] = 0x00;
//		CAN_MultiData_Answer[6] = 0x06;
//		CAN_MultiData_Answer[7] = 0x00;
//		Can_Send_BMS_Msg(CAN_MultiData_Answer, 8,0x1CECF456);
//		}
//  else if(CAN_MultiData_RX_OK_Flg)
//		{
//		CAN_MultiData_RX_OK_Flg = 0;
//		CAN_MultiData_Answer[0] = 0x13;
//		CAN_MultiData_Answer[1] = 0x0d;
//		CAN_MultiData_Answer[2] = 0x00;
//		CAN_MultiData_Answer[3] = 0x02;
//		CAN_MultiData_Answer[4] = 0xFF;
//		CAN_MultiData_Answer[5] = 0x00;
//		CAN_MultiData_Answer[6] = 0x06;
//		CAN_MultiData_Answer[7] = 0x00;
//		Can_Send_BMS_Msg(CAN_MultiData_Answer, 8,0x1CECF456);
//		}
//	else
//	 {
//  	if(CAN_Recongnition_1C02_Flg)
//		  {
//		   CAN_Recongnition_TX1801_Data[0] = 0xAA;
//		  }
//		 else
//		  {
//		  CAN_Recongnition_TX1801_Data[0] = 0;
//		  }
//		  CAN_Recongnition_TX1801_Data[1] = 0;
//	    CAN_Recongnition_TX1801_Data[2] = 0;
//		  CAN_Recongnition_TX1801_Data[3] = 0;
//		  CAN_Recongnition_TX1801_Data[4] = 0;
//	  	CAN_Recongnition_TX1801_Data[5] = 0xFF;
//	  	CAN_Recongnition_TX1801_Data[6] = 0xFF;
//	    CAN_Recongnition_TX1801_Data[7] = 0xFF;
//		  Can_Send_BMS_Msg(CAN_Recongnition_TX1801_Data, 8,0x1801F456);
//	 }
// }
//	else
//	  {
//		 if((CAN_ParameterConfiguration_1009_Flg)&&(CAN_ParameterConfiguration_1009_Data[0] == 0xAA))
//		   {
//		   if(ucBat_Status)
//		    {
//		    CAN_ParameterConfiguration_TX100A_Data[0] = 0xAA;
//		    }
//		   else
//		    {
//		     CAN_ParameterConfiguration_TX100A_Data[0] = 0x00;
//		    }
//				Can_Send_BMS_Msg(CAN_ParameterConfiguration_TX100A_Data, 1 , 0x100AF456);
//       }
//		else
//		  {

//			CAN_ParameterConfiguration_TX1807_Data[0] = Hex_To_BCD(LCD_Seconds);
//		  CAN_ParameterConfiguration_TX1807_Data[1] = Hex_To_BCD(LCD_Minute);
//	    CAN_ParameterConfiguration_TX1807_Data[2] = Hex_To_BCD(LCD_Hour);
//		  CAN_ParameterConfiguration_TX1807_Data[3] = Hex_To_BCD(LCD_Date);
//		  CAN_ParameterConfiguration_TX1807_Data[4] = Hex_To_BCD(LCD_Month);
//		  CAN_ParameterConfiguration_TX1807_Data[5] = Hex_To_BCD(LCD_Year);
//		  CAN_ParameterConfiguration_TX1807_Data[6] = Hex_To_BCD(0x20);
//	    CAN_ParameterConfiguration_TX1807_Data[7] = 0xFF;
//		  Can_Send_BMS_Msg(CAN_ParameterConfiguration_TX1807_Data, 8,0x1807F456); 	 //ʱ��ͬ������
//
//		  CAN_ParameterConfiguration_TX1808_Data[0] = 0x2C;
//		  CAN_ParameterConfiguration_TX1808_Data[1] = 0x01;
//	    CAN_ParameterConfiguration_TX1808_Data[2] = 0xB4;
//		  CAN_ParameterConfiguration_TX1808_Data[3] = 0x00;
//		  CAN_ParameterConfiguration_TX1808_Data[4] = 0xAC;
//		  CAN_ParameterConfiguration_TX1808_Data[5] = 0x0D;
//		  CAN_ParameterConfiguration_TX1808_Data[6] = 0xA0;
//	    CAN_ParameterConfiguration_TX1808_Data[7] = 0x0F;
//		  Can_Send_BMS_Msg(CAN_ParameterConfiguration_TX1808_Data, 8,0x1808F456);
//		   }
//    if(CAN_Charge_EN_1810_Flg)
//		  {
//			CAN_MultiData_request_Flg = 0;
//		  CAN_Msg_StepState = CAN_Msg_In_Charge;
//			CAN_Recongnition_1C06_Flg = 0;
//		  }
//	  }
// }

//if(CAN_Msg_StepState == CAN_Msg_In_ChargeEnd)
//  {
//		uc_CAN101A_Tx_Flg = 0;
//		CAN_ChargeEnd_TX181D_Data[0] = OBC_Charge_Time;
//		CAN_ChargeEnd_TX181D_Data[1] = OBC_Charge_Time >> 8;
//	  CAN_ChargeEnd_TX181D_Data[2] = (unsigned int)(OBC_Energy_Cal /100);
//		CAN_ChargeEnd_TX181D_Data[3] = (unsigned int)(OBC_Energy_Cal /100) >> 8;
//		CAN_ChargeEnd_TX181D_Data[4] = 0;
//		CAN_ChargeEnd_TX181D_Data[5] = 0;
//		CAN_ChargeEnd_TX181D_Data[6] = 0;
//	  CAN_ChargeEnd_TX181D_Data[7] = 0;
//		Can_Send_BMS_Msg(CAN_ChargeEnd_TX181D_Data, 8,0x181Df456);
//	}
//
//if((CAN_RX_OverTime_Msg1.all&0x01) ||  (CAN_RX_OverTime_Msg2.all&0x0f) ||  (CAN_RX_OverTime_Msg3.all&0x3F) ||(CAN_RX_OverTime_Msg4.all&0x01))
//	{
//	CAN_ChargeEnd_TX081F_Data[0] = CAN_RX_OverTime_Msg1.all;
//	CAN_ChargeEnd_TX081F_Data[1] = CAN_RX_OverTime_Msg2.all;
//	CAN_ChargeEnd_TX081F_Data[2] = CAN_RX_OverTime_Msg3.all;
//	CAN_ChargeEnd_TX081F_Data[3] = CAN_RX_OverTime_Msg4.all;
//	Can_Send_BMS_Msg(CAN_ChargeEnd_TX081F_Data, 4,0x081FF456);
//	}
//
//
//}
//else if (ucTag50ms)
//{
//ucTag50ms = 0;
//if(CAN_Msg_StepState == CAN_Msg_In_Charge)
//  {
//	if(CAN_MultiData_request_Flg  == 1)
//		{
//		CAN_MultiData_request_Flg = 0;
//		CAN_MultiData_Answer[0] = 0x11;
//		CAN_MultiData_Answer[1] = 0x02;
//		CAN_MultiData_Answer[2] = 0x01;
//		CAN_MultiData_Answer[3] = 0xFF;
//		CAN_MultiData_Answer[4] = 0xFF;
//		CAN_MultiData_Answer[5] = 0x00;
//		CAN_MultiData_Answer[6] = 0x11;
//		CAN_MultiData_Answer[7] = 0x00;
//		Can_Send_BMS_Msg(CAN_MultiData_Answer, 8,0x1CECF456);
//		}
//  else if(CAN_MultiData_RX_OK_Flg)
//		{
//		CAN_MultiData_RX_OK_Flg = 0;
//		CAN_MultiData_Answer[0] = 0x13;
//		CAN_MultiData_Answer[1] = 0x09;
//		CAN_MultiData_Answer[2] = 0x00;
//		CAN_MultiData_Answer[3] = 0x02;
//		CAN_MultiData_Answer[4] = 0xFF;
//		CAN_MultiData_Answer[5] = 0x00;
//		CAN_MultiData_Answer[6] = 0x11;
//		CAN_MultiData_Answer[7] = 0x00;
//		Can_Send_BMS_Msg(CAN_MultiData_Answer, 8,0x1CECF456);
//		}
//		else
//		{
//	  uiI_Output = 4000 -uiI_Output;
//	  CAN_Charge_TX1812_Data[0] = uiV_Output;
//		CAN_Charge_TX1812_Data[1] = uiV_Output >> 8;
//	  CAN_Charge_TX1812_Data[2] = uiI_Output;
//		CAN_Charge_TX1812_Data[3] = uiI_Output >> 8;
//		CAN_Charge_TX1812_Data[4] = OBC_Charge_Time;
//		CAN_Charge_TX1812_Data[5] = OBC_Charge_Time >> 8;
//		CAN_Charge_TX1812_Data[6] = OBC_Charge_Status;
//	  CAN_Charge_TX1812_Data[7] = 0x00;
//		Can_Send_BMS_Msg(CAN_Charge_TX1812_Data, 8,0x1812F456);
//		}
//
//	}
//
//}
//else
//{
//if(0)   //  Flg.all &0x0FE7  δ����ع�����BMS���Ľ��չ��ϴ˴���������  ����Э��BMS���չ������⴦�� ��ع��ϴ˴������� ��Э�������з�����
//	{
//		if(Flg.bit.Input_Err)
//		{
//  	OBC_Err_type_Msg1.Bits.Bit7_8 = 0x01;
//		}
//		if(Flg.bit.OTP_Err)
//		{
//		OBC_Err_type_Msg1.Bits.Bit5_6 = 0x01;
//		}
//		if((Flg.bit.I_Output_OCP_Err) || (Flg.bit.Hardware_Err)|| (Flg.bit.CAN_LLC_Err)|| (Flg.bit.Usart_LLC_Err)||(Flg.bit.Usart_PFC_Err)||(Flg.bit.V_Bus_390VErr))
//		{
//		OBC_Err_type_Msg2.Bits.Bit3_4 = 0x01;
//		}
//		if((Flg.bit.V_Output_OVP_Err)||(Flg.bit.V_Output_UVP_Err))
//		{
//		OBC_Err_type_Msg3.Bits.Bit3_4 = 0x01;
//		}

//	  CAN_Charge_TX101A_Data[0] = 0x10;
//		CAN_Charge_TX101A_Data[1] = OBC_Err_type_Msg1.all;
//	  CAN_Charge_TX101A_Data[2] = OBC_Err_type_Msg2.all;
//		CAN_Charge_TX101A_Data[3] = OBC_Err_type_Msg3.all;
//		Can_Send_BMS_Msg(CAN_Charge_TX101A_Data, 4,0x101AF456);
//    uc_CAN101A_Tx_Flg = 1;
//	}
//else if(CAN_Charge_EN_1019_Flg)
//		{
//		CAN_Charge_TX101A_Data[0] = OBC_Charge_Stop_Reason_BMS_Stop;
//		CAN_Charge_TX101A_Data[1] = 0;
//	  CAN_Charge_TX101A_Data[2] = 0;
//		CAN_Charge_TX101A_Data[3] = 0;
//		Can_Send_BMS_Msg(CAN_Charge_TX101A_Data, 4,0x101AF456);
//	  uc_CAN101A_Tx_Flg = 1;
//		}
//	  if(CAN_ChargeEnd_181C_Flg)
//	    {
//	    CAN_Msg_StepState = CAN_Msg_In_ChargeEnd;
//      CAN_Recongnition_1C06_Flg = 0;
//	    }
//
//}
/////////////////////////////�����ڸ��׶η�����Ϣ����Ϣ����//////////////////////
//}



























