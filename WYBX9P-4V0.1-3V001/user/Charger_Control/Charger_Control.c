
#include "Charger_Control.h"
#include "System_Init.h"
#include "usart.h"
#include "stm32f10x_usart.h"
#include "CAN.h"
#include "PCF8563.h"

///////////////其他变量定义////////////////
unsigned char ucBat_Status = 0;



///////////////LED指示灯模块变量定义///////////
unsigned int uiShowLedCnt=0;
unsigned char ucLedInitalStatusStep = 0;



///////////////盲充模块变量定义///////////

#define  SampleChangeVoltageTime        10 //等待一个稳定时间1000ms
volatile struct strSysTemInfo g_strSysInfo = {0};//系统一些状态结构体的定义

unsigned int uiChargeVoltageSet = 0;
unsigned int uiChargeCurrentSet = 0;
unsigned char ucChangeStepCnt = 0;

///////////////输出参数设置变量定义///////////
float f_I_OutputCoef = 1;		         //输出降功率参数
unsigned int V_Set = 0;              //BMS设置值转换为DSP输出设置值
unsigned int I_Set = 0;              //BMS设置值转换为DSP输出设置值
unsigned int V_BMS_Set = 0;  //BSM设置输出电压
unsigned int I_BMS_Set = 0;  //BMS设置输出电流


/////////////CAN通讯数组定义/////////////////
uint8_t CAN_LLC_RX_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_LLC_RX_Data2[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_PFC_RX_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_BMS_RX_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_BMS_TX_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_TX_LLC_Data[8] = {0,0,0,0,0,0,0,0};


//////////////////////充电机故障检测模块变量/////////////

unsigned char ucOTPCnt = 0;          //过温保护计数
unsigned char ucOTPGrade1Cnt = 0;    //高温降功率阶段判断计数  降功率分3档
unsigned char ucOTPGrade2Cnt = 0;
unsigned char ucOTPRecoverCnt = 0;

unsigned char ucOTP_Flg = 0;          //过温保护标志位
unsigned char uc_Output_En_Flg = 0;   //输出状态标志位


unsigned char uiV_Output_OVP_Err_Cnt = 0;   //输出过压故障计数
unsigned char uiV_Output_UVP_Err_Cnt = 0;   //输出欠压故障计数
unsigned char uiV_Output_UVP_Delay_Cnt = 0; //输出欠压故障检测延时计数
unsigned char uiI_Output_Err_Cnt = 0;        //输出过流故障计数

unsigned char ucCAN_LLC_Rx_ErrCnt = 0;     //与LLC模块通信超时计数
unsigned char ucCAN_BMS_Rx_ErrCnt = 0;     //与BMS模块通信超时计数

unsigned int uiLCD_Display_V_Set = 0;      //LCD电池界面显示设置电压参数   （包含手动设置参数和BMS设置参数）
unsigned int uiLCD_Display_I_Set = 0;      //LCD电池界面显示设置电压参数
////////////故障标志位联合体定义////////
union BITS Flg;


///////////////////输出参数计算联合体///////////////
union
{
    char V_OutPut_Coef[3];
    float f_V_OutPut_Coef;
} V_D_Coef;                         //输出电压设置系数
union
{
    char V_OutPut_Offset[3];
    float f_V_OutPut_Offset;
} V_D_Offset;                       //输出电压设置偏移量
union
{
    char V_ADC_Coef[3];
    float f_V_ADC_Coef;
} V_A_Coef;                         //输出电压采样系数
union
{
    char V_ADC_Offset[3];
    float f_V_ADC_Offset;
} V_A_Offset;                       //输出电压采样偏移量    (电流输出若线性度不好 考虑分段标定  预留三组系数 暂时使用一组)
union
{
    char I_OutPut_Coef[3];
    float f_I_OutPut_Coef;
} I_D_Coef;                         //输出电流设置系数
union
{
    char I_OutPut_Offset[3];
    float f_I_OutPut_Offset;
} I_D_Offset;                       //输出电流设置偏移量
union
{
    char I_OutPut_Coef1[3];
    float f_I_OutPut_Coef1;
} I_D_Coef1;                        //输出电流设置系数2
union
{
    char I_OutPut_Offset1[3];
    float f_I_OutPut_Offset1;
} I_D_Offset1;                     //输出电流设置偏移量2
union
{
    char I_OutPut_Coef2[3];
    float f_I_OutPut_Coef2;
} I_D_Coef2;                       //输出电流设置系数3
union
{
    char I_OutPut_Offset2[3];
    float f_I_OutPut_Offset2;
} I_D_Offset2;                     //输出电流设置偏移量3
union
{
    char I_ADC_Coef[3];
    float f_I_ADC_Coef;
} I_A_Coef;                        //输出电流采样系数
union
{
    char I_ADC_Offset[3];
    float f_I_ADC_Offset;
} I_A_Offset;                       //输出电流采样偏移量

///////////////////输出参数计算联合体///////////////


//////////////////杭叉国标非车载通信协议参数/////////////////

//传输协议功能   ID 0x1CEC56F4  BMS请求多包传送  0x1CECf456  OBC回复命令     0x1CEB56F4  BMS发送信息

unsigned char CAN_Msg_StepState = 0;                   //CAN通讯阶段信息
unsigned char CAN_HandShake_1827_Flg = 0;              //接收到BMS握手报文标志  0x182756F4
unsigned char CAN_Recongnition_1C02_Flg = 0;           //接收到BMS辨识报文标志  0x1C0256F4
unsigned char CAN_Recongnition_1C06_Flg = 0;           //接收到BMS蓄电池参数报文标志  0x1C0256F4    (传输协议功能  多包传送)
unsigned char CAN_ParameterConfiguration_1009_Flg = 0; //接收到BMS准备就绪报文标志  0x100956F4
unsigned char CAN_Charge_EN_1810_Flg = 0;              //接收到BMS输出请求参数设置报文标志  0x181056F4
unsigned char CAN_Charge_EN_1813_Flg = 0;              //接收到BMS电池状态报文标志  0x181356F4
unsigned char CAN_Charge_EN_1019_Flg = 0;              //接收到BMS终止充电报文标志  0x101956F4
unsigned char CAN_ChargeEnd_181C_Flg = 0;              //接收到BMS充电过程统计报文标志  0x181C56F4
unsigned char CAN_MultiData_request_Flg = 0;           //多包传送请求标志
unsigned char CAN_MultiData_RX_OK_Flg = 0;             //多包传送接收正常标志

unsigned char uc_CAN101A_Tx_Flg = 0;                   //OBC故障主动终止发送报文标志

unsigned char OBC_Charge_Status = 0;                   //充电机状态信息   BMS需反馈是否启动
unsigned int OBC_Charge_Time = 0;                      //充电机反馈充电时间参数

float OBC_Energy_Cal = 0;        //OBC输出能量统计  采用间隔1分钟取两组参数求平均的方式
float f_V_Output1;               //电压参数1
float f_V_Output2;               //电压参数2
float f_I_Output1;               //电流参数1
float f_I_Output2;               //电流参数2
unsigned int uiTimer60S_Cnt = 0; //60s计数

unsigned int uiBMS_Set_Max_V_Output = 0;               //电池最高电压参数

unsigned char CAN_HandShake_1827_Data[2] = {0};                //OBC接收握手报文数据
unsigned char CAN_Recongnition_1C02_Data[56] = {0};            //OBC接收辨识报文数据
unsigned char CAN_Recongnition_1C06_Data[16] = {0};            //OBC接收蓄电池参数报文数据
unsigned char CAN_ParameterConfiguration_1009_Data[1] = {0};   //OBC接电池准备状态报文数据
unsigned char CAN_Charge_1810_Data[5] = {0};                   //OBC接收BMS充电输出设置数据
unsigned char CAN_Charge_1C11_Data[16] = {0};                  //OBC接收BMS充电总状态数据
unsigned char CAN_Charge_1813_Data[7] = {0};                   //OBC接收BMS蓄电池状态数据
unsigned char CAN_Charge_1019_Data[4] = {0};                   //OBC接收BMS终止充电数据
unsigned char CAN_ChargeEnd_181C_Data[7] = {0};                //OBC接收BMS统计参数数据
unsigned char CAN_ChargeEnd_081E_Data[4] = {0};                //OBC接收BMS错误（通信故障）数据


unsigned char CAN_Rx_MultiData[8] = {0};                       //OBC接收多包传送数据
unsigned char CAN_MultiData_request[8] = {0};                  //OBC接收多包传送请求数据
unsigned char CAN_MultiData_Answer[8] = {0};                   //OBC反馈允许多包传送请求数据


////////////////////多包协议传输阶段 不同的多包传送使能标志///////////
unsigned char CAN_Recongnition_1C02_MultiData_request_Flg = 0;
unsigned char CAN_Recongnition_1C06_MultiData_request_Flg = 0;
unsigned char CAN_Charge_1C11_MultiData_request_Flg = 0;

unsigned char CAN_HandShake_Tx1826_Data[3] = {0};              //OBC发送握手报文数据
unsigned char CAN_Recongnition_TX1801_Data[8] = {0};           //OBC发送辨识报文数据
unsigned char CAN_ParameterConfiguration_TX1807_Data[8] = {0}; //时间同步信息数据
unsigned char CAN_ParameterConfiguration_TX1808_Data[8] = {0}; //OBC发送最大输出能力报文数据
unsigned char CAN_ParameterConfiguration_TX100A_Data[1] = {0}; //OBC发送准备状态报文数据
unsigned char CAN_Charge_TX1812_Data[8] = {0};                 //OBC发送充电状态报文数据
unsigned char CAN_Charge_TX101A_Data[4] = {0};                 //OBC发送终止充电报文数据
unsigned char CAN_ChargeEnd_TX181D_Data[8] = {0};              //OBC发送统计参数报文数据
unsigned char CAN_ChargeEnd_TX081F_Data[4] = {0};              //OBC发送错误（通信错误）报文数据

unsigned int BMS_Set_I_Output = 0;                            //BMS请求输出电流参数

unsigned char OBC_Charge_Stop_Reason_BMS_Stop = 0;            //OBC故障主动停止原因参数

union  OBC_Err_type     OBC_Err_type_Msg1;                    //OBC故障反馈参数联合体
union  OBC_Err_type     OBC_Err_type_Msg2;
union  OBC_Err_type     OBC_Err_type_Msg3;

union  CAN_RX_OverTime  CAN_RX_OverTime_Msg1;                 //OBC通信超时故障联合体
union  CAN_RX_OverTime  CAN_RX_OverTime_Msg2;
union  CAN_RX_OverTime  CAN_RX_OverTime_Msg3;
union  CAN_RX_OverTime  CAN_RX_OverTime_Msg4;

unsigned int ui_CAN1C02_Rx_Err_Cnt = 0;                      //OBC接收不同帧ID超时计数
unsigned int ui_CAN1C06_Rx_Err_Cnt = 0;
unsigned int ui_CAN1009_Rx_Err_Cnt = 0;
unsigned int ui_CAN1810_Rx_Err_Cnt = 0;
unsigned int ui_CAN1C11_Rx_Err_Cnt = 0;
unsigned int ui_CAN1019_Rx_Err_Cnt = 0;
unsigned int ui_CAN181C_Rx_Err_Cnt = 0;



//////////////////杭叉国标非车载通信协议参数/////////////////


//////////////////盲充充电过程管理函数///////////////////////////
void ChargeProcess(void)
{
    if (g_strSysInfo.ucSystemChargeStepState != SystemInChargeFinish)//如果充电没有完成 一直在充电过程中循环
    {
        switch(g_strSysInfo.ucSystemChargeStepState)
        {
        case SystemInPreChargeStep1 :
        {
            while(SystemInPreChargeStep1 == g_strSysInfo.ucSystemChargeStepState) //如果是预充电调整电压电流
            {
                //如果是预充电  要实时监控电压是否在变化，电流有自我调节
                ChargeStepManage(g_strSysInfo.strPreChangeStep1.uiChargeChangeVoltage,
                                 g_strSysInfo.strPreChangeStep1.uiChargeChangeCurrent, g_strSysInfo.strPreChangeStep1.uiChargeConstantVoltage,
                                 g_strSysInfo.strPreChangeStep1.uiChargeConstantCurrent, g_strSysInfo.strPreChangeStep1.uiChargeLimitTime);
            }
            break;
        }
        case SystemInPreChargeStep2 :
        {
            while(SystemInPreChargeStep2 == g_strSysInfo.ucSystemChargeStepState) //如果是预充电调整电压电流
            {
                //如果是预充电  要实时监控电压是否在变化，电流有自我调节
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
        case SystemInChargeFinish://充电完成
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
    unsigned char ucChargeModeJudge = 0;//充电模式判断变量

    if((0 == uiChangeVoltage) && (0 == uiChangeCurrent) && (0 == uiConstantVoltage) && (0 == uiConstantCurrent) && (0 == uiChargeLimitTime))
    {
        g_strSysInfo.ucSystemChargeStepState += 1;
        return;//如果该段的充电参数为0说明不要此段的充电曲线，转入到下一个状态。并且不作充电处理
    }
    if(uiChangeVoltage && uiConstantCurrent)ucChargeModeJudge = ChargeModeInConstantCurrent;
    else if(uiChangeCurrent && uiConstantVoltage)ucChargeModeJudge = ChargeModeInConstantVoltage;
    else return;//非在恒压，恒流模式。是错误的模式

    if(ChargeModeInConstantCurrent == ucChargeModeJudge)//如果是在恒流模式
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
    else if(ChargeModeInConstantVoltage == ucChargeModeJudge)//如果是在恒压模式
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
    V_D_Coef.f_V_OutPut_Coef = 21.9;                    //输出电压电流系数及偏移量设置
    V_D_Offset.f_V_OutPut_Offset = 175;
    I_D_Coef.f_I_OutPut_Coef = 17.18;
    I_D_Offset.f_I_OutPut_Offset = -343;
    V_Set = Voltage;
    I_Set = Currennt;
    uiLCD_Display_V_Set = Voltage;
    uiLCD_Display_I_Set = Currennt;
    if(V_Set > 1100)     //BMS设置输出电压电流限幅
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

    I_Set = I_Set *f_I_OutputCoef;   //降功率设置   f_I_OutputCoef为降功率系数

    f_OutputTemp = V_Set;
    f_OutputTemp = f_OutputTemp * V_D_Coef.f_V_OutPut_Coef + V_D_Offset.f_V_OutPut_Offset;  //设置电压值转换为LLC单片机环路工作的参数
    V_Set = f_OutputTemp;
    f_OutputTemp = I_Set;
    f_OutputTemp = f_OutputTemp * I_D_Coef.f_I_OutPut_Coef + I_D_Offset.f_I_OutPut_Offset;

    I_Set = f_OutputTemp;

    CAN_TX_LLC_Data[0] = V_Set >> 8;     //CAN通讯发送电压电流设置的数据
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
        if((!(Flg.all&0x0FFF))&&uiV_Output>420&&CAN_BMS_Rx_Flg&&(CC1_Control == 0)&&(CAN_BMS_RX_Data[4] == 0x00))    //充电机故障状态及检测保护  发送停止输出命令
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
    Can_Send_LLC_Msg(CAN_TX_LLC_Data);      //CAN通讯发送数据
}


/*******************************************************************************
 函数块名称:  void OBC_Status_Manage(void)
 功能:        温度保护、输入过欠压、输出过压、过流、欠压、通信故障保护
 输入参数:    无
 输出参数:    无
 其他说明:
*******************************************************************************/
void OBC_Status_Manage(void)
{
/////////////////////////////温度保护//////////////////////////////////////
// LLC模块高于85度，BUCK模块高于90度过温保护
// LLC模块低于70度，BUCK模块低于75度恢复满功率
//	LLC模块高于80度，BUCK模块高于85度降功率至90%
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

/////////////////////////////温度保护///////////////////////////////////////

/////////////////////////////输出状态保护//////////////////////////////////////

    if(uiI_Output > 1100)     //输出过压保护  连续检测5S
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

    if(uiV_Output > 1100)      //输出过流保护  连续检测5S
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

    if(uc_Output_En_Flg)    //欠压检测保护延时10S  输出后延时等待充电机输出后检测是否输出欠压
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
    if((uiV_Output < 420)&&(uiV_Output_UVP_Delay_Cnt >= 100))//欠压检测保护
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

    if(uiV_Output <= 420)         //检测到电池电压
    {
        ucBat_Status = 0;
        Flg.bit.Batarry_Err = 1;
    }
    else
    {
        ucBat_Status = 1;
        Flg.bit.Batarry_Err = 0;
    }
    /////////////////////////////输出状态保护//////////////////////////////////////

    /////////////////////////////通信超时保护//////////////////////////////////////
    ucCAN_LLC_Rx_ErrCnt++;             //与LLC模块CAN通讯超时检测  连续5S
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
    if(ucCAN_BMS_Rx_ErrCnt >= 50)     //与NMS模块CAN通讯超时检测  连续5S
    {
        ucCAN_BMS_Rx_ErrCnt = 50;
        Flg.bit.CAN_BMS_Err = 1;
    }
    else
    {
        Flg.bit.CAN_BMS_Err = 0;
    }
    /////////////////////////////通信超时保护//////////////////////////////////////



}

void ShowChargeStatusLed(void)
{
    uiShowLedCnt++;
    uiShowLedCnt %=100000;
    /*****************************上电初始状态  五个指示灯循环*******************/
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
    /*****************************上电初始状态  五个指示灯循环*******************/


    /*****************************正常工作状态                *******************/
    else
    {
        if(CAN_BMS_Rx_Flg)                          //与外界通信指示灯控制 接收到BMS通讯时 默认通信状态  使能通讯灯
        {
            C_EnOutStateLed1();
        }
        else
        {
            C_DisEnOutStateLed1();
        }

        if(CAN_BMS_RX_Data[4] == 0x01)  //接收到关机信号  点亮充足指示灯
        {
            C_EnOutStateLed2();
            C_DisEnOutStateLed3();
            C_DisEnOutStateLed4();
            C_DisEnOutStateLed5();
        }
        else
        {
            /*****************************故障状态                *******************/
            if(Flg.all & 0x0FF7)           //出现故障时 关闭恒压 工作指示灯 点亮故障灯
            {
                C_DisEnOutStateLed3();
                C_DisEnOutStateLed4();
                C_EnOutStateLed5();
            }
            /*****************************正常指示状态               *******************/
            else
            {
                C_DisEnOutStateLed5();
                if(uc_Output_En_Flg == 0&&uiI_Output > 30)    //输出使能且输出电流大于3A为工作状态
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

///////////////////////通用协议//////////////////////////////////
void CAN_Msg_Handle(void)
{
//反馈充电机状态到BMS
    CAN_BMS_TX_Data[0] = CAN_LLC_RX_Data[0];   //前两位为输出电压
    CAN_BMS_TX_Data[1] = CAN_LLC_RX_Data[1];
    CAN_BMS_TX_Data[2] = CAN_LLC_RX_Data[2];	 //三四位为输出电流
    CAN_BMS_TX_Data[3] = CAN_LLC_RX_Data[3];
    CAN_BMS_TX_Data[4] = (Flg.all&0xFF1F);     //第五六位为故障代码
    CAN_BMS_TX_Data[5] = (Flg.all&0xFFE0) >> 5;
    CAN_BMS_TX_Data[6] = CAN_PFC_RX_Data[7] ;  //预留位
    CAN_BMS_TX_Data[7] = CAN_PFC_RX_Data[6] ;

    Can_Send_BMS_Msg(CAN_BMS_TX_Data,8,0x18FF50E6);      //反馈充电机信息到BMS
}



////////////////////杭叉CAN协议////////////////////////////////

//void CAN_Msg_Handle(void)
//{

//	///////////////////////////充电机在各阶段接收BMS报文超时判断//////////////////////
////	if((CAN_Msg_StepState == CAN_Msg_In_Recongnition)&&(CAN_Recongnition_1C06_Flg == 0))       //辨识阶段  判断接收BMS车辆辨识报文超时 5s
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
////	if(CAN_Msg_StepState == CAN_Msg_In_ParameterConfiguration)                  //参数配置阶段  �
////	  {
////		if(CAN_Recongnition_1C06_Flg == 0)                                         //接收参数配置报文(5S)超时
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
////		  ui_CAN1009_Rx_Err_Cnt++;                                               //接收电池准备就绪（60S）报文超�
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
////	if(CAN_Msg_StepState == CAN_Msg_In_Charge)            //充电阶段
////	  {
////	    ui_CAN1810_Rx_Err_Cnt++;
////		  if(ui_CAN1810_Rx_Err_Cnt >= 100)                 //接收电池充电需求报文超时
////		    {
////		     CAN_RX_OverTime_Msg3.Bits.Bit3_4 = 0x01;
////		    }
////			else
////			  {
////			  CAN_RX_OverTime_Msg3.Bits.Bit3_4 = 0x00;
////			  }
////			 ui_CAN1C11_Rx_Err_Cnt++;
////		  if(ui_CAN1C11_Rx_Err_Cnt >= 500)                 //接收电池总状态报文超时
////		    {
////		     CAN_RX_OverTime_Msg3.Bits.Bit1_2 = 0x01;
////		    }
////			else
////			  {
////			  CAN_RX_OverTime_Msg3.Bits.Bit1_2 = 0x00;
////			  }
////
////		if(uc_CAN101A_Tx_Flg)                             //接收BMS充电终止保卫超时
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
////if(CAN_Msg_StepState == CAN_Msg_In_ChargeEnd)         //充电结束阶段  接收BMS统计报文超时
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
/////////////////////////////充电机在各阶段接收BMS报文超时判断//////////////////////
//
//
/////////////////////////////充电机在各阶段反馈信息及信息处理//////////////////////
//	uiV_Output= CAN_LLC_RX_Data[0] * 256 + CAN_LLC_RX_Data[1];
//	uiI_Output = CAN_LLC_RX_Data[2] * 256 + CAN_LLC_RX_Data[3];
//	OBC_Charge_Time = ui100msCnt /600;
/////////////////////////////充电机输出能量计算//////////////////////
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
/////////////////////////////充电机输出能量计算//////////////////////
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
//		  Can_Send_BMS_Msg(CAN_ParameterConfiguration_TX1807_Data, 8,0x1807F456); 	 //时间同步参数
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
//if(0)   //  Flg.all &0x0FE7  未检测电池故障与BMS报文接收故障此处不做处理  （此协议BMS接收故障另外处理 电池故障此处不处理 在协议中另有反馈）
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
/////////////////////////////充电机在各阶段反馈信息及信息处理//////////////////////
//}



























