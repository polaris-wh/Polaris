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

extern unsigned int uiPFC_Temp1_AsTrue;         //OBC内部温度换算为真是温度参数
extern unsigned int uiPFC_Temp2_AsTrue;
extern unsigned int uiPFC_Temp3_AsTrue;
extern unsigned int uiLLC_Temp1_AsTrue;
extern unsigned int uiLLC_Temp2_AsTrue;

extern unsigned int uiV_Output;  //充电机输出电压参数  100mv单位
extern unsigned int uiI_Output;  //充电机输出电电流参数 100mA单位

extern unsigned int V_BMS_Set;  //BSM设置输出电压
extern unsigned int I_BMS_Set;  //BMS设置输出电流

extern uint8_t CAN_LLC_RX_Data[8];
extern uint8_t CAN_LLC_RX_Data2[8];
extern uint8_t CAN_PFC_RX_Data[8];
extern uint8_t CAN_BMS_RX_Data[8];
extern uint8_t CAN_BMS_TX_Data[8];
extern uint8_t CAN_TX_LLC_Data[8];
extern unsigned long g_uiNtcTable_External[135];    //温度电阻值对应表

extern unsigned char LCD_Rx_Data[110];  //LCD串口接收数据数组
extern u8 LCD_Page_MSg_Flg;    //接收到LCD发送的页面信息标志
extern u8 LCD_Manual_Flg;       //LCD手动设置输出按钮信息信息标志
extern u16 LCD_Page_Msg_Data;   //LCD当前页面数据
extern u16 LCD_Manual_Data;   //LCD手动设置输出按钮信息
extern u16 LCD_Manual_V_OutputData;   //LCD手动设置电压输出参数信息
extern u16 LCD_Manual_I_OutputData;   //LCD手动设置电流输出参数信息
extern u16 LCD_Manual_ReturnData;   //LCD手动设置返回主页面参数
extern u16 LCD_Manual_CrcData;     //LCD进入手动设置校验参数

extern u16 LCD_Manual_SetData;     //LCD进入手动设置按钮返回参数
extern u8 LCD_Year;                //LCD时间参数
extern u8 LCD_Month;
extern u8 LCD_Date;
extern u8 LCD_Hour;
extern u8 LCD_Minute;
extern u8 LCD_Seconds;



extern u16 LCD_Set_Year;                    //LCD设置时间参数
extern u16 LCD_Set_Month;
extern u16 LCD_Set_Date;
extern u16 LCD_Set_Hour;
extern u16 LCD_Set_Minute;
extern u16 LCD_Set_Seconds;

extern unsigned char ucCAN_LLC_Rx_ErrFlg;
extern unsigned char ucCAN_LLC_Rx_ErrCnt;   //与LLC模块通信超时计数
extern unsigned char ucCAN_BMS_Rx_ErrFlg;
extern unsigned char ucCAN_BMS_Rx_ErrCnt;   //与BMS模块通信超时计数



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
    unsigned int uiChargeLimitTime;         //此阶段充许最大充电时间  1s  max 65535s = 18h
    unsigned int uiChargeChangeVoltage;     //在此阶段转换到下一个阶段电压值  10mV  如果此参数不为0 以恒压方式充电
    unsigned int uiChargeChangeCurrent;     //在此阶段转换到下一个阶段电流值 10mA  如果此参数不为0 以恒流方式充电
    unsigned int uiChargeConstantVoltage;   //在此阶段要求恒压值，这时侯转换点的电压值为0  10mV
    unsigned int uiChargeConstantCurrent;   //在此阶段要求恒流值，这时侯转换点的电流值为0  10mA
};//每个充电状态，阶段。电池的所有特性

struct strSysTemInfo
{
    unsigned char ucSystemState;                    //系统工作状态
    unsigned char ucSystemChargeStepState;          //系统充电状态步骤
    unsigned char ucSystemLedError;                 //系统指示灯错误指示状态
    unsigned char ucCmdStatus;                      //是否接收工作命令状态
    unsigned char ucOutputMode;                     //输出模式选择，1加热模式，0充电模式
    unsigned char ucChargeHardWareEnFlag;           //充电继电器，风扇是否开启标志
    unsigned char ucBattStatus;                     //电池状态
    unsigned char ucPFCStatus;                      //PFC状态
    u32 uiExternBatTemp;                            //外部电池温度采样
    u32 uiInternAirTemp;                            //内部外界温度采样
    u32 uiBattleVoltage;                            //电池电压采样
    u32 uiChargeVoltage;                            //充电电压采样  变压器输出电压
    u32 uiChargeCurrent;                            //充电电流采样
    u32 uiPowerIn;                                  //输入电压采样
    u32 uiSetChargeVolValue100mVUnit;               //程序设定充电电压赋值全局变量100mV
    u32 uiSetChargeCurValue100mAUnit;               //程序设定充电电流赋值全局变量100mA
    u32 uiMaxChargeVoltage100mVUnit;                //最大电充电电压
    u32 uiMinChargeVoltage100mVUnit;                //最小电充电电压
    u32 uiMaxChargeCurrent100mAUnit;                //最大充电电流

    struct strPerStepChargeInfo strPreChangeStep1;  //第一阶段
    struct strPerStepChargeInfo strPreChangeStep2;
    struct strPerStepChargeInfo strChangeStep2;     //阶段不一定是恒流还是恒压
    struct strPerStepChargeInfo strChangeStep3;
    struct strPerStepChargeInfo strChangeStep4;
    struct strPerStepChargeInfo strChangeStep5;
    struct strPerStepChargeInfo strChangeStep6;
};

#define SystemNoBatt                0x00//系统在无电池状态
#define SystemInPreChargeStep1      0x01//系统在预充电1状态
#define SystemInPreChargeStep2      0x02//系统在预充电2状态
#define SystemInChargeStep2         0x03//系统在充电状态2
#define SystemInChargeStep3         0x04//系统在充电状态3
#define SystemInChargeStep4         0x05//系统在充电状态4
#define SystemInChargeStep5         0x06//系统在充电状态5
#define SystemInChargeStep6         0x07//系统在充电状态6
#define SystemInChargeFinish        0x08//系统充电完成


#define ChargeModeInConstantVoltage   0x01//在恒压模式的定义
#define ChargeModeInConstantCurrent   0x02//在恒压模式的定义

//////////////////杭叉国标非车载通信协议参数/////////////////
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

////////////////////多包协议传输阶段 不同的多包传送使能标志///////////
extern unsigned char CAN_Recongnition_1C02_MultiData_request_Flg;
extern unsigned char CAN_Recongnition_1C06_MultiData_request_Flg;
extern unsigned char CAN_Charge_1C11_MultiData_request_Flg;


extern unsigned char CAN_Msg_StepState;                   //CAN通讯阶段信息
extern unsigned char CAN_HandShake_1827_Flg;              //接收到BMS握手报文标志  0x182756F4
extern unsigned char CAN_Recongnition_1C02_Flg;           //接收到BMS辨识报文标志  0x1C0256F4
extern unsigned char CAN_Recongnition_1C06_Flg;           //接收到BMS蓄电池参数报文标志  0x1C0256F4    (传输协议功能  多包传送)
extern unsigned char CAN_ParameterConfiguration_1009_Flg; //接收到BMS准备就绪报文标志  0x100956F4
extern unsigned char CAN_Charge_EN_1810_Flg;              //接收到BMS输出请求参数设置报文标志  0x181056F4
extern unsigned char CAN_Charge_EN_1813_Flg;              //接收到BMS电池状态报文标志  0x181356F4
extern unsigned char CAN_Charge_EN_1019_Flg;              //接收到BMS终止充电报文标志  0x101956F4
extern unsigned char CAN_ChargeEnd_181C_Flg;              //接收到BMS充电过程统计报文标志  0x181C56F4
extern unsigned char CAN_MultiData_request_Flg;           //多包传送请求标志
extern unsigned char CAN_MultiData_RX_OK_Flg;             //多包传送接收正常标志


extern unsigned char CAN_HandShake_1827_Data[2];                //OBC接收握手报文数据
extern unsigned char CAN_Recongnition_1C02_Data[56];            //OBC接收辨识报文数据
extern unsigned char CAN_Recongnition_1C06_Data[16];            //OBC接收蓄电池参数报文数据

extern unsigned char CAN_ParameterConfiguration_1009_Data[1];   //OBC接电池准备状态报文数据
extern unsigned char CAN_Charge_1810_Data[5];                   //OBC接收BMS充电输出设置数据
extern unsigned char CAN_Charge_1C11_Data[16];                  //OBC接收BMS充电总状态数据
extern unsigned char CAN_Charge_1813_Data[7];                   //OBC接收BMS蓄电池状态数据
extern unsigned char CAN_Charge_1019_Data[4];                   //OBC接收BMS终止充电数据
extern unsigned char CAN_ChargeEnd_181C_Data[7];                //OBC接收BMS统计参数数据
extern unsigned char CAN_ChargeEnd_081E_Data[4];                //OBC接收BMS错误（通信故障）数据

extern unsigned char CAN_ParameterConfiguration_TX1808_Data[8]; //OBC发送最大输出能力报文数据
extern unsigned char CAN_ParameterConfiguration_TX100A_Data[1]; //OBC发送准备状态报文数据
extern unsigned char CAN_Rx_MultiData[8];                       //OBC接收多包传送数据
extern unsigned char CAN_MultiData_request[8];                  //OBC接收多包传送请求数据



extern unsigned int ui_CAN1C02_Rx_Err_Cnt;     //OBC接收不同帧ID超时计数
extern unsigned int ui_CAN1C06_Rx_Err_Cnt;
extern unsigned int ui_CAN1009_Rx_Err_Cnt;
extern unsigned int ui_CAN1810_Rx_Err_Cnt;
extern unsigned int ui_CAN1C11_Rx_Err_Cnt;
extern unsigned int ui_CAN1019_Rx_Err_Cnt;
extern unsigned int ui_CAN181C_Rx_Err_Cnt;


extern unsigned char OBC_Charge_Status;                   //充电机状态信息   BMS需反馈是否启动
extern unsigned int OBC_Charge_Time;                      //充电机反馈充电时间参数
extern unsigned char uc_Output_En_Flg;   //输出状态标志位

extern float OBC_Energy_Cal;        //OBC输出能量统计  采用间隔1分钟取两组参数求平均的方式
extern float f_V_Output1;               //电压参数1
extern float f_V_Output2;               //电压参数2
extern float f_I_Output1;               //电流参数1
extern float f_I_Output2;               //电流参数2
extern unsigned int uiTimer60S_Cnt; //60s计数
//////////////////杭叉国标非车载通信协议参数/////////////////





#endif
