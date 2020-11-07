
#include "Charger_Control.h"
#include "System_Init.h"
#include "usart.h"
#include "stm32f10x_usart.h"
#include "CAN.h"
#include "PCF8563.h"

///////////////∆‰À˚±‰¡ø∂®“Â////////////////
unsigned char ucBat_Status = 0;



///////////////LED÷∏ æµ∆ƒ£øÈ±‰¡ø∂®“Â///////////
unsigned int uiShowLedCnt=0;
unsigned char ucLedInitalStatusStep = 0;



///////////////√§≥‰ƒ£øÈ±‰¡ø∂®“Â///////////

#define  SampleChangeVoltageTime        10 //µ»¥˝“ª∏ˆŒ»∂® ±º‰1000ms
volatile struct strSysTemInfo g_strSysInfo = {0};//œµÕ≥“ª–©◊¥Ã¨Ω·ππÃÂµƒ∂®“Â

unsigned int uiChargeVoltageSet = 0;
unsigned int uiChargeCurrentSet = 0;
unsigned char ucChangeStepCnt = 0;

/////////////// ‰≥ˆ≤Œ ˝…Ë÷√±‰¡ø∂®“Â///////////
float f_I_OutputCoef = 1;		         // ‰≥ˆΩµπ¶¬ ≤Œ ˝
unsigned int V_Set = 0;              //BMS…Ë÷√÷µ◊™ªªŒ™DSP ‰≥ˆ…Ë÷√÷µ
unsigned int I_Set = 0;              //BMS…Ë÷√÷µ◊™ªªŒ™DSP ‰≥ˆ…Ë÷√÷µ
unsigned int V_BMS_Set = 0;  //BSM…Ë÷√ ‰≥ˆµÁ—π
unsigned int I_BMS_Set = 0;  //BMS…Ë÷√ ‰≥ˆµÁ¡˜


/////////////CANÕ®—∂ ˝◊È∂®“Â/////////////////
uint8_t CAN_LLC_RX_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_LLC_RX_Data2[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_PFC_RX_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_BMS_RX_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_BMS_TX_Data[8] = {0,0,0,0,0,0,0,0};
uint8_t CAN_TX_LLC_Data[8] = {0,0,0,0,0,0,0,0};


//////////////////////≥‰µÁª˙π ’œºÏ≤‚ƒ£øÈ±‰¡ø/////////////

unsigned char ucOTPCnt = 0;          //π˝Œ¬±£ª§º∆ ˝
unsigned char ucOTPGrade1Cnt = 0;    //∏ﬂŒ¬Ωµπ¶¬ Ω◊∂Œ≈–∂œº∆ ˝  Ωµπ¶¬ ∑÷3µµ
unsigned char ucOTPGrade2Cnt = 0;
unsigned char ucOTPRecoverCnt = 0;

unsigned char ucOTP_Flg = 0;          //π˝Œ¬±£ª§±Í÷æŒª
unsigned char uc_Output_En_Flg = 0;   // ‰≥ˆ◊¥Ã¨±Í÷æŒª


unsigned char uiV_Output_OVP_Err_Cnt = 0;   // ‰≥ˆπ˝—ππ ’œº∆ ˝
unsigned char uiV_Output_UVP_Err_Cnt = 0;   // ‰≥ˆ«∑—ππ ’œº∆ ˝
unsigned char uiV_Output_UVP_Delay_Cnt = 0; // ‰≥ˆ«∑—ππ ’œºÏ≤‚—” ±º∆ ˝
unsigned char uiI_Output_Err_Cnt = 0;        // ‰≥ˆπ˝¡˜π ’œº∆ ˝

unsigned char ucCAN_LLC_Rx_ErrCnt = 0;     //”ÎLLCƒ£øÈÕ®–≈≥¨ ±º∆ ˝
unsigned char ucCAN_BMS_Rx_ErrCnt = 0;     //”ÎBMSƒ£øÈÕ®–≈≥¨ ±º∆ ˝

unsigned int uiLCD_Display_V_Set = 0;      //LCDµÁ≥ÿΩÁ√Êœ‘ æ…Ë÷√µÁ—π≤Œ ˝   £®∞¸∫¨ ÷∂Ø…Ë÷√≤Œ ˝∫ÕBMS…Ë÷√≤Œ ˝£©
unsigned int uiLCD_Display_I_Set = 0;      //LCDµÁ≥ÿΩÁ√Êœ‘ æ…Ë÷√µÁ—π≤Œ ˝
////////////π ’œ±Í÷æŒª¡™∫œÃÂ∂®“Â////////
union BITS Flg;


/////////////////// ‰≥ˆ≤Œ ˝º∆À„¡™∫œÃÂ///////////////
union
{
    char V_OutPut_Coef[3];
    float f_V_OutPut_Coef;
} V_D_Coef;                         // ‰≥ˆµÁ—π…Ë÷√œµ ˝
union
{
    char V_OutPut_Offset[3];
    float f_V_OutPut_Offset;
} V_D_Offset;                       // ‰≥ˆµÁ—π…Ë÷√∆´“∆¡ø
union
{
    char V_ADC_Coef[3];
    float f_V_ADC_Coef;
} V_A_Coef;                         // ‰≥ˆµÁ—π≤…—˘œµ ˝
union
{
    char V_ADC_Offset[3];
    float f_V_ADC_Offset;
} V_A_Offset;                       // ‰≥ˆµÁ—π≤…—˘∆´“∆¡ø    (µÁ¡˜ ‰≥ˆ»Ùœﬂ–‘∂»≤ª∫√ øº¬«∑÷∂Œ±Í∂®  ‘§¡Ù»˝◊Èœµ ˝ ‘› ± π”√“ª◊È)
union
{
    char I_OutPut_Coef[3];
    float f_I_OutPut_Coef;
} I_D_Coef;                         // ‰≥ˆµÁ¡˜…Ë÷√œµ ˝
union
{
    char I_OutPut_Offset[3];
    float f_I_OutPut_Offset;
} I_D_Offset;                       // ‰≥ˆµÁ¡˜…Ë÷√∆´“∆¡ø
union
{
    char I_OutPut_Coef1[3];
    float f_I_OutPut_Coef1;
} I_D_Coef1;                        // ‰≥ˆµÁ¡˜…Ë÷√œµ ˝2
union
{
    char I_OutPut_Offset1[3];
    float f_I_OutPut_Offset1;
} I_D_Offset1;                     // ‰≥ˆµÁ¡˜…Ë÷√∆´“∆¡ø2
union
{
    char I_OutPut_Coef2[3];
    float f_I_OutPut_Coef2;
} I_D_Coef2;                       // ‰≥ˆµÁ¡˜…Ë÷√œµ ˝3
union
{
    char I_OutPut_Offset2[3];
    float f_I_OutPut_Offset2;
} I_D_Offset2;                     // ‰≥ˆµÁ¡˜…Ë÷√∆´“∆¡ø3
union
{
    char I_ADC_Coef[3];
    float f_I_ADC_Coef;
} I_A_Coef;                        // ‰≥ˆµÁ¡˜≤…—˘œµ ˝
union
{
    char I_ADC_Offset[3];
    float f_I_ADC_Offset;
} I_A_Offset;                       // ‰≥ˆµÁ¡˜≤…—˘∆´“∆¡ø

/////////////////// ‰≥ˆ≤Œ ˝º∆À„¡™∫œÃÂ///////////////


//////////////////∫º≤Êπ˙±Í∑«≥µ‘ÿÕ®–≈–≠“È≤Œ ˝/////////////////

//¥´ ‰–≠“Èπ¶ƒ‹   ID 0x1CEC56F4  BMS«Î«Û∂‡∞¸¥´ÀÕ  0x1CECf456  OBCªÿ∏¥√¸¡Ó     0x1CEB56F4  BMS∑¢ÀÕ–≈œ¢

unsigned char CAN_Msg_StepState = 0;                   //CANÕ®—∂Ω◊∂Œ–≈œ¢
unsigned char CAN_HandShake_1827_Flg = 0;              //Ω” ’µΩBMSŒ’ ÷±®Œƒ±Í÷æ  0x182756F4
unsigned char CAN_Recongnition_1C02_Flg = 0;           //Ω” ’µΩBMS±Ê ∂±®Œƒ±Í÷æ  0x1C0256F4
unsigned char CAN_Recongnition_1C06_Flg = 0;           //Ω” ’µΩBMS–ÓµÁ≥ÿ≤Œ ˝±®Œƒ±Í÷æ  0x1C0256F4    (¥´ ‰–≠“Èπ¶ƒ‹  ∂‡∞¸¥´ÀÕ)
unsigned char CAN_ParameterConfiguration_1009_Flg = 0; //Ω” ’µΩBMS◊º±∏æÕ–˜±®Œƒ±Í÷æ  0x100956F4
unsigned char CAN_Charge_EN_1810_Flg = 0;              //Ω” ’µΩBMS ‰≥ˆ«Î«Û≤Œ ˝…Ë÷√±®Œƒ±Í÷æ  0x181056F4
unsigned char CAN_Charge_EN_1813_Flg = 0;              //Ω” ’µΩBMSµÁ≥ÿ◊¥Ã¨±®Œƒ±Í÷æ  0x181356F4
unsigned char CAN_Charge_EN_1019_Flg = 0;              //Ω” ’µΩBMS÷’÷π≥‰µÁ±®Œƒ±Í÷æ  0x101956F4
unsigned char CAN_ChargeEnd_181C_Flg = 0;              //Ω” ’µΩBMS≥‰µÁπ˝≥ÃÕ≥º∆±®Œƒ±Í÷æ  0x181C56F4
unsigned char CAN_MultiData_request_Flg = 0;           //∂‡∞¸¥´ÀÕ«Î«Û±Í÷æ
unsigned char CAN_MultiData_RX_OK_Flg = 0;             //∂‡∞¸¥´ÀÕΩ” ’’˝≥£±Í÷æ

unsigned char uc_CAN101A_Tx_Flg = 0;                   //OBCπ ’œ÷˜∂Ø÷’÷π∑¢ÀÕ±®Œƒ±Í÷æ

unsigned char OBC_Charge_Status = 0;                   //≥‰µÁª˙◊¥Ã¨–≈œ¢   BMS–Ë∑¥¿° «∑Ò∆Ù∂Ø
unsigned int OBC_Charge_Time = 0;                      //≥‰µÁª˙∑¥¿°≥‰µÁ ±º‰≤Œ ˝

float OBC_Energy_Cal = 0;        //OBC ‰≥ˆƒ‹¡øÕ≥º∆  ≤…”√º‰∏Ù1∑÷÷”»°¡Ω◊È≤Œ ˝«Û∆Ωæ˘µƒ∑Ω Ω
float f_V_Output1;               //µÁ—π≤Œ ˝1
float f_V_Output2;               //µÁ—π≤Œ ˝2
float f_I_Output1;               //µÁ¡˜≤Œ ˝1
float f_I_Output2;               //µÁ¡˜≤Œ ˝2
unsigned int uiTimer60S_Cnt = 0; //60sº∆ ˝

unsigned int uiBMS_Set_Max_V_Output = 0;               //µÁ≥ÿ◊Ó∏ﬂµÁ—π≤Œ ˝

unsigned char CAN_HandShake_1827_Data[2] = {0};                //OBCΩ” ’Œ’ ÷±®Œƒ ˝æ›
unsigned char CAN_Recongnition_1C02_Data[56] = {0};            //OBCΩ” ’±Ê ∂±®Œƒ ˝æ›
unsigned char CAN_Recongnition_1C06_Data[16] = {0};            //OBCΩ” ’–ÓµÁ≥ÿ≤Œ ˝±®Œƒ ˝æ›
unsigned char CAN_ParameterConfiguration_1009_Data[1] = {0};   //OBCΩ”µÁ≥ÿ◊º±∏◊¥Ã¨±®Œƒ ˝æ›
unsigned char CAN_Charge_1810_Data[5] = {0};                   //OBCΩ” ’BMS≥‰µÁ ‰≥ˆ…Ë÷√ ˝æ›
unsigned char CAN_Charge_1C11_Data[16] = {0};                  //OBCΩ” ’BMS≥‰µÁ◊‹◊¥Ã¨ ˝æ›
unsigned char CAN_Charge_1813_Data[7] = {0};                   //OBCΩ” ’BMS–ÓµÁ≥ÿ◊¥Ã¨ ˝æ›
unsigned char CAN_Charge_1019_Data[4] = {0};                   //OBCΩ” ’BMS÷’÷π≥‰µÁ ˝æ›
unsigned char CAN_ChargeEnd_181C_Data[7] = {0};                //OBCΩ” ’BMSÕ≥º∆≤Œ ˝ ˝æ›
unsigned char CAN_ChargeEnd_081E_Data[4] = {0};                //OBCΩ” ’BMS¥ÌŒÛ£®Õ®–≈π ’œ£© ˝æ›


unsigned char CAN_Rx_MultiData[8] = {0};                       //OBCΩ” ’∂‡∞¸¥´ÀÕ ˝æ›
unsigned char CAN_MultiData_request[8] = {0};                  //OBCΩ” ’∂‡∞¸¥´ÀÕ«Î«Û ˝æ›
unsigned char CAN_MultiData_Answer[8] = {0};                   //OBC∑¥¿°‘ –Ì∂‡∞¸¥´ÀÕ«Î«Û ˝æ›


////////////////////∂‡∞¸–≠“È¥´ ‰Ω◊∂Œ ≤ªÕ¨µƒ∂‡∞¸¥´ÀÕ πƒ‹±Í÷æ///////////
unsigned char CAN_Recongnition_1C02_MultiData_request_Flg = 0;
unsigned char CAN_Recongnition_1C06_MultiData_request_Flg = 0;
unsigned char CAN_Charge_1C11_MultiData_request_Flg = 0;

unsigned char CAN_HandShake_Tx1826_Data[3] = {0};              //OBC∑¢ÀÕŒ’ ÷±®Œƒ ˝æ›
unsigned char CAN_Recongnition_TX1801_Data[8] = {0};           //OBC∑¢ÀÕ±Ê ∂±®Œƒ ˝æ›
unsigned char CAN_ParameterConfiguration_TX1807_Data[8] = {0}; // ±º‰Õ¨≤Ω–≈œ¢ ˝æ›
unsigned char CAN_ParameterConfiguration_TX1808_Data[8] = {0}; //OBC∑¢ÀÕ◊Ó¥Û ‰≥ˆƒ‹¡¶±®Œƒ ˝æ›
unsigned char CAN_ParameterConfiguration_TX100A_Data[1] = {0}; //OBC∑¢ÀÕ◊º±∏◊¥Ã¨±®Œƒ ˝æ›
unsigned char CAN_Charge_TX1812_Data[8] = {0};                 //OBC∑¢ÀÕ≥‰µÁ◊¥Ã¨±®Œƒ ˝æ›
unsigned char CAN_Charge_TX101A_Data[4] = {0};                 //OBC∑¢ÀÕ÷’÷π≥‰µÁ±®Œƒ ˝æ›
unsigned char CAN_ChargeEnd_TX181D_Data[8] = {0};              //OBC∑¢ÀÕÕ≥º∆≤Œ ˝±®Œƒ ˝æ›
unsigned char CAN_ChargeEnd_TX081F_Data[4] = {0};              //OBC∑¢ÀÕ¥ÌŒÛ£®Õ®–≈¥ÌŒÛ£©±®Œƒ ˝æ›

unsigned int BMS_Set_I_Output = 0;                            //BMS«Î«Û ‰≥ˆµÁ¡˜≤Œ ˝

unsigned char OBC_Charge_Stop_Reason_BMS_Stop = 0;            //OBCπ ’œ÷˜∂ØÕ£÷π‘≠“Ú≤Œ ˝

union  OBC_Err_type     OBC_Err_type_Msg1;                    //OBCπ ’œ∑¥¿°≤Œ ˝¡™∫œÃÂ
union  OBC_Err_type     OBC_Err_type_Msg2;
union  OBC_Err_type     OBC_Err_type_Msg3;

union  CAN_RX_OverTime  CAN_RX_OverTime_Msg1;                 //OBCÕ®–≈≥¨ ±π ’œ¡™∫œÃÂ
union  CAN_RX_OverTime  CAN_RX_OverTime_Msg2;
union  CAN_RX_OverTime  CAN_RX_OverTime_Msg3;
union  CAN_RX_OverTime  CAN_RX_OverTime_Msg4;

unsigned int ui_CAN1C02_Rx_Err_Cnt = 0;                      //OBCΩ” ’≤ªÕ¨÷°ID≥¨ ±º∆ ˝
unsigned int ui_CAN1C06_Rx_Err_Cnt = 0;
unsigned int ui_CAN1009_Rx_Err_Cnt = 0;
unsigned int ui_CAN1810_Rx_Err_Cnt = 0;
unsigned int ui_CAN1C11_Rx_Err_Cnt = 0;
unsigned int ui_CAN1019_Rx_Err_Cnt = 0;
unsigned int ui_CAN181C_Rx_Err_Cnt = 0;



//////////////////∫º≤Êπ˙±Í∑«≥µ‘ÿÕ®–≈–≠“È≤Œ ˝/////////////////


//////////////////√§≥‰≥‰µÁπ˝≥Ãπ‹¿Ì∫Ø ˝///////////////////////////
void ChargeProcess(void)
{
    if (g_strSysInfo.ucSystemChargeStepState != SystemInChargeFinish)//»Áπ˚≥‰µÁ√ª”–ÕÍ≥… “ª÷±‘⁄≥‰µÁπ˝≥Ã÷–—≠ª∑
    {
        switch(g_strSysInfo.ucSystemChargeStepState)
        {
        case SystemInPreChargeStep1 :
        {
            while(SystemInPreChargeStep1 == g_strSysInfo.ucSystemChargeStepState) //»Áπ˚ «‘§≥‰µÁµ˜’˚µÁ—πµÁ¡˜
            {
                //»Áπ˚ «‘§≥‰µÁ  “™ µ ±º‡øÿµÁ—π «∑Ò‘⁄±‰ªØ£¨µÁ¡˜”–◊‘Œ“µ˜Ω⁄
                ChargeStepManage(g_strSysInfo.strPreChangeStep1.uiChargeChangeVoltage,
                                 g_strSysInfo.strPreChangeStep1.uiChargeChangeCurrent, g_strSysInfo.strPreChangeStep1.uiChargeConstantVoltage,
                                 g_strSysInfo.strPreChangeStep1.uiChargeConstantCurrent, g_strSysInfo.strPreChangeStep1.uiChargeLimitTime);
            }
            break;
        }
        case SystemInPreChargeStep2 :
        {
            while(SystemInPreChargeStep2 == g_strSysInfo.ucSystemChargeStepState) //»Áπ˚ «‘§≥‰µÁµ˜’˚µÁ—πµÁ¡˜
            {
                //»Áπ˚ «‘§≥‰µÁ  “™ µ ±º‡øÿµÁ—π «∑Ò‘⁄±‰ªØ£¨µÁ¡˜”–◊‘Œ“µ˜Ω⁄
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
        case SystemInChargeFinish://≥‰µÁÕÍ≥…
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
    unsigned char ucChargeModeJudge = 0;//≥‰µÁƒ£ Ω≈–∂œ±‰¡ø

    if((0 == uiChangeVoltage) && (0 == uiChangeCurrent) && (0 == uiConstantVoltage) && (0 == uiConstantCurrent) && (0 == uiChargeLimitTime))
    {
        g_strSysInfo.ucSystemChargeStepState += 1;
        return;//»Áπ˚∏√∂Œµƒ≥‰µÁ≤Œ ˝Œ™0Àµ√˜≤ª“™¥À∂Œµƒ≥‰µÁ«˙œﬂ£¨◊™»ÎµΩœ¬“ª∏ˆ◊¥Ã¨°£≤¢«“≤ª◊˜≥‰µÁ¥¶¿Ì
    }
    if(uiChangeVoltage && uiConstantCurrent)ucChargeModeJudge = ChargeModeInConstantCurrent;
    else if(uiChangeCurrent && uiConstantVoltage)ucChargeModeJudge = ChargeModeInConstantVoltage;
    else return;//∑«‘⁄∫„—π£¨∫„¡˜ƒ£ Ω°£ «¥ÌŒÛµƒƒ£ Ω

    if(ChargeModeInConstantCurrent == ucChargeModeJudge)//»Áπ˚ «‘⁄∫„¡˜ƒ£ Ω
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
    else if(ChargeModeInConstantVoltage == ucChargeModeJudge)//»Áπ˚ «‘⁄∫„—πƒ£ Ω
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
    V_D_Coef.f_V_OutPut_Coef = 21.9;                    // ‰≥ˆµÁ—πµÁ¡˜œµ ˝º∞∆´“∆¡ø…Ë÷√
    V_D_Offset.f_V_OutPut_Offset = 175;
    I_D_Coef.f_I_OutPut_Coef = 17.18;
    I_D_Offset.f_I_OutPut_Offset = -343;
    V_Set = Voltage;
    I_Set = Currennt;
    uiLCD_Display_V_Set = Voltage;
    uiLCD_Display_I_Set = Currennt;
    if(V_Set > 1100)     //BMS…Ë÷√ ‰≥ˆµÁ—πµÁ¡˜œﬁ∑˘
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

    I_Set = I_Set *f_I_OutputCoef;   //Ωµπ¶¬ …Ë÷√   f_I_OutputCoefŒ™Ωµπ¶¬ œµ ˝

    f_OutputTemp = V_Set;
    f_OutputTemp = f_OutputTemp * V_D_Coef.f_V_OutPut_Coef + V_D_Offset.f_V_OutPut_Offset;  //…Ë÷√µÁ—π÷µ◊™ªªŒ™LLCµ•∆¨ª˙ª∑¬∑π§◊˜µƒ≤Œ ˝
    V_Set = f_OutputTemp;
    f_OutputTemp = I_Set;
    f_OutputTemp = f_OutputTemp * I_D_Coef.f_I_OutPut_Coef + I_D_Offset.f_I_OutPut_Offset;

    I_Set = f_OutputTemp;

    CAN_TX_LLC_Data[0] = V_Set >> 8;     //CANÕ®—∂∑¢ÀÕµÁ—πµÁ¡˜…Ë÷√µƒ ˝æ›
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
        if((!(Flg.all&0x0FFF))&&uiV_Output>420&&CAN_BMS_Rx_Flg&&(CC1_Control == 0)&&(CAN_BMS_RX_Data[4] == 0x00))    //≥‰µÁª˙π ’œ◊¥Ã¨º∞ºÏ≤‚±£ª§  ∑¢ÀÕÕ£÷π ‰≥ˆ√¸¡Ó
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
    Can_Send_LLC_Msg(CAN_TX_LLC_Data);      //CANÕ®—∂∑¢ÀÕ ˝æ›
}


/*******************************************************************************
 ∫Ø ˝øÈ√˚≥∆:  void OBC_Status_Manage(void)
 π¶ƒ‹:        Œ¬∂»±£ª§°¢ ‰»Îπ˝«∑—π°¢ ‰≥ˆπ˝—π°¢π˝¡˜°¢«∑—π°¢Õ®–≈π ’œ±£ª§
  ‰»Î≤Œ ˝:    Œﬁ
  ‰≥ˆ≤Œ ˝:    Œﬁ
 ∆‰À˚Àµ√˜:
*******************************************************************************/
void OBC_Status_Manage(void)
{
/////////////////////////////Œ¬∂»±£ª§//////////////////////////////////////
// LLCƒ£øÈ∏ﬂ”⁄85∂»£¨BUCKƒ£øÈ∏ﬂ”⁄90∂»π˝Œ¬±£ª§
// LLCƒ£øÈµÕ”⁄70∂»£¨BUCKƒ£øÈµÕ”⁄75∂»ª÷∏¥¬˙π¶¬ 
//	LLCƒ£øÈ∏ﬂ”⁄80∂»£¨BUCKƒ£øÈ∏ﬂ”⁄85∂»Ωµπ¶¬ ÷¡90%
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

/////////////////////////////Œ¬∂»±£ª§///////////////////////////////////////

///////////////////////////// ‰≥ˆ◊¥Ã¨±£ª§//////////////////////////////////////

    if(uiI_Output > 1100)     // ‰≥ˆπ˝—π±£ª§  ¡¨–¯ºÏ≤‚5S
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

    if(uiV_Output > 1100)      // ‰≥ˆπ˝¡˜±£ª§  ¡¨–¯ºÏ≤‚5S
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

    if(uc_Output_En_Flg)    //«∑—πºÏ≤‚±£ª§—” ±10S   ‰≥ˆ∫Û—” ±µ»¥˝≥‰µÁª˙ ‰≥ˆ∫ÛºÏ≤‚ «∑Ò ‰≥ˆ«∑—π
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
    if((uiV_Output < 420)&&(uiV_Output_UVP_Delay_Cnt >= 100))//«∑—πºÏ≤‚±£ª§
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

    if(uiV_Output <= 420)         //ºÏ≤‚µΩµÁ≥ÿµÁ—π
    {
        ucBat_Status = 0;
        Flg.bit.Batarry_Err = 1;
    }
    else
    {
        ucBat_Status = 1;
        Flg.bit.Batarry_Err = 0;
    }
    ///////////////////////////// ‰≥ˆ◊¥Ã¨±£ª§//////////////////////////////////////

    /////////////////////////////Õ®–≈≥¨ ±±£ª§//////////////////////////////////////
    ucCAN_LLC_Rx_ErrCnt++;             //”ÎLLCƒ£øÈCANÕ®—∂≥¨ ±ºÏ≤‚  ¡¨–¯5S
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
    if(ucCAN_BMS_Rx_ErrCnt >= 50)     //”ÎNMSƒ£øÈCANÕ®—∂≥¨ ±ºÏ≤‚  ¡¨–¯5S
    {
        ucCAN_BMS_Rx_ErrCnt = 50;
        Flg.bit.CAN_BMS_Err = 1;
    }
    else
    {
        Flg.bit.CAN_BMS_Err = 0;
    }
    /////////////////////////////Õ®–≈≥¨ ±±£ª§//////////////////////////////////////



}

void ShowChargeStatusLed(void)
{
    uiShowLedCnt++;
    uiShowLedCnt %=100000;
    /*****************************…œµÁ≥ı º◊¥Ã¨  ŒÂ∏ˆ÷∏ æµ∆—≠ª∑*******************/
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
    /*****************************…œµÁ≥ı º◊¥Ã¨  ŒÂ∏ˆ÷∏ æµ∆—≠ª∑*******************/


    /*****************************’˝≥£π§◊˜◊¥Ã¨                *******************/
    else
    {
        if(CAN_BMS_Rx_Flg)                          //”ÎÕ‚ΩÁÕ®–≈÷∏ æµ∆øÿ÷∆ Ω” ’µΩBMSÕ®—∂ ± ƒ¨»œÕ®–≈◊¥Ã¨   πƒ‹Õ®—∂µ∆
        {
            C_EnOutStateLed1();
        }
        else
        {
            C_DisEnOutStateLed1();
        }

        if(CAN_BMS_RX_Data[4] == 0x01)  //Ω” ’µΩπÿª˙–≈∫≈  µ„¡¡≥‰◊„÷∏ æµ∆
        {
            C_EnOutStateLed2();
            C_DisEnOutStateLed3();
            C_DisEnOutStateLed4();
            C_DisEnOutStateLed5();
        }
        else
        {
            /*****************************π ’œ◊¥Ã¨                *******************/
            if(Flg.all & 0x0FF7)           //≥ˆœ÷π ’œ ± πÿ±’∫„—π π§◊˜÷∏ æµ∆ µ„¡¡π ’œµ∆
            {
                C_DisEnOutStateLed3();
                C_DisEnOutStateLed4();
                C_EnOutStateLed5();
            }
            /*****************************’˝≥£÷∏ æ◊¥Ã¨               *******************/
            else
            {
                C_DisEnOutStateLed5();
                if(uc_Output_En_Flg == 0&&uiI_Output > 30)    // ‰≥ˆ πƒ‹«“ ‰≥ˆµÁ¡˜¥Û”⁄3AŒ™π§◊˜◊¥Ã¨
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

///////////////////////Õ®”√–≠“È//////////////////////////////////
void CAN_Msg_Handle(void)
{
//∑¥¿°≥‰µÁª˙◊¥Ã¨µΩBMS
    CAN_BMS_TX_Data[0] = CAN_LLC_RX_Data[0];   //«∞¡ΩŒªŒ™ ‰≥ˆµÁ—π
    CAN_BMS_TX_Data[1] = CAN_LLC_RX_Data[1];
    CAN_BMS_TX_Data[2] = CAN_LLC_RX_Data[2];	 //»˝ÀƒŒªŒ™ ‰≥ˆµÁ¡˜
    CAN_BMS_TX_Data[3] = CAN_LLC_RX_Data[3];
    CAN_BMS_TX_Data[4] = (Flg.all&0xFF1F);     //µ⁄ŒÂ¡˘ŒªŒ™π ’œ¥˙¬Î
    CAN_BMS_TX_Data[5] = (Flg.all&0xFFE0) >> 5;
    CAN_BMS_TX_Data[6] = CAN_PFC_RX_Data[7] ;  //‘§¡ÙŒª
    CAN_BMS_TX_Data[7] = CAN_PFC_RX_Data[6] ;

    Can_Send_BMS_Msg(CAN_BMS_TX_Data,8,0x18FF50E6);      //∑¥¿°≥‰µÁª˙–≈œ¢µΩBMS
}



////////////////////∫º≤ÊCAN–≠“È////////////////////////////////

//void CAN_Msg_Handle(void)
//{

//	///////////////////////////≥‰µÁª˙‘⁄∏˜Ω◊∂ŒΩ” ’BMS±®Œƒ≥¨ ±≈–∂œ//////////////////////
////	if((CAN_Msg_StepState == CAN_Msg_In_Recongnition)&&(CAN_Recongnition_1C06_Flg == 0))       //±Ê ∂Ω◊∂Œ  ≈–∂œΩ” ’BMS≥µ¡æ±Ê ∂±®Œƒ≥¨ ± 5s
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
////	if(CAN_Msg_StepState == CAN_Msg_In_ParameterConfiguration)                  //≤Œ ˝≈‰÷√Ω◊∂Œ  ±
////	  {
////		if(CAN_Recongnition_1C06_Flg == 0)                                         //Ω” ’≤Œ ˝≈‰÷√±®Œƒ(5S)≥¨ ±
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
////		  ui_CAN1009_Rx_Err_Cnt++;                                               //Ω” ’µÁ≥ÿ◊º±∏æÕ–˜£®60S£©±®Œƒ≥¨ 
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
////	if(CAN_Msg_StepState == CAN_Msg_In_Charge)            //≥‰µÁΩ◊∂Œ
////	  {
////	    ui_CAN1810_Rx_Err_Cnt++;
////		  if(ui_CAN1810_Rx_Err_Cnt >= 100)                 //Ω” ’µÁ≥ÿ≥‰µÁ–Ë«Û±®Œƒ≥¨ ±
////		    {
////		     CAN_RX_OverTime_Msg3.Bits.Bit3_4 = 0x01;
////		    }
////			else
////			  {
////			  CAN_RX_OverTime_Msg3.Bits.Bit3_4 = 0x00;
////			  }
////			 ui_CAN1C11_Rx_Err_Cnt++;
////		  if(ui_CAN1C11_Rx_Err_Cnt >= 500)                 //Ω” ’µÁ≥ÿ◊‹◊¥Ã¨±®Œƒ≥¨ ±
////		    {
////		     CAN_RX_OverTime_Msg3.Bits.Bit1_2 = 0x01;
////		    }
////			else
////			  {
////			  CAN_RX_OverTime_Msg3.Bits.Bit1_2 = 0x00;
////			  }
////
////		if(uc_CAN101A_Tx_Flg)                             //Ω” ’BMS≥‰µÁ÷’÷π±£Œ¿≥¨ ±
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
////if(CAN_Msg_StepState == CAN_Msg_In_ChargeEnd)         //≥‰µÁΩ· ¯Ω◊∂Œ  Ω” ’BMSÕ≥º∆±®Œƒ≥¨ ±
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
/////////////////////////////≥‰µÁª˙‘⁄∏˜Ω◊∂ŒΩ” ’BMS±®Œƒ≥¨ ±≈–∂œ//////////////////////
//
//
/////////////////////////////≥‰µÁª˙‘⁄∏˜Ω◊∂Œ∑¥¿°–≈œ¢º∞–≈œ¢¥¶¿Ì//////////////////////
//	uiV_Output= CAN_LLC_RX_Data[0] * 256 + CAN_LLC_RX_Data[1];
//	uiI_Output = CAN_LLC_RX_Data[2] * 256 + CAN_LLC_RX_Data[3];
//	OBC_Charge_Time = ui100msCnt /600;
/////////////////////////////≥‰µÁª˙ ‰≥ˆƒ‹¡øº∆À„//////////////////////
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
/////////////////////////////≥‰µÁª˙ ‰≥ˆƒ‹¡øº∆À„//////////////////////
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
//		  Can_Send_BMS_Msg(CAN_ParameterConfiguration_TX1807_Data, 8,0x1807F456); 	 // ±º‰Õ¨≤Ω≤Œ ˝
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
//if(0)   //  Flg.all &0x0FE7  Œ¥ºÏ≤‚µÁ≥ÿπ ’œ”ÎBMS±®ŒƒΩ” ’π ’œ¥À¥¶≤ª◊ˆ¥¶¿Ì  £®¥À–≠“ÈBMSΩ” ’π ’œ¡ÌÕ‚¥¶¿Ì µÁ≥ÿπ ’œ¥À¥¶≤ª¥¶¿Ì ‘⁄–≠“È÷–¡Ì”–∑¥¿°£©
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
/////////////////////////////≥‰µÁª˙‘⁄∏˜Ω◊∂Œ∑¥¿°–≈œ¢º∞–≈œ¢¥¶¿Ì//////////////////////
//}



























