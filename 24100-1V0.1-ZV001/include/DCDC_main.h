//########################################################
//
// FILE:   dcdc100a.h
//
// TITLE:  dcdc100a variables definitions.
//
//########################################################
// Running on TMS320LF280xA   DCDC part   H641AU211                
// External clock is 20MHz, PLL * 10/2 , CPU-Clock 100 MHz
// Date: from March 16, 2006 to Nov 30, 2006  , (C) www & mhp & lsy
// Version:1.00     Change Date: April 4, 2006 , (C) www & mhp & lsy
//########################################################
#ifndef DCDC_MAIN_H
#define DCDC_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif




/*------------��������-------------------------*/

#define DEAD_TIME       15          //�����ű۵�����ʱ�� 40*10ns=400ns
#define PHASE_DELAY     15          //�ڹ��ͺ���ܵ�ʱ�� 20*10ns=200ns
#define TMAX            900         //��󿪹����� ��Ӧ��Ϳ���Ƶ�� 80kHz
#define TMIN            200         //��С��������Ҳ�ǳ�ʼ����Ƶ�� ��Ӧ��߿���Ƶ�� 111kHz


/*------------��������-------------------------*/
        // the whole time of module working (hour)

/*------------�ṹ����-------------------------*/
/*struct uintData
{
    unsigned int uiLD;
    unsigned int uiHD;
};
typedef struct uintData uintStructData;
*/
//extern volatile unsigned int uiMdlAddr;
////////////////////////////////////////////////////////////////
/************************��������*******************************/
interrupt void dcdc_isr(void);
interrupt void xint1_isr(void);
extern void dcdcloop_init(void);
// Prototype statements for functions found within this file.
extern void InitEPwm1Example(void);
extern void InitEPwm2Example(void);
extern void InitEPwm3Example(void);

extern void MTimer0(void);
extern void MTimer1(void);
extern void AdcChuli(void);
extern void IoChuli(void);
extern void SpiChuli(void);
extern void KeyChuli(void);
extern void CDChuli(void);
extern void GetSciData(void);
extern void SendSciData(void);
extern void ShowChargeProcessLed(void);
#define C_DisOutStateGreenLed()  GpioDataRegs.GPASET.bit.GPIO3 = 1
#define C_EnOutStateGreenLed()   GpioDataRegs.GPACLEAR.bit.GPIO3 = 1
#define C_DisOutStateRedLed()    GpioDataRegs.GPASET.bit.GPIO2 = 1
#define C_EnOutStateRedLed()     GpioDataRegs.GPACLEAR.bit.GPIO2 = 1



#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  // end of DSP280x_ADC_H definition
