
 // TI File $Revision: /main/5 $
// Checkin $Date: December 16, 2009   16:43:59 $
//###########################################################################
//
// FILE:    DSP2803x_Adc.c
//
// TITLE:   DSP2803x ADC Initialization & Support Functions.
//
//###########################################################################
// $TI Release: f2803x Support Library v200 $
// $Release Date: Tue Jul 24 10:01:39 CDT 2012 $
//###########################################################################

#include "DSP2803x_Device.h"     // Headerfile Include File
#include "DSP2803x_Examples.h"   // Examples Include File

#define ADC_usDELAY  1000L

//---------------------------------------------------------------------------
// InitAdc:
//---------------------------------------------------------------------------
// This function initializes ADC to a known state.
//
// NOTE: ADC INIT IS DIFFERENT ON 2803x DEVICES COMPARED TO OTHER 28X DEVICES
//
void InitAdc(void)
{
    extern void DSP28x_usDelay(Uint32 Count);

    // *IMPORTANT*
    // The Device_cal function, which copies the ADC calibration values from TI reserved
    // OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
    // Boot ROM. If the boot ROM code is bypassed during the debug process, the
    // following function MUST be called for the ADC to function according
    // to specification. The clocks to the ADC MUST be enabled before calling this
    // function.
    // See the device data manual and/or the ADC Reference
    // Manual for more information.

        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
        (*Device_cal)();
        EDIS;

    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
    // after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
    // CPU_RATE define statement in the DSP2802x_Examples.h file must
    // contain the correct CPU clock period in nanoseconds.
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCBGPWD  = 1;      // Power ADC BG
    AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;      // Power reference
    AdcRegs.ADCCTL1.bit.ADCPWDN   = 1;      // Power ADC
    AdcRegs.ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;      // Select interal BG
    AdcRegs.ADCCTL1.bit.TEMPCONV = 1;       //Disable Tempconv
    EDIS;

    DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels

// 设置ADC采样通道

    EALLOW;
    AdcRegs.ADCSOC0CTL.bit.CHSEL 	= 5;	//set SOC0 channel select to ADCINA5  =Tcpu
    AdcRegs.ADCSOC1CTL.bit.CHSEL 	= 0;	//set SOC1 channel select to ADCINB0  =Io
    AdcRegs.ADCSOC2CTL.bit.CHSEL 	= 1;	//set SOC2 channel select to ADCINB1  =Io
    AdcRegs.ADCSOC3CTL.bit.CHSEL 	= 2; 	//set SOC3 channel select to ADCINA0  =Vo
	AdcRegs.ADCSOC4CTL.bit.CHSEL 	= 3;	//set SOC4 channel select to ADCINA2  =Vbat
	AdcRegs.ADCSOC5CTL.bit.CHSEL 	= 8;	//set SOC5 channel select to ADCINB2  =Io
    AdcRegs.ADCSOC6CTL.bit.CHSEL 	= 9;	//set SOC6 channel select to ADCINB3  =Io
    AdcRegs.ADCSOC7CTL.bit.CHSEL 	= 14;	//set SOC7 channel select to ADCINB6  =Tem3

    AdcRegs.ADCSOC8CTL.bit.CHSEL 	= 5;	//set SOC8 channel select to ADCINA5  =Tcpu
	AdcRegs.ADCSOC9CTL.bit.CHSEL	= 0;    //set SOC9 channel select to ADCINB0  =Io
	AdcRegs.ADCSOC10CTL.bit.CHSEL 	= 1;	//set SOC10 channel select to ADCINB1  =Io
	AdcRegs.ADCSOC11CTL.bit.CHSEL 	= 2;	//set SOC11 channel select to ADCINA1  =Vo
    AdcRegs.ADCSOC12CTL.bit.CHSEL 	= 3;	//set SOC12 channel select to ADCINA10 =Tmos
    AdcRegs.ADCSOC13CTL.bit.CHSEL 	= 8;	//set SOC13 channel select to ADCINB2  =Io
    AdcRegs.ADCSOC14CTL.bit.CHSEL 	= 9;	//set SOC14 channel select to ADCINB3  =Io
 	AdcRegs.ADCSOC15CTL.bit.CHSEL	= 15;   //set SOC15 channel select to ADCINB7 =Tem4
	
// 设置ADCSOCx的触发源	
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL 	= 0;	//set SOC0 start trigger on ePWM2A ADCSOCA, due to round-robin SOC0 converts first then SOC1
	AdcRegs.ADCSOC1CTL.bit.TRIGSEL 	= 0;	//set SOC1 start trigger on ePWM2A ADCSOCA, due to round-robin SOC1 converts first then SOC2
	AdcRegs.ADCSOC2CTL.bit.TRIGSEL 	= 0; 	//set SOC2 start trigger on ePWM2A ADCSOCA, due to round-robin SOC2 converts first then SOC3
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL 	= 0;	//set SOC2 start trigger on ePWM2A ADCSOCA, due to round-robin SOC2 converts first then SOC3
	AdcRegs.ADCSOC4CTL.bit.TRIGSEL 	= 0;	//set SOC3 start trigger on ePWM2A ADCSOCA, due to round-robin SOC3 converts first then SOC4
	AdcRegs.ADCSOC5CTL.bit.TRIGSEL 	= 0;	//set SOC4 start trigger on ePWM2A ADCSOCA, due to round-robin SOC3 converts first then SOC4
    AdcRegs.ADCSOC6CTL.bit.TRIGSEL 	= 0;	//set SOC4 start trigger on ePWM2A ADCSOCA, due to round-robin SOC3 converts first then SOC4
    AdcRegs.ADCSOC7CTL.bit.TRIGSEL 	= 0;	//set SOC4 start trigger on ePWM2A ADCSOCA, due to round-robin SOC3 converts first then SOC4
    AdcRegs.ADCSOC8CTL.bit.TRIGSEL 	= 0;    //set SOC4 start trigger on ePWM2A ADCSOCA, due to round-robin SOC3 converts first then SOC4
	AdcRegs.ADCSOC9CTL.bit.TRIGSEL 	= 0;    //set SOC4 start trigger on ePWM2A ADCSOCA, due to round-robin SOC3 converts first then SOC4
	AdcRegs.ADCSOC10CTL.bit.TRIGSEL = 0;	//set SOC3 start trigger on ePWM2A ADCSOCA, due to round-robin SOC3 converts first then SOC4
	AdcRegs.ADCSOC11CTL.bit.TRIGSEL = 0;	//set SOC4 start trigger on ePWM2A ADCSOCA, due to round-robin SOC3 converts first then SOC4
    AdcRegs.ADCSOC12CTL.bit.TRIGSEL = 0;	//set SOC4 start trigger on ePWM2A ADCSOCA, due to round-robin SOC3 converts first then SOC4
    AdcRegs.ADCSOC13CTL.bit.TRIGSEL = 0;	//set SOC4 start trigger on ePWM2A ADCSOCA, due to round-robin SOC3 converts first then SOC4
    AdcRegs.ADCSOC14CTL.bit.TRIGSEL = 0;    //set SOC4 start trigger on ePWM2A ADCSOCA, due to round-robin SOC3 converts first then SOC4
 	AdcRegs.ADCSOC15CTL.bit.TRIGSEL = 0;    //set SOC4 start trigger on ePWM2A ADCSOCA, due to round-robin SOC3 converts first then SOC4

// 设置采样保持时间
	AdcRegs.ADCSOC0CTL.bit.ACQPS 	= 6;	//set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC1CTL.bit.ACQPS 	= 6;	//set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC2CTL.bit.ACQPS 	= 6;	//set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC3CTL.bit.ACQPS 	= 6;	//set SOC3 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    AdcRegs.ADCSOC4CTL.bit.ACQPS 	= 6;	//set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC5CTL.bit.ACQPS 	= 6;	//set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC6CTL.bit.ACQPS 	= 6;	//set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    AdcRegs.ADCSOC7CTL.bit.ACQPS 	= 6;	//set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    AdcRegs.ADCSOC8CTL.bit.ACQPS 	= 6;    //set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC9CTL.bit.ACQPS 	= 6;    //set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    AdcRegs.ADCSOC10CTL.bit.ACQPS 	= 6;	//set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC11CTL.bit.ACQPS 	= 6;	//set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC12CTL.bit.ACQPS 	= 6;	//set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    AdcRegs.ADCSOC13CTL.bit.ACQPS 	= 6;	//set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    AdcRegs.ADCSOC14CTL.bit.ACQPS 	= 6;    //set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
 	AdcRegs.ADCSOC15CTL.bit.ACQPS 	= 6;    //set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)

	EDIS;

    InitAdcAio();

}

//配置ADC端口
void InitAdcAio()
{

   EALLOW;

/* Configure ADC pins using AIO regs*/
// This specifies which of the possible AIO pins will be Analog input pins.
// NOTE: AIO1,3,5,7-9,11,13,15 are analog inputs in all AIOMUX1 configurations.
// Comment out other unwanted lines.

    GpioCtrlRegs.AIOMUX1.bit.AIO2 = 1;    // Configure AIO2 for A2 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO4 = 1;    // Configure AIO4 for A4 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO6 = 1;    // Configure AIO6 for A6 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO10 = 1;   // Configure AIO10 for B2 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO12 = 1;   // Configure AIO12 for B4 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO14 = 1;   // Configure AIO14 for B6 (analog input) operation

    EDIS;
}



void StartAdc(void) //起动AD
{
    AdcRegs.ADCSOCFRC1.all = 0xffff; 
    while(AdcRegs.ADCCTL1.bit.ADCBSY ==0 );//等待转换完成
	asm(" NOP ");
    asm(" NOP ");
    asm(" CLRC SXM ");
}



//AD0,AD5,AD13有问题
//AD12不太好
//AD10可用环路


//===========================================================================
// End of file.
//===========================================================================
