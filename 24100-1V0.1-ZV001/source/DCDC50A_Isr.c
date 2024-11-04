/************************************************************************
ProjectName： 24100-1V0.1-ZV001
Author     ： WH
Version    ： ZV001
Date       ： 2024-10-25

************************************************************************/


#include "DSP2803x_Device.h"     // DSP280x Headerfile Include File
#include "DSP2803x_Examples.h"   // DSP280x Examples Include File

#pragma CODE_SECTION(dcdc_isr, "ramfuncs");
#pragma CODE_SECTION(xint1_isr, "ramfuncs");
//interrupt void dcdc_isr(void);
//unsigned int      uiErrTD;
extern unsigned int     uiActionReady;
#define     MEP_SF                          3520        //TBD
#define     DCDC_PWMMIN                     2560        //TBD
#define     DCDC_VOLTMOD_CURRLIM            3500*8      //150A*3500*8/32768=128A
#define     VOLT_SHIFT                      12
#define     CURR_SHIFT                      13

/***************************************************************************
*    Function Name:     ctldcdc_isr
***************************************************************************/ 
void dcdcloop_init(void)
{
/*  Uint16  i;
    Uint16 *addr;

    for(i=0x801D; i<0x802B; i++)
    {
        addr = (unsigned int *)i;
        *addr=0;
    }
*/
    // voltage loop PI
    Dcdcisr.ui_Dcdc_Volt_K1 = 4096;     //Q12   2nd prototype parameter 4096
    Dcdcisr.ui_Dcdc_Volt_K2 = 0;        //                   0
    Dcdcisr.ui_Dcdc_Volt_K3 = 2718;     //3.95        4911   ERR0比较重要！2718(1.0)
    Dcdcisr.ui_Dcdc_Volt_K4 = 4463; //3.95        7400   //eRR1       4463(1.0)
    Dcdcisr.ui_Dcdc_Volt_K5 = 1787;     //3.95        2800  //err2       1787(1.0)
    Dcdcisr.ui_Dcdc_Volt_K6 = 0;        //                  //

    // current loop PI ( Kp = K4-2*K5 ; Ki = K3-K4+K5 ; Kd = K5 )

    Dcdcisr.ui_Dcdc_Curr_K1 = 8192;     // Q13    2nd prototype parameter
    Dcdcisr.ui_Dcdc_Curr_K2 = 0;            //
    Dcdcisr.ui_Dcdc_Curr_K3 = 1013;     //      4052
    Dcdcisr.ui_Dcdc_Curr_K4 = 830;      //    3321
    Dcdcisr.ui_Dcdc_Curr_K5 = 0;            //   4052

    Dcdcisr.ui_Dcdc_Curr_Filt_K1 = 30850;   //15Hz calculate frequency:1.5625K Q15
    Dcdcisr.ui_Dcdc_Curr_Filt_K2 = 959;

    Dcdcisr.ui_Dcdc_Temp_Ctrl = 50;
    Dcdcisr.ui_Dcdc_Temp_Set1 = 35;     //-15   875Bh
    Dcdcisr.ui_Dcdc_Temp_Set2 = 40;     //-10   875Ch
    Dcdcisr.ui_Dcdc_K_flag = 0;             //      875Dh

    //Dcdcisr.ui_Dcdc_debug = 1;
    Dcdcisr.ui_Dcdc_Vdebug0 = 200;
    Dcdcisr.ui_Dcdc_Vdebug1 = 10;
    Dcdcisr.ui_Dcdc_Volt_Ref = 17244;      //80v---28740

    Dcdcisr.ui_Dcdc_Vversion = 0x101;
    Dcdcisr.ui_Dcdc_Dversion = 1;           // baseline B08:for archive
    Dcdcisr.ui_Dcdc_Tversion = 0x0;     //mask AC derating when off, mask protect when AC cut
//  uiVersionNoSw = 101;

    Dcdcisr.ui_Dcdc_Curr_Ref = 1024;        //25A=25mV*80/3V *4095 *8=2730*8=21840
    Dcdcisr.ui_Dcdc_Curr_140A = 24464;      //140A--28A=28mV*80/3V *4095*8=3058*8=24464
    Dcdcisr.ui_Dcdc_Curr_160A = 30720;      // change to 150A  3840*8

    Dcdcisr.ui_Dcdc_Duty_Ramp = DCDC_PWMMIN;        // 2*64
    Dcdcisr.ui_Dcdc_Duty_Short = DCDC_PWMMIN;      //2*64
    Dcdcisr.ui_Dcdc_Duty_Permit = DCDC_PWMMIN;    //55*64
    Dcdcisr.ul_Dcdc_Power_Lim = 0x80000000;     //2560*8*512*54;    //100A=160A*2560*8/32768
    Dcdcisr.ui_Dcdc_Power_Lim = 30720;         //30A=30mV*80/3V *4095*8=3276*8=26208
    //Dcdcisr.ui_Dcdc_Pwm_Set = 0x0;
    Dcdcisr.ui_Dcdc_Curr_delta = 0;     // (82k//2k)*2.5/(3M+(82k//2k)) * (82/2) /3 *4096 = 91

//  Dcdcisr.ui_Dcdc_Duty_Ramp = 128;
    Dcdcisr.ui_Dcdc_Duty_Short = 128;
    Dcdcisr.ui_Dcdc_Short_Flag = 0;
}

////////////////////////////////////////////////////////
///ePWM  interrupt
///////////////////////////////////////////////////////
interrupt void dcdc_isr(void)
{
 
    signed int              Dcdc_Tmp;
    signed long             Dcdc_TmpLong;
    unsigned long           Dcdc_uTmpLong;
    unsigned int            uiErrOut;

///////////hxs 过流保护////////////////////////////////
    if((GpioDataRegs.GPADAT.bit.GPIO6 == 1))
       {
         asm("    NOP");
         asm("    NOP");
         asm("    NOP");
         asm("    NOP");
         asm("    NOP");
         if((GpioDataRegs.GPADAT.bit.GPIO6 == 1)&&(uiActionReady >= 1))
            {
              uiActionReady = 0;
         //   GpioDataRegs.GPBSET.bit.GPIO34 = 1;                   //关DC继电器
        //  EALLOW;
        //  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;                 // Configure GPIO0 as EPWM1A
        //  GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;                 // Configure GPIO1 as EPWM1B
        //  GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;                 // Configure GPIO2 as EPWM2A
        //  GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;                 // Configure GPIO3 as EPWM2B
        //  EDIS;

              EPwm3Regs.AQCSFRC.all = 0x5;                      //强制PWM1A,PWM1B输出低
        //    EPwm2Regs.AQCSFRC.all = 0x5;                      //强制PWM2A,PWM2B输出低
            }
        }

    //GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;          // testp1 clear

    //Read ADC
    Dcdcisr.ui_Dcdc_Curr_Adc = (AdcResult.ADCRESULT2) + (AdcResult.ADCRESULT1) + (AdcResult.ADCRESULT14) + (AdcResult.ADCRESULT13)
                            +(AdcResult.ADCRESULT6) +( AdcResult.ADCRESULT5) + (AdcResult.ADCRESULT10) + (AdcResult.ADCRESULT9) ;

    Dcdcisr.ui_Dcdc_Volt_Adc = (AdcResult.ADCRESULT3)+(AdcResult.ADCRESULT11);      // use Adc value
    //   Dcdcisr.ui_Dcdc_Vbat_Adc = AdcResult.ADCRESULT3;
     //Compare with current limit mode
    Dcdcisr.i_Dcdc_Curr_Err2 = Dcdcisr.i_Dcdc_Curr_Err1;
    Dcdcisr.i_Dcdc_Curr_Err1 = Dcdcisr.i_Dcdc_Curr_Err0;

    Dcdcisr.ui_Dcdc_Curr_Out2 = Dcdcisr.ui_Dcdc_Curr_Out1;
    Dcdcisr.ui_Dcdc_Curr_Out1 = Dcdcisr.ui_Dcdc_Curr_Out0;

    Dcdcisr.ui_Dcdc_Run_Mode &= 0xfffc;
    if (Dcdcisr.ui_Dcdc_Power_Lim>=Dcdcisr.ui_Dcdc_Curr_Ref)
    {
        Dcdc_Tmp = Dcdcisr.ui_Dcdc_Curr_Ref;
        Dcdcisr.ui_Dcdc_Run_Mode |= 0x0002;
    }
    else
    {
        Dcdc_Tmp = Dcdcisr.ui_Dcdc_Power_Lim;
        Dcdcisr.ui_Dcdc_Run_Mode |= 0x0001;
    }

    Dcdcisr.i_Dcdc_Volt_Err2 = Dcdcisr.i_Dcdc_Volt_Err1;
    Dcdcisr.i_Dcdc_Volt_Err1 = Dcdcisr.i_Dcdc_Volt_Err0;

    Dcdcisr.ui_Dcdc_Volt_Out2 = Dcdcisr.ui_Dcdc_Volt_Out1;
    Dcdcisr.ui_Dcdc_Volt_Out1 = Dcdcisr.ui_Dcdc_Volt_Out0;

//  GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;            // testp1 clear
    AdcRegs.ADCSOCFRC1.all = 0xffff;

    //Update vars
    Dcdcisr.i_Dcdc_Volt_Err0 = (signed int)(Dcdcisr.ui_Dcdc_Volt_Ref - (Dcdcisr.ui_Dcdc_Volt_Adc<<2)); //Q15

    /*if (uiActionReady>=3)
    {
        if(Dcdcisr.i_Dcdc_Volt_Err0 < -Dcdcisr.ui_Dcdc_Vdebug0)     //0.2V
        {
            Dcdcisr.i_Dcdc_Volt_Err2 -= Dcdcisr.ui_Dcdc_Vdebug1;//(abs(Dcdcisr.i_Dcdc_Volt_Err1) >>4);
        }
        else if(Dcdcisr.i_Dcdc_Volt_Err0 > Dcdcisr.ui_Dcdc_Vdebug0)
        {
            Dcdcisr.i_Dcdc_Volt_Err2 += Dcdcisr.ui_Dcdc_Vdebug1;//(Dcdcisr.i_Dcdc_Volt_Err1 >>4);
        }
    }*/

    //DCDC Voltage Regulator, HV:Q15*Q12, LV:Q15*Q9

    Dcdc_TmpLong = (long)Dcdcisr.ui_Dcdc_Volt_K1*Dcdcisr.ui_Dcdc_Volt_Out1 +(long)Dcdcisr.ui_Dcdc_Volt_K2*Dcdcisr.ui_Dcdc_Volt_Out2;
    Dcdc_TmpLong = Dcdc_TmpLong +(long)Dcdcisr.i_Dcdc_Volt_Err0 * Dcdcisr.ui_Dcdc_Volt_K3 - (long)Dcdcisr.i_Dcdc_Volt_Err1 * Dcdcisr.ui_Dcdc_Volt_K4
                +(long)Dcdcisr.i_Dcdc_Volt_Err2 * Dcdcisr.ui_Dcdc_Volt_K5 ;

 // Dcdcisr.l_Dcdc_Vresult1 = Dcdc_TmpLong;
    Dcdc_TmpLong =  Dcdc_TmpLong>>VOLT_SHIFT;
    //Range check   HV:Q15*Q12, LV:Q15*Q9
    if(Dcdc_TmpLong >= ( long)Dcdcisr.ui_Dcdc_Duty_Ramp)
    {
        Dcdc_TmpLong = ( long)Dcdcisr.ui_Dcdc_Duty_Ramp;
    }
    else if(Dcdc_TmpLong <= (long)DCDC_PWMMIN)
    {
        Dcdc_TmpLong = (long)DCDC_PWMMIN;
    }

    Dcdcisr.ui_Dcdc_Volt_Out0 = (unsigned int)(Dcdc_TmpLong );

//  GpioDataRegs.GPASET.bit.GPIO7 = 1;          // testp1 set

    //Update current loop vars



    Dcdcisr.i_Dcdc_Curr_Err0 = Dcdc_Tmp - (signed int)Dcdcisr.ui_Dcdc_Curr_delta -(signed int)Dcdcisr.ui_Dcdc_Curr_Adc ;  //Q15

    // avoid high current, quicken loop calculation
    if ((Dcdcisr.ui_Dcdc_Curr_Adc>=Dcdcisr.ui_Dcdc_Curr_140A)||1)       //140A
    {
        Dcdcisr.i_Dcdc_Curr_Err2 = Dcdcisr.i_Dcdc_Curr_Err0;
    }
    else
    {
        Dcdcisr.i_Dcdc_Curr_Err2 = 0;
    }

    // avoid high current, limit  duty to 120A short duty(9.5%)
    if (Dcdcisr.ui_Dcdc_Curr_Adc>=Dcdcisr.ui_Dcdc_Curr_160A)        //160A
    {
        Dcdcisr.ui_Dcdc_Duty_Permit = DCDC_PWMMIN;      //55*64;
    }
    else
    {
        Dcdcisr.ui_Dcdc_Duty_Permit = Dcdcisr.ui_Dcdc_Duty_Ramp;
    }

    //DCDC MOSFET stress is exceed 600V, judge short according to voltage,  limit duty and increase slowly, mhp 061123
    if ((Dcdcisr.ui_Dcdc_Volt_Adc<=400)&&(uiActionReady>=3)&&(Dcdcisr.ui_Dcdc_Short_Flag == 0)) // 1000/4096/2*61=7.44V
    {
        Dcdcisr.ui_Dcdc_Duty_Short = DCDC_PWMMIN;
        Dcdcisr.ui_Dcdc_Short_Flag = 1;
    }

    if ((Dcdcisr.ui_Dcdc_Duty_Short<Dcdcisr.ui_Dcdc_Duty_Permit)&&(Dcdcisr.ui_Dcdc_Short_Flag == 1))
        Dcdcisr.ui_Dcdc_Duty_Permit = Dcdcisr.ui_Dcdc_Duty_Short;


    //DCDC Current Regulator, HV:Q15*Q15, LV:Q15*Q12
    Dcdc_TmpLong = (long)Dcdcisr.ui_Dcdc_Curr_Out1*Dcdcisr.ui_Dcdc_Curr_K1 - (long)Dcdcisr.ui_Dcdc_Curr_Out2*Dcdcisr.ui_Dcdc_Curr_K2;
    Dcdc_TmpLong = Dcdc_TmpLong + (long)Dcdcisr.i_Dcdc_Curr_Err0 * Dcdcisr.ui_Dcdc_Curr_K3
            - (long)Dcdcisr.i_Dcdc_Curr_Err1 * Dcdcisr.ui_Dcdc_Curr_K4  + (long)Dcdcisr.i_Dcdc_Curr_Err2 * Dcdcisr.ui_Dcdc_Curr_K5;
 // Dcdcisr.l_Dcdc_Iresult1 = Dcdc_TmpLong;
    Dcdc_TmpLong = Dcdc_TmpLong>>CURR_SHIFT;

    //Range check
    if(Dcdc_TmpLong >= ( long)Dcdcisr.ui_Dcdc_Duty_Permit)
    {
        Dcdcisr.ui_Dcdc_Curr_Out0 = Dcdcisr.ui_Dcdc_Duty_Permit;
    }
    else if(Dcdc_TmpLong <= (long)DCDC_PWMMIN)
    {
        Dcdcisr.ui_Dcdc_Curr_Out0 = DCDC_PWMMIN;
    }
    else
        Dcdcisr.ui_Dcdc_Curr_Out0 = (unsigned int)(Dcdc_TmpLong);

    // compare the voltage loop and current loop, get little one
    if(Dcdcisr.ui_Dcdc_Volt_Out0>Dcdcisr.ui_Dcdc_Curr_Out0)
    {
        Dcdcisr.ui_Dcdc_Pwm_Out = Dcdcisr.ui_Dcdc_Curr_Out0;
        Dcdcisr.ui_Dcdc_Run_Mode |= 0x0010;
    }
    else if ((Dcdcisr.ui_Dcdc_Volt_Out0>Dcdcisr.ui_Dcdc_Curr_Out0-64)&&(Dcdcisr.ui_Dcdc_Run_Mode&0x0010))
    {
        Dcdcisr.ui_Dcdc_Pwm_Out = Dcdcisr.ui_Dcdc_Curr_Out0;
        //Dcdcisr.ui_Dcdc_Run_Mode |= 0x0010;
    }
    else
    {
        Dcdcisr.ui_Dcdc_Pwm_Out = Dcdcisr.ui_Dcdc_Volt_Out0;
        Dcdcisr.ui_Dcdc_Run_Mode = 0;
    }

//  if (Dcdcisr.ui_Dcdc_Pwm_Out < 0x500)        // for no load
//      Dcdcisr.ui_Dcdc_Pwm_Out = 0x40;
    
////###################################
/////////hxs PFM+PWM  ////////////////
    uiErrOut = Dcdcisr.ui_Dcdc_Pwm_Out >> 6;
    if (uiErrOut>=TMAX)       //最低频率限定 =10M/(700*2) =71KHZ
       uiErrOut = TMAX;
    if (uiErrOut <= 40)
       uiErrOut = 40;

    Dcdcisr.ui_Dcdc_PRD = uiErrOut;
    
 
    if (Dcdcisr.ui_Dcdc_PRD <= TMIN)       //最高频率限定 =10M/(550*2) =91KHZ
       {
       Dcdcisr.ui_Dcdc_PRD = TMIN;
       Dcdcisr.ui_Dcdc_ErrTD = 120 - (uiErrOut/12);
       if(Dcdcisr.ui_Dcdc_ErrTD >= 115)
          Dcdcisr.ui_Dcdc_ErrTD = 115;

       EPwm3Regs.DBRED = Dcdcisr.ui_Dcdc_ErrTD ;
       EPwm3Regs.CMPB = (Dcdcisr.ui_Dcdc_PRD>>1) + Dcdcisr.ui_Dcdc_ErrTD;

       }
    else
       {
        EPwm3Regs.DBRED = DEAD_TIME ;
        EPwm3Regs.CMPB = (Dcdcisr.ui_Dcdc_PRD>>1) + DEAD_TIME;
       }
    EPwm1Regs.TBPRD = Dcdcisr.ui_Dcdc_PRD;
    EPwm2Regs.TBPRD = Dcdcisr.ui_Dcdc_PRD;
    EPwm3Regs.TBPRD = Dcdcisr.ui_Dcdc_PRD;

    //#ifdef HV

        //Duty = CMPA-CMPB, CMPB=fixed value, EPwm1A & EPwm2A used
        EPwm3Regs.CMPA.half.CMPA = Dcdcisr.ui_Dcdc_PRD>>1;  // - (uiErrOut>>1);

        EPwm2Regs.CMPA.half.CMPA = (Dcdcisr.ui_Dcdc_PRD>>1);
        EPwm2Regs.CMPB = uiErrOut - DEAD_TIME;



     //   EPwm3Regs.CMPA.half.CMPA = Dcdcisr.ui_Dcdc_PRD>>1 ;
     //   EPwm3Regs.CMPB = Dcdcisr.ui_Dcdc_PRD>>1 ;
    //#endif    // endif HV

//  GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;            // testp1 clear

    Dcdcisr.ui_Dcdc_Start_Cnt++;
    if (Dcdcisr.ui_Dcdc_Start_Cnt >= 50)
        Dcdcisr.ui_Dcdc_Start_Cnt = 50;

    // for mem test (continually)
    /*if (uiActionReady>=3)
    {
        if (Dcdcisr.ui_Mem_Debug_Flag)
        {
            if (Dcdcisr.ui_Mem_Debug_Num<1024)
            {
                Dcdcisr.ui_Dcdc_Mem[Dcdcisr.ui_Debug_Num] = Dcdcisr.ui_Dcdc_Curr_Adc0;
            }

            Dcdcisr.ui_Mem_Debug_Num++;
            if (Dcdcisr.ui_Mem_Debug_Num>=1024)
            {
                Dcdcisr.ui_Mem_Debug_Num=0;
                Dcdcisr.ui_Mem_Debug_Flag = 0;
            }
        }
    }*/

    Dcdcisr.ui_Dcdc_Curr_Cnt++;
    Dcdcisr.ul_Dcdc_Curr_Sum = Dcdcisr.ul_Dcdc_Curr_Sum + (unsigned long)Dcdcisr.ui_Dcdc_Curr_Adc;
    Dcdcisr.ul_Dcdc_Volt_Sum = Dcdcisr.ul_Dcdc_Volt_Sum + (unsigned long)Dcdcisr.ui_Dcdc_Volt_Adc;
 //   Dcdcisr.ul_Dcdc_Vbat_Sum = Dcdcisr.ul_Dcdc_Vbat_Sum + (unsigned long)Dcdcisr.ui_Dcdc_Vbat_Adc;

    if (Dcdcisr.ui_Dcdc_Curr_Cnt >= 64)
    {
        Dcdcisr.ui_Dcdc_Curr_Cnt = 0;
        Dcdcisr.ui_Dcdc_Curr_Ave0 = (unsigned int)(Dcdcisr.ul_Dcdc_Curr_Sum>>7);
        Dcdc_uTmpLong = (unsigned long)Dcdcisr.ui_Dcdc_Curr_Filt_K1*Dcdcisr.ui_Dcdc_Curr_Dis0+(((unsigned long)Dcdcisr.ui_Dcdc_Curr_Filt_K1*Dcdcisr.ui_Dcdc_Curr_Dis0_Lo)>>15)
            +(unsigned long)Dcdcisr.ui_Dcdc_Curr_Filt_K2*Dcdcisr.ui_Dcdc_Curr_Ave0+(unsigned long)Dcdcisr.ui_Dcdc_Curr_Filt_K2*Dcdcisr.ui_Dcdc_Curr_Ave1;

        Dcdcisr.ul_Dcdc_Curr_Sum = 0;
        Dcdcisr.ui_Dcdc_Curr_Dis0 = (unsigned int)(Dcdc_uTmpLong >> 15);
        Dcdcisr.ui_Dcdc_Curr_Dis0_Lo = (unsigned int)(Dcdc_uTmpLong & 0x7FFF);
        Dcdcisr.ui_Dcdc_Curr_Ave1 = Dcdcisr.ui_Dcdc_Curr_Ave0;

//        Dcdcisr.ul_Dcdc_Vbat_Ave = Dcdcisr.ul_Dcdc_Vbat_Sum>>3;
        Dcdcisr.ul_Dcdc_Volt_Ave = Dcdcisr.ul_Dcdc_Volt_Sum>>5;
        if(Dcdcisr.ul_Dcdc_Volt_Ave<500)
            Dcdcisr.ul_Dcdc_Volt_Ave = 500;
        Dcdc_uTmpLong = Dcdcisr.ul_Dcdc_Power_Lim/Dcdcisr.ul_Dcdc_Volt_Ave;
        if (Dcdc_uTmpLong>=32760)
            Dcdcisr.ui_Dcdc_Power_Lim = 32760;
        else if (Dcdc_uTmpLong<100)
            Dcdcisr.ui_Dcdc_Power_Lim = 100;
        else
            Dcdcisr.ui_Dcdc_Power_Lim = (unsigned int)Dcdc_uTmpLong;
//      Dcdcisr.ul_Dcdc_Vbat_Sum = 0;
        Dcdcisr.ul_Dcdc_Volt_Sum = 0;
    }
/*  else
    {
        // Tempctrl<-15
        if ((Dcdcisr.ui_Dcdc_Temp_Ctrl <Dcdcisr.ui_Dcdc_Temp_Set1)&&(Dcdcisr.ui_Dcdc_K_flag == 0))
        {
            //Kv_dzsp = 1.0 20060530
            Dcdcisr.ui_Dcdc_Volt_K3 = 2718;         // gain 1.0
            Dcdcisr.ui_Dcdc_Volt_K4 = 4463;         // gain 1.0
            Dcdcisr.ui_Dcdc_Volt_K5 = 1787;         // gain 1.0

            Dcdcisr.ui_Dcdc_K_flag = 1;
        }
        // Tempctrl>-10
        else if ((Dcdcisr.ui_Dcdc_Temp_Ctrl >Dcdcisr.ui_Dcdc_Temp_Set2)&&Dcdcisr.ui_Dcdc_K_flag)
        {
            //Kv_dzsp = 4.0 20060530
            //Dcdcisr.ui_Dcdc_Volt_K3 = 10874;  //4.0       11961(4.4)
            //Dcdcisr.ui_Dcdc_Volt_K4 = 17854;  //4.0       19639(4.4)
            //Dcdcisr.ui_Dcdc_Volt_K5 = 7148;   //4.0       7863(4.4)

            Dcdcisr.ui_Dcdc_Volt_K3 = 9823;     //3.6       11961(4.4)
            Dcdcisr.ui_Dcdc_Volt_K4 = 16087;    //3.6       19639(4.4)
            Dcdcisr.ui_Dcdc_Volt_K5 = 6452;     //3.6       7863(4.4)

            Dcdcisr.ui_Dcdc_K_flag = 0;
        }
    }
*/
    if (Dcdcisr.ui_Dcdc_Curr_Max<Dcdcisr.ui_Dcdc_Curr_Adc)
        Dcdcisr.ui_Dcdc_Curr_Max = Dcdcisr.ui_Dcdc_Curr_Adc;

    //DCDC MOSFET stress is exceed 600V when short,  limit duty increase slowly, mhp 061123
    if (Dcdcisr.ui_Dcdc_Short_Flag == 1)
    {
        if (Dcdcisr.ui_Dcdc_Duty_Short<0x500)
            Dcdcisr.ui_Dcdc_Duty_Short =  Dcdcisr.ui_Dcdc_Duty_Short + 2;
        else
            Dcdcisr.ui_Dcdc_Duty_Short =  Dcdcisr.ui_Dcdc_Duty_Short + 8;
        if (Dcdcisr.ui_Dcdc_Duty_Short>=55*64)
            Dcdcisr.ui_Dcdc_Short_Flag = 2;
    }

    if ((Dcdcisr.ui_Dcdc_Volt_Adc>=2685)&&(Dcdcisr.ui_Dcdc_Short_Flag == 2))        //20/61*4096*2
        Dcdcisr.ui_Dcdc_Short_Flag = 0;

    //GpioDataRegs.GPASET.bit.GPIO7 = 1;            // testp1 set

    EPwm1Regs.ETCLR.bit.INT = 1;                // Clear INT flag for this timer
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; // Acknowledge this int to receive more int from group 3

}

interrupt void xint1_isr(void)
{
         asm("    NOP");
         asm("    NOP");
         asm("    NOP");
         asm("    NOP");
         asm("    NOP");
         if((GpioDataRegs.GPADAT.bit.GPIO6 == 1)&&0)
            {
              uiActionReady = 0;
         //   GpioDataRegs.GPBSET.bit.GPIO34 = 1;                   //关DC继电器
        //  EALLOW;
        //  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;                 // Configure GPIO0 as EPWM1A
        //  GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;                 // Configure GPIO1 as EPWM1B
        //  GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;                 // Configure GPIO2 as EPWM2A
        //  GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;                 // Configure GPIO3 as EPWM2B
        //  EDIS;

              EPwm3Regs.AQCSFRC.all = 0x5;                      //强制PWM1A,PWM1B输出低
        //    EPwm2Regs.AQCSFRC.all = 0x5;                      //强制PWM2A,PWM2B输出低
            }
       
    // Acknowledge this interrupt to get more from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


//===========================================================================
// No more.
//===========================================================================
