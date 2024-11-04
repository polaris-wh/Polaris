// TI File $Revision: /main/2 $
// Checkin $Date: February 24, 2009   15:54:31 $
//###########################################################################
//
// FILE:   DSP2803x_EPwm.c
//
// TITLE:  DSP2803x EPwm Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2803x C/C++ Header Files V1.10 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "DSP2803x_Device.h"     // DSP2803x Headerfile Include File
#include "DSP2803x_Examples.h"   // DSP2803x Examples Include File

//---------------------------------------------------------------------------
// InitEPwm:
//---------------------------------------------------------------------------
// This function initializes the EPwm(s) to a known state.
//
void InitEPwm(void)
{
  InitEPwmGpio();
  
  
   // Initialize EPwm1/2/3/4/5/6/7
   InitEPwm1Example();
   InitEPwm2Example();
//   InitEPwm3Example();
   InitEPwm4Example();
//   InitEPwm6Example();
   //tbd...
   EPwm1Regs.ETSEL.bit.INTEN = 1;      //启用中断
}

//---------------------------------------------------------------------------
// Example: InitEPwmGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as EPwm pins

void InitEPwm1Example()
{
   EPwm1Regs.AQCSFRC.all=0x05;            // 关驱动
   EPwm1Regs.TBPRD =  TMAX ;                        // Set timer period
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                      // Clear counter

   // Setup TBCLK
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;     // Count up-down
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // 主模块模式
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       //系统时钟分频得到时钟频率
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;          //
   EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;     //启用输出同步


   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

//   EPwm1Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
//   EPwm1Regs.ETSEL.bit.SOCASEL	= 2;		// Select SOC from from CPMA on upcount
//   EPwm1Regs.ETPS.bit.SOCAPRD 	= 2;		// Generate pulse on 1st event
   EPwm1Regs.ETSEL.bit.INTEN  = 0;               //启用EPWM中断
   EPwm1Regs.ETSEL.bit.INTSEL = 1;               //CTR=0时
   EPwm1Regs.ETPS.bit.INTPRD = 3;                //3次触发
   // Set actions

   ///////////////////变压器更改相序/////////////////
//   EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;	        //CTR=0则EPWM1A出高
//   EPwm1Regs.AQCTLA.bit.CBD = AQ_CLEAR;         //增计数时CTR=CMPA则EPWM1A出低
   ///////////////////变压器更改相序/////////////////
   EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            //CTR=0则EPWM1A出高
   EPwm1Regs.AQCTLA.bit.PRD = AQ_CLEAR;         //增计数时CTR=CMPA则EPWM1A出低


 //  EPwm1Regs.AQCTLB.bit.PRD = AQ_SET;           // SET PWM1B on event A, up count
 //  EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;         // CLEAR PWM1B on event A, down count

 
  // Set DB Mode
   EPwm1Regs.DBCTL.bit.IN_MODE= DBA_ALL;					//EPWM1A作为输入
   EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;   				//EPWM1B取反
   EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;   			//EPWM1B下降延迟,EPWM1A上升延迟 
   EPwm1Regs.DBRED = DEAD_TIME ;                		     //EPWM2A上升延迟时间12=12*16nS=200nS
   EPwm1Regs.DBFED = DEAD_TIME ;                			//EPWM1A上升延迟时间20=20*10nS=200nS 
 
     
  // Setup compare 
   EPwm1Regs.CMPA.half.CMPA = DEAD_TIME;			// Set compare A value
   EPwm1Regs.CMPB = TMAX - DEAD_TIME ;              // Set Compare B value
   

  }




void InitEPwm2Example()
{
   EPwm2Regs.AQCSFRC.all=0x05;            // 关驱动
   EPwm2Regs.TBPRD =  TMAX ;                        // Set timer period
   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                      // Clear counter

   // Setup TBCLK
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;    // Count up-down
   EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // 从模块模式
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       //系统时钟分频得到时钟频率
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;          //
   EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;      //同步输入模式


   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   ///////////////////变压器更改相序/////////////////
   // Set actions
//   EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;	            		//增计数时CTR=CMPA则EPWM2A出高
//   EPwm2Regs.AQCTLA.bit.PRD = AQ_CLEAR;             		//增计数时CTR=CTR则EPWM2A出低
//   EPwm2Regs.AQCTLB.bit.PRD = AQ_SET;	            		//减计数时CTR=CMPB则EPWM2B出高
//   EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;             		//减计数时CTR=0则EPWM2B出低
   ///////////////////变压器更改相序/////////////////

   EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;                        //增计数时CTR=CMPA则EPWM2A出高
   EPwm2Regs.AQCTLA.bit.CBD = AQ_CLEAR;                     //增计数时CTR=CTR则EPWM2A出低
   EPwm2Regs.AQCTLB.bit.PRD = AQ_SET;                       //减计数时CTR=CMPB则EPWM2B出高
   EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;                     //减计数时CTR=0则EPWM2B出低
   
   
   EPwm2Regs.DBCTL.bit.IN_MODE= DBA_ALL;					//EPWM2A作为输入
   EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;   				//B取反
   EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;   			//用死区控制 
   EPwm2Regs.DBRED = DEAD_TIME;                			        //EPWM2A上升延迟时间12=12*16nS=200nS
   EPwm2Regs.DBFED = DEAD_TIME;
    

 // Setup compare 
   EPwm2Regs.CMPA.half.CMPA = DEAD_TIME;			// Set compare A value
   EPwm2Regs.CMPB = TMAX - DEAD_TIME ;              // Set Compare B value


 }
/*
void InitEPwm3Example()
{
   EPwm3Regs.AQCSFRC.all=0x05;            // 关驱动
   EPwm3Regs.TBPRD = TMAX;                        // Set timer period
   EPwm3Regs.TBPHS.half.TBPHS = 0x0546;           // TMAX - DEAD_TIME = 1500 - 150 =1350
   EPwm3Regs.TBCTR = 0x0000;                      // Clear counter

   // Setup TBCLK
   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up-down
   EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // 从模式
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       //系统时钟分频得到时钟频率
   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
   EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;                //同步流通


   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

  
 
   // Set actions
   EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // SET PWM3A on event A, up count
   EPwm3Regs.AQCTLA.bit.CBD = AQ_SET;           // CLEAR PWM3A on event A, down count
//   EPwm3Regs.AQCTLB.bit.CAU = AQ_SET;          // CLEAR PWM3B on event B, up count
//   EPwm3Regs.AQCTLB.bit.PRD = AQ_CLEAR;            //SET PWM3B on event B, down count

   
   EPwm3Regs.DBCTL.bit.IN_MODE= DBA_ALL;					//EPWM3A作为输
   EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;   				//EPWM3B取反
   EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;   		//EPWM3A,EPWM3B都用死区 
   EPwm3Regs.DBRED = DEAD_TIME;                			     //EPWM2A上升延迟时间
   EPwm3Regs.DBFED = DEAD_TIME;
 // Setup compare 
   EPwm3Regs.CMPA.half.CMPA = 1;			// Set compare A value
   EPwm3Regs.CMPB = TMAX - 1;                           // Set Compare B value
   
      
 }
*/

void InitEPwm4Example()
{
   EPwm4Regs.AQCSFRC.all=0x05;            // 关驱动
   EPwm4Regs.TBPRD =  TMAX ;                        // Set timer period
   EPwm4Regs.TBPHS.half.TBPHS = 24;           // Phase is 0
   EPwm4Regs.TBCTR = 0x0000;                      // Clear counter

   // Setup TBCLK
   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // 从模式
   EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       //系统时钟分频得到时钟频率
   EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;
   EPwm4Regs.TBCTL.bit.SYNCOSEL= TB_SYNC_IN;                //同步流通


   EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
   EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

 
 
   // Set actions
   EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;             // SET PWM3A on event A, up count
   EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;           // CLEAR PWM3A on event A, down count
   EPwm4Regs.AQCTLB.bit.PRD = AQ_SET;           // SET PWM1B on event A, up count
   EPwm4Regs.AQCTLB.bit.CBD = AQ_CLEAR;         // CLEAR PWM1B on event A, down count
 
  
   EPwm4Regs.DBCTL.bit.IN_MODE= DBA_ALL;					//EPWM4A作为输入
   EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;   				//B取反
   EPwm4Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;   			//不用死区控制
   EPwm4Regs.DBRED = DEAD_TIME ;                			        //EPWM4A上升延迟时间12=12*16nS=200nS
   EPwm4Regs.DBFED = DEAD_TIME ;
  // Setup compare 
   EPwm4Regs.CMPA.half.CMPA = 15;			// Set compare A value
   EPwm4Regs.CMPB = TMAX - 15;              // Set Compare B value
 //  EPwm4Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
  // EPwm4Regs.TBCTR = 0x0000;                  // Clear counter
   }



//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//

void InitEPwmGpio(void)
{
   InitEPwm1Gpio();
   InitEPwm2Gpio();
 //  InitEPwm3Gpio();

   InitEPwm4Gpio();

 //  InitEPwm5Gpio();

 //  InitEPwm6Gpio();




}

void InitEPwm1Gpio(void)
{
   EALLOW;

/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)

/* Configure EPWM-1 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM1 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;   // Configure GPIO1 as EPWM1B

    EDIS;
}

void InitEPwm2Gpio(void)
{
   EALLOW;

/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM3B)

/* Configure EPwm-2 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM2 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;   // Configure GPIO3 as EPWM2B

    EDIS;
}

void InitEPwm3Gpio(void)
{
   EALLOW;

/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)

/* Configure EPwm-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM3 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;   // Configure GPIO5 as EPWM3B

    EDIS;
}

#if DSP28_EPWM4
void InitEPwm4Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO6 (EPWM4A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up on GPIO7 (EPWM4B)

/* Configure EPWM-4 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM4 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B

    EDIS;
}
#endif // endif DSP28_EPWM4

#if DSP28_EPWM5
void InitEPwm5Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;    // Disable pull-up on GPIO8 (EPWM5A)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;    // Disable pull-up on GPIO9 (EPWM5B)

/* Configure EPWM-5 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM5 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B

    EDIS;
}
#endif // endif DSP28_EPWM5

#if DSP28_EPWM6
void InitEPwm6Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;    // Disable pull-up on GPIO10 (EPWM6A)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;    // Disable pull-up on GPIO11 (EPWM6B)

/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;   // Configure GPIO10 as EPWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;   // Configure GPIO11 as EPWM6B

    EDIS;
}
#endif // endif DSP28_EPWM6

#if DSP28_EPWM7
void InitEPwm7Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 1;    // Disable pull-up on GPIO40 (EPWM7A)
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1;    // Disable pull-up on GPIO41 (EPWM7B)

/* Configure EPWM-7 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM7 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 1;   // Configure GPIO40 as EPWM7A
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 1;   // Configure GPIO41 as EPWM7B

    EDIS;
}
#endif // endif DSP28_EPWM7


//---------------------------------------------------------------------------
// Example: InitEPwmSyncGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as EPwm Synch pins
//

void InitEPwmSyncGpio(void)
{

//   EALLOW;

/* Configure EPWMSYNCI  */

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

// GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO6 (EPWMSYNCI)
   GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;   // Enable pull-up on GPIO32 (EPWMSYNCI)

/* Set qualification for selected pins to asynch only */
// This will select synch to SYSCLKOUT for the selected pins.
// Comment out other unwanted lines.

// GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 0;   // Synch to SYSCLKOUT GPIO6 (EPWMSYNCI)
   GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 0;  // Synch to SYSCLKOUT GPIO32 (EPWMSYNCI)

/* Configure EPwmSync pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPwmSync functional pins.
// Comment out other unwanted lines.

// GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 2;    // Configures GPIO6 for EPWMSYNCI operation
   GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 2;   // Configures GPIO32 for EPWMSYNCI operation.

/* Configure EPWMSYNC0  */

/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// Comment out other unwanted lines.

// GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO6 (EPWMSYNCO)
   GpioCtrlRegs.GPBPUD.bit.GPIO33 = 1;   // Disable pull-up on GPIO33 (EPWMSYNCO)

// GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 3;   // Configures GPIO6 for EPWMSYNCO
   GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 2;  // Configures GPIO33 for EPWMSYNCO

}

//---------------------------------------------------------------------------
// Example: InitTzGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as Trip Zone (TZ) pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//

void InitTzGpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
   GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;    // Enable pull-up on GPIO12 (TZ1)
// GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;    // Enable pull-up on GPIO15 (TZ1)
// GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;    // Enable pull-up on GPIO13 (TZ2)
// GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;    // Enable pull-up on GPIO16 (TZ2)
   GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up on GPIO28 (TZ2)
   GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up on GPIO14 (TZ3)
// GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;    // Enable pull-up on GPIO17 (TZ3)
   GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up on GPIO29 (TZ3)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;  // Asynch input GPIO12 (TZ1)
// GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (TZ1)
//   GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;  // Asynch input GPIO13 (TZ2)
// GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;  // Asynch input GPIO16 (TZ2)
   GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (TZ2)
   GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3;  // Asynch input GPIO14 (TZ3)
// GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;  // Asynch input GPIO17 (TZ3)
   GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;  // Asynch input GPIO29 (TZ3)


/* Configure TZ pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be TZ functional pins.
// Comment out other unwanted lines.
   GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // Configure GPIO12 as TZ1
//   GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;  // Configure GPIO15 as TZ1
//   GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // Configure GPIO13 as TZ2
// GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 3;  // Configure GPIO16 as TZ2
   GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;  // Configure GPIO28 as TZ2
//   GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;  // Configure GPIO14 as TZ3
// GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 3;  // Configure GPIO17 as TZ3
   GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 3;  // Configure GPIO29 as TZ3

   EDIS;

}

//===========================================================================
// End of file.
//===========================================================================
