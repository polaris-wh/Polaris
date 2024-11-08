// TI File $Revision: /main/1 $
// Checkin $Date: December 5, 2008   18:01:09 $
//###########################################################################
//
// FILE:	DSP2803x_Sci.c
//
// TITLE:	DSP2803x SCI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2803x C/C++ Header Files V1.10 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "DSP2803x_Device.h"     // DSP2803x Headerfile Include File
#include "DSP2803x_Examples.h"   // DSP2803x Examples Include File

//---------------------------------------------------------------------------
// InitSci:
//---------------------------------------------------------------------------
// This function initializes the SCI(s) to a known state.
//
void InitSci(void)
{
    // Initialize SCI-A:
   InitSciGpio();

   SciaRegs.SCICCR.all = 0x0007;               // 1 stop bit,  No loopback 
                                               // No parity,8 char bits,
                                               // async mode, idle-line protocol
   SciaRegs.SCICTL1.all = 0x0003;              // disenable TX, RX, internal SCICLK, 
                                               // Disable RX ERR, SLEEP, TXWAKE
//   SciaRegs.SCIHBAUD = 487>>8;                 //Baund:9600bps(LSPCLK = 37.5MHz)
//   SciaRegs.SCILBAUD = 487 & 0x00FF;

   SciaRegs.SCIHBAUD    =0x0000;  // 9600 baud @LSPCLK = 15MHz (60 MHz SYSCLK).
   SciaRegs.SCILBAUD    =0x00C2;
  
   SciaRegs.SCICTL2.bit.TXINTENA = 0;          //disable SCI TX interrupt
   SciaRegs.SCICTL2.bit.RXBKINTENA = 0;        //disable SCI RX interrupt
   SciaRegs.SCICCR.bit.LOOPBKENA =0;           // disable loop back  for test self

   SciaRegs.SCIFFTX.all = 0xE040;              //SCIRST, SCI FIFO enable,
                                               //TX FIFO interrupt disable,
											   
   SciaRegs.SCIFFRX.all = 0x604f;              //RX FIFO interrupt enable,
                                               //RX FIFO interrupt level=16bytes
   SciaRegs.SCIFFCT.all = 0x00;
   SciaRegs.SCICTL1.all = 0x0023;             // Relinquish SCI from Reset 
   SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;     //Enable FIFO operation
   SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;      //Enable FIFO operation
   
    //tbd...

}

//---------------------------------------------------------------------------
// Example: InitSciGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as SCI pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
// Caution:
// Only one GPIO pin should be enabled for SCITXDA/B operation.
// Only one GPIO pin shoudl be enabled for SCIRXDA/B operation.
// Comment out other unwanted lines.

void InitSciGpio()
{
   InitSciaGpio();
}

void InitSciaGpio()
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.
// This will enable the pullups for the specified pins.

	GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;     // Enable pull-up for GPIO7  (SCIRXDA)

	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;	   // Enable pull-up for GPIO29 (SCITXDA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;	   // Enable pull-up for GPIO12 (SCITXDA)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
//	GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 3;   // Asynch input GPIO7 (SCIRXDA)

/* Configure SCI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA operation
//	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 2;    // Configure GPIO7  for SCIRXDA operation

	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation
//	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 2;   // Configure GPIO12 for SCITXDA operation

    EDIS;
}

//===========================================================================
// End of file.
//===========================================================================
