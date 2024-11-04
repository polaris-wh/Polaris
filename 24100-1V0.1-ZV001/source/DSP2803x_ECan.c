// TI File $Revision: /main/2 $
// Checkin $Date: March 3, 2009   17:00:01 $
//###########################################################################
//
// FILE:	DSP2803x_ECan.c
//
// TITLE:	DSP2803x Enhanced CAN Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2803x C/C++ Header Files V1.10 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "DSP2803x_Device.h"     // DSP28 Headerfile Include File
#include "DSP2803x_Examples.h"   // DSP28 Examples Include File


//lyb_add_start
/////////////////////////////





//---------------------------------------------------------------------------
// InitECan:
//---------------------------------------------------------------------------
// This function initializes the eCAN module to a known state.
//
void InitECan(void)
{
   InitECana();
   InitECanaGpio();
}

void InitECana(void)		// Initialize eCAN-A module
{

/* Create a shadow register structure for the CAN control registers. This is
 needed, since only 32-bit access is allowed to these registers. 16-bit access
 to these registers could potentially corrupt the register contents or return
 false data. This is especially true while writing to/reading from a bit
 (or group of bits) among bits 16 - 31 */

struct ECAN_REGS ECanaShadow;

	EALLOW;		// EALLOW enables access to protected bits

/* Configure eCAN RX and TX pins for CAN operation using eCAN regs*/

    ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
    ECanaShadow.CANTIOC.bit.TXFUNC = 1;
    ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;

    ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
    ECanaShadow.CANRIOC.bit.RXFUNC = 1;
    ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all;

/* Configure eCAN for HECC mode - (reqd to access mailboxes 16 thru 31) */
									// HECC mode also enables time-stamping feature

	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.SCB = 1;
	ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

//	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
//	ECanaShadow.CANMC.bit.DBO = 1;
//	ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

/* Initialize all bits of 'Message Control Register' to zero */
// Some bits of MSGCTRL register come up in an unknown state. For proper operation,
// all bits (including reserved bits) of MSGCTRL must be initialized to zero

    ECanaMboxes.MBOX0.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX1.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX2.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX3.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX4.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX5.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX6.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX7.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX8.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX9.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX10.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX11.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX12.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX13.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX14.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX15.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX16.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX17.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX18.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX19.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX20.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX21.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX22.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX23.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX24.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX25.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX26.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX27.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX28.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX29.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX30.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX31.MSGCTRL.all = 0x00000000;

// TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
//	as a matter of precaution.

	ECanaRegs.CANTA.all	= 0xFFFFFFFF;	/* Clear all TAn bits */

	ECanaRegs.CANRMP.all = 0xFFFFFFFF;	/* Clear all RMPn bits */

	ECanaRegs.CANGIF0.all = 0xFFFFFFFF;	/* Clear all interrupt flag bits */
	ECanaRegs.CANGIF1.all = 0xFFFFFFFF;

/* Configure bit timing parameters for eCANA*/

	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    // Wait until the CPU has been granted permission to change the configuration registers
    do
    {
      ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 1 );  		// Wait for CCE bit to be set..

   	ECanaShadow.CANBTC.all = 0;
    /* The following block is only for 60 MHz SYSCLKOUT. (30 MHz CAN module clock Bit rate = 1 Mbps
       See Note at end of file. */

    ECanaShadow.CANBTC.bit.BRPREG = 11;           //2 为1M            5为500K           11为250k
    ECanaShadow.CANBTC.bit.TSEG2REG = 1;
    ECanaShadow.CANBTC.bit.TSEG1REG = 6;

    ECanaShadow.CANBTC.bit.SAM = 1;
    ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

//	ECanaShadow.CANOPC.all = ECanaRegs.CANOPC.all;
//    ECanaShadow.CANOPC.bit.OPC17 = 1;
//	ECanaShadow.CANOPC.bit.OPC16 = 1;
//    ECanaRegs.CANOPC.all = ECanaShadow.CANOPC.all;

    // Wait until the CPU no longer has permission to change the configuration registers
    do
    {
      ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 0 ); 		// Wait for CCE bit to be  cleared..

/* Disable all Mailboxes  */
 	ECanaRegs.CANME.all = 0;		// Required before writing the MSGIDs

  //--------------------------msgBox0----------------------------------    
    ECanaMboxes.MBOX0.MSGID.bit.AME = 1;    //接收屏蔽使能位  0:所有标识符必须匹配以接收消息  1:所有都接收
    ECanaMboxes.MBOX0.MSGID.bit.AAM = 0;    //自动应答模式位  仅对配置为发送的邮箱有效
    
    ECanaMboxes.MBOX0.MSGID.bit.IDE = 1;    //0:标准帧   1:扩展帧
    ECanaMboxes.MBOX0.MSGID.all = 0x98FF00A1;  //只接收主板发的数据(001 **** ****b)


    //--------------------------msgBox1----------------------------------    

    //--------------------------msgBox2----------------------------------    
    ECanaMboxes.MBOX2.MSGID.bit.AME = 1;    //接收屏蔽使能位  0:所有标识符必须匹配以接收消息  1:所有都接收
    ECanaMboxes.MBOX2.MSGID.bit.AAM = 0;    //自动应答模式位  仅对配置为发送的邮箱有效
    
    ECanaMboxes.MBOX2.MSGID.bit.IDE = 1;    //0:标准帧   1:扩展帧
//    ECanaMboxes.MBOX2.MSGID.all = 0x98A7E3D3;  //只接收主板发的数据(011 0000 0100b)
    ECanaMboxes.MBOX2.MSGID.all = 0x9827D9D0;  //只接收主板发的数据(001 **** ****b)
    
	//---------------------------------------------------------------
	//中断屏蔽配置
	ECanaShadow.CANMIM.all = ECanaRegs.CANMIM.all;
	ECanaShadow.CANMIM.bit.MIM0 = 0;
  	ECanaShadow.CANMIM.bit.MIM2 = 0;
	ECanaRegs.CANMIM.all = ECanaShadow.CANMIM.all;

	//中断等级配置
	ECanaShadow.CANMIL.all = ECanaRegs.CANMIL.all;
	ECanaShadow.CANMIL.bit.MIL0 = 1;
  	ECanaShadow.CANMIL.bit.MIL2 = 1;
	ECanaRegs.CANMIL.all = ECanaShadow.CANMIL.all;

	//设置邮箱为接收邮箱
  	ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
  	ECanaShadow.CANMD.bit.MD0 = 1;				//接收邮箱
  	ECanaShadow.CANMD.bit.MD2 = 1; 				//接收邮箱
  	ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;

  	//使能邮箱
	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME0 = 1;  				//激活邮箱
  	ECanaShadow.CANME.bit.ME2 = 1;					//激活邮箱
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;
	
	//---------------------
	//CAN全局中断
	ECanaShadow.CANGIM.all = ECanaRegs.CANGIM.all;
	ECanaShadow.CANGIM.bit.I1EN = 0;
	ECanaRegs.CANGIM.all = ECanaShadow.CANGIM.all;


  //---------------------------msgBox8---------------------------
	//配置为发送
	ECanaRegs.CANTRS.bit.TRS8 = 0;

	ECanaMboxes.MBOX8.MSGID.all = 0;
	ECanaMboxes.MBOX8.MSGID.bit.AME = 0;
	ECanaMboxes.MBOX8.MSGID.bit.AAM = 0;
	ECanaMboxes.MBOX8.MSGID.all = 0x9806F4E5;
	
	ECanaMboxes.MBOX8.MSGCTRL.all = 0;
	ECanaMboxes.MBOX8.MSGCTRL.bit.RTR = 0;			//没有远程帧请求
	ECanaMboxes.MBOX8.MSGCTRL.bit.TPL = 8;

	ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
  	ECanaShadow.CANMD.bit.MD8 = 0;                  //设置为发送邮箱                    
  	ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;

	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME8 = 1;  				//使能邮箱8
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;

	//---------------------------msgBox9---------------------------
	//配置为发送
	ECanaRegs.CANTRS.bit.TRS9 = 0;

	ECanaMboxes.MBOX9.MSGID.all = 0;
	ECanaMboxes.MBOX9.MSGID.bit.AME = 0;
	ECanaMboxes.MBOX9.MSGID.bit.AAM = 0;
	ECanaMboxes.MBOX9.MSGID.all = 0x98FF50E5;
	
	ECanaMboxes.MBOX9.MSGCTRL.all = 0;
	ECanaMboxes.MBOX9.MSGCTRL.bit.RTR = 0;			//没有远程帧请求
	ECanaMboxes.MBOX9.MSGCTRL.bit.TPL = 9;

	ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
    ECanaShadow.CANMD.bit.MD9 = 0;                  //设置为发送邮箱                    
    ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;

	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME9 = 1;  				//使能邮箱9
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;
  //---------------------------msgBox10---------------------------
	//配置为发送

    EDIS;
}

//---------------------------------------------------------------------------
// Example: InitECanGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as eCAN pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
// Caution:
// Only one GPIO pin should be enabled for CANTXA operation.
// Only one GPIO pin shoudl be enabled for CANRXA operation.
// Comment out other unwanted lines.

void InitECanGpio(void)
{
   InitECanaGpio();
}

void InitECanaGpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected CAN pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;	    // Enable pull-up for GPIO30 (CANRXA)
 	GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;	    // Enable pull-up for GPIO31 (CANTXA)

/* Set qualification for selected CAN pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;   // Asynch qual for GPIO30 (CANRXA)

/* Configure eCAN-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAN functional pins.

    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;	// Configure GPIO30 for CANRXA operation
 	GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;	// Configure GPIO31 for CANTXA operation

    EDIS;
}

/* Note: Bit timing parameters must be chosen based on the network parameters such as
   the sampling point desired and the propagation delay of the network. The propagation
   delay is a function of length of the cable, delay introduced by the
   transceivers and opto/galvanic-isolators (if any).

   The parameters used in this file must be changed taking into account the above mentioned
   factors in order to arrive at the bit-timing parameters suitable for a network.
*/




void CanErrorHandle(void)
{
  struct ECAN_REGS ECanaShadow;

  ECanaShadow.CANES.all = ECanaRegs.CANES.all;
  if (ECanaShadow.CANES.bit.BO | ECanaShadow.CANES.bit.EP | ECanaShadow.CANES.bit.EW)
  {
    InitECana();
  }
}

//===========================================================================
// End of file.
//===========================================================================



