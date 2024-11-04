// TI File $Revision: /main/1 $
// Checkin $Date: December 5, 2008   18:01:01 $
//###########################################################################
//
// FILE:	DSP2803x_Gpio.c
//
// TITLE:	DSP2803x General Purpose I/O Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2803x C/C++ Header Files V1.10 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "DSP2803x_Device.h"     // DSP2803x Headerfile Include File
#include "DSP2803x_Examples.h"   // DSP2803x Examples Include File

//---------------------------------------------------------------------------
// InitGpio:
//---------------------------------------------------------------------------
// This function initializes the Gpio to a known (default) state.
//
// For more details on configuring GPIO's as peripheral functions,
// refer to the individual peripheral examples and/or GPIO setup example.
void InitGpio(void)
{
   EALLOW;

   // Each GPIO pin can be:
   // a) a GPIO input/output
   // b) peripheral function 1
   // c) peripheral function 2
   // d) peripheral function 3
   // By default, all are GPIO Inputs
   GpioCtrlRegs.GPAMUX1.all = 0x0000;     // GPIO functionality GPIO0-GPIO15
   GpioCtrlRegs.GPAMUX2.all = 0x0000;     // GPIO functionality GPIO16-GPIO31
   GpioCtrlRegs.GPBMUX1.all = 0x0000;     // GPIO functionality GPIO32-GPIO44
   GpioCtrlRegs.AIOMUX1.all = 0x0000;     // Dig.IO funct. applies to AIO2,4,6,10,12,14

   GpioCtrlRegs.GPADIR.all = 0x0000;      // GPIO0-GPIO31 are GP inputs
   GpioCtrlRegs.GPBDIR.all = 0x0000;      // GPIO32-GPIO44 are inputs
   GpioCtrlRegs.AIODIR.all = 0x0000;      // AIO2,4,6,19,12,14 are digital inputs

   // Each input can have different qualification
   // a) input synchronized to SYSCLKOUT
   // b) input qualified by a sampling window
   // c) input sent asynchronously (valid for peripheral inputs only)
   GpioCtrlRegs.GPAQSEL1.all = 0x0000;    // GPIO0-GPIO15 Synch to SYSCLKOUT
   GpioCtrlRegs.GPAQSEL2.all = 0x0000;    // GPIO16-GPIO31 Synch to SYSCLKOUT
   GpioCtrlRegs.GPBQSEL1.all = 0x0000;    // GPIO32-GPIO44 Synch to SYSCLKOUT

   // Pull-ups can be enabled or disabled.
   GpioCtrlRegs.GPAPUD.all = 0x0000;      // Pullup's enabled GPIO0-GPIO31
   GpioCtrlRegs.GPBPUD.all = 0x0000;      // Pullup's enabled GPIO32-GPIO44
   //GpioCtrlRegs.GPAPUD.all = 0xFFFF;    // Pullup's disabled GPIO0-GPIO31
   //GpioCtrlRegs.GPBPUD.all = 0xFFFF;    // Pullup's disabled GPIO32-GPIO44
   EDIS;
/*
   EALLOW;	  
// ����E_PEM����Ϊ��������ڿ���PWM�����   
   GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;    // ��ʹ��GPIO33���ŵ���������
   GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;   // ����GPIO33����Ϊһ��IO��
   GpioCtrlRegs.GPBDIR.bit.GPIO33  = 1;   // ����GPIO33����Ϊ�������


// ��������GPIO6�����ȿ���   
   GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // ��ʹ��GPIO5���ŵ���������
   GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;   // ����GPIO5����Ϊһ��IO��
   GpioCtrlRegs.GPADIR.bit.GPIO6  = 1;    // ����GPIO5����Ϊ�������
   GpioDataRegs.GPADAT.bit.GPIO6  =1;
// ����̵�������GPIO19
   GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;    // ��ʹ��GPIO5���ŵ���������
   GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;   // ����GPIO5����Ϊһ��IO��
   GpioCtrlRegs.GPADIR.bit.GPIO19  = 1;   // ����GPIO5����Ϊ�������
   GpioDataRegs.GPADAT.bit.GPIO19  =0;    // ��ʼ�������Ϊ0
// ��ɫָʾ�ƿ���GPIO24
   GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;    // ��ʹ��GPIO5���ŵ���������
   GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;   // ����GPIO5����Ϊһ��IO��
   GpioCtrlRegs.GPADIR.bit.GPIO24  = 1;   // ����GPIO5����Ϊ�������
   GpioDataRegs.GPADAT.bit.GPIO24  =0;    // ��ʼ�������Ϊ0
// �������ܱ�־����GPIO11
   GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;    // ��ʹ��GPIO5���ŵ���������
   GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;   // ����GPIO5����Ϊһ��IO��
   GpioCtrlRegs.GPADIR.bit.GPIO20  = 1;   // ����GPIO5����Ϊ�������
   GpioDataRegs.GPADAT.bit.GPIO20  =0;     // ��ʼ�������Ϊ0


   // ͨѶ״̬ ָʾ�� �̵�
   GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;    // ��ʹ��GPIO5���ŵ���������
   GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;   // ����GPIO5����Ϊһ��IO��
   GpioCtrlRegs.GPADIR.bit.GPIO17  = 1;   // ����GPIO5����Ϊ�������
   GpioDataRegs.GPADAT.bit.GPIO17  =0;     // ��ʼ�������Ϊ0
   // ���
   GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;    // ��ʹ��GPIO5���ŵ���������
   GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;   // ����GPIO5����Ϊһ��IO��
   GpioCtrlRegs.GPADIR.bit.GPIO8  = 1;   // ����GPIO5����Ϊ�������
   GpioDataRegs.GPADAT.bit.GPIO8  =0;     // ��ʼ�������Ϊ0
   // �Ƶ�
   GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;    // ��ʹ��GPIO5���ŵ���������
   GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;   // ����GPIO5����Ϊһ��IO��
   GpioCtrlRegs.GPADIR.bit.GPIO16  = 1;   // ����GPIO5����Ϊ�������
   GpioDataRegs.GPADAT.bit.GPIO16  =0;     // ��ʼ�������Ϊ0

   //����������  
   //�����ѹ��ѹ����
   GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;    // ��ʹ��GPIO5���ŵ���������
   GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;   // ����GPIO5����Ϊһ��IO��
   GpioCtrlRegs.GPADIR.bit.GPIO9  = 0;   // ����GPIO5����Ϊ��������
   //��ؽ���ͷ��ӱ���
   GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // ��ʹ��GPIO5���ŵ���������
   GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;   // ����GPIO5����Ϊһ��IO��
   GpioCtrlRegs.GPADIR.bit.GPIO28  = 0;   // ����GPIO5����Ϊ��������
  //���عܹ�������
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // ��ʹ��GPIO5���ŵ���������
   GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;   // ����GPIO5����Ϊһ��IO��
   GpioCtrlRegs.GPADIR.bit.GPIO29  = 0;   // ����GPIO5����Ϊ��������

  //�����ʱ�������
   GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;    // ʹ��GPIO33���ŵ���������
   GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;   // ����GPIO33����Ϊһ��IO��
   GpioCtrlRegs.GPBDIR.bit.GPIO33  = 0;   // ����GPIO33����Ϊ��������
// ������Ƿѹ��������   
   GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;    // ʹ��GPIO33���ŵ���������
   GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;   // ����GPIO33����Ϊһ��IO��
   GpioCtrlRegs.GPBDIR.bit.GPIO32  = 0;   // ����GPIO33����Ϊ��������


   EDIS;
   */
}

//===========================================================================
// End of file.
//===========================================================================
