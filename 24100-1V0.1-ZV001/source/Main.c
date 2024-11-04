/************************************************************************
ProjectName： 24100-1V0.1-ZV001
Author     ： WH
Version    ： ZV001
Date       ： 2024-10-25

************************************************************************/

#define GLOBAL_Q 24
#include "DSP2803x_Device.h"       // Headerfile Include File
#include "DSP2803x_Examples.h"     // Examples Include File
#include "math.h"

unsigned int Iref = 2900;           //电流初始参数
unsigned int Vref = 15000;          //电压初始参数
unsigned int T_Imax = 1000;         //温度限电流参数
unsigned int P_Imax = 1000;         //温度限电流参数
unsigned int PowerIn_Imax = 1000;   //温度限电流参数
unsigned int Imax = 1000;           //最大电流限制参数


unsigned int  PowerIn_Grade_1_Cnt = 0;    //交流限参数档位1计数
unsigned int  PowerIn_Grade_2_Cnt = 0;    //交流限参数档位2计数
unsigned int  PowerIn_Grade_3_Cnt = 0;    //交流限参数档位3计数
unsigned int  PowerIn_Grade_4_Cnt = 0;    //交流限参数档位4计数

unsigned char Output_Data[11];              //输出参数数组
unsigned char uc_Start_Demarcate_Flg = 0;   //标定参数命令标志


struct uiBITS{
    unsigned int B0:1;
    unsigned int B1:1;
    unsigned int B2:1;
    unsigned int B3:1;
    unsigned int B4:1;
    unsigned int B5:1;
    unsigned int B6:1;
    unsigned int B7:1;

    unsigned int B8:1;
    unsigned int B9:1;
    unsigned int B10:1;
    unsigned int B11:1;
    unsigned int B12:1;
    unsigned int B13:1;
    unsigned int B14:1;
    unsigned int B15:1;
   };
union FLAG_BITS{
    struct uiBITS bit;
    unsigned int all;
};

union FLAG_BITS RX1flag;                   //串口通信接收标志结构体
union FLAG_BITS DcdcStatus;                //充电机状态标志结构体
union FLAG_BITS CANStatus;                 //CAN反馈状态结构体

myCanMsg_t canRxMsg,canTxMsg;

//定时器计时对下列标志位置位
unsigned int uiIoSam;                    //状态处理标志
unsigned int uiAdcSam;                   //ADC参数处理标志
unsigned int uiSciSam;                   //串口参数处理标志



unsigned long int ledcnt;                 //LED灯状态闪烁变换计数
unsigned char uc_SCI_Err = 0;             //串口通信故障计数超时计数
unsigned char uc_SCI_Err_Flg = 0;         //串口通信故障吃超时标志־
unsigned char uc_SCI_Err_Cnt = 0;         //串口通信超时次数计时  三次后不再恢复

unsigned char uc_Temperature_Err_Flg = 1;  //内部温度探头短路或者未连接故障标志位
unsigned char uc_Temperature_Err_Cnt = 0;  //内部温度探头短路或者未连接故障检测计数
unsigned char uc_OVP_Cnt = 0;              //输出过压故障检测计数
unsigned char uc_UVP_Cnt = 0;              //输出欠压故障检测计数
unsigned char uc_OCP_Cnt = 0;              //输出过流故障检测计数
unsigned char uc_OTP_Cnt = 0;              //内部过温故障检测计数
unsigned char uc_Power_In_Err_Cnt = 0;     //输入电压故障检测计数
unsigned char uc_OTP_Flg = 0;              //内部过温故障标志位
unsigned char uc_Err_Led = 0;              //故障LED灯变量
unsigned char ucShowErroLedTimeCont = 0;   //故障LED灯显示次数计数
unsigned int ui_V_Power_In = 0;            //输入电压参数
unsigned char uc_100mS_Flg=0;              //100ms定时标志
unsigned char uc_10S_Flg=0;                //10S定时标志
unsigned char uc_Extern_Temperaure_Err_Cnt = 0;      //外部温度故障计数（短路或者未连接）
unsigned char uc_Extern_Temperaure_Recover_Cnt = 0;  //外部温度恢复正常计数
unsigned char uc_Extern_Temperaure_Err_Flg = 0;      //外部温度故障标志位

unsigned int CMPR_Softstart=0;             //软启动参数
unsigned int uiBaseTimer=0;                //定时器计数参数
unsigned int uiSSTimer=0;                  //输出阶段启动延时
unsigned int uiActionReady=0;              //输出控制阶段控制

unsigned int uiSciSendnum=0;               //数据发送数量计数   发送完再进行数据更新
unsigned int uiSciGetnum=0;                //收到串口数据计数
unsigned int uiSciGetsum=0;                //接收查询定时时间计数
unsigned int uiSciGetOK=0;                 //接收查询定时标志


unsigned char ucTxbuffer[14] = {0,0,0,0,0,0,0,0,0,0,0};     //发送数据数组
unsigned char RX1Data[14] = {0,0,0,0,0,0,0,0,0,0,0};        //串口接收数据数组
unsigned char PFCData[11] = {0,0,0,0,0,0,0,0,0,0,0};        //PFC数据数组
unsigned char CANData[8] = {0,0,0,0,0,0,0,0};               //CAN通讯数组
unsigned int uiRxVset;                                      //电压设置参数
unsigned int uiRxIset;                                      //电流设置参数
unsigned int uiRxVset_Coef_Value;                           //比例运算后电压设置参数
unsigned int uiRxIset_Coef_Value;                           //比例运算后电流设置参数
unsigned int Vset;                                        //CAN请求电压设置参数
unsigned int Iset;                                         //CAN请求电流设置参数

long lAdcVoSum[41];                           //输出电压采集多次数据数组
long lAdcIoSum[41];                           //输出电流采集多次数据数组
long lAdcVbatSum[41];                         //电池电压采集多次数据数组
long lAdcTdspSum[41];                         //DSP温度采集多次数据数组
long lAdcExTSum[41];                          //外部温度采集多次数据数组
long lAdcTp1Sum[41];                          //内部温度1采集多次数据数组
long lAdcTp2Sum[41];                          //内部温度2采集多次数据数组
long lAdcTp3Sum[41];                          //内部温度3采集多次数据数组
unsigned int  uiAdcVo;                        //输出电压计算后数据
unsigned int  uiAdcIo;                        //输出电流计算后数据
unsigned int  uiAdcVbat;                      //电池电压计算后数据
unsigned int  ui_Old_AdcVbat;                 //上一次电池电压计算后数据   采用数据滤波方式  3/4 + 1/4
int  iAdcTdsp;                                //DSP温度计算后数据  可以是负数
unsigned int  uiAdcTdsp;                      //DSP温度计算后数据  iAdcTdsp +1000后数据
unsigned int  uiAdcTp1;                       //内部温度1计算后数据
unsigned int  uiAdcTp2;                       //内部温度2计算后数据
unsigned int  uiAdcTp3;                       //内部温度3计算后数据
unsigned int  uiAdcExT;                       //外部温度计算后数据
unsigned int  uiAdcNum=0;                     //ADC多次采样数据计数
long lAdcVo;                                  //输出电压多次采样计算数据
long lAdcIo;                                  //输出电流多次采样计算数据
long lAdcVbat;                                //电池电压多次采样计算数据
long lAdcTdsp;                                //DSP温度多次采样计算数据
long lAdcTp1;                                 //内部温度1多次采样计算数据
long lAdcTp2;                                 //内部温度2多次采样计算数据
long lAdcTp3;                                 //内部温度3多次采样计算数据
long lAdcExT;                                 //外部温度多次采样计算数据

unsigned int CANONOFF=0;                      //CAN通讯使能和停止输出标志
unsigned int CAN_bus_empty=0;                 //在接收数据时不发送数据
unsigned int uiCANTxTimer=0;                  //CAN通讯定时发送标志
unsigned int uiCanFailTime=0;                 //通讯超时计数

void main(void)

{

   InitSysCtrl();                            // 系统时钟初始化

   InitGpio();                               //GPIO初始化

   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);     //将程序复制到RAM运行   提高代码运行速度
   InitFlash();                                                          //Flash初始化

   DINT;                                                                 //关闭中断

   InitPieCtrl();                                                        //外设初始化

   IER = 0x0000;                                                         //中断使能寄存器清零
   IFR = 0x0000;                                                         //中断标志寄存器清零

   InitPieVectTable();                                                   //初始化中断向量表

   EALLOW;                                                               //解除寄存器保护
   PieVectTable.XINT1 = &xint1_isr;                                      //IO中断配置   PFC使能LLC输出
   PieVectTable.EPWM1_INT = &dcdc_isr;                                   //PWM中断配置
   EDIS;                                                                 //恢复寄存器保护

   IER |= M_INT1;                                                        //中断使能寄存器配置
   IER |= M_INT3;                                                        //中断标志寄存器配置

// Enable EPWM1
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;                                   // Enable the PIE block
   PieCtrlRegs.PIEIER1.bit.INTx4 = 1;                                   // Enable PIE Gropu 1 INT4 =XINT1

   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;                                   //PFC使能DSP输出中断


   EALLOW;                                                              //解除寄存器保护
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;                               //关闭PWM的时基时钟
   EDIS;                                                                //恢复寄存器保护

   InitCpuTimers();                                                     //定时器0初始化
   InitAdc();                                                           //ADC初始化
   InitEPwm();                                                          //PWM初始化
   InitSci();                                                           //串口通信初始化
   InitECan();                                                          //CAN通讯初始化

   EALLOW;                                              //解除寄存器保护
   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;                  //Configure GPIO2 as OUTPUT LED-RED
   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;                  //Configure GPIO3 as OUTPUT LED-GREE
   GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;                  // Configure GPIO0 as EPWM3A
   GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;                  // Configure GPIO1 as EPWM3B

   GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;                 // Configure GPIO28 as SCI_RX

   GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;                   //Configure GPIO2 as OUTPUT LED-RED
   GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;                   //Configure GPIO3 as OUTPUT LED-GREE
   GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;                  //Configure GPIO16 as OUTPUT DCRLY

   GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 0;                 // XINT1 Synch to SYSCLKOUT only
   GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 6;            // XINT1 is GPIO6

   EDIS;                                                //恢复寄存器保护

   XIntruptRegs.XINT1CR.bit.POLARITY = 1;               // up edge interrupt

   GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;                //输出继电器控制

   dcdcloop_init();                                     //电压电流输出环路参数初始化

   EALLOW;                                              //解除寄存器保护
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;               //关闭PWM的时基时钟
   EDIS;

   EINT;                                                //使能全局中断
   ERTM;                                                //使能调试事件

   DcdcStatus.all = 0;                                  //状态位清零

   while(1)
   {
       asm(" NOP");                                     //空函数
       ServiceDog();                                    //喂狗
       MTimer0();                                       //定时器0循环处理
       IoChuli();                                       //系统运行状态处理
       AdcChuli();                                      //ADC参数处理
       GetSciData();                                    //获取串口数据
       SendSciData();                                   //发送串口数据

       if(MsgRx(&canRxMsg))                             //判断是否有CAN数据
         {
            CanMsgHandle();                             //CAN数据解析处理
         }
       if(uiCANTxTimer==1)                              //1S钟发送一次
         {
            CanInfoTx();                                //CAN发送函数
            uiCANTxTimer=0;
         }
   }

}

//###########################################################################
//                             CPU��ʱ��0����
//###########################################################################
void MTimer0(void)
{
    if (CpuTimer0Regs.TCR.bit.TIF)                   //5mS中断一次
    {
        CpuTimer0Regs.TCR.bit.TIF = 1;
        uiBaseTimer++;
        uiBaseTimer %=60000;
        ServiceDog();                                //喂狗

        uiSciGetsum++;                               //获取串口数据计数，60ms查询一次
       if(uiSciGetsum >=11)
          {
           uiSciGetsum = 0;
           uiSciGetOK = 1;                          //串口查询标志位使能
          }

        if ((uiBaseTimer)%4 ==3)                   //20ms处理一下系统状态及ADC采样参数
          {
          uiIoSam = 1;
          uiAdcSam = 1;
          }

        if ((uiBaseTimer)%20 ==19)                 //串口 通信超时计数  100ms加1   连续60S收不到后报错
           {
             uiSciSam = 1;
             uc_SCI_Err++;
             if(uc_SCI_Err > 600)
               {
               uc_SCI_Err = 600;
               }
           }
        if((uiBaseTimer%200)==199)
              {
               uiCANTxTimer = 1;
              }
        if ((uiBaseTimer)%2000 ==1999)            //10S定时标志位
           {
            uc_10S_Flg = 1;
           }

       if((uiBaseTimer%20)==19)
          {
           uc_100mS_Flg = 1;                                       //100mS定时标志位置位
             if(uc_Temperature_Err_Flg == 1)                       //第一次商店检测内部温度检测探头是否短路或者断路
               {
                if ((uiAdcTp1 >54)&&(uiAdcTp1 < 3379) &&(uiAdcTp2 > 54) &&(uiAdcTp2 < 3379))       //短路或断路检测
                  {
                   if((uiAdcTp1 >122)&&(uiAdcTp1 < 3175) &&(uiAdcTp2 > 122) &&(uiAdcTp2 < 3175))   //温度过低或者过高检测
                   {
                   uc_Temperature_Err_Cnt++;
                   }
                  }
                else
                  {
                  uc_Temperature_Err_Cnt = 0;
                  }
                if(uc_Temperature_Err_Cnt > 10)                                                    //连续判断10次后   0为正常  1为异常
                  {
                  uc_Temperature_Err_Flg = 0;
                  }
                else
                  {
                  uc_Temperature_Err_Flg = 1;
                  }
               }

             if(uiAdcExT > 2434)                                                                   //外部温度探头检测外部温度异常告警
               {
               uc_Extern_Temperaure_Err_Cnt++;
               }
             else
               {
               uc_Extern_Temperaure_Err_Cnt = 0;
               }
             if(uc_Extern_Temperaure_Err_Cnt >= 100)                                              //连续判断100次
               {
               uc_Extern_Temperaure_Err_Cnt = 100;
               uc_Extern_Temperaure_Err_Flg = 1;
               }


             if(uc_Extern_Temperaure_Err_Flg == 1)                                               //外部温度探头检测外部温度异常告警后恢复检测
               {
               if(uiAdcExT < 1855)
                 {
                 uc_Extern_Temperaure_Recover_Cnt++;
                 }
               else
                 {
                 uc_Extern_Temperaure_Recover_Cnt = 0;
                 }
               if(uc_Extern_Temperaure_Recover_Cnt >= 100)                                      //连续判断100次
                 {
                 uc_Extern_Temperaure_Recover_Cnt = 0;
                 uc_Extern_Temperaure_Err_Flg = 0;
                 }
               }

          }

       if ((uiBaseTimer)%100 ==99)                                                              //LED指示灯  每0.5s处理一次
           {
           ShowChargeProcessLed();
           }

    ///////////////////////////////////DCDC输出过程处理/////////////////////////////////////////
       if((uiBaseTimer>=200)&&(uiActionReady==0))                              //收到PFC使能输出信号且上电至少1S后进行处理
         {
          if ((uc_Temperature_Err_Flg == 0)||(uc_Start_Demarcate_Flg))         //内部温度探头连接正常  或者是标定模式  允许进行输出
             {
              EALLOW;
              GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;                   // Configure GPIO2 as EPWM3A
              GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;                   // Configure GPIO3 as EPWM3B
              EDIS;
              EPwm3Regs.AQCSFRC.all = 0x05;                         //关闭PWM输出
              uiSSTimer = 0;                                        //输出过程延时1S
              CMPR_Softstart = 0;                                   //软起参数设置为0
              uiActionReady = 1;                                    //进入阶段1
             }
         }
        if(uiActionReady==1)                                        //阶段1处理
          {
           uiSSTimer++;
           if (uiSSTimer>=200)                                      //输出过程延时1S
              {
              uiSSTimer = 0;
              uiActionReady = 2;                                    //进入阶段2
              }
          }
        if (uiActionReady==2)                                       //阶段2处理
           {
            GpioDataRegs.GPASET.bit.GPIO16 = 1;                   //使能输出继电器
            Dcdcisr.ui_Dcdc_Volt_Ref = Vref;                       //设置初始电压参数
            Dcdcisr.ui_Dcdc_Curr_Ref = 1000;                       //设置初始电流参数
            CMPR_Softstart++;                                      //缓启动参数递增
            EPwm3Regs.AQCSFRC.all = 0x0;                            //使能PWM3A,PWM3B输出
            if (CMPR_Softstart>=40)                                 //唤起参数达到40后使能GPIO输出PWM
               {
               EALLOW;
               GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;                  // Configure GPIO6 as EPWM3A
               GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;                  // Configure GPIO7 as EPWM3B
               EDIS;
               }
            if (CMPR_Softstart>=TMAX)                              //缓起参数达到最大，即输出频率达到最低时进入阶段3
               {
               uiActionReady = 3;
               CMPR_Softstart = TMAX;                              //缓起参数设置为最大
               }
            Dcdcisr.ui_Dcdc_Duty_Ramp = (unsigned int)CMPR_Softstart<<6;   //设置PWM 的占空比
          }
        if(uiActionReady==3)                                       //阶段3   输出过程中电压电流调整
          {
            if(uc_Start_Demarcate_Flg)                             //标定模式
            {
            if (Dcdcisr.ui_Dcdc_Curr_Ref <= (Iref -8))             //电流输出增大
               {
               Dcdcisr.ui_Dcdc_Curr_Ref += 8;
               if (Dcdcisr.ui_Dcdc_Curr_Ref >= Iref)
                  {
                  Dcdcisr.ui_Dcdc_Curr_Ref = Iref;
                  }
               }
            if (Dcdcisr.ui_Dcdc_Curr_Ref >= (Iref +8))            //电流输出减小
               {
               Dcdcisr.ui_Dcdc_Curr_Ref -= 8;
               if (Dcdcisr.ui_Dcdc_Curr_Ref <= Iref)
                  {
                  Dcdcisr.ui_Dcdc_Curr_Ref = Iref;
                  }
               }

             if(uiRxVset >25000)        //电压电流输出最大限制
               uiRxVset = 25000;
             if(uiRxVset <12000)
               uiRxVset = 12000;
             if(uiRxIset >10000)
               uiRxIset = 10000;
             if(uiRxIset <600)
               uiRxIset = 600;
             Dcdcisr.ui_Dcdc_Volt_Ref = uiRxVset;                //电压输出直接给定  不需要缓慢上升或者下降
             Iref = uiRxIset;                                    //设定电流参数
            }
           else                                                  //CAN通讯输出或者曲线运行
            {
            if (Dcdcisr.ui_Dcdc_Curr_Ref <= (Iref -8))           //电流增大
               {
               Dcdcisr.ui_Dcdc_Curr_Ref += 8;
               if (Dcdcisr.ui_Dcdc_Curr_Ref >= Iref)
                  {
                  Dcdcisr.ui_Dcdc_Curr_Ref = Iref;
                  }
               }
            if (Dcdcisr.ui_Dcdc_Curr_Ref >= (Iref +8))          //电流减小
                {
                Dcdcisr.ui_Dcdc_Curr_Ref -= 8;
                if (Dcdcisr.ui_Dcdc_Curr_Ref <= Iref)
                   {
                   Dcdcisr.ui_Dcdc_Curr_Ref = Iref;
                   }
                }


            if(uiRxVset >25000)                                //电压电流输出最大限制
              uiRxVset = 25000;
            if(uiRxVset <12000)
              uiRxVset = 12000;
            if(uiRxIset >10000)
              uiRxIset = 10000;
            if(uiRxIset <600)
              uiRxIset = 600;

            Dcdcisr.ui_Dcdc_Volt_Ref = uiRxVset;              //电压参数直接设置
            Iref = uiRxIset;                                  //设定电流参数

             }
          }

    }
}

///////////////////////////////////////////////////////////////////////////
////运行状态处理
////////////////////////////////////////////////////////////////////////////
void IoChuli(void)
{
  if(uiIoSam)    //定时处理标志
  {
    uiIoSam = 0;
    ServiceDog();   //喂狗
    ////////////////内外部温度探头异常状态处理/////////////////////////
     if((uc_Temperature_Err_Flg == 1)||(uc_Extern_Temperaure_Err_Flg==1))
        {
        DcdcStatus.bit.B0 = 1;
        }
     else
        {
        DcdcStatus.bit.B0 = 0;
        }

    ///////////////串口通信超时处理/////////////////////////
    if(uc_SCI_Err_Cnt < 3)          //最多三次后不恢复
    {
       if(uc_SCI_Err >= 600)       //单次60S超时检测
          {
          DcdcStatus.bit.B1 = 1;
          uc_SCI_Err_Flg = 1;
          }
       else
          {
            DcdcStatus.bit.B1 = 0;
          }
    }
    else
    {
    DcdcStatus.bit.B1 = 1;
    uc_SCI_Err_Flg = 1;
    }
    if((DcdcStatus.bit.B1 == 0)&&(uc_SCI_Err_Flg == 1))   //超时保护计数
      {
        uc_SCI_Err_Cnt++;
        if(uc_SCI_Err_Cnt >= 3 )
          {
            uc_SCI_Err_Cnt = 3;
           }
        uc_SCI_Err_Flg = 0;
      }
  }
}

///////////////充电指示灯状态处理//////////////////////
void ShowChargeProcessLed(void)
{
    ledcnt++;
    ledcnt %= 10000;

    if((DcdcStatus.all)||(PFCData[9]&0x08))
      {
        if(DcdcStatus.bit.B1)                  //DSP接收PFC串口通信超时状态显示
           {
            uc_Err_Led = 11;
           }
        else if(PFCData[9]&0x08)              //PFC接收DSP串口通信超时状态显示
           {
            uc_Err_Led = 12;
           }
        else
           {
           uc_Err_Led = 0;
           }

       //故障灯LED状态指示
       ucShowErroLedTimeCont++;
       ucShowErroLedTimeCont %= 100;
       if(ucShowErroLedTimeCont <= ( uc_Err_Led<< 1))         //闪烁次数计数   亮一次灭一次
         {
          if( ledcnt % 2 == 0)
            {
            C_DisOutStateGreenLed();
            C_EnOutStateRedLed();
            }
          else
            {
            C_DisOutStateGreenLed();
            C_DisOutStateRedLed();
            }
         }
       else                                                  //闪烁一定次数后 停一下  再进行循环
         {
          C_DisOutStateGreenLed();
          C_DisOutStateRedLed();
         }
       if(ucShowErroLedTimeCont > ((uc_Err_Led << 1) + 1))   //闪烁一定次数后 停一下  再进行循环
         {
         ucShowErroLedTimeCont = 0;
         }
      }
     else                                           //正常运行阶段指示灯  盲充阶段及CAN控制指示灯状态
      {
        uc_Err_Led = 0;
        if(0)                                    //充电中是红灯闪烁
          {
              if( ledcnt % 2 == 0)
               {
                C_DisOutStateGreenLed();
                C_EnOutStateRedLed();
               }
               else
               {
               C_DisOutStateGreenLed();
               C_DisOutStateRedLed();
               }
          }
        else if (0)               //充满为绿灯常亮
            {
            C_EnOutStateGreenLed();
            C_DisOutStateRedLed();
            }
        else                                   //待机为红绿交替闪烁
           {
           if ( ledcnt % 2 == 1)
              {
               C_DisOutStateGreenLed();
               C_EnOutStateRedLed();
              }
           else
              {
              C_EnOutStateGreenLed();
              C_DisOutStateRedLed();
              }
            }
       }

}
//////////////////////////////////////////////


 /////////////////////////////////////////////////////////
  /////串口通信数据发送//////
  /////////////////////////////////////////////////////////
void SendSciData(void)
{
  unsigned int a;
  if(uiSciSam)                                                //定时处理串口通信数据标志  100mS处理一次  发送数据到PFC板
  {
        if (SciaRegs.SCICTL2.all&0x80)                       //判断发送寄存器是都空闲
         {
            if ( uiSciSendnum>=14)                            //分14次发送14个数据  发送完后再更新
            {
                 uiSciSendnum = 0;
                 uiSciSam = 0;                               //  14个数据发送完后清零发送标志
                     ucTxbuffer[0] = 0xA5;
                     ucTxbuffer[1] = 0xA5;
                     ucTxbuffer[2] = 0x29;

                     a = uiAdcVbat >>8;                        //电池电压
                     ucTxbuffer[4] = (unsigned char)a;
                     a = uiAdcVbat & 0xff;
                     ucTxbuffer[3] = (unsigned char)a;

                     a = uiAdcIo >>8;                          //输出电流
                     ucTxbuffer[6] = (unsigned char)a;
                     a = uiAdcIo & 0xff;
                     ucTxbuffer[5] = (unsigned char)a;

                     a = uiAdcTp1 >>8;                        //内部温度1
                     ucTxbuffer[8] = (unsigned char)a;
                     a = uiAdcTp1 & 0xff;
                     ucTxbuffer[7] = (unsigned char)a;

                     a = uiAdcTp2 >>8;                       //内部温度2
                     ucTxbuffer[10] = (unsigned char)a;
                     a = uiAdcTp2 & 0xff;
                     ucTxbuffer[9] = (unsigned char)a;

                     a = DcdcStatus.all & 0xff;             //运行状态信息
                     ucTxbuffer[11] = (unsigned char)a;

                     a = ucTxbuffer[2] + ucTxbuffer[3] + ucTxbuffer[4] + ucTxbuffer[5] + ucTxbuffer[6]
                       + ucTxbuffer[7] + ucTxbuffer[8] + ucTxbuffer[9] + ucTxbuffer[10] + ucTxbuffer[11];          //求和校验
                     a = a & 0xff;
                     ucTxbuffer[12] = (unsigned char)a;

                     ucTxbuffer[13] = 0x55;
            }
            else
            {
                SciaRegs.SCITXBUF = ucTxbuffer[ uiSciSendnum];                    //每次发送一个字节数据
                uiSciSendnum++;
            }
      }
  }
}
  /////////////////////////////////////////////////////////
  /////SCI ���մ���//////
  /////////////////////////////////////////////////////////

void GetSciData(void)
 {
    unsigned char data,a;
    asm(" NOP");
   if(uiSciGetOK)                                              //定时读取串口通信数据标志
   {
     if(SciaRegs.SCIFFRX.bit.RXFFST >=1)                       //判断接收状态寄存器是否有数据
     {
      data = SciaRegs.SCIRXBUF.all;                            //读数据

      if(RX1flag.bit.B7 == 1)                                  //处理数据状态标志
        {
           RX1Data[uiSciGetnum] = data;                        //存储数据
           uiSciGetnum++;
           if(uiSciGetnum >=11)                                 //接收数据大于11字节后再处理
           {
               uiSciGetnum = 0;
               a = RX1Data[0] + RX1Data[1] + RX1Data[2] + RX1Data[3] + RX1Data[4]
                       + RX1Data[5] + RX1Data[6] + RX1Data[7] + RX1Data[8] + RX1Data[9];
               a = (a & 0xff);                                   //判断求和校验数据是否匹配
               if(RX1Data[10] == a)
                  {
                    uc_SCI_Err = 0;                              //清零串口通信故障标志
                    RX1flag.bit.B6 = 1;                          //接收状态
                    a = 0;
                    if(RX1Data[0] == 0x19)                       //判断接收数据标志
                    {
                        PFCData[0] = RX1Data[0];
                        PFCData[1] = RX1Data[1];
                        PFCData[2] = RX1Data[2];
                        PFCData[3] = RX1Data[3];
                        PFCData[4] = RX1Data[4];
                        PFCData[5] = RX1Data[5];
                        PFCData[6] = RX1Data[6];
                        PFCData[7] = RX1Data[7];
                        PFCData[8] = RX1Data[8];
                        PFCData[9] = RX1Data[9];
                        PFCData[10] = RX1Data[10];

                    }
                 }
               else
                  {
                   RX1flag.bit.B6 = 0;
                  }
               RX1flag.bit.B7 = 0;
           }
        }

        if(data == 0xa5)                        //接收数据帧头判断
        {
           if(RX1flag.bit.B1 == 1)             //接收到帧头的第二个A5数据
           {
              RX1flag.bit.B7 = 1;              //标明收到两个帧头数据 可以接收正常数据
              RX1flag.bit.B1 = 0;              //清除接收到第一个帧头A5数据标志
              uiSciGetnum = 0;                 //接收数据数据从0开始
           }

           if(RX1flag.bit.B0 == 1)             //接收到帧尾后的第一个A5数据
           {
               RX1flag.bit.B1 = 1;             //标明收到第一个A5帧头
               RX1flag.bit.B0 = 0;             //清零收到帧尾标志
           }

        }
        else
        {
           RX1flag.bit.B1 = 0;
        }
       if(data == 0x55)                         //接收到帧尾
            RX1flag.bit.B0 = 1;
    }
     if (SciaRegs.SCIRXST.bit.RXERROR)          //接受故障处理
        {
            SciaRegs.SCICCR.all = 0x0007;
            SciaRegs.SCICTL1.all = 0x0003;
            SciaRegs.SCICTL2.all = 0x0000;

            SciaRegs.SCIHBAUD = 0x0000;
            SciaRegs.SCILBAUD = 0x00c3;

            SciaRegs.SCIRXBUF.all= 0x00;
            SciaRegs.SCITXBUF = 0x00;
            SciaRegs.SCICTL1.all = 0x0023;
            uiSciGetnum = 0;                     //各标志位清零
            RX1flag.bit.B7 = 0;
            uiSciGetsum = 0;
            uiSciGetOK = 0;
       }

   }
 }

/////////////////CAN发送函数///////////////////
void CanInfoTx(void)
{
    struct ECAN_REGS ECanaShadow;
    unsigned int a;
    if(CAN_bus_empty==0)               //CAN通讯发送 在接收数据时不发送数据
    {
        a = uiAdcVo;
        if(a<80) a=0;
       canRxMsg.dataH.byteH.byte6= (unsigned char)((a & 0xff00)>>8);
       canRxMsg.dataH.byteH.byte7= (unsigned char)(a & 0xff);

       a = uiAdcIo;
       if(a<20)a=0;
       canRxMsg.dataH.byteH.byte4= (unsigned char)((a & 0xff00)>>8);
       canRxMsg.dataH.byteH.byte5= (unsigned char)(a & 0xff);

       a=0;
       canRxMsg.dataL.byteL.byte2= (unsigned char)((a & 0xff00)>>8);
       canRxMsg.dataL.byteL.byte3= (unsigned char)(a & 0xff);

       a=0;
       canRxMsg.dataL.byteL.byte0= (unsigned char)((a & 0xff00)>>8);
       canRxMsg.dataL.byteL.byte1= (unsigned char)(a & 0xff);

       ECanaMboxes.MBOX9.MSGCTRL.bit.DLC = 8;
       ECanaMboxes.MBOX9.MDH.all = canRxMsg.dataH.all;
       ECanaMboxes.MBOX9.MDL.all = canRxMsg.dataL.all;

       EALLOW;
       ECanaRegs.CANME.bit.ME9 = 0;
       ECanaMboxes.MBOX9.MSGID.all = 0x980000A1;
       ECanaRegs.CANME.bit.ME9 = 1;
       EDIS;

       ECanaShadow.CANTRS.all = ECanaRegs.CANTRS.all;
       ECanaShadow.CANTRS.bit.TRS9 = 1;
       ECanaShadow.CANTA.bit.TA9 = 1;
       ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
    }
}



Uint32 MsgRx(myCanMsg_t* pCanRxMsg)
{
    struct ECAN_REGS ECanaShadow;
    Uint32 res = 0;

    ECanaShadow.CANRMP.all = ECanaRegs.CANRMP.all;
    if (ECanaShadow.CANRMP.bit.RMP0 == 1)
    {
        ECanaRegs.CANTA.bit.TA0 = 1;                              //
        ECanaRegs.CANRMP.bit.RMP0 = 1;
        if (ECanaRegs.CANRML.bit.RML0 == 0)                       //
        {
            pCanRxMsg->id = ECanaMboxes.MBOX0.MSGID.all;          //
            pCanRxMsg->dlc = ECanaMboxes.MBOX0.MSGCTRL.bit.DLC;   //
            pCanRxMsg->dataH.all = ECanaMboxes.MBOX0.MDH.all;     //
            pCanRxMsg->dataL.all = ECanaMboxes.MBOX0.MDL.all;     //

            res = 1;
        }
        CAN_bus_empty=1;                                          //
    }
    else
    {
        CAN_bus_empty=0;                                         //
    }

    if (ECanaShadow.CANRMP.bit.RMP2 == 1)
    {
        ECanaRegs.CANTA.bit.TA2 = 1;
        ECanaRegs.CANRMP.bit.RMP2 = 1;                           //

        if (ECanaRegs.CANRML.bit.RML2 == 0)
        {
            pCanRxMsg->id = ECanaMboxes.MBOX2.MSGID.all;         //
            pCanRxMsg->dlc = ECanaMboxes.MBOX2.MSGCTRL.bit.DLC;  //
            pCanRxMsg->dataH.all = ECanaMboxes.MBOX2.MDH.all;    //
            pCanRxMsg->dataL.all = ECanaMboxes.MBOX2.MDL.all;    //

            res = 1;
        }
        CAN_bus_empty=1;
    }
    else
    {
        CAN_bus_empty=0;                                        //
    }

    return res;
}

void CanMsgHandle(void)
{
    if (canRxMsg.id == 0x98FF00A1)
    {
        uiCanFailTime = 0;
        CANONOFF = canRxMsg.dataH.byteH.byte4 & 0x01;           //判断字节4是否使能输出

        Vset = (canRxMsg.dataL.byteL.byte0<<8) + canRxMsg.dataL.byteL.byte1;                //高字节在前  字节0和1为电压
        Iset = (canRxMsg.dataL.byteL.byte2<<8) + canRxMsg.dataL.byteL.byte3;

        CANData[0] = canRxMsg.dataL.byteL.byte0;
        CANData[1] = canRxMsg.dataL.byteL.byte1;
        CANData[2] = canRxMsg.dataL.byteL.byte2;
        CANData[3] = canRxMsg.dataL.byteL.byte0;
        CANData[4] = canRxMsg.dataL.byteL.byte0;
        CANData[5] = canRxMsg.dataL.byteL.byte0;
        CANData[6] = canRxMsg.dataL.byteL.byte0;
        CANData[7] = canRxMsg.dataL.byteL.byte0;
    }
    uiRxVset = Vset;
    uiRxIset = Iset;
}


////////////////////////////////////////////////////////////////////////////
//Adc采样计算处理
///////////////////////////////////////////////////////////////////////////
void AdcChuli(void)
{
  unsigned int m=0;
  long lAdcTemp;
  if(uiAdcSam)
  {
     uiAdcSam = 0;
     ServiceDog();

     lAdcVoSum[40] = Dcdcisr.ul_Dcdc_Volt_Ave;       //输出电压
     lAdcIoSum[40] = Dcdcisr.ui_Dcdc_Curr_Dis0;      //输出电流
     lAdcVbatSum[40] = AdcResult.ADCRESULT4;         //电池电压
     lAdcTdspSum[40] = AdcResult.ADCRESULT0;         //DSP温度
     lAdcTp1Sum[40] = AdcResult.ADCRESULT7;          //内部温度1
     lAdcTp2Sum[40] = AdcResult.ADCRESULT8;          //内部温度2
     lAdcTp3Sum[40] = AdcResult.ADCRESULT12;         //内部温度3
     lAdcExTSum[40] = AdcResult.ADCRESULT15;         //外内部温度

     lAdcVoSum[uiAdcNum] = lAdcVoSum[40];            //依次存储40个数据
     lAdcIoSum[uiAdcNum] = lAdcIoSum[40];
     lAdcVbatSum[uiAdcNum] = lAdcVbatSum[40];
     lAdcTdspSum[uiAdcNum] = lAdcTdspSum[40];
     lAdcTp1Sum[uiAdcNum] = lAdcTp1Sum[40];
     lAdcTp2Sum[uiAdcNum] = lAdcTp2Sum[40];
     lAdcTp3Sum[uiAdcNum] = lAdcTp3Sum[40];
     lAdcExTSum[uiAdcNum] = lAdcExTSum[40];

     uiAdcNum++;
     if(uiAdcNum>=40)
     {
       uiAdcNum = 0;
       lAdcVo = 0;
       lAdcIo = 0;
       lAdcVbat = 0;
       lAdcTdsp = 0;
       lAdcTp1 = 0;
       lAdcTp2 = 0;
       lAdcTp3 = 0;
       lAdcExT = 0;
       for(m=0;m<40;m++)                             //40个数据求和
         {
         lAdcVo = lAdcVo + lAdcVoSum[m];
         lAdcIo = lAdcIo + lAdcIoSum[m];
         lAdcVbat = lAdcVbat + lAdcVbatSum[m];
         lAdcTdsp = lAdcTdsp + lAdcTdspSum[m];
         lAdcTp1 = lAdcTp1 + lAdcTp1Sum[m];
         lAdcTp2 = lAdcTp2 + lAdcTp2Sum[m];
         lAdcTp3 = lAdcTp3 + lAdcTp3Sum[m];
         lAdcExT = lAdcExT + lAdcExTSum[m];
         }
     }
    lAdcVo = lAdcVo + lAdcVoSum[40] - lAdcVoSum[uiAdcNum];            //每次读取ADC数据后将本次数据与最新一次做差值处理
    lAdcIo = lAdcIo + lAdcIoSum[40] - lAdcIoSum[uiAdcNum];
    lAdcVbat = lAdcVbat + lAdcVbatSum[40] - lAdcVbatSum[uiAdcNum];
    lAdcTdsp = lAdcTdsp + lAdcTdspSum[40] - lAdcTdspSum[uiAdcNum];
    lAdcTp1 = lAdcTp1 + lAdcTp1Sum[40] - lAdcTp1Sum[uiAdcNum];
    lAdcTp2 = lAdcTp2 + lAdcTp2Sum[40] - lAdcTp2Sum[uiAdcNum];
    lAdcTp3 = lAdcTp3 + lAdcTp3Sum[40] - lAdcTp3Sum[uiAdcNum];
    lAdcExT = lAdcExT + lAdcExTSum[40] - lAdcExTSum[uiAdcNum];

    lAdcTemp = labs(lAdcVo + 1200 );                                 //比例换算  转换为真实电压
    uiAdcVo = (unsigned int)(lAdcTemp/658);

    lAdcTemp = labs(lAdcIo);
    uiAdcIo = (unsigned int)(lAdcTemp/833);

    lAdcTemp = labs(lAdcVbat * 4);
    uiAdcVbat = (unsigned int)(lAdcTemp/658);

    uiAdcVbat = ui_Old_AdcVbat*3 + uiAdcVbat;
    uiAdcVbat = uiAdcVbat>>2 ;
    ui_Old_AdcVbat = uiAdcVbat;

    lAdcTemp = (lAdcTdsp-27300) * 5;
    iAdcTdsp = (int)(lAdcTemp/273);
    uiAdcTdsp = (unsigned int)(iAdcTdsp + 1000);

    lAdcTemp = (lAdcTp1)/40;
    uiAdcTp1 = (unsigned int)(labs(4096 - lAdcTemp));

    lAdcTemp = (lAdcTp2)/40;
    uiAdcTp2 = (unsigned int)(labs(4096 - lAdcTemp));

    lAdcTemp = (lAdcTp3)/40;
    uiAdcTp3 = (unsigned int)(labs(4096 - lAdcTemp));

    lAdcTemp = (lAdcExT)/40;
    uiAdcExT = (unsigned int)(labs(4096 - lAdcTemp));


  }
}


