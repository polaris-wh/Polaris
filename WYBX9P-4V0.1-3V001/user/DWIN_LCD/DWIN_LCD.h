#ifndef __DWIN_LCD_H__
#define __DWIN_LCD_H__


void LCD_Page_Manage(void);
void LCD_Page_Change(unsigned  short usPage);
void LCD_Display(void);




//////////////////迪文LCD参数设置命令及地址信息/////////////////
#define SendHeard1                 0x5A       //迪文串口通信帧头
#define SendHeard2                 0xA5
#define WriteLCDDataOrder          0x82       //写入数据命令
#define ReadLCDDataOrder           0x83       //读取数据命令
#define ChangeLCDPageOrder         0x5A01     //改变页面命令

//系统变量地址
#define Date_Set_Addr              0x009C     //设置时间参数地址
#define Date_Read_Addr             0x0010     //读取时间参数地址
#define Page_Msg_Addr              0x0014     //页面信息参数地址


#define OutputVoltageAddr          0x1000     //输出电压地址
#define OutputCurrentAddr          0x1010     //输出电流地址
#define BattaryCapacity            0x1020     //电池容量地址
#define ChageTimeAddr              0x1030     //充电时间地址
#define BattaryTypeAddr            0x1040     //电池类型地址
#define ChageStatusAddr            0x1050     //充电状态地址
#define V_Input_Addr               0x1060     //输入电压地址
#define I_Input_Addr               0x1070     //输入电流地址
#define V_Bus_Addr                 0x1080     //母线电压地址
#define T_PFC1Addr                 0x1090     //Buck模块温度探头1地址
#define T_PFC2Addr                 0x10A0     //Buck模块温度探头2地址
#define T_PFC3Addr                 0x10B0     //Buck模块温度探头3地址
#define T_LLC1Addr                 0x10C0     //LLC模块温度探头1地址
#define T_LLC2Addr                 0x10D0     //LLC模块温度探头2地址
#define Err_MSg_Addr               0x10E0     //故障信息地址
#define Manual_Set_V_Output_Addr   0x10F0     //手动设置输出电压地址
#define Manual_Set_I_Output_Addr   0x1100     //手动设置输出电流地址
#define Manual_Start_Stop_Addr     0x1110     //手动设置输出启停按钮地址
#define Manual_Set_Reurn_Addr      0x1120     //手动界面返回主界面按钮地址
#define LCD_Set_Year_Addr          0x1130     //LCD设置时间-年地址
#define LCD_Set_Month_Addr         0x1140     //LCD设置时间-月地址
#define LCD_Set_Date_Addr          0x1150     //LCD设置时间-日地址
#define LCD_Set_Hour_Addr          0x1160     //LCD设置时间-时地址
#define LCD_Set_Minute_Addr        0x1170     //LCD设置时间-分地址
#define LCD_Set_Seconnds_Addr      0x1180     //LCD设置时间-秒地址
#define Manual_Set_Password_Addr   0x1190     //手动设置参数密码地址
#define Manual_Set_Parameter_Addr  0x11A0     //手动设置输出参数按钮地址



#define Incident_Time_Start_Addr      0x11B0  //事件起始时间地址
#define Incident_Content_Start_Addr0x 0x11C0  //事件起始内容地址    间隔十个地址  最多十五个事件内容  结束地址为0x1380


#define Bat_Msg_Highest_Temp_Addr     0x1390  //电池最高电池温度点参数地址
#define Bat_Msg_Lowest_Temp_Addr      0x13A0  //电池最低电池温度点参数地址
#define Bat_Msg_Highest_Voltage_Addr  0x13B0  //电池最高单体电压参数地址
#define Bat_Msg_Lowest_Voltage_Addr   0x13C0  //电池最低单体电压参数地址
#define Set_V_Output_Addr             0x13D0  //设置输出电压参数地址
#define Set_I_Output_Addr             0x13E0  //设置输出电电流参数地址
#define Bat_SOC_Addr                  0x13F0  //电池SOC百分比参数地址

#endif






