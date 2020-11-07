#ifndef __DWIN_LCD_H__
#define __DWIN_LCD_H__


void LCD_Page_Manage(void);
void LCD_Page_Change(unsigned  short usPage);
void LCD_Display(void);




//////////////////����LCD�������������ַ��Ϣ/////////////////
#define SendHeard1                 0x5A       //���Ĵ���ͨ��֡ͷ
#define SendHeard2                 0xA5
#define WriteLCDDataOrder          0x82       //д����������
#define ReadLCDDataOrder           0x83       //��ȡ��������
#define ChangeLCDPageOrder         0x5A01     //�ı�ҳ������

//ϵͳ������ַ
#define Date_Set_Addr              0x009C     //����ʱ�������ַ
#define Date_Read_Addr             0x0010     //��ȡʱ�������ַ
#define Page_Msg_Addr              0x0014     //ҳ����Ϣ������ַ


#define OutputVoltageAddr          0x1000     //�����ѹ��ַ
#define OutputCurrentAddr          0x1010     //���������ַ
#define BattaryCapacity            0x1020     //���������ַ
#define ChageTimeAddr              0x1030     //���ʱ���ַ
#define BattaryTypeAddr            0x1040     //������͵�ַ
#define ChageStatusAddr            0x1050     //���״̬��ַ
#define V_Input_Addr               0x1060     //�����ѹ��ַ
#define I_Input_Addr               0x1070     //���������ַ
#define V_Bus_Addr                 0x1080     //ĸ�ߵ�ѹ��ַ
#define T_PFC1Addr                 0x1090     //Buckģ���¶�̽ͷ1��ַ
#define T_PFC2Addr                 0x10A0     //Buckģ���¶�̽ͷ2��ַ
#define T_PFC3Addr                 0x10B0     //Buckģ���¶�̽ͷ3��ַ
#define T_LLC1Addr                 0x10C0     //LLCģ���¶�̽ͷ1��ַ
#define T_LLC2Addr                 0x10D0     //LLCģ���¶�̽ͷ2��ַ
#define Err_MSg_Addr               0x10E0     //������Ϣ��ַ
#define Manual_Set_V_Output_Addr   0x10F0     //�ֶ����������ѹ��ַ
#define Manual_Set_I_Output_Addr   0x1100     //�ֶ��������������ַ
#define Manual_Start_Stop_Addr     0x1110     //�ֶ����������ͣ��ť��ַ
#define Manual_Set_Reurn_Addr      0x1120     //�ֶ����淵�������水ť��ַ
#define LCD_Set_Year_Addr          0x1130     //LCD����ʱ��-���ַ
#define LCD_Set_Month_Addr         0x1140     //LCD����ʱ��-�µ�ַ
#define LCD_Set_Date_Addr          0x1150     //LCD����ʱ��-�յ�ַ
#define LCD_Set_Hour_Addr          0x1160     //LCD����ʱ��-ʱ��ַ
#define LCD_Set_Minute_Addr        0x1170     //LCD����ʱ��-�ֵ�ַ
#define LCD_Set_Seconnds_Addr      0x1180     //LCD����ʱ��-���ַ
#define Manual_Set_Password_Addr   0x1190     //�ֶ����ò��������ַ
#define Manual_Set_Parameter_Addr  0x11A0     //�ֶ��������������ť��ַ



#define Incident_Time_Start_Addr      0x11B0  //�¼���ʼʱ���ַ
#define Incident_Content_Start_Addr0x 0x11C0  //�¼���ʼ���ݵ�ַ    ���ʮ����ַ  ���ʮ����¼�����  ������ַΪ0x1380


#define Bat_Msg_Highest_Temp_Addr     0x1390  //�����ߵ���¶ȵ������ַ
#define Bat_Msg_Lowest_Temp_Addr      0x13A0  //�����͵���¶ȵ������ַ
#define Bat_Msg_Highest_Voltage_Addr  0x13B0  //�����ߵ����ѹ������ַ
#define Bat_Msg_Lowest_Voltage_Addr   0x13C0  //�����͵����ѹ������ַ
#define Set_V_Output_Addr             0x13D0  //���������ѹ������ַ
#define Set_I_Output_Addr             0x13E0  //������������������ַ
#define Bat_SOC_Addr                  0x13F0  //���SOC�ٷֱȲ�����ַ

#endif






