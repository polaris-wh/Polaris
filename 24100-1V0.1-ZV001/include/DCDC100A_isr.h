/***************************************************************************
*             Defines
***************************************************************************/
#ifndef DCDCisr_H
#define DCDCisr_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************
*            Variables Definition
***************************************************************************/
struct dcdc_100a_isr
{

//unsigned int			ui_Dcdc_Mem[1024];			// 8300H~8700H

unsigned int			ui_Dcdc_Volt_Ref;			// 8000H	voltage reference
unsigned int			ui_Dcdc_Curr_Ref;			// 8001H	current reference
unsigned int 			ui_Dcdc_Curr_delta;			// 8002H	current offset

unsigned int			ui_Dcdc_Volt_K1;			// 8003H
unsigned int			ui_Dcdc_Volt_K2;			// 8004H
unsigned int			ui_Dcdc_Volt_K3;			// 8005H
unsigned int			ui_Dcdc_Volt_K4;			// 8006H
unsigned int			ui_Dcdc_Volt_K5;			// 8007H
unsigned int			ui_Dcdc_Volt_K6;			// 8008H

unsigned int			ui_Dcdc_Curr_K1;			// 8009H
unsigned int			ui_Dcdc_Curr_K2;			// 800AH
unsigned int			ui_Dcdc_Curr_K3;			// 800BH
unsigned int			ui_Dcdc_Curr_K4;			// 800CH
unsigned int			ui_Dcdc_Curr_K5;			// 800DH

unsigned int			ui_Dcdc_Curr_140A;			// 800EH	quicken loop if current>140A
unsigned int			ui_Dcdc_Curr_160A;			// 800FH	constant duty10% if current>150A

unsigned int			ui_Dcdc_Run_Mode;			// 8010H  rectifier run in voltage /current/ power loop mode
unsigned int			ui_Dcdc_Duty_Permit;		// 8011H  the max PWM duty permitted(including short circuit)
unsigned int			ui_Dcdc_Duty_Ramp;			// 8012H	the max PWM duty for loop 
unsigned int			ui_Dcdc_Pwm_Out;			// 8013H	the result of v and i loop, to set CMPRA & CMPRB
unsigned int 			ui_Dcdc_Pwm_Set;			// 8014H	set pwm duty in open loop for the first prototype, rsd
unsigned int			ui_Dcdc_Start_Cnt;			// 8015H  for softstart count

unsigned long           ul_Dcdc_Volt_Sum;			// 8016H	add voltage adc of 64 int
unsigned long			ul_Dcdc_Volt_Ave;			// 8018H  average adc of 64 int
unsigned long			ul_Dcdc_Power_Lim;			// 801AH	power reference(5800*power limit)
unsigned int			ui_Dcdc_Power_Lim;			// 801CH	power reference(ul_Dcdc_Power_Lim/Vo)

unsigned int			ui_Dcdc_Volt_Adc;			// 801DH
signed int				i_Dcdc_Volt_Err0;			// 801EH
signed int				i_Dcdc_Volt_Err1;			// 801FH
signed int				i_Dcdc_Volt_Err2;			// 8020H
unsigned int			ui_Dcdc_Volt_Out0;			// 8021H
unsigned int			ui_Dcdc_Volt_Out1;			// 8022H
unsigned int			ui_Dcdc_Volt_Out2;			// 8023H

unsigned int			ui_Dcdc_Curr_Adc;			// 8024H
signed int				i_Dcdc_Curr_Err0;			// 8025H  the current current error
signed int				i_Dcdc_Curr_Err1;			// 8026H  the last current error
signed int				i_Dcdc_Curr_Err2;			// 8027H  the second last current error
unsigned int			ui_Dcdc_Curr_Out0;			// 8028H
unsigned int			ui_Dcdc_Curr_Out1;			// 8029H
unsigned int			ui_Dcdc_Curr_Out2;			// 802AH
unsigned int			ui_Dcdc_Curr_Max;			// 802BH

unsigned long 			ul_Dcdc_Curr_Sum;			// 802CH	64 current sum
unsigned int 			ui_Dcdc_Curr_Cnt;			// 802EH	count 64
unsigned int			ui_Dcdc_Curr_Filt_K1;		// 802FH	current display filter factor
unsigned int			ui_Dcdc_Curr_Filt_K2;		// 8030H	current display filter factor
unsigned int 			ui_Dcdc_Curr_Ave0;			// 8031H	current display filter input
unsigned int 			ui_Dcdc_Curr_Ave1;			// 8032H	current display filter input
unsigned int			ui_Dcdc_Curr_Dis0;			// 8033H	current display filter result hi
unsigned int			ui_Dcdc_Curr_Dis0_Lo;		// 8034H	current display filter result loi


unsigned int			ui_Dcdc_Temp_Ctrl;			// 8035H	environ temp+50,use for loop change in low temperature
unsigned int			ui_Dcdc_Temp_Set1;			// 8036H	for loop change in low temperature -15 degree
unsigned int			ui_Dcdc_Temp_Set2;			// 8037H	for loop change in low temperature -10 degree
unsigned int			ui_Dcdc_K_flag;				// 8038H	flag for loop change in low temperature 

unsigned int			ui_Dcdc_Short_Flag;			// 8039H
unsigned int			ui_Dcdc_Duty_Short;			// 803AH	duty for short

unsigned int			ui_Dcdc_PRD;				// 803BH
signed long		      	l_Dcdc_Iresult1;			// 803CH	current loop result(before limit, for debug)
unsigned int			ui_Mem_Debug_Num;	    	// 803EH	 for mem test (continually)
unsigned int			ui_Mem_Debug_Flag;	    	// 803FH   for mem test (continually)


unsigned int	        ui_Dcdc_debug;				// 8040H	to set the DCDC board work without PFC board
unsigned int            ui_Dcdc_Vversion;			// 8041H	version for archive
unsigned int            ui_Dcdc_Dversion;			// 8042H	version for baseline
unsigned int			ui_Dcdc_Tversion;			// 8043H	version for develop

unsigned int			ui_Dcdc_Vdebug0;			// 8044H
unsigned int			ui_Dcdc_Vdebug1;			// 8045H
unsigned int			ui_Dcdc_ErrTD;			// 8046H
unsigned long			ul_Dcdc_Vbat_Sum;			// 8048H
unsigned long			ul_Dcdc_Vbat_Ave;			// 804AH
unsigned long			ul_Dcdc_Rsd3;				// 804CH
unsigned long			ul_Dcdc_Rsd4;				// 804EH

};

//volatile struct dcdc_100a_isr Dcdcisr;
extern volatile struct dcdc_100a_isr Dcdcisr;


#ifdef __cplusplus
}
#endif /* extern "C" */

#endif

