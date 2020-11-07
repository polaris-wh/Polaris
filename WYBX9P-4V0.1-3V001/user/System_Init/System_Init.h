#ifndef __System_H
#define __System_H
#include "stm32f10x.h"
#include "stdio.h"


#define OBC_Work_In_CAN_Mode 0x01
#define OBC_Work_In_MC_Mode  0x02

void RCC_Configuration(void);
void System_Init(void);
void Gpio_Init(void);
void Timer2_Init(void);
void SPI3_Init(void);
void SPI3_SetSpeed(u8 SPI_BaudRatePrescaler); 
u8 SPI3_ReadWriteByte(u8 TxData);

void Parameter_Init(void);



#define MCU_CTL_Led1_Port              GPIOA
#define MCU_CTL_Led1_Pin               GPIO_Pin_4 

#define MCU_CTL_Led2_Port              GPIOA
#define MCU_CTL_Led2_Pin               GPIO_Pin_5   

#define MCU_CTL_Led3_Port              GPIOA
#define MCU_CTL_Led3_Pin               GPIO_Pin_6   

#define MCU_CTL_Led4_Port              GPIOA
#define MCU_CTL_Led4_Pin               GPIO_Pin_7  

#define MCU_CTL_Led5_Port              GPIOC
#define MCU_CTL_Led5_Pin               GPIO_Pin_4   

#define MCU_CTL_Rly1_Port              GPIOD
#define MCU_CTL_Rly1_Pin               GPIO_Pin_11  

#define MCU_CTL_Rly2_Port              GPIOD
#define MCU_CTL_Rly2_Pin               GPIO_Pin_12  

#define MCU_CC1_Port                   GPIOA
#define MCU_CC1_Pin                    GPIO_Pin_10  

#define RTC_SCL_Port                   GPIOB
#define RTC_SCL_Pin                    GPIO_Pin_8  

#define RTC_SDA_Port                   GPIOB
#define RTC_SDAPin                     GPIO_Pin_9  

#define IIC_24C0XX_SCL_Port            GPIOB
#define IIC_24C0XX_SCL_Pin             GPIO_Pin_10  

#define IIC_24C0XX_SDA_Port            GPIOB
#define IIC_24C0XX_SDAPin              GPIO_Pin_11 

#define IIC_24C0XX_Read_EN_Port        GPIOB
#define IIC_24C0XX_Read_EN_Pin         GPIO_Pin_2

#define SD_CS_EN_Port                  GPIOB
#define SD_CS_EN_Pin                   GPIO_Pin_6

#define SD_CS_ENABLE()                    GPIO_ResetBits(SD_CS_EN_Port, SD_CS_EN_Pin)
#define SD_CS_DISABLE()                   GPIO_SetBits(SD_CS_EN_Port, SD_CS_EN_Pin)

#define IIC_24C0XX_Read_EN()              GPIO_ResetBits(IIC_24C0XX_Read_EN_Port, IIC_24C0XX_Read_EN_Pin)
#define IIC_24C0XX_Read_DisEN()           GPIO_SetBits(IIC_24C0XX_Read_EN_Port, IIC_24C0XX_Read_EN_Pin)

#define IIC_24C0XX_SCL_High()             GPIO_SetBits(IIC_24C0XX_SCL_Port, IIC_24C0XX_SCL_Pin)
#define IIC_24C0XX_SCL_Low()              GPIO_ResetBits(IIC_24C0XX_SCL_Port, IIC_24C0XX_SCL_Pin)

#define IIC_24C0XX_SDA_High()             GPIO_SetBits(IIC_24C0XX_SDA_Port, IIC_24C0XX_SDAPin)
#define IIC_24C0XX_SDA_Low()              GPIO_ResetBits(IIC_24C0XX_SDA_Port, IIC_24C0XX_SDAPin)

#define IIC_24C0XX_SDA_Status()           GPIO_ReadInputDataBit(IIC_24C0XX_SDA_Port, IIC_24C0XX_SDAPin);

#define RTC_SCL_High()             GPIO_SetBits(RTC_SCL_Port, RTC_SCL_Pin)
#define RTC_SCL_Low()              GPIO_ResetBits(RTC_SCL_Port, RTC_SCL_Pin)

#define RTC_SDA_High()             GPIO_SetBits(RTC_SDA_Port, RTC_SDAPin)
#define RTC_SDA_Low()              GPIO_ResetBits(RTC_SDA_Port, RTC_SDAPin)

#define RTC_SDA_Status()           GPIO_ReadInputDataBit(RTC_SDA_Port, RTC_SDAPin);

#define C_EnOutStateLed1()         GPIO_SetBits(MCU_CTL_Led1_Port, MCU_CTL_Led1_Pin)
#define C_DisEnOutStateLed1()      GPIO_ResetBits(MCU_CTL_Led1_Port, MCU_CTL_Led1_Pin)

#define C_EnOutStateLed2()         GPIO_SetBits(MCU_CTL_Led2_Port, MCU_CTL_Led2_Pin)
#define C_DisEnOutStateLed2()      GPIO_ResetBits(MCU_CTL_Led2_Port, MCU_CTL_Led2_Pin)

#define C_EnOutStateLed3()         GPIO_SetBits(MCU_CTL_Led3_Port, MCU_CTL_Led3_Pin)
#define C_DisEnOutStateLed3()      GPIO_ResetBits(MCU_CTL_Led3_Port, MCU_CTL_Led3_Pin)

#define C_EnOutStateLed4()         GPIO_SetBits(MCU_CTL_Led4_Port, MCU_CTL_Led4_Pin)
#define C_DisEnOutStateLed4()      GPIO_ResetBits(MCU_CTL_Led4_Port, MCU_CTL_Led4_Pin)

#define C_EnOutStateLed5()         GPIO_SetBits(MCU_CTL_Led5_Port, MCU_CTL_Led5_Pin)
#define C_DisEnOutStateLed5()      GPIO_ResetBits(MCU_CTL_Led5_Port, MCU_CTL_Led5_Pin)






#define CC1_Control    GPIO_ReadInputDataBit(MCU_CC1_Port,MCU_CC1_Pin)


extern volatile unsigned char ucTag10ms;
extern volatile unsigned char ucTag50ms;
extern volatile unsigned char ucTag100ms;
extern volatile unsigned char ucTag250ms;
extern volatile unsigned char ucTag500ms;
extern volatile unsigned char ucTag5s;
extern unsigned int ui100msCnt;
extern volatile unsigned char CAN_BMS_Rx_Flg;

extern unsigned char ucOBC_Work_Mode;

void IWDG_Configuration(void);
void reloadWDG(void);



#endif
