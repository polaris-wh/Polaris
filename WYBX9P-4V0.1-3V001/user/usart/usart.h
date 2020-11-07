#ifndef __USART_H__
#define __USART_H__

void USART2_Configuration(void);
void USART3_Configuration(void);
void USART5_Configuration(void);
void USART_Send(USART_TypeDef* USARTx, uint8_t *Dat,uint16_t len);
void USART_STR(USART_TypeDef* USARTx,char *str);
unsigned char Usart_Deal(unsigned char usart_dat);
void LCD_Usart_Deal(unsigned char LCD_Usart_dat);






//串口命令状态定义
#define USART_HEAD      0
#define USART_CMD	      1
#define USART_LEN	      2
#define USART_DATA      3
#define USART_CRC       4
#define USART_END       5


//LCD串口命令状态定义
#define LCD_USART_HEAD1      0
#define LCD_USART_HEAD2      1
#define LCD_USART_LEN	       2
#define LCD_USART_CMD	       3
#define LCD_USART_Data       4
#define LCD_USART_END        5


#endif

