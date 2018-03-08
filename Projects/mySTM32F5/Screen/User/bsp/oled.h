#ifndef __OLED_H_
#define __OLED_H_
#include "stm32f10x.h"

#define isCMD  0
#define isDATA 1

#define OLED_SCLK_Port  (GPIOA)
#define OLED_SCLK_Pin  (GPIO_Pin_1)
#define OLED_MOSI_Port  (GPIOA)
#define OLED_MOSI_Pin  (GPIO_Pin_2)
#define OLED_RES_Port  (GPIOA)
#define OLED_RES_Pin  (GPIO_Pin_3)
#define OLED_DC_Port  (GPIOA)
#define OLED_DC_Pin  (GPIO_Pin_4)
#define OLED_CS_Port  (GPIOA)
#define OLED_CS_Pin  (GPIO_Pin_5)

#define SIZE 16
#define XLevelL		0x02
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xA5
#define X_WIDTH 	128
#define Y_WIDTH 	64
//-----------------OLED¶Ë¿Ú¶¨Òå----------------  					   

void delay_ms(uint32_t uCount);
		     
void OLED_WriteByte(uint8_t dat,uint8_t cmd);	       							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr);
void OLED_ShowNum(uint8_t x,uint8_t y,u32 num,uint8_t len,uint8_t size2);
void OLED_ShowString(uint8_t x,uint8_t y, uint8_t *p);	 
void OLED_SetPos(uint8_t x, uint8_t y);
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t no);
void OLED_DrawBMP(uint8_t x0, uint8_t y0,uint8_t x1, uint8_t y1,uint8_t BMP[]);


#define OLED_SCLK(a)	if (a)	\
					GPIO_SetBits(OLED_SCLK_Port,OLED_SCLK_Pin);\
					else		\
					GPIO_ResetBits(OLED_SCLK_Port,OLED_SCLK_Pin)

#define OLED_MOSI(a)	if (a)	\
					GPIO_SetBits(OLED_MOSI_Port,OLED_MOSI_Pin);\
					else		\
					GPIO_ResetBits(OLED_MOSI_Port,OLED_MOSI_Pin)
					
#define OLED_RES(a)	if (a)	\
					GPIO_SetBits(OLED_RES_Port,OLED_RES_Pin);\
					else		\
					GPIO_ResetBits(OLED_RES_Port,OLED_RES_Pin)

#define OLED_DC(a)	if (a)	\
					GPIO_SetBits(OLED_DC_Port,OLED_DC_Pin);\
					else		\
					GPIO_ResetBits(OLED_DC_Port,OLED_DC_Pin)
                                          
#define OLED_CS(a)	if (a)	\
					GPIO_SetBits(OLED_CS_Port,OLED_CS_Pin);\
					else		\
					GPIO_ResetBits(OLED_CS_Port,OLED_CS_Pin)


#endif  
	 



