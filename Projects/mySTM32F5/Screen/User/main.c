#include "oled.h"
#include "stm32f10x.h"
#include "nRF905.h"

void Delay(__IO uint32_t nCount)
{
	for(; nCount != 0; nCount--);
}

int main(void)
{
	uint16_t il=0,ih=0;
	GPIO_InitTypeDef ledio;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOE, ENABLE);
	ledio.GPIO_Pin=GPIO_Pin_2;	
	ledio.GPIO_Mode= GPIO_Mode_Out_PP;
	ledio.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_Init(GPIOE, &ledio);
	ledio.GPIO_Pin=GPIO_Pin_10;
	GPIO_Init(GPIOC, &ledio);
	GPIO_SetBits(GPIOE, GPIO_Pin_2);
	GPIO_SetBits(GPIOC, GPIO_Pin_10);
	
	OLED_Init();
	OLED_Clear();
//	nRF905_Init();
//  nRF905_Conf();
	
/*
	OLED_ShowChinese(0,0,0);
	OLED_ShowChinese(2,0,1);
	OLED_ShowChinese(4,0,2);
	OLED_ShowChar(48,0,'C');
	OLED_ShowChinese(7,0,3);
	OLED_ShowChinese(9,0,4);
	
	for(il=0;il<8;il++)
		OLED_ShowChinese(2*il,2,5+il);
	for(il=0;il<8;il++)
		OLED_ShowChinese(2*il,4,13+il);
	for(il=0;il<8;il++)
		OLED_ShowChinese(2*il,6,21+il);
	while(1);
*/	

	OLED_ShowChinese(4,0,0);
	OLED_ShowChinese(6,0,1);
	OLED_ShowChinese(8,0,2);
	OLED_ShowChinese(10,0,3);
	OLED_ShowChinese(0,2,4);
	OLED_ShowChinese(2,2,6);
	OLED_ShowChinese(0,4,5);
	OLED_ShowChinese(2,4,6);	
	OLED_ShowString(0,0,"OLED");
	OLED_ShowChar(96,0,':');
	OLED_ShowChar(32,2,':');
	OLED_ShowChar(32,4,':');
	
	while(1)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_2);
		GPIO_ResetBits(GPIOC,GPIO_Pin_10);
		OLED_ShowNum(40,4,++il,4,16);
		Delay(0x400000);
		GPIO_ResetBits(GPIOE,GPIO_Pin_2);
		GPIO_SetBits(GPIOC,GPIO_Pin_10);
		OLED_ShowNum(40,2,++ih,4,16);
		Delay(0x400000);

	}

}






