#include "stm32f10x.h"
#include "nRF905.h"

void delay(uint16_t t)
{
  while(t--);
}

typedef struct RFconfig
{
  uint8_t n;
  uint8_t buf[10];
}RFconfig;

const RFconfig nRF905Conf = 
{
  10,
  0x4c, 0x0c, 0x44, 0x20, 0x20, 0xcc, 0xcc, 0xcc, 0xcc, 0x58
};

uint8_t TxBuf[32],RxBuf[32];

uint8_t SpiWR(uint8_t Send)
{
  uint8_t i;
  uint8_t Receive=0;
  for (i=0;i<8;i++)
  {
    nRF905_SCLK(0);
    if (Send&0x80)
      nRF905_MOSI(1);
    else
      nRF905_MOSI(0);
    Send=Send<<1;

    nRF905_SCLK(1);
    if (nRF905_MISO)
      Receive=Receive+0x01;
    Receive=Receive<<1;
  }
    return Receive;
}

void nRF905_Conf(void)
{
  uint8_t i;
  nRF905_CSN(0);
  SpiWR(WCR);
  for(i=0; i<nRF905Conf.n; i++)
    SpiWR(nRF905Conf.buf[i]);
  nRF905_CSN(1);
}
    
void SendPacket(void)
{
  uint8_t i;
  nRF905_CSN(0);
  SpiWR(WTP);
  for(i=0;i<32;i++)
    SpiWR(TxBuf[i]);
  nRF905_CSN(1);
  delay(TIMES);
  nRF905_CSN(0);
  for(i=0;i<4;i++)
    SpiWR(nRF905Conf.buf[i+5]);
  nRF905_CSN(1);
  nRF905_TRXCE(1);
  delay(TIMES);
  nRF905_TRXCE(0);
}

void nRF905_Init(void)
{
//Output pins
	GPIO_InitTypeDef nRF905;
	nRF905.GPIO_Pin=nRF905_SCLK_Pin;	
	nRF905.GPIO_Mode= GPIO_Mode_Out_PP;
	nRF905.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_Init(nRF905_SCLK_Port, &nRF905);
	
	nRF905.GPIO_Pin=nRF905_MOSI_Pin;
	GPIO_Init(nRF905_MOSI_Port, &nRF905);
	nRF905.GPIO_Pin=nRF905_CSN_Pin;
	GPIO_Init(nRF905_CSN_Port, &nRF905);
	nRF905.GPIO_Pin=nRF905_TRXCE_Pin;
	GPIO_Init(nRF905_TRXCE_Port, &nRF905);
	nRF905.GPIO_Pin=nRF905_PWRUP_Pin;
	GPIO_Init(nRF905_PWRUP_Port, &nRF905);
	nRF905.GPIO_Pin=nRF905_TXEN_Pin;
	GPIO_Init(nRF905_TXEN_Port, &nRF905);
	nRF905.GPIO_Pin=LED1_Pin;
	GPIO_Init(LED1_Port, &nRF905);
	nRF905.GPIO_Pin=LED2_Pin;
	GPIO_Init(LED2_Port, &nRF905);
	
//input pins
	nRF905.GPIO_Pin=nRF905_MISO_Pin;	
	nRF905.GPIO_Mode= GPIO_Mode_IN_FLOATING;
	nRF905.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_Init(nRF905_MISO_Port, &nRF905);
	
	nRF905.GPIO_Pin=nRF905_AM_Pin;
	GPIO_Init(nRF905_AM_Port, &nRF905);
	nRF905.GPIO_Pin=nRF905_DR_Pin;
	GPIO_Init(nRF905_DR_Port, &nRF905);
	nRF905.GPIO_Pin=nRF905_CD_Pin;
	GPIO_Init(nRF905_CD_Port, &nRF905);
  
  
  nRF905_CSN(1);	// Spi 	disable						
  nRF905_SCLK(0);	// Spi clock line init high
  nRF905_PWRUP(1);	// nRF905 power on
  nRF905_TRXCE(0);	// Set nRF905 in standby mode
  nRF905_TXEN(0);	// set radio in Rx mode
}
