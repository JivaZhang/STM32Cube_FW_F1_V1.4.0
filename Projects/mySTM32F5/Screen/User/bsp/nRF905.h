#ifndef __NRF905_H
#define __NRF905_H
#include "stm32f10x.h"

#define nRF905_TRXCE_Port  (GPIOC)
#define nRF905_TRXCE_Pin  (GPIO_Pin_5)
#define nRF905_AM_Port  (GPIOC)
#define nRF905_AM_Pin  (GPIO_Pin_4)
#define nRF905_MISO_Port  (GPIOC)
#define nRF905_MISO_Pin  (GPIO_Pin_3)
#define nRF905_SCLK_Port  (GPIOC)
#define nRF905_SCLK_Pin  (GPIO_Pin_2)

#define nRF905_TXEN_Port  (GPIOE)
#define nRF905_TXEN_Pin  (GPIO_Pin_7)
#define nRF905_PWRUP_Port  (GPIOE)
#define nRF905_PWRUP_Pin  (GPIO_Pin_8)
#define nRF905_CD_Port  (GPIOE)
#define nRF905_CD_Pin  (GPIO_Pin_9)
#define nRF905_DR_Port  (GPIOE)
#define nRF905_DR_Pin  (GPIO_Pin_10)
#define nRF905_MOSI_Port  (GPIOE)
#define nRF905_MOSI_Pin  (GPIO_Pin_11)
#define nRF905_CSN_Port  (GPIOE)
#define nRF905_CSN_Pin  (GPIO_Pin_12)

#define LED1_Port  (GPIOE)
#define LED1_Pin  (GPIO_Pin_2)
#define LED2_Port  (GPIOC)
#define LED2_Pin  (GPIO_Pin_10)

#define nRF905_SCLK(a)	if (a)	\
					GPIO_SetBits(nRF905_SCLK_Port,nRF905_SCLK_Pin);\
					else		\
					GPIO_ResetBits(nRF905_SCLK_Port,nRF905_SCLK_Pin)

#define nRF905_MOSI(a)	if (a)	\
					GPIO_SetBits(nRF905_MOSI_Port,nRF905_MOSI_Pin);\
					else		\
					GPIO_ResetBits(nRF905_MOSI_Port,nRF905_MOSI_Pin)
					
#define nRF905_CSN(a)	if (a)	\
					GPIO_SetBits(nRF905_CSN_Port,nRF905_CSN_Pin);\
					else		\
					GPIO_ResetBits(nRF905_CSN_Port,nRF905_CSN_Pin)

#define nRF905_TRXCE(a)	if (a)	\
					nRF905_TRXCE_Port->ODR |= (uint8_t)nRF905_TRXCE_Pin;\
					else		\
					nRF905_TRXCE_Port->ODR &= (uint8_t)(~nRF905_TRXCE_Pin)
                                          
#define nRF905_PWRUP(a)	if (a)	\
					GPIO_SetBits(nRF905_PWRUP_Port,nRF905_PWRUP_Pin);\
					else		\
					GPIO_ResetBits(nRF905_PWRUP_Port,nRF905_PWRUP_Pin)
                                          
#define nRF905_TXEN(a)	if (a)	\
					GPIO_SetBits(nRF905_TXEN_Port,nRF905_TXEN_Pin);\
					else		\
					GPIO_ResetBits(nRF905_TXEN_Port,nRF905_TXEN_Pin)
                                          
#define LED1(a)	if (a)	\
					GPIO_SetBits(LED1_Port,LED1_Pin);\
					else		\
					GPIO_ResetBits(LED1_Port,LED1_Pin)
                                          
#define LED2(a)	if (a)	\
					GPIO_SetBits(LED2_Port,LED2_Pin);\
					else		\
					GPIO_ResetBits(LED2_Port,LED2_Pin)

#define nRF905_MISO     GPIO_ReadInputDataBit(nRF905_MISO_Port,nRF905_MISO_Pin)
#define nRF905_AM       GPIO_ReadInputDataBit(nRF905_AM_Port,nRF905_AM_Pin)
#define nRF905_DR       GPIO_ReadInputDataBit(nRF905_DR_Port,nRF905_DR_Pin)
#define nRF905_CD       GPIO_ReadInputDataBit(nRF905_CD_Port,nRF905_CD_Pin)    

#define WCR     0x00            // Write configuration register command
#define RCR     0x10            // Read  configuration register command
#define WTP     0x20            // Write TX Payload  command
#define RTP     0x21            // Read  TX Payload  command
#define WTA     0x22            // Write TX Address  command
#define RTA     0x23            // Read  TX Address  command
#define RRP     0x24            // Read  RX Payload  command
#define TIMES   1000            // Dealy time

#define SetTxMode()     nRF905_TXEN(1);nRF905_TRXCE(0);delay(TIMES)
#define SetRxMode()     nRF905_TXEN(0);nRF905_TRXCE(1);delay(TIMES)

extern uint8_t TxBuf[32],RxBuf[32];

extern uint8_t SpiWR(uint8_t Send);
extern void nRF905_Conf(void);
extern void SendPacket(void);
extern void nRF905_Init(void);
extern void delay(uint16_t t);

#endif
