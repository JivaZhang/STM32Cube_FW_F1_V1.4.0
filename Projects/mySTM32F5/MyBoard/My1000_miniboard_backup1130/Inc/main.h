/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
//#define CLASSIC_BOARD


#ifdef CLASSIC_BOARD
	#define DW_RESET_Pin GPIO_PIN_0
	#define DW_RESET_GPIO_Port GPIOA
	#define DW_NSS_Pin GPIO_PIN_4
	#define DW_NSS_GPIO_Port GPIOA
	#define DW_SCK_Pin GPIO_PIN_5
	#define DW_SCK_GPIO_Port GPIOA
	#define DW_MISO_Pin GPIO_PIN_6
	#define DW_MISO_GPIO_Port GPIOA
	#define DW_MOSI_Pin GPIO_PIN_7
	#define DW_MOSI_GPIO_Port GPIOA
	#define DW_WUP_Pin GPIO_PIN_0
	#define DW_WUP_GPIO_Port GPIOB
	#define LED_RED_Pin GPIO_PIN_10
	#define LED_RED_GPIO_Port GPIOB

	#define DW_IRQn_Pin GPIO_PIN_5
	#define DW_IRQn_GPIO_Port GPIOB
	
	
	#define TA_SW1_GPIO                 GPIOC
	#define TA_TorA					 GPIO_PIN_4
	
	#define SWITCH_IO_GROUP  GPIOB
	#define SWITCH_IO_PAIR_GROUP  GPIOA
  #define	SWITCH_BIT5      GPIO_PIN_1
	#define	SWITCH_BIT6      GPIO_PIN_8
	#define	SWITCH_BIT7      GPIO_PIN_11
	#define	SWITCH_BIT8      GPIO_PIN_12
	//	#define PB2_BOOT1_Pin GPIO_PIN_2
//	#define PB2_BOOT1_GPIO_Port GPIOB

	#define UART_TX_Pin GPIO_PIN_9
	#define UART_TX_GPIO_Port GPIOA
	#define UART_RX_Pin GPIO_PIN_10
	#define UART_RX_GPIO_Port GPIOA    //all UART1
	#define USARTx  USART1

	#define LED7_Pin GPIO_PIN_8
	#define LED7_GPIO_Port GPIOC
	#define LED8_Pin GPIO_PIN_9
	#define LED8_GPIO_Port GPIOC
#else

	#define DW_RESET_Pin GPIO_PIN_0
	#define DW_RESET_GPIO_Port GPIOA
	#define DW_NSS_Pin GPIO_PIN_4
	#define DW_NSS_GPIO_Port GPIOA
	#define DW_SCK_Pin GPIO_PIN_5
	#define DW_SCK_GPIO_Port GPIOA
	#define DW_MISO_Pin GPIO_PIN_6
	#define DW_MISO_GPIO_Port GPIOA
	#define DW_MOSI_Pin GPIO_PIN_7
	#define DW_MOSI_GPIO_Port GPIOA
	#define DW_WUP_Pin GPIO_PIN_0
	#define DW_WUP_GPIO_Port GPIOB
	#define LED_RED_Pin GPIO_PIN_10
	#define LED_RED_GPIO_Port GPIOB

	#define DW_IRQn_Pin GPIO_PIN_5
	#define DW_IRQn_GPIO_Port GPIOB
	
	
	#define TA_SW1_GPIO                 GPIOC
	#define TA_TorA					 GPIO_PIN_1
	
	#define SWITCH_IO_GROUP  GPIOC

  #define	SWITCH_BIT5      GPIO_PIN_2
	#define	SWITCH_BIT6      GPIO_PIN_3
	#define	SWITCH_BIT7      GPIO_PIN_4
	#define	SWITCH_BIT8      GPIO_PIN_5
	
	
	#define UART_TX_Pin GPIO_PIN_10
	#define UART_TX_GPIO_Port GPIOC
	#define UART_RX_Pin GPIO_PIN_11
	#define UART_RX_GPIO_Port GPIOC   //all UART4
	#define USARTx  UART4

	


#endif



//#define LCD_RS_Pin GPIO_PIN_6
//#define LCD_RS_GPIO_Port GPIOB
#define LCD_NSS_Pin GPIO_PIN_7
#define LCD_NSS_GPIO_Port GPIOB
#define LCD_SCK_Pin GPIO_PIN_13
#define LCD_SCK_GPIO_Port GPIOB
#define LCD_MISO_Pin GPIO_PIN_14
#define LCD_MISO_GPIO_Port GPIOB
#define LCD_MOSI_Pin GPIO_PIN_15
#define LCD_MOSI_GPIO_Port GPIOB
//#define LCD_RW_Pin GPIO_PIN_6
//#define LCD_RW_GPIO_Port GPIOC







#define DW1000_RSTn					DW_RESET_Pin
#define DW1000_RSTn_GPIO			DW_RESET_GPIO_Port
#define DECAIRQ                     DW_IRQn_Pin
#define DECAIRQ_GPIO                DW_IRQn_GPIO_Port




/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
