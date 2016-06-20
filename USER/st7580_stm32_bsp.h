/**
  ******************************************************************************
  * @file    st7580_stm32_bsp.c
  * @author  IMS Systems Lab & Technical Marketing
  * @version V3.1.0
  * @date    09-May-2012
  * @brief  Header file for st7580_stm32_bsp.c module.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ST7580_STM32_EVAL_H
#define __ST7580_STM32_EVAL_H

#ifdef __cplusplus
  extern "C" {
#endif 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/**
 The table below gives an overview of the hardware resources supported.
     - LCD: TFT Color LCD (Parallel (FSMC) and Serial (SPI))
     - IOE: IO Expander on I2C
     - sFLASH: serial SPI FLASH (M25Pxxx)
     - sEE: serial I2C EEPROM (M24C08, M24C32, M24C64)
     - TSENSOR: Temperature Sensor (LM75)
     - SD: SD Card memory (SPI and SDIO (SD Card MODE)) 
  =========================================================+
    ST7580 BOARD   | LED | Buttons  | USB Ports | SD (SPI) |
  =========================================================+
    EVAL_ST7580    |  3  |    2     |     1     |    YES   |
  ---------------------------------------------------------+      
  =========================================================+
*/
/** 
  * @brief  Uncomment the line corresponding to the STMicroelectronics evaluation
  *   board used in your application.
  *   
  *  Tip: To avoid modifying this file each time you need to switch between these
  *       boards, you can define the board in your toolchain compiler preprocessor.    
  */ 
                    


/* Includes ------------------------------------------------------------------*/
#include "AppTypes.h"
#include "string.h"

#define COMn                             2

#define EVAL_COM1                        USART3
#define EVAL_COM1_CLK                    RCC_APB1Periph_USART3
#define EVAL_COM1_TX_PIN                 GPIO_Pin_10
#define EVAL_COM1_TX_GPIO_PORT           GPIOB
#define EVAL_COM1_TX_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define EVAL_COM1_RX_PIN                 GPIO_Pin_11
#define EVAL_COM1_RX_GPIO_PORT           GPIOB
#define EVAL_COM1_RX_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define EVAL_COM1_IRQn                   USART3_IRQn

#define EVAL_COM2                        USART6
#define EVAL_COM2_CLK                    RCC_APB2Periph_USART6
#define EVAL_COM2_TX_PIN                 GPIO_Pin_6
#define EVAL_COM2_TX_GPIO_PORT           GPIOC
#define EVAL_COM2_TX_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define EVAL_COM2_RX_PIN                 GPIO_Pin_7
#define EVAL_COM2_RX_GPIO_PORT           GPIOC
#define EVAL_COM2_RX_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define EVAL_COM2_IRQn                   USART6_IRQn

#define SD_DETECT_PIN                    GPIO_Pin_11                 /* PF.11 */
#define SD_DETECT_GPIO_PORT              GPIOF                       /* GPIOF */
#define SD_DETECT_GPIO_CLK               RCC_AHB1Periph_GPIOB
#define SDIO_FIFO_ADDRESS                ((uint32_t)0x40018080)

#define TIMER_NOERR             ((uint8_t)0x02)


//T_REQ
#define BSP_T_REQ_END  GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET)
#define BSP_T_REQ_START  GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET)

#define BSP_MODEM_RESET  GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET)
#define BSP_MODEM_START  GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET)


typedef void * xComPortHandle;  

typedef enum {
  COM1 = 0,
  COM2 = 1
} COM_TypeDef;   


typedef struct {
  u8 Enable;
  u32 Counter;
} TIMER_T;

//CSMA-CA   Channel status Check (read gpio status pin)
#define CHANNEL_STATUS  GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)

void          STM_EVAL_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct);
void          RTS_Write( uint8_t value );
void          EnableMCO(void);
void          BSP_Init( unsigned long ulWantedBaud );
void          SER_TimerInit(void);
signed long   xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, long xBlockTime );
void          vSerialPutBuff( xComPortHandle pxPort, signed char * pcString, unsigned short usStringLength );
void          vSerialSendStart(void);


u32   TimerStart(TIMER_T *timer);
u8    TimerExpired(TIMER_T *timer);
u8    TimerSet(TIMER_T *timer, u32 value);
u32   TimerGet(TIMER_T *timer);


#ifdef __cplusplus
  }
#endif 

#endif /* __ST7580_STM32_EVAL_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
