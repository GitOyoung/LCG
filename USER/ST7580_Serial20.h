/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : ST7580_Serial20.h
* Author             : IMS Systems Lab
* Version            : V0.1
* Date               : 03/25/2009
* Description        : ST7580 communication management with handshaking
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/
#ifndef _ST7580_SERIAL_H_
#define _ST7580_SERIAL_H_


#include "st7580_stm32_bsp.h"

#define BUFF_SIZE 254
#define STATUS_SIZE 2
#define COM_BAUD_RATE	( 57600 )
#define comBUFFER_LEN	1

#define STX 0x02
#define STXR 0x03
#define ACK 0x06
#define NAK 0x15
#define Q_MARK 0x3F

#define CMD_ERROR -1
#define CMD_OK  0

#define IS_SET  0x01
#define IS_TX   0x02
#define IS_RX   0x04
#define RESET_MASK  0x30
#define TXIMP_MASK  0x1F
#define TEMP_MASK   0x60
// Keep Alive Section
#define KEEP_ALIVE   

////CSMACA
#define UART_BYTE_TRANSFER_TIME    (uint16_t) 220  /* time to transfer 1 single byte on the uart (217 us) */
#define UART_CONST_CONTRIB         (uint16_t) 460  /* constant delay through uart transmission (460 us) */
#define PREAMBLE_BYTE_DURATION     (uint16_t) 840  /* duration of a single byte of the preamble (832 us) */
#define PREAMBLE_CONST_CONTRIB     (uint16_t) 320  /* constant delay for preamble transmission (316 us) */
#define SENSING_SAFE_INTERVAL      (uint16_t) 220//200  /* safe interval for sensing added to expected PL_RX_ON raising time (us) */
#define SENSING_SLOT_DIFFERENCE    (uint16_t) 900//1000  /* amount of time between sensing and slot end/begin (us) */

#define USE_CUSTOM_ST7580

#ifdef USE_CUSTOM_ST7580
#define T_REQ_END  GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET)
#define T_REQ_START  GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET)

#define MODEM_RESET  GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET)
#define MODEM_START  GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET)

#endif//USE_EVAL_ST7580

#ifdef USE_EVAL_ST7580
#define T_REQ_END  GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET)
#define T_REQ_START  GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET)

#define MODEM_RESET  GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET)
#define MODEM_START  GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET)

#endif//USE_EVAL_ST7580

#ifdef JB_BASIC
#define T_REQ_END  GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_SET)
#define T_REQ_START  GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_RESET)
#endif

#ifdef DELTA_DPS_240VB
#define T_REQ_END  GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET)
#define T_REQ_START  GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET)
#endif//DELTA_DPS_240VB

typedef struct {
  u8 stx;
  u8 length;
  u8 cmd;
  u8 data[256];  
  u16 chksum;
} SER_FRAME_T;

typedef struct{
  u8 id;
  u8 Rxpar_len;
  u8 Txpar_len;
  u8 Rxparam[255];
  u8 Txparam[255];
} SER_CMD_FRAME_T;

typedef struct {
  u8 start;
  u8 status0;
  u8 status1;
  u8 reserved;
} SER_STATUS_T;


enum {
  M_CONFIGURED = 1,
  M_NOT_CONFIGURED,
  M_WAIT_INIT,
  M_FRAME_ACKED,
  M_FRAME_NOT_ACKED,
  M_WAIT_ACK,
  M_FRAME_RECEIVED,
  M_IDLE,
  M_FRAME_VALID,
  M_FRAME_NOT_VALID,
  M_FRAME_DELIVERED,
  M_WAIT_STATUS,
  M_FRAME_CONFIRMED,
  M_FRAME_NOT_CONFIRMED,
  M_WAIT_CONFIRM,
  M_TO_ELAPSED,
  M_CSMA_CA,
  M_KEEP_ALIVE,
  M_KA_WAIT,
  M_KA_OK,
  M_KA_FAILED,
};


u8 ModemComFSM( SER_CMD_FRAME_T* frame, u8 Tx);
void SER_Receive( char cValue );
//void SER_TimerInit(void);
void SER_SetTimeOut(u16 Time);
u8 SER_CheckTimeOut(void);
//u16 SER_GetTick(void);
void SER_TimerHandler(void);
void SER_StopTimeOut(void);
//void vSerialPutBuff( xComPortHandle pxPort, signed char * pcString, unsigned short usStringLength );

#endif //_ST7580_SERIAL_H_