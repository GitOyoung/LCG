/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : ST7580_Serial20.c
* Author             : IMS Systems Lab
* Version            : V1.0
* Date               : 08/03/2009
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


/* Demo program include files. */
#include <stdlib.h>
#include "ST7580_Serial20.h"

#define comRX_BLOCK_TIME  1
#define TIME_OUT_TICK 50000
//#define COLL_AVOID

#ifdef COLL_AVOID
#define DIFS_TIME (2 *8 + 4) //8 mSec. (due to modem header recognition) + DIFS time
#define CW_MIN    1
#define CW_MAX  60 // collision window max (number of expected station)
#define SLOT_TIME 5 //20 //mSec. tempo di vulnerabilita' = 2 volte il Tempo di propagazione
                     // backoff-time = random[0-CW] x SLOT_TIME
#define WAS_FREE    1
#define WAS_BUSY    (!WAS_FREE)
#define  MAC_DATA_REQ 0x50
#define  SS_DATA_REQ  0x54
#endif //COLL_AVOID

#define UIDMCU0              ((u32 *)0x1FFFF7EE)
#define UIDMCU1              ((u32 *)0x1FFFF7EF)
#define UIDMCU2              ((u32 *)0x1FFFF7F0)
#define UIDMCU3              ((u32 *)0x1FFFF7F1)
#define UIDMCU4              ((u32 *)0x1FFFF7F2)
#define UIDMCU5              ((u32 *)0x1FFFF7F3)

//CSMA ADJUST Time
#define PACKET_SIZE     5
#define UART_BYTE_TRANSFER_TIME    (uint16_t) 220  /* time to transfer 1 single byte on the uart (217 us) */
#define UART_CONST_CONTRIB         (uint16_t) 460  /* constant delay through uart transmission (460 us) */
#define PREAMBLE_BYTE_DURATION     (uint16_t) 840  /* duration of a single byte of the preamble (832 us) */
#define PREAMBLE_CONST_CONTRIB     (uint16_t) 320  /* constant delay for preamble transmission (316 us) */
#define PREAMBLE_PREAM_LEN      2

#define TIMER_SENSING_TIME  (PACKET_SIZE + 5) * UART_BYTE_TRANSFER_TIME + UART_CONST_CONTRIB +\
        (PREAMBLE_PREAM_LEN+2+1) * PREAMBLE_BYTE_DURATION + PREAMBLE_CONST_CONTRIB +\
        + SENSING_SAFE_INTERVAL 


static u8 state = 0;//Start State
static u32 TimeOutCount = TIME_OUT_TICK;

s8 SER_Send_CMD( u8 op, u8 par_len, u8* param );
SER_STATUS_T* SER_GET_STATUS(void);
SER_FRAME_T* SER_GET_FRAME(void);
u8* SER_GET_ACK(void);
void SER_SEND_ACK (void);
void SER_SEND_NAK (void);
u8 CheckTimeOut(void);
void SetTimeOut(u32 tick);
extern void USB_TimeOut(u8 status);

/*-----------------------------------------------------------*/

static SER_FRAME_T TxFrame;
static SER_FRAME_T RxFrame;
static SER_STATUS_T SerStatus;
static u8 SerAck;

#ifdef COLL_AVOID
TIMER_T SensingTimer = {APP_FALSE, 0};
TIMER_T BackOffTimer = {APP_FALSE, 0};
u32 CollWin = CW_MAX;
#endif //COLL_AVOID

TIMER_T TimeoutTimer = {APP_FALSE, 0};

u16 s1mSecTime = 0;
extern char sensing_elapsed,slot_elapsed;

u8 SerBuff[260];
u8 RxBuff[260];
static u32 SerCount = 0;
extern u32 RxBytes;
extern u32 RxPending;
extern u32 RxState;
static u32 SerLength = 0;
u8 FirstC;

extern TIMER_T csmaca_backoff_timer,timerAS,TransmissionTimeout,hostIFwaiting;

#ifdef COLL_AVOID
/*******************************************************************************
* Function Name  : StartSensingTimer
* Description    : StartSensingTimer
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void StartSensingTimer(u8 ActiveRand)
{
  SensingTimer.Counter = DIFS_TIME * (1 + ActiveRand*rand() % 10);
  SensingTimer.Enable = APP_TRUE;  
}

/*******************************************************************************
* Function Name  : StopBackOffTimer
* Description    : StopBackOffTimer
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void StopBackOffTimer(void)
{
  BackOffTimer.Enable = APP_FALSE;  
}

/*******************************************************************************
* Function Name  : StartBackOffTimer
* Description    : StartBackOffTimer
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void StartBackOffTimer(void)
{
  BackOffTimer.Enable = APP_TRUE;  
}

/*******************************************************************************
* Function Name  : SetBackOffTimer
* Description    : SetBackOffTimer
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SetBackOffTimer(u32 value)
{
  BackOffTimer.Counter = value;  
}

/*******************************************************************************
* Function Name  : SensingTimerExpired
* Description    : SensingTimerExpired
* Input          : None
* Output         : None
* Return         : TRUE/FALSE
*******************************************************************************/
u8 SensingTimerExpired(void)
{
  if ( (SensingTimer.Counter != 0) 
    || (SensingTimer.Enable != APP_TRUE) ) return APP_FALSE;
  SensingTimer.Enable = APP_FALSE;
  return APP_TRUE;
}

/*******************************************************************************
* Function Name  : BackOffTimerExpired
* Description    : BackOffTimerExpired
* Input          : None
* Output         : None
* Return         : TRUE/FALSE
*******************************************************************************/
u8 BackOffTimerExpired(void)
{
  if ( (BackOffTimer.Counter != 0) 
    || (BackOffTimer.Enable != APP_TRUE) ) return APP_FALSE;
  BackOffTimer.Enable = APP_FALSE;
  return APP_TRUE;
}

/*******************************************************************************
* Function Name  : CalcBackOff
* Description    : backoff-time = random[0-CW] x SLOT_TIME
* Input          : None
* Output         : None
* Return         : backoff-time
*******************************************************************************/
u32 CalcBackOff(void)
{
  static u32 Backoff;

  Backoff = (rand() % CollWin + 1) * SLOT_TIME;
  return Backoff;

}
#endif //COLL_AVOID

/*******************************************************************************
* Function Name  : SER_TimerHandler
* Description    : IRQ handler for timer, 1ms to sincrhronize the communication
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SER_TimerHandler(void)
{
    unsigned short capture = 0;  
    /* operations for channel 1 event - Keep Alive timer */
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
    {
        s1mSecTime++;           /* NEEDED FOR SYNCHRONIZE COMMUNICATIONS WITH ST7580 ! */
        
        #ifdef  KEEP_ALIVE
          if((TimeoutTimer.Counter > 0) && (TimeoutTimer.Enable == APP_TRUE)){
            TimeoutTimer.Counter--;
        }
        #endif 
          
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);                /* clear pending interrupt flag */
        capture = TIM_GetCapture1(TIM2);                        /* get the updated counter */
        TIM_SetCompare1(TIM2, capture + 1000 );                 /* set the new compare value */
    }

    /* operations for channel 2 event */
    if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
    {
        /* do stuff */
//        slot_elapsed = 1;

        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);                /* clear pending interrupt flag */
        capture = TIM_GetCapture2(TIM2);                        /* get the updated counter */
        TIM_SetCompare2(TIM2, capture + TIMER_SENSING_TIME );                 /* set the new compare value */
    }

    /* operations for channel 3 event */
    if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
    {
//        if (csmaca_backoff_timer.Enable == APP_TRUE && csmaca_backoff_timer.Counter>0)
//        {
//            csmaca_backoff_timer.Counter--;
//        }

        TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);                /* clear pending interrupt flag */
        /* set sensing timer also */
        capture = TIM_GetCapture3(TIM2);                        /* get the updated counter */
        TIM_SetCompare3(TIM2, capture + TIMER_SENSING_TIME );               /* set the new compare value */

        /* SENSING */
//        sensing_elapsed = 1;
    }

}

/*******************************************************************************
* Function Name  : SER_SetTimer
* Description    : SER_SetTimer
* Input          : Time in 1mSec step 
* Output         : None
* Return         : None
*******************************************************************************/
void SER_SetTimeOut(u16 Time)
{
  TimeoutTimer.Counter = Time;
  TimeoutTimer.Enable = APP_TRUE;  
}

/*******************************************************************************
* Function Name  : SER_StopTimer
* Description    : SER_StopTimer
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SER_StopTimeOut(void)
{
  TimeoutTimer.Enable = APP_FALSE;  
}
/*******************************************************************************
* Function Name  : SER_CheckTimeOut
* Description    : SER_CheckTimeOut
* Input          : None 
* Output         : None
* Return         : TRUE/FALSE
*******************************************************************************/
u8 SER_CheckTimeOut(void)
{
  if ( (TimeoutTimer.Counter == 0) && (TimeoutTimer.Enable == APP_TRUE)) {
    TimeoutTimer.Enable = APP_FALSE;
    return APP_TRUE;
  }
  return APP_FALSE;
}

/*******************************************************************************
* Function Name  : SER_Send_CMD
* Description    : Send command to the ST7580 power line modem.
* Input          : command code, parameters lenght, pointer to parameters
* Output         : None
* Return         : Error code CMD_ERROR/CMD_OK
*******************************************************************************/
s8 SER_Send_CMD( u8 op, u8 par_len, u8* param )
{
  s8 status;
  u32 i;
  u16 ChkSum = 0;
  if ( par_len > BUFF_SIZE ) {  
    status = CMD_ERROR;
  } else {
    TxFrame.stx = STX;
    TxFrame.length = par_len; //3 + par_len;
    ChkSum+=TxFrame.length;
    TxFrame.cmd = op;
    ChkSum+=TxFrame.cmd;
    for ( i=0; i<par_len; i++ ) {
      TxFrame.data[i] = param[i];
      ChkSum+=TxFrame.data[i];
    }
    TxFrame.chksum = ChkSum;
    TxFrame.data[i++] = TxFrame.chksum & 0xFF;
    TxFrame.data[i] = TxFrame.chksum >> 8;
    
    vSerialPutBuff( NULL, (signed char*)&TxFrame, TxFrame.length + 5 );
    status = CMD_OK;
  }
  return status;
}


/*******************************************************************************
* Function Name  : SER_GET_FRAME
* Description    : Read received frame from the ST7580 power line modem.
* Input          : None
* Output         : None
* Return         : Pointer to the received frame
*******************************************************************************/
SER_FRAME_T* SER_GET_FRAME( void )
{
  SER_FRAME_T* ret = NULL;
  u8 pRxBuffer[260];
  u32 i;
  u16 chksum = 0;
  
  if( xSerialGetChar( NULL, pRxBuffer, comRX_BLOCK_TIME ) )
  {
    if (( pRxBuffer[0] == STX )||( pRxBuffer[0] == STXR )) //valid frame
    {
      RxFrame.stx = pRxBuffer[0];
      RxFrame.length = pRxBuffer[1];
      chksum += RxFrame.length;
      RxFrame.cmd = pRxBuffer[2];
      chksum += RxFrame.cmd;
      for ( i=0; i<RxFrame.length; i++ ){
        RxFrame.data[i] = pRxBuffer[i+3];
        chksum += RxFrame.data[i];
      }
      RxFrame.chksum = (pRxBuffer[i+3+1] << 8) | pRxBuffer[i+3] ;
      if ( RxFrame.chksum == chksum ) { //no frame error
        ret = &RxFrame;
        SER_SEND_ACK();
      } else {
        SER_SEND_NAK();
      }
    }
  }
  return ret;
}

/*******************************************************************************
* Function Name  : SER_GET_STATUS
* Description    : Read status frame from the ST7580 power line modem.
* Input          : None
* Output         : None
* Return         : Pointer to the status frame
*******************************************************************************/
SER_STATUS_T* SER_GET_STATUS( void )
{
  SER_STATUS_T* ret = NULL;
  s8 pRxBuffer[260];
    
  if( xSerialGetChar( NULL, pRxBuffer, comRX_BLOCK_TIME ) )
  {
    if ( pRxBuffer[0] == Q_MARK ) //valid status
    {
      SerStatus.start = pRxBuffer[0];        
      SerStatus.status0 = pRxBuffer[1];
      ret = &SerStatus;
    }
  }
  return ret;
}

/*******************************************************************************
* Function Name  : SER_GET_ACK
* Description    : Read ack frame from the ST7580 power line modem.
* Input          : None
* Output         : None
* Return         : Pointer to the ack frame
*******************************************************************************/
u8* SER_GET_ACK( void )
{
  u8* ret = NULL;
  s8 pRxBuffer[260];
  
  if( xSerialGetChar( NULL, pRxBuffer, comRX_BLOCK_TIME ) )
  {
    if ( pRxBuffer[0] == ACK || pRxBuffer[0] == NAK ) //valid ack
    {
      SerAck = pRxBuffer[0];
      ret = &SerAck;
    }
  }
  return ret;
}

/*******************************************************************************
* Function Name  : SER_SEND_ACK
* Description    : Send ack frame to the ST7580 power line modem.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SER_SEND_ACK ( void )
{
  static signed char Tx = ACK;
  vSerialPutBuff( NULL, (signed char*)&Tx, 1 );
}


/*******************************************************************************
* Function Name  : SER_SEND_NAK
* Description    : Send not ack frame to the ST7580 power line modem.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SER_SEND_NAK ( void )
{
  static signed char Tx = NAK;
  vSerialPutBuff( NULL, (signed char*)&Tx, 1 );
}


/*******************************************************************************
* Function Name  : ModemComFSM
* Description    : FSM function for communication with ST7580.
* Input          : apdu, transmission/reception procedure command
* Output         : apdu
* Return         : FSM status
*******************************************************************************/
u8 debug_flag = 0;
u8 ModemComFSM( SER_CMD_FRAME_T* apdu, u8 Tx )
{
  u8 len;
  u8* ConfRsp;
  static SER_STATUS_T* c_status;
  SER_FRAME_T* frame;
  static u8 u8_cmd;
  static u8 u8_cmd_len;
  static u8* parameters;
  static u8 ret;

  #ifdef COLL_AVOID
  static u8 ChannelStatus = WAS_BUSY;
  u32 BackOffTime = 0;
  #endif  //COLL_AVOID

  u32 i;

  switch(state) {
  case 0:
    #ifdef COLL_AVOID
      srand( *UIDMCU0 ^ *UIDMCU1 ^ *UIDMCU2 ^ *UIDMCU3 ^ *UIDMCU4 ^ *UIDMCU5 );
    #endif //COLL_AVOID
    state = 2;
    break;
  case 1: //wait ack
    ConfRsp = SER_GET_ACK();
    ret = M_WAIT_ACK;
    if ( ConfRsp != NULL ) {
      SetTimeOut(TIME_OUT_TICK);
      if ( (*ConfRsp) == ACK ) {
        *ConfRsp = 0;
        state    = 4;
        ret      = M_FRAME_ACKED;
      } else {
        *ConfRsp = 0;
        state    = 0;
        ret      = M_FRAME_NOT_ACKED;
      }
    }
    if (CheckTimeOut()) {
      SetTimeOut(TIME_OUT_TICK);
      ret   = M_TO_ELAPSED;
      state = 2;
      T_REQ_END;
    }
    break;
  case 2: //idle
    ret = M_IDLE;
    frame = SER_GET_FRAME();
    if ( debug_flag == 1 ) debug_flag = 0;    
    if ( Tx == 0 ){//Check RX data
      if ( frame != NULL ) {
        apdu->id        = frame->cmd;
        len             = frame->length;
        apdu->Rxpar_len = len;
        for (i=0; i<len; i++){
          apdu->Rxparam[i] = frame->data[i];
        }
        ret = M_FRAME_RECEIVED;
      }
    }
    else if ( Tx == 1 ) {// TX Data
      if ( apdu != NULL ){
        SetTimeOut(TIME_OUT_TICK);
        u8_cmd     = apdu->id;
        u8_cmd_len = apdu->Txpar_len;
        parameters = apdu->Txparam;
        SetTimeOut(TIME_OUT_TICK);
        T_REQ_START;
        
        state = 3;
        
        ret = M_FRAME_VALID;
      }
    }
    else if ( Tx == 2) {// Keep Alive
#ifdef KEEP_ALIVE
      SetTimeOut(TIME_OUT_TICK);
      T_REQ_START;
      state = 11;
#endif// KEEP_ALIVE          
      ret = M_FRAME_VALID;
    } else {
      ret = M_FRAME_NOT_VALID;
    }
    break;
  case 3: //get status/send command
    ret = M_WAIT_STATUS;
    if ( (c_status = SER_GET_STATUS()) != NULL ){
      T_REQ_END;
      SetTimeOut(TIME_OUT_TICK);
      SER_Send_CMD( u8_cmd, u8_cmd_len, parameters ); 
      state = 1;
      ret   = M_FRAME_DELIVERED;
    }
    if ( CheckTimeOut() ){
      SetTimeOut(TIME_OUT_TICK);
      ret   = M_TO_ELAPSED;
      state = 2;
      T_REQ_END;
    }
    break;
  case 4: //check confirm
    frame = SER_GET_FRAME();
    ret = M_WAIT_CONFIRM;
    if ( frame != NULL ) {
      SetTimeOut(TIME_OUT_TICK);
      if ( frame->cmd == apdu->id + 1 ) {//check command confirm
        ret = M_FRAME_CONFIRMED;
      } else {
        ret = M_FRAME_NOT_CONFIRMED;
        debug_flag = 1;
      }
      apdu->id        = frame->cmd;
      len             = frame->length;
      apdu->Rxpar_len = len;
      for (i = 0; i < len; i++) {
        apdu->Rxparam[i] = frame->data[i];
      }
      state = 2;
    }
    if ( CheckTimeOut() ) {
      SetTimeOut(TIME_OUT_TICK);
      ret   = M_TO_ELAPSED;
      state = 2;
      T_REQ_END;
    }    
    break;
    
#ifdef KEEP_ALIVE    
  case 11://Keep Alive Process
    ret = M_KA_WAIT;
    if ((c_status = SER_GET_STATUS()) != NULL ){
      T_REQ_END; 
      state = 2;
      ret   = M_KA_OK;
    }
    else if( CheckTimeOut() ){
      T_REQ_END;
      ret   = M_KA_FAILED;
      state = 2;
      T_REQ_END;
    }
    break;
#endif //KEEP_ALIVE
  } 
  return ret;
} /*lint !e715 !e818 pvParameters is required for a task function even if it is not referenced. */


/*******************************************************************************
* Function Name  : ModemReadRx
* Description    : ModemReadRx
* Input          : 
* Output         : 
* Return         : SER_STATUS_T
*******************************************************************************/
u8 ModemReadRx( void )
{

  static SER_STATUS_T* c_stat;  
  T_REQ_START;
  c_stat = SER_GET_STATUS();
  T_REQ_END;
  if ( c_stat != NULL && (c_stat->status0 & IS_RX) == IS_RX ) { //modem is receiving
     return 1;
  } 
  
  return 0;
}

/*******************************************************************************
* Function Name  : CheckTimeOut
* Description    : CheckTimeOut
* Input          : 
* Output         : 
* Return         : SER_STATUS_T
*******************************************************************************/
u8 CheckTimeOut(void)
{ 
  TimeOutCount--;
  return ( TimeOutCount == 0 ) ? 1 : 0;
}

/*******************************************************************************
* Function Name  : SetTimeOut
* Description    : SetTimeOut
* Input          : 
* Output         : 
* Return         : SER_STATUS_T
*******************************************************************************/
void SetTimeOut( u32 tick )
{
  TimeOutCount = tick;
}

/*************************************/
void SER_Receive( char cValue )
{
  if( SerCount == 0) {
    if ( cValue == ACK || cValue == NAK ) {
      SerBuff[SerCount] = cValue;
      SerCount++;
      if ( RxBytes == 0 ) {
        memcpy( RxBuff, SerBuff, SerCount );
        RxBytes = SerCount; 
      } else {
          RxPending = SerCount;
      }
      SerCount = 0;
    } else {
      if ( (cValue == Q_MARK) || (cValue == STX) || (cValue == STXR) ) {
        FirstC            = cValue;
        SerBuff[SerCount] = cValue;
        SerCount++;
      } else { //no valid sequence
        SerCount   = 0;
        FirstC     = 0;
        SerLength  = 0;
        SerBuff[0] = 0;
      }
    }
  }  else {
    switch (FirstC) {
    case Q_MARK: //status
      SerBuff[SerCount] = cValue;
      SerCount++;
      if ( SerCount >= STATUS_SIZE ) {
        T_REQ_END;
        FirstC = 0;
        if ( RxBytes == 0 ) {
          memcpy( RxBuff, SerBuff, SerCount );
          RxBytes = SerCount; 
        } else {
          RxPending = SerCount;
        }        
        SerCount = 0;
      }
      break;
    case STX: //Frame
    case STXR: //Frame Repeated  
      if ( SerCount == 1 ) {
        SerLength = cValue;
      }
      SerBuff[SerCount] = cValue;
      SerCount++;
      if ( SerCount >= (SerLength + 2 + 3) ) { //payload + command + checksum
        FirstC = 0;
        SerLength = 0;
        if ( RxBytes == 0 ) {
          memcpy( RxBuff, SerBuff, SerCount );
          RxBytes = SerCount; 
        } else {
          RxPending = SerCount;
        }
        SerCount = 0;
      }
      break;
    default:
      SerCount   = 0;
      FirstC     = 0;
      SerLength  = 0;
      SerBuff[0] = 0;
      RxState    = 0;
    }
  }
}

/*-----------------------------------------------------------*/
