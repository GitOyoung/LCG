/**
  ******************************************************************************
  * @file    st7580_stm32_bsp.c
  * @author  IMS Systems Lab & Technical Marketing
  * @version V3.1.0
  * @date    09-May-2012
  * @brief  
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
  
/* Includes ------------------------------------------------------------------*/
#include "st7580_stm32_bsp.h"


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"

void vUARTInterruptHandler( void );
void vUART_USBInterruptHandler( void );
void Modem_GPIO_Config(void);
void Modem_USART_Config( unsigned long ulWantedBaud  );

USART_TypeDef* COM_USART[COMn] = {EVAL_COM1, EVAL_COM2}; 

GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT, EVAL_COM2_TX_GPIO_PORT};

GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT, EVAL_COM2_RX_GPIO_PORT};
 
const uint32_t COM_USART_CLK[COMn] = {EVAL_COM1_CLK, EVAL_COM2_CLK};

const uint32_t COM_TX_PORT_CLK[COMn] = {EVAL_COM1_TX_GPIO_CLK, EVAL_COM2_TX_GPIO_CLK};
 
const uint32_t COM_RX_PORT_CLK[COMn] = {EVAL_COM1_RX_GPIO_CLK, EVAL_COM2_RX_GPIO_CLK};

const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN, EVAL_COM2_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN, EVAL_COM2_RX_PIN};


/* xSerialGetChar Global Variables */
extern u8 RxBuff[260];
u8 RxState          = 0;
u32 RxPending       = 0;
extern u8 SerBuff[260];
u32 RxBytes         = 0;
static u32 SerCount = 0;
extern u16 s1mSecTime;

/* UARTInterruptHandler Global Variables */
#define TX_SER_BUFF_SIZE  260
#define serNO_BLOCK ( 0 )
static u32 TxHead = 0;
static u32 TxEnd  = 0;
static u8 xCharsForTx[TX_SER_BUFF_SIZE];
signed long xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, long xBlockTime );
extern void SER_Receive( char cValue );

/* vUART_USBInterruptHandler Global Variables */
extern void USART_To_USB_Send_Data(void);


/*******************************************************************************
* Function Name  : STM_EVAL_COMInit
* Description    : STM_EVAL_COMInit USATA PER USB!!!!
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void STM_EVAL_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM], ENABLE);

  /* Enable UART clock */
  if (COM == COM1)
  {
    RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE); 
  }
  else
  {
    RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
  }

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin   = COM_TX_PIN[COM];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);
  
  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Pin   = COM_RX_PIN[COM];
  GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(COM_USART[COM], USART_InitStruct);
    
  /* Enable USART */
  USART_Cmd(COM_USART[COM], ENABLE);
}

/**
  * @brief  DeInitializes the SDIO interface.
  * @param  None
  * @retval None
  */
void SD_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /*!< Disable SDIO Clock */
  SDIO_ClockCmd(DISABLE);
  
  /*!< Set Power State to OFF */
  SDIO_SetPowerState(SDIO_PowerState_OFF);

  /*!< DeInitializes the SDIO peripheral */
  SDIO_DeInit();
  
  /*!< Disable the SDIO AHB Clock */
  RCC_AHB2PeriphClockCmd(RCC_APB2Periph_SDIO, DISABLE);

  /*!< Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /*!< Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/**
  * @brief  Initializes the SD Card and put it into StandBy State (Ready for 
  *         data transfer).
  * @param  None
  * @retval None
  */
void SD_LowLevel_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /*!< GPIOC and GPIOD Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | SD_DETECT_GPIO_CLK, ENABLE);

  /*!< Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /*!< Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /*!< Configure SD_SPI_DETECT_PIN pin: SD Card detect pin */
  GPIO_InitStructure.GPIO_Pin   = SD_DETECT_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);

  /*!< Enable the SDIO AHB Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);

  /*!< Enable the DMA2 Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
}

/**
  * @brief  Configures the DMA2 Channel4 for SDIO Tx request.
  * @param  BufferSRC: pointer to the source buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
//void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize)
//{

//  DMA_InitTypeDef DMA_InitStructure;


//}

/**
  * @brief  Configures the DMA2 Channel4 for SDIO Rx request.
  * @param  BufferDST: pointer to the destination buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
//void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize)
//{
//  DMA_InitTypeDef DMA_InitStructure;

//}

/*******************************************************************************
* Function Name  : SD_DMAEndOfTransferStatus.
* Description    : SD_DMAEndOfTransferStatus.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
//uint32_t SD_DMAEndOfTransferStatus(void)
//{
//  return (uint32_t)0;
//}

/*******************************************************************************
* Function Name  : RTS_Write.
* Description    : RTS_Write.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void RTS_Write( uint8_t value )
{
     //for EVALST7580. use pin 8 for ipp001v2
  GPIO_WriteBit(GPIOA, GPIO_Pin_1, (BitAction)(value & 0x1) );  
}

void Modem_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  
   
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE );

  /* Configure modem Reset pin (PC15)*/
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOC, &GPIO_InitStructure );                
  GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET); 
  
   /* Configure modem T_REQ pin (PA1)*/
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; //GPIO_Mode_Out_OD; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOA, &GPIO_InitStructure );                
  GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
  
    /* Configure modem Rx pin (PC13)*/
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOC, &GPIO_InitStructure );
   
  /* Configure USART2 Rx (PA3) as input floating */
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init( GPIOA, &GPIO_InitStructure );

  /* Configure USART2 Tx (PA2) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init( GPIOA, &GPIO_InitStructure );
  
}


/*******************************************************************************
* Function Name  : Modem_USART_Config.
* Description    : Modem_USART_Config.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Modem_USART_Config( unsigned long ulWantedBaud  )
{
  USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef USART_ClockInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd( RCC_APB2Periph_USART6, ENABLE );
 
  USART_InitStructure.USART_BaudRate            = ulWantedBaud;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_ClockInitStructure.USART_Clock          = USART_Clock_Disable;
  USART_ClockInitStructure.USART_CPOL           = USART_CPOL_Low;
  USART_ClockInitStructure.USART_CPHA           = USART_CPHA_2Edge;
  USART_ClockInitStructure.USART_LastBit        = USART_LastBit_Disable;
  
  USART_ClockInit( USART6, &USART_ClockInitStructure);
  USART_Init( USART6, &USART_InitStructure );
                  
  USART_ClearITPendingBit(USART6, USART_IT_RXNE);
  USART_ClearITPendingBit(USART6, USART_IT_TXE);
  USART_ITConfig( USART6, USART_IT_RXNE, ENABLE );
  
  NVIC_InitStructure.NVIC_IRQChannel                   = USART6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init( &NVIC_InitStructure );
  
  USART_Cmd( USART6, ENABLE ); 
}



/*******************************************************************************
* Function Name  : EnableMCO.
* Description    : EnableMCO.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EnableMCO( void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );
  /* Output HSE clock on MCO pin */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);
}

/*******************************************************************************
* Function Name  : BSP_Init.
* Description    : BSP_Init.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void BSP_Init( unsigned long ulWantedBaud )
{
  /* Modem Config*/
  Modem_GPIO_Config();
  
  /* Configure modem Reset*/           
  Modem_USART_Config(ulWantedBaud );

  /* Timer Init*/
  SER_TimerInit();
}

/*******************************************************************************
* Function Name  : SER_TimerInit
* Description    : Initialize a timer for timeout management with 1 mSec resolution.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SER_TimerInit(void)
{
/* SETUP e START 1000 uSec. (1KHz) SYS TICK TIMER  base clock = 36MHz */
  
  // ((36 )/(36 + 1) = 1MHz -> 1000/1000=1KHz (1mS)
  
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
  
  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period        = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler     = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM2, 71, TIM_PSCReloadMode_Immediate); //div72

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse       = 1000;
  TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
  
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
  
  TIM_Cmd(TIM2, ENABLE);
  /* Output Compare Timing Mode configuration: Channel2 - Elio CSMA-CA*/
  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse       = 65535; //TIMER_SLOT_DURATION;
  TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

  TIM_OC2Init(TIM2, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

  /* Output Compare Timing Mode configuration: Channel3 - Elio  CSMA-CA*/
  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse       = 65535; //TIMER_SENSING_TIME;
  TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

  TIM_OC3Init(TIM2, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);

  TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);

  TIM_Cmd(TIM2, ENABLE);
}

/*******************************************************************************
* Function Name  : xSerialGetChar
* Description    : Initialize a timer for timeout management with 1 mSec resolution.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
signed long xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, long xBlockTime )
{
/* The port handle is not required as this driver only supports one port. */

  signed long ret = APP_FALSE;
  static u16 cTime;
  ( void ) pxPort;
  if ( RxBytes != 0 ) {
    memcpy( pcRxedChar, RxBuff, RxBytes );
    RxBytes = 0;
    RxState = 0;
    ret = APP_TRUE;
  }
  if ( RxPending != 0 ) {
      memcpy( RxBuff, SerBuff, RxPending );
      RxBytes = RxPending;
      RxPending = 0;
  }
  if ( SerCount <= 0 ) return ret;
   //check incomplete frames
  if ( RxState == 0 ) {
    cTime = s1mSecTime;
    RxState = 1;
  } else {
    if(s1mSecTime != cTime ) {
      if ( s1mSecTime > cTime ) {
        if ( (s1mSecTime - cTime) > 100 ) {
          SerCount = 0;
          RxState = 0;
        }
      } else if ( (s1mSecTime + (0xFFFF - cTime)) > 100 ) {
        SerCount = 0;
        RxState = 0;
      }
    }
  }
  
  return ret;
}

/*******************************************************************************
* Function Name  : vUARTInterruptHandler
* Description    : vUARTInterruptHandler
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
void vUARTInterruptHandler( void )
{
  char cChar;
  if( USART_GetITStatus( USART6, USART_IT_TXE ) == SET ) {
    /* The interrupt was caused by the THR becoming empty.  Are there any
    more characters to transmit? */
    if ( TxHead != TxEnd ) {
      cChar  = xCharsForTx[TxHead];
      TxHead = (TxHead + 1) % TX_SER_BUFF_SIZE;
      USART_SendData( USART6, cChar );
    } else {
      USART_ITConfig( USART6, USART_IT_TXE, DISABLE );
    }
    USART_ClearITPendingBit(USART6, USART_IT_TXE);
  }
  
  if( USART_GetITStatus( USART6, USART_IT_RXNE ) == SET ) {
    cChar = USART_ReceiveData( USART6 );
    SER_Receive( cChar );
    USART_ClearITPendingBit(USART6, USART_IT_RXNE);
  }
  if( USART_GetITStatus( USART6, USART_IT_ORE ) == SET ) {
    cChar = USART_ReceiveData( USART6 );
  } 
}
/**
 * Function Name   :  vSerialSendStart
 * Description     :  Start Send the Data of Buffer
 * Input           :  None
 * Output          :  None
 */
void vSerialSendStart(void) 
{
  signed char cChar;
  if( TxHead == TxEnd) return;
  USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
  cChar  = xCharsForTx[TxHead];
  TxHead = (TxHead + 1) % TX_SER_BUFF_SIZE;
  USART_SendData( USART6, cChar );
}

void vSerialPutBuff( xComPortHandle pxPort, signed char * pcString, unsigned short usStringLength )
{
  signed char *pxNext;

  /* A couple of parameters that this port does not use. */
  ( void ) pxPort;

  /* NOTE: This implementation does not handle the queue being full as no
  block time is used! */

  /* The port handle is not required as this driver only supports UART1. */
  ( void ) pxPort;

  /* Send each character in the string, one at a time. */
  pxNext = pcString;
  while( usStringLength-- ) {
    xSerialPutChar( pxPort, *pxNext, serNO_BLOCK );
    pxNext++;
  }
}

signed long xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, long xBlockTime )
{
  u32 next = ((TxEnd + 1) % TX_SER_BUFF_SIZE);
  /*buffer full -> escape, return failed*/
  if ( next == TxHead )  return 0;
  xCharsForTx[TxEnd] = cOutChar;    
  TxEnd              = next;
  USART_ITConfig( USART2, USART_IT_TXE, ENABLE );
  return 1;
}

/*******************************************************************************
* Function Name  : vUART_USBInterruptHandler
* Description    : vUART_USBInterruptHandler
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/

void vUART_USBInterruptHandler( void )
{
  extern  u8 SwitchSelec;
  if (USART_GetITStatus(EVAL_COM2, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(EVAL_COM2, USART_IT_RXNE);
    /* Send the received data to the PC Host*/
//    USART_To_USB_Send_Data();
  }

  /* If overrun condition occurs, clear the ORE flag and recover communication */
  if (USART_GetFlagStatus(EVAL_COM2, USART_FLAG_ORE) != RESET) {
    USART_ClearITPendingBit(EVAL_COM2, USART_FLAG_ORE);
    (void)USART_ReceiveData(EVAL_COM2);
  } 
}


//CSMA-CA Section
/*******************************************************************************
* Function Name  : TimerStart
* Description    : TimerStart CSMA Section
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
u32 TimerStart (TIMER_T *timer)
{
    timer->Enable = APP_TRUE;
    return timer->Counter;
}
/*******************************************************************************
* Function Name  : TimerExpired
* Description    : TimerExpired CSMA Section
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
u8 TimerExpired (TIMER_T *timer)
{
  if (timer->Enable != APP_TRUE 
    || timer->Counter != 0) return APP_FALSE;  
  timer->Enable = APP_FALSE;     
  return APP_TRUE;
}
/*******************************************************************************
* Function Name  : TimerStop
* Description    : TimerStop CSMA Section
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
u32 TimerStop (TIMER_T *timer)
{
    timer->Enable = APP_FALSE;
    return timer->Counter;
}
/*******************************************************************************
* Function Name  : TimerSet
* Description    : TimerSet CSMA Section
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
u8 TimerSet (TIMER_T *timer, u32 value)
{
    timer->Counter = value;
    return TIMER_NOERR;
}
/*******************************************************************************
* Function Name  : TimerUpdate
* Description    : TimerUpdate CSMA Section
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
void TimerUpdate (TIMER_T *timer, long value)
{
    timer->Counter += value;
}
/*******************************************************************************
* Function Name  : isTimerActive
* Description    : isTimerActive CSMA Section
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
u8 isTimerActive (TIMER_T *timer)
{
    return timer->Enable;
}
/*******************************************************************************
* Function Name  : TimerGet
* Description    : TimerGet CSMA Section
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
u32 TimerGet (TIMER_T *timer)
{
    return timer->Counter;
}
/*******************************************************************************
* Function Name  : getTimeStamp
* Description    : getTimeStamp CSMA Section
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
u32 getTimeStamp()
{
    return s1mSecTime;
}



    
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
