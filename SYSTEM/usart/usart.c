/*
* file: usart.c
* author: Jeans Oyoung
* created: June 23, 2016
 */
#include "usart.h"
#include "stm32f4xx_usart.h"
#include "buffer.h"

#include <stdio.h>
#include <string.h>


void USART1_Init(void);
void USART2_Init(void);
void USART3_Init(void);
void USART4_Init(void);
void USART5_Init(void);
void USART6_Init(void);

static USART_TypeDef *Coms[COMn] = {0, 0, 0, 0, 0, 0 };
static Buffer *ComBuffers[COMn] = {0, 0, 0, 0, 0, 0 };


#ifdef  USE_USART1
static ReceiveHandler U1_Receive;
static Buffer Com1TxBuffer;
#endif

#ifdef  USE_USART2
static ReceiveHandler U2_Receive;
static Buffer Com2TxBuffer;
#endif

#ifdef  USE_USART3
static ReceiveHandler U3_Receive;
static Buffer Com3TxBuffer;
#endif

#ifdef  USE_USART4
static ReceiveHandler U4_Receive;
static Buffer Com4TxBuffer;
#endif

#ifdef  USE_USART5
static ReceiveHandler U5_Receive;
static Buffer Com5TxBuffer;
#endif

#ifdef  USE_USART6
static ReceiveHandler U6_Receive;
static Buffer Com6TxBuffer;
#endif




void InitCom(Com_TypeDef com)
{
	switch(com)
	{
#ifdef USE_USART1
		case COM1:
			USART1_Init();break;
#endif
#ifdef USE_USART2
		case COM2:
			USART2_Init();break;
#endif
#ifdef USE_USART3
		case COM3:
			USART3_Init();break;
#endif
#ifdef USE_USART4
		case COM4:
			USART4_Init();break;
#endif
#ifdef USE_USART5
		case COM5:
			USART5_Init();break;
#endif
#ifdef USE_USART6
		case COM6:
			USART6_Init();break;
#endif
		default:break;
	}
}

u8 SerialPutChar(Com_TypeDef com, const char value)
{
	Buffer *txBuffer = ComBuffers[com];
	if(BF_Full(txBuffer)) return 0;
	BF_Write(txBuffer, &value, 1);
	SerialSendStart(com);
	return 1;
}

void SerialPutBuffer(Com_TypeDef com,  const char *buffer, int size)
{
	const char *pb = buffer;
	while(size-- && SerialPutChar(com, *pb)) {
		pb++;
	}
}



void SerialPutString(Com_TypeDef com, const char *string)
{
	int length = strlen(string);
	SerialPutBuffer(com, string, length);
}

void SerialSendStart(Com_TypeDef com)
{
	USART_TypeDef *Ux = Coms[com];
	USART_ITConfig(Ux, USART_IT_TXE, ENABLE );

}
	
void SerialHandlerSet(Com_TypeDef com, ReceiveHandler handler)
{
	switch(com)
	{
		
#ifdef USE_USART1
		case COM1:
			U1_Receive = handler;break;
#endif
#ifdef USE_USART2
		case COM2:
			U2_Receive = handler;break;
#endif
#ifdef USE_USART3
		case COM3:
			U3_Receive = handler;break;
#endif
#ifdef USE_USART4
		case COM4:
			U4_Receive = handler;break;
#endif
#ifdef USE_USART5
		case COM5:
			U5_Receive = handler;break;
#endif
#ifdef USE_USART6
		case COM6:
			U6_Receive = handler;break;
#endif
		default:break;
	}
}

/*---------------------private functions--------------------------------------------*/
	
#ifdef USE_USART1
/*
* 串口1初始化
*/
void USART1_Init()
{
	
	//TODO
	GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
  //串口1对应引脚复用映射
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
  
  //USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
  USART_InitStructure.USART_BaudRate = COM1_BAUDRATE;//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  
  USART_Cmd(USART1, ENABLE);  //使能串口1 
  
  USART_ClearFlag(USART1, USART_FLAG_TC);
  
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 3;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;    //子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;      //IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器、

	
	//
	Coms[COM1] = USART1;
	BF_Init(&Com1TxBuffer);
	ComBuffers[COM1] = &Com1TxBuffer;
}

/*
* 串口1中断
*/
void USART1_IRQHandler()
{

  if( USART_GetITStatus( USART1, USART_IT_TXE ) == SET ) {
    /* The interrupt was caused by the THR becoming empty.  Are there any
    more characters to transmit? */

    if(BF_Empty(&Com1TxBuffer)) {
			USART_ITConfig( USART1, USART_IT_TXE, DISABLE );
		} else {
			USART_SendData(USART1, BF_ReadByte(&Com1TxBuffer));
		}
    USART_ClearITPendingBit(USART1, USART_IT_TXE);
  }
  
  if( USART_GetITStatus( USART1, USART_IT_RXNE ) == SET ) {
    if(U1_Receive) U1_Receive( USART_ReceiveData( USART1 ));
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
  if( USART_GetITStatus( USART1, USART_IT_ORE ) == SET ) {
		USART_ReceiveData( USART1);
  }	
}


#endif /*----------USE_USART1  END-------*/

#ifdef USE_USART2
/*
* 串口2初始化
*/	
void USART2_Init()
{
	//TODO
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);


	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 

	USART_InitStructure.USART_BaudRate = COM2_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;        
	USART_Init(USART2, &USART_InitStructure);         
	USART_Cmd(USART2, ENABLE);  //使能串口2
	 
	USART_ClearFlag(USART2, USART_FLAG_TC);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//
	//Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =3;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;       //
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //
	
	NVIC_Init(&NVIC_InitStructure); //
	Coms[COM2] = USART2;
	BF_Init(&Com2TxBuffer);
	ComBuffers[COM2] = &Com2TxBuffer;
}

/*
* 串口2中断
*/
void USART2_IRQHandler()
{
  if( USART_GetITStatus( USART2, USART_IT_TXE ) == SET ) {
    if(BF_Empty(&Com2TxBuffer)) {
			USART_ITConfig(USART2, USART_IT_TXE,DISABLE);
		} else {
			USART_SendData(USART2, BF_ReadByte(&Com2TxBuffer));
		}
    USART_ClearITPendingBit(USART2, USART_IT_TXE);
  }
  
  if( USART_GetITStatus( USART2, USART_IT_RXNE ) == SET ) {
    if(U2_Receive) U2_Receive( USART_ReceiveData( USART2 ));
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
  if( USART_GetITStatus( USART2, USART_IT_ORE ) == SET ) {
    USART_ReceiveData( USART2);
  } 		
}
#endif /*----------USE_USART2  END-------*/

#ifdef USE_USART3
/*
* 串口3初始化
*/
void USART3_Init()
{
	//TODO
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11
	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10、GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); // PB10 PB11

	 //USART3
	USART_InitStructure.USART_BaudRate = COM3_BAUDRATE;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
	USART_Init(USART3, &USART_InitStructure); //初始化串口3

	USART_Cmd(USART3, ENABLE);  //使能串口3
	 
	USART_ClearFlag(USART3, USART_FLAG_TC);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//
	//Usart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =3;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;       //
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //
	NVIC_Init(&NVIC_InitStructure); //
	
	Coms[COM3] = USART3;
	BF_Init(&Com3TxBuffer);
	ComBuffers[COM3] = &Com3TxBuffer;
}

/*
* 串口3中断
*/
void USART3_IRQHandler()
{
	char cChar;
  if( USART_GetITStatus( USART3, USART_IT_TXE ) == SET ) {
    /* The interrupt was caused by the THR becoming empty.  Are there any
    more characters to transmit? */

    if(BF_Empty(&Com3TxBuffer)) {
			USART_ITConfig( USART3, USART_IT_TXE, DISABLE );
		} else {
			USART_SendData(USART3, BF_ReadByte(&Com3TxBuffer));
		}
    USART_ClearITPendingBit(USART3, USART_IT_TXE);
  }
  
  if( USART_GetITStatus( USART3, USART_IT_RXNE ) == SET ) {
    cChar = USART_ReceiveData( USART3 );
    if(U3_Receive) U3_Receive( cChar );
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
  }
  if( USART_GetITStatus( USART3, USART_IT_ORE ) == SET ) {
    cChar = USART_ReceiveData( USART3);
  } 	
}


#endif /*----------USE_USART3  END-------*/

#ifdef USE_USART4
/*
* 串口4初始化
*/
void USART4_Init()
{
	
}

/*
* 串口4中断
*/
void USART4_IRQHandler()
{
	
}

#endif /*----------USE_USART4  END-------*/

#ifdef USE_USART5
/*
* 串口5初始化
*/
void USART5_Init()
{
	
}

/*
* 串口5中断
*/
void USART5_IRQHandler()
{
	
}


#endif /*----------USE_USART5  END-------*/

#ifdef USE_USART6
/*
* 串口6初始化
*/
void USART6_Init()
{
	//TODO
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOC6
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOC7
	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOC6、GPIOC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); // PC6 PC7

	 //USART6 
	USART_InitStructure.USART_BaudRate = COM6_BAUDRATE;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
	USART_Init(USART6, &USART_InitStructure); //初始化串口6

	USART_Cmd(USART6, ENABLE);  //使能串口6
	 
	USART_ClearFlag(USART6, USART_FLAG_TC);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//
	//Usart6 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =3;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =4;       //
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //
	NVIC_Init(&NVIC_InitStructure); //
	//
	Coms[COM6] = USART6;
	BF_Init(&Com6TxBuffer);
	ComBuffers[COM6] = &Com6TxBuffer;
}

/*
* 串口6中断
*/
void USART6_IRQHandler()
{
  if( USART_GetITStatus( USART6, USART_IT_TXE ) == SET ) {

    if(BF_Empty(&Com6TxBuffer)) {
			USART_ITConfig( USART6, USART_IT_TXE, DISABLE );
		} else {
			USART_SendData(USART6, BF_ReadByte(&Com6TxBuffer));
		}
    USART_ClearITPendingBit(USART6, USART_IT_TXE);
  }
  
  if( USART_GetITStatus( USART6, USART_IT_RXNE ) == SET ) {
    if(U6_Receive) U6_Receive( USART_ReceiveData( USART6 ) );
    USART_ClearITPendingBit(USART6, USART_IT_RXNE);
  }
  if( USART_GetITStatus( USART6, USART_IT_ORE ) == SET ) {
		USART_ReceiveData( USART6 );
  } 
}



#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif

#endif /*----------USE_USART6  END-------*/
