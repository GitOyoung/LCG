#ifndef __USART_H
#define __USART_H



#ifdef __cplusplus
extern "C" {
#endif
	
/*---------------definitions------------------------------------*/
	




#define USE_USART1 1
//#define USE_USART2 1
#define USE_USART3 1
//#define USE_USART4 1
//#define USE_USART5 1
#define USE_USART6 1
	
/*-------------------enums-------------------------------------------*/

/*---------------public enums-----------------------------*/
	

	enum {
		COM1_BAUDRATE  = 115200,
		COM3_BAUDRATE  = 57600,
		COM6_BAUDARTE  = 57600			
	};
	
	typedef enum {
		COM1 = 0,
		COM2,
		COM3,
		COM4,
		COM5,
		COM6,
		COMn
	} Com_TypeDef;
	
	typedef void (*ReceiveHandler)(char);
	
	void InitCom(Com_TypeDef com);
	void SerialPutBuffer(Com_TypeDef com,  const char *buffer, int size, int immediate);
	void SerialPutString(Com_TypeDef com, const char *string);
	void SerialSendStart(Com_TypeDef com);
	
	void SerialHandlerSet(Com_TypeDef com, ReceiveHandler handler);
	

#ifdef __cplusplus
}
#endif
#endif

/*----------------------end-------------------*/

