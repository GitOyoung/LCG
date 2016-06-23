#ifndef _ST7580_H_
#define _ST7580_H_


#ifdef __cplusplus
extern "C" {
#endif

	#include "st7580_conf.h"
	
	typedef unsigned char u8;
	typedef unsigned short u16;
	
	enum {
		STX 	= 0x02,
		STXR 	= 0x03,
		ACK		= 0x06,
		NAK 	= 0x15,
		Q_MARK = 0x3f
	};
	
	typedef enum { //MIB Objects Act
		MIB_M_CONFIG = 0,
		MIB_PHY_CONFIG,
		MIB_SS_KEYS,
		MIB_RESERVED,
		MIB_LAST_DI,
		MIB_LAST_TX_CONF,
		MIB_PHY_DATA,
		MIB_DL_DATA,
		MIB_SS_DATA,
		MIB_HOST_INT_TO,
		MIB_FW_REL,
		MIB_SYS_CONF,
	} MIB_INDEX_OBJ_T;
	
	typedef struct {
		u8 stx;
		u8 length;
		u8 cmd;
		u8 data[256];  
		u16 chksum;
	} LocalFrame;
	
	
	typedef struct {
		u8 start;
		u8 status0;
		u8 status1;
		u8 reserved;
	} Status_TypeDef;
	
	typedef struct{
		u8 id;
		u8 Rxpar_len;
		u8 Txpar_len;
		u8 Rxparam[255];
		u8 Txparam[255];
	} CommandFrame;

	void STA_SendCmd(u8 op, u8 plen, u8 *params);
	void STB_SendCmd(u8 op, u8 plen, u8 *params);
	
	LocalFrame *STA_GetFrame(void);
	LocalFrame *STB_GetFrame(void);
	
	Status_TypeDef *STA_GetStatus(void);
	Status_TypeDef *STB_GetStatus(void);
	
	
	u8* STA_GET_ACK(void);
	u8* STB_GET_ACK(void);
	
	void STA_SendAck(void);
	void STB_Send_Ack(void);
	
	void STA_SendNoAck(void);
	void STB_SendNoAck(void);
	
	
	void STA_ReceiveHandler(char byte);
	void STB_ReceiveHandler(char byte);
	
//	void BIO_ResetRequest(LocalFrame *lf);
//	void MIB_WriteRequest(LocalFrame *lf, MIB_INDEX_OBJ_T index, const char *data, int n);
//	void MIB_ReadRequest(LocalFrame *lf, MIB_INDEX_OBJ_T index);
//	void MIB_EraseRequest(LocalFrame *lf, MIB_INDEX_OBJ_T index);
//	
//	void PingRequest(LocalFrame *lf);
//	void PHY_DataRequest(LocalFrame *lf, const char *data, int n);
//		
//	void MIB_ReadIndex(MIB_INDEX_OBJ_T index, char *data);
//	void MIB_WriteIndex(MIB_INDEX_OBJ_T index, const char *data, int n);
	

#ifdef __cplusplus
}
#endif

#endif
/*-------------------end-----------------------*/
