#ifndef _BUFFER_H_
#define _BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

/*------------------definitions---------------------------------------*/	
	

	
/*-------------------enums-------------------------------------------*/	
	enum {
		BufferSize = 260
	};
	
	typedef enum {
		BF_FALSE = 0,
		BF_TRUE = !BF_FALSE
	} BF_BOOL;
	
	typedef struct {
		char data[BufferSize];
		int head;
		int tail;
	} Buffer;
	
	void BF_Init(Buffer *buffer);
	BF_BOOL BF_Empty(const Buffer *buffer);
	BF_BOOL BF_Full(const Buffer *buffer);
	int BF_ReadByte(Buffer *buffer);
	BF_BOOL BF_WriteByte(Buffer *buffer, const char byte);
	int BF_SizeGet(const Buffer *buffer);
	int BF_Read(Buffer *buffer, char *des, int size);
	int BF_Write(Buffer *buffer, const char *src, int size);

#ifdef __cplusplus
}
#endif
#endif

