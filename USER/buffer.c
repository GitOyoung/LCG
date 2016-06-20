

#include "buffer.h"

void BF_Init(Buffer *buffer)
{
	buffer->head = 0;
	buffer->tail = 0;
}
BF_BOOL BF_Empty(const Buffer *buffer)
{
	if(buffer->head == buffer->tail) return BF_TRUE;
	else return BF_FALSE;
}
BF_BOOL BF_Full(const Buffer *buffer)
{
	if( (buffer->tail + 1) % BufferSize == buffer->head) return BF_TRUE;
	else return BF_FALSE;
}
int BF_ReadByte(Buffer *buffer)
{
	char byte = -1;
	if(BF_Empty(buffer)) return -1;
	byte = buffer->data[buffer->head++];
	if(buffer->head >= BufferSize) buffer->head = 0;
	return byte;
}

BF_BOOL BF_WriteByte(Buffer *buffer, const char byte)
{
	if(BF_Full(buffer)) return BF_FALSE;
	buffer->data[buffer->tail++] = byte;
	if(buffer->tail >= BufferSize) buffer->tail = 0;
	return BF_TRUE;
}

int BF_SizeGet(const Buffer *buffer)
{
	int head = buffer->head;
	int tail = buffer->tail;
	return head < tail ? (tail - head) : (tail + BufferSize - head);
}
int BF_Read(Buffer *buffer, char *des, int size)
{
	int index = 0;
	int realSize = BF_SizeGet(buffer);
	if(realSize > size) realSize = size;
	for(; index < realSize; ++index) {
		des[index] = BF_ReadByte(buffer);
	}
	return realSize;
}
int BF_Write(Buffer *buffer, const char *src, int size)
{
	int writedCount = 0;
	while(writedCount < size) {
		if(BF_WriteByte(buffer, src[writedCount]) == BF_FALSE) break;
		writedCount++;
	}
	return writedCount;
}
