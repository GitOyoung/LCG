#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "includes.h"
#include "os_app_hooks.h"

#include <stdio.h>


void U3_RecvHandler(char c)
{
	SerialPutBuffer(COM3, &c, 1, 1);
}

int main(void)
{
	delay_init(168);
	InitCom(COM3);
	SerialHandlerSet(COM3, U3_RecvHandler);	
	char message[] = "Hello, World! Made by Oyoung\r\n";
	while(1) {
		SerialPutString(COM3, message);
		delay_ms(1000);
	}
	
}
