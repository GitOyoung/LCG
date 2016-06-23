#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "includes.h"
#include "os_app_hooks.h"

#include <stdio.h>


void U2_RecvHandler(char c)
{
	SerialPutBuffer(COM2, &c, 1);
}

int main(void)
{
	delay_init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	InitCom(COM2);
	SerialHandlerSet(COM2, U2_RecvHandler);	
	char message[] = "Hello, World! Made by Oyoung\r\n";
	while(1) {
		SerialPutString(COM2, message);
		delay_ms(1000);
	}
	
}
