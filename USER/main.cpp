#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "includes.h"
#include "os_app_hooks.h"

#include <stdio.h>

#include "gpio.h"

int main(void)
{
	delay_init(168);
	InitCom(COM1);	
	char message[] = "Hello, World! Made by Oyoung\r\n";
	while(1) {
		SerialPutString(COM1, message);
		delay_ms(100);
	}
	
}
