//uart.cpp
#include <Arduino.h>
#include <inttypes.h>
#include <stdio.h>
#include "config.h"
#include "uart.h"
#include "stdio.h"

FILE _uart0io;

FILE _uart1io;

void serial_read()
{  
	static char buff[30] = {0};
	static byte n = 0;
	while(Serial1.available())
	{
		buff[n] = Serial1.read();
		if(buff[n++] == '\n')
		{
			buff[n] = '\0';
			Serial1.print(buff);
			n = 0;
		}
	}
}

int uart1_putchar(char c, FILE *)
{
	Serial1.write(c);
	return 0;
}
int uart1_getchar(FILE *)
{
	return Serial1.read();
}


void uart0_init(void)
{
	Serial.begin(UART0_BDR, SERIAL_8N1); //serial0 - USB
	//fdev_setup_stream(uart0io, uart0_putchar, uart0_getchar, _FDEV_SETUP_WRITE | _FDEV_SETUP_READ); //setup uart in/out stream
}

void uart1_init(void)
{
	Serial1.begin(UART1_BDR, SERIAL_8N1); //serial1
	//fdev_setup_stream(uart1io, uart1_putchar, uart1_getchar, _FDEV_SETUP_WRITE | _FDEV_SETUP_READ); //setup uart in/out stream
}
