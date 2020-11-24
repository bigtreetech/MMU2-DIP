//uart.h
#ifndef UART_H
#define UART_H
#include "stdio.h"

extern FILE _uart1io;
#define uart1io (&_uart1io)

void serial_read(void);

void uart1_init(void);


#endif //_UART_H
