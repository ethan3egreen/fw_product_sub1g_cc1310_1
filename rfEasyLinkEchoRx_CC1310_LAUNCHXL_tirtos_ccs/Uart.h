#ifndef UART_H_
#define UART_H_

#include "stdint.h"

void Uart_init();
void send_Uart(uint8_t *buffer, size_t size);
int inttochar(int input);
void hextochar(int input,char *result);

#endif
