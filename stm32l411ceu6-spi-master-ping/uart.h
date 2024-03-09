#ifndef UART_H__
#define UART_H__

#include "main.h"

void uart_config(void);
int uart_transmit(char *data, int size);

#endif /* UART_H__ */