#ifndef UART_H
#define UART_H

#include "stm32f7xx.h"

void usart_init(void);
void usart_putc(uint8_t ch);
uint8_t usart_getc(void);
void usart_puts(const char *str);
void usart_gets(char *buffer, uint32_t max_len);

#endif // UART_H
