#ifndef UART_H
#define UART_H

#include "stm32f7xx.h"

void usart_init(void);
void usart_putc(uint8_t ch);
uint8_t usart_getc();
void usart_puts(const char *str);
void usart_gets(char *buffer, uint32_t max_len);
// void USART6_IRQHandler(void);
void print_uint32_hex(uint32_t val);
void print_uint64_hex(uint64_t val);

// Ring buffer access
int uart_rx_available(void);
uint8_t uart_rx_read(void);
#endif // UART_H
