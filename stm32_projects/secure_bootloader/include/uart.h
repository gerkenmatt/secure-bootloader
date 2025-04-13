#ifndef UART_H
#define UART_H

#include "stm32f7xx.h"

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

#define USART_BAUDRATE 115200
#define SYSCLK_FREQ    16000000
#define USARTDIV       (SYSCLK_FREQ / USART_BAUDRATE)
#define AF7            0x07  // Alternate function 7 for USART3

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------

/**
 * Initializes the USART3 peripheral        
 */
void usart_init(void);

/**
 * Sends a single character via USART3
 * 
 * @param ch The character to send
 */
void usart_putc(uint8_t ch);

/**
 * Receives a single character via USART3
 * 
 * @return The character received
 */
uint8_t usart_getc(void);

/**
 * Sends a string via USART3
 * 
 * @param str The string to send
 */
void usart_puts(const char *str);

/**
 * Receives a string via USART3
 * 
 * @param buffer The buffer to store the received string
 * @param max_len The maximum length of the string to receive
 */
void usart_gets(char *buffer, uint32_t max_len);

/**
 * Prints a 32-bit unsigned integer in hexadecimal format
 * 
 * @param val The value to print
 */
void print_uint32_hex(uint32_t val);

/**
 * Prints a 64-bit unsigned integer in hexadecimal format
 * 
 * @param val The value to print
 */
void print_uint64_hex(uint64_t val);

#endif // UART_H
