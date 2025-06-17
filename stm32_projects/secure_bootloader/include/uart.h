#ifndef UART_H
#define UART_H

#include "stm32f7xx.h"
#include <stdbool.h> 

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
 * @brief Tries to get a character from the UART receive buffer.
 * This function is NON-BLOCKING.
 *
 * @param data Pointer to a byte where the received character will be stored.
 * @return true if a character was successfully read, false if the buffer was empty.
 */
bool usart_getc(uint8_t* data);

/**
 * Sends a string via USART3
 * 
 * @param str The string to send
 */
void usart_puts(const char *str);

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

/**
 * Prints a 8-bit unsigned integer in hexadecimal format
 * 
 * @param val The value to print
 */
void print_uint8_hex(uint8_t val);

void print_uint16_hex(uint16_t val) ;

#endif // UART_H
