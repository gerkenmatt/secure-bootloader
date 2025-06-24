/**
 * @file uart.h
 * @brief UART communication interface for STM32 bootloader.
 */
#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// Public Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Initializes the UART peripheral for communication.
 * Hardcoded for USART6 (PG14-TX, PG9-RX).
 */
void uart_init(void);

/**
 * @brief Sends a single character via UART (blocking).
 * @param ch The character to send.
 */
void uart_putc(uint8_t ch);

/**
 * @brief Tries to get a character from the UART receive buffer (non-blocking).
 * @param p_data Pointer to a byte where the received character will be stored.
 * @return true if a character was successfully read, false if the buffer was empty.
 */
bool uart_getc(uint8_t* p_data);

/**
 * @brief Sends a null-terminated string via UART (blocking).
 * @param str The string to send.
 */
void uart_puts(const char *str);

/**
 * @brief Sends a formatted string via UART, similar to printf.
 * @param fmt The format string.
 * @param ... Variable arguments for the format string.
 */
void uart_printf(const char *fmt, ...);

#endif // UART_H
