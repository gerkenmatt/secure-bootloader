#include "uart.h"

int main(void) {
    char buffer[100];

    usart_init();
    __enable_irq();

    // usart_puts("Waiting for USART6 RX...\r\n");
    // while (1) {
    //     if (USART6->ISR & USART_ISR_ORE) {
    //         volatile uint8_t dummy = USART6->RDR;  // Clear ORE by reading RDR
    //         USART6->ICR |= USART_ICR_ORECF;        // Clear ORE flag
    //         usart_puts("[ORE cleared]\r\n");
    //     }

    //     if (USART6->ISR & USART_ISR_RXNE) {
    //         uint8_t ch = USART6->RDR;
    //         usart_puts("Got: ");
    //         usart_putc(ch);
    //         usart_puts("\r\n");
    //         }
    // }
    usart_puts("USART bridge ready. Type something:\r\n");

    usart_puts("CR1: "); print_uint32_hex(USART6->CR1); usart_puts("\r\n");
    usart_puts("ISR: "); print_uint32_hex(USART6->ISR); usart_puts("\r\n");

    while (1) {
        // if (uart_rx_available()) {
        //     uint8_t ch = uart_rx_read();
        //     usart_putc(ch);  // echo
        // }
        if (USART6->ISR & USART_ISR_RXNE) {
            uint8_t ch = USART6->RDR;
            usart_putc(ch);  // echo to USART3
        }
    }

    // while (1) {
        // NVIC_SetPendingIRQ(USART6_IRQn);
        // for(int i = 0; i < 1000000; i++) {
        //     __asm("nop");
        // }
        // NVIC_ClearPendingIRQ(USART6_IRQn);
        // usart_puts("Test\r\n");
        // usart_gets(buffer, sizeof(buffer));
        // usart_puts("You typed: ");
        // usart_puts(buffer);
        // usart_puts("\r\n");
    // }
}