#include "uart.h"

#define USART_BAUDRATE 115200
#define SYSCLK_FREQ    16000000
#define USARTDIV       (SYSCLK_FREQ / USART_BAUDRATE)
#define AF7            0x07  // Alternate function 7 for USART3

void usart_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    GPIOD->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOD->MODER |=  (2U << (8 * 2)) | (2U << (9 * 2));

    GPIOD->AFR[1] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)));
    GPIOD->AFR[1] |=  (AF7 << (0 * 4)) | (AF7 << (1 * 4));

    USART3->BRR = USARTDIV;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE;
    USART3->CR1 |= USART_CR1_UE;
}

void usart_putc(uint8_t ch) {
    USART3->TDR = ch;
    while (!(USART3->ISR & USART_ISR_TC));
}

uint8_t usart_getc(void) {
    while (!(USART3->ISR & USART_ISR_RXNE));
    return USART3->RDR;
}

void usart_puts(const char *str) {
    while (*str) {
        usart_putc(*str++);
    }
}

void usart_gets(char *buffer, uint32_t max_len) {
    uint32_t i = 0;
    while (i < (max_len - 1)) {
        char ch = usart_getc();
        usart_putc(ch);
        if (ch == '\r' || ch == '\n') break;
        buffer[i++] = ch;
    }
    buffer[i] = '\0';
}
