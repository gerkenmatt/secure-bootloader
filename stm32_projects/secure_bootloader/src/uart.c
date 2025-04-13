#include "uart.h"

#define USART_BAUDRATE 115200
#define SYSCLK_FREQ    16000000
#define USARTDIV       (SYSCLK_FREQ / USART_BAUDRATE)
#define AF7            0x07  // Alternate function 7 for USART3
#define AF8            0x08  // Alternate function 8 for USART6 (PG14/PG9)


void usart_init(void) {
    // Enable clocks for GPIOD and GPIOG (USART3 on PD8/PD9, USART6 on PG14/PG9)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOGEN;

    // Enable USART3 and USART6 clocks
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

    // USART3 GPIO setup (PD8 = TX, PD9 = RX)
    GPIOD->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOD->MODER |=  (2U << (8 * 2)) | (2U << (9 * 2));
    GPIOD->AFR[1] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)));
    GPIOD->AFR[1] |=  (AF7 << (0 * 4)) | (AF7 << (1 * 4));

    // USART6 GPIO setup (PG14 = TX, PG9 = RX)
    GPIOG->MODER &= ~((3U << (14 * 2)) | (3U << (9 * 2)));
    GPIOG->MODER |=  (2U << (14 * 2)) | (2U << (9 * 2));
    GPIOG->AFR[1] &= ~(0xF << ((14 - 8) * 4)); // AFR[1] for PG14
    GPIOG->AFR[1] |=  (AF8 << ((14 - 8) * 4));
    GPIOG->AFR[1] &= ~(0xF << ((9 - 8) * 4));  // AFR[1] for PG9
    GPIOG->AFR[1] |=  (AF8 << ((9 - 8) * 4));

    // USART3 config
    USART3->BRR = USARTDIV;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE;
    USART3->CR1 |= USART_CR1_UE;

    // USART6 config
    USART6->BRR = USARTDIV;
    USART6->CR1 = USART_CR1_TE | USART_CR1_RE;
    USART6->CR1 |= USART_CR1_UE;

    // Enable RXNE interrupt
    // USART6->CR1 |= USART_CR1_RXNEIE;

    // Enable USART6 interrupt in NVIC
    // NVIC_EnableIRQ(USART6_IRQn);
    NVIC_DisableIRQ(USART6_IRQn);
}

void usart_putc( uint8_t ch) {
    USART3->TDR = ch;
    while (!(USART3->ISR & USART_ISR_TC));
}

uint8_t usart_getc() {
    while (!(USART6->ISR & USART_ISR_RXNE));
    return USART6->RDR;
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
void print_uint32_hex(uint32_t val) {
    char hex[9];
    for (int i = 7; i >= 0; i--) {
        int nibble = (val >> (i * 4)) & 0xF;
        hex[7 - i] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
    }
    hex[8] = '\0';
    usart_puts(hex);
}

void print_uint64_hex(uint64_t val) {
    char hex[17];
    for (int i = 15; i >= 0; i--) {
        int nibble = (val >> (i * 4)) & 0xF;
        hex[15 - i] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
    }
    hex[16] = '\0';
    usart_puts(hex);
}