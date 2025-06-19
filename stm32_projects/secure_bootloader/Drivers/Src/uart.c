#include "uart.h"
#include <stdint.h>
#include <stdbool.h>

#define USART_BAUDRATE 115200
#define SYSCLK_FREQ    16000000
#define USARTDIV       (SYSCLK_FREQ / USART_BAUDRATE)
#define AF7            0x07  // Alternate function 7 for USART3
#define AF8            0x08  // Alternate function 8 for USART6 (PG14/PG9)

#define UART_RX_BUFFER_SIZE 256 // A power of 2 is most efficient

typedef struct 
{
    uint8_t buffer[UART_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} ring_buffer_t;

static ring_buffer_t rx_buffer;

void ring_buffer_init(void) 
{
    rx_buffer.head = 0;
    rx_buffer.tail = 0;
}

// Write data from ISR
void ring_buffer_write(uint8_t data) 
{
    uint16_t next_head = (rx_buffer.head + 1) % UART_RX_BUFFER_SIZE;
    if (next_head != rx_buffer.tail) 
    { 
        // Avoid overwriting the tail
        rx_buffer.buffer[rx_buffer.head] = data;
        rx_buffer.head = next_head;
    }
}

// Read data from main application
bool ring_buffer_read(uint8_t* data) 
{
    if (rx_buffer.head == rx_buffer.tail) 
    {
        return false; // Buffer is empty
    }
    *data = rx_buffer.buffer[rx_buffer.tail];
    rx_buffer.tail = (rx_buffer.tail + 1) % UART_RX_BUFFER_SIZE;
    return true;
}

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

    ring_buffer_init();

    // Enable USART3 RXNE interrupt
    USART6->CR1 |= USART_CR1_RXNEIE; 
    USART6->CR1 |= USART_CR1_UE; // Enable USART last

    NVIC_SetPriority(USART6_IRQn, 0); // Set a high priority
    NVIC_EnableIRQ(USART6_IRQn);
}

void usart_putc( uint8_t ch) 
{
    // USART6->TDR = ch;
    // while (!(USART6->ISR & USART_ISR_TC));
    
    while (!(USART6->ISR & USART_ISR_TXE));
    USART6->TDR = ch;
}

bool usart_getc(uint8_t* data) 
{
    return ring_buffer_read(data);
}

void usart_puts(const char *str) 
{
    while (*str) 
    {
        usart_putc(*str++);
    }
}


void print_uint8_hex(uint8_t val) 
{
    char hex[3];
    for (int i = 1; i >= 0; i--) 
    {
        int nibble = (val >> (i * 4)) & 0xF;
        hex[1 - i] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
    }
    hex[2] = '\0';
    usart_puts(hex);
}

void print_uint32_hex(uint32_t val) 
{
    char hex[9];
    for (int i = 7; i >= 0; i--) 
    {
        int nibble = (val >> (i * 4)) & 0xF;
        hex[7 - i] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
    }
    hex[8] = '\0';
    usart_puts(hex);
}

void print_uint64_hex(uint64_t val) 
{
    char hex[17];
    for (int i = 15; i >= 0; i--) 
    {
        int nibble = (val >> (i * 4)) & 0xF;
        hex[15 - i] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
    }
    hex[16] = '\0';
    usart_puts(hex);
}

void print_uint16_hex(uint16_t val) 
{
    char hex[5];
    for (int i = 3; i >= 0; i--) 
    {
        int nibble = (val >> (i * 4)) & 0xF;
        hex[3 - i] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
    }
    hex[4] = '\0';
    usart_puts(hex);
}

void USART6_IRQHandler(void) 
{
    // Check if the interrupt was caused by a received byte (RXNE flag)
    if (USART6->ISR & USART_ISR_RXNE) 
    {
        // The flag is cleared by reading the data register
        uint8_t received_byte = (uint8_t)USART6->RDR;
        ring_buffer_write(received_byte);
    }

    // Check for and clear error flags
    if (USART6->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) 
    {
        // Overrun, Framing, or Noise error occurred.
        // Clear the flags by writing to the ICR register.
        USART6->ICR |= USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;
    }
}