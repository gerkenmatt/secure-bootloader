#include "uart.h"
#include "stm32f767xx.h"

// --- Private Driver Configuration ---
#define UART_PERIPH         USART6
#define UART_IRQn           USART6_IRQn
#define UART_IRQHandler     USART6_IRQHandler

#define UART_GPIO_PORT      GPIOG
#define UART_TX_PIN         14
#define UART_RX_PIN         9
#define UART_ALT_FUNC       8 // AF8 for USART6 on GPIOG

#define UART_TX_PIN_AF_POS  ((UART_TX_PIN - 8) * 4)
#define UART_RX_PIN_AF_POS  ((UART_RX_PIN - 8) * 4)

#define UART_GPIO_CLK_EN()  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN)
#define UART_PERIPH_CLK_EN() (RCC->APB2ENR |= RCC_APB2ENR_USART6EN)

// --- Baud Rate Calculation ---
#define UART_SYSCLK_FREQ    16000000UL
#define UART_BAUDRATE       115200UL
#define UART_BAUD_DIV       (UART_SYSCLK_FREQ / UART_BAUDRATE)

// --- Ring Buffer for RX ---
#define UART_RX_BUFFER_SIZE 256 // Power of 2 for efficient '%' -> '&' optimization

// --- Type Definitions ---
typedef struct {
    uint8_t buffer[UART_RX_BUFFER_SIZE];
    volatile uint16_t head; // Written by ISR
    volatile uint16_t tail; // Read by main application
} ring_buffer_t;

// --- Static Variables ---
static ring_buffer_t rx_buffer = {0};

// --- Function Prototypes ---
static void ring_buffer_write(uint8_t data);
static bool ring_buffer_read(uint8_t* data);
static void print_hex_value(uint64_t val, int num_nibbles);

// --- Public Function Implementations ---

void uart_init(void) 
{
    // 1. Enable clocks
    UART_GPIO_CLK_EN();
    UART_PERIPH_CLK_EN();

    // 2. Configure GPIO pins to Alternate Function
    // Clear mode bits for TX and RX pins
    UART_GPIO_PORT->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER14);
    // Set mode to Alternate Function (10)
    UART_GPIO_PORT->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER14_1);

    // Set Alternate Function Register for pins 9 and 14 (AF8)
    // using our self-calculating macros.
    // First, clear the 4-bit fields for the pins.
    UART_GPIO_PORT->AFR[1] &= ~((0xF << UART_RX_PIN_AF_POS) | (0xF << UART_TX_PIN_AF_POS));
    // Now, set the alternate function value (AF8) in those fields.
    UART_GPIO_PORT->AFR[1] |= ((uint32_t)UART_ALT_FUNC << UART_RX_PIN_AF_POS) | ((uint32_t)UART_ALT_FUNC << UART_TX_PIN_AF_POS);

    // 3. Configure UART Peripheral
    // Disable UART first to safely change settings
    UART_PERIPH->CR1 &= ~USART_CR1_UE;

    // Set baud rate
    UART_PERIPH->BRR = UART_BAUD_DIV;

    // Configure Control Register 1: 8N1, enable TX, enable RX
    UART_PERIPH->CR1 = USART_CR1_TE | USART_CR1_RE;
    UART_PERIPH->CR2 = 0;
    UART_PERIPH->CR3 = 0;

    // 4. Configure Interrupts
    // Initialize buffer state
    rx_buffer.head = 0;
    rx_buffer.tail = 0;
    
    // Enable RXNE (Receive Not Empty) interrupt
    UART_PERIPH->CR1 |= USART_CR1_RXNEIE;
    
    // Set interrupt priority (bootloader might want the highest priority)
    NVIC_SetPriority(UART_IRQn, 0);
    NVIC_EnableIRQ(UART_IRQn);
    
    // 5. Enable UART
    UART_PERIPH->CR1 |= USART_CR1_UE;
}

void uart_putc( uint8_t ch) 
{
    // Wait until the transmit data register is empty
    while (!(USART6->ISR & USART_ISR_TXE));
    USART6->TDR = ch;
}

bool uart_getc(uint8_t* data) 
{
    return ring_buffer_read(data);
}

void uart_puts(const char *str) 
{
    while (*str) 
        uart_putc(*str++);
}

// --- Hex Printing Helpers ---

static void print_hex_value(uint64_t val, int num_nibbles) 
{
    char hex_str[17];
    hex_str[num_nibbles] = '\0';
    for (int i = num_nibbles - 1; i >= 0; i--) 
    {
        uint8_t nibble = (val >> (i * 4)) & 0xF;
        hex_str[(num_nibbles - 1) - i] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
    }
    uart_puts(hex_str);
}

void uart_print_hex64(uint64_t val) 
{
    print_hex_value(val, 16);
}

void uart_print_hex32(uint32_t val) 
{
    print_hex_value(val, 8);
}

void uart_print_hex16(uint16_t val) 
{
    print_hex_value(val, 4);
}

void uart_print_hex8(uint8_t val) 
{
    print_hex_value(val, 2);
}

// --- Ring Buffer Implementation ---

// Called from ISR to write data
static void ring_buffer_write(uint8_t data)
{
    uint16_t next_head = (rx_buffer.head + 1) % UART_RX_BUFFER_SIZE;
    if (next_head != rx_buffer.tail) 
    { 
        // Avoid overwriting the tail
        rx_buffer.buffer[rx_buffer.head] = data;
        rx_buffer.head = next_head;
    }
    //TODO: add error handling for buffer overflow
}

// Called from main application to read data
static bool ring_buffer_read(uint8_t* data) 
{
    if (rx_buffer.head == rx_buffer.tail) 
    {
        return false; // Buffer is empty
    }
    
    *data = rx_buffer.buffer[rx_buffer.tail];
    rx_buffer.tail = (rx_buffer.tail + 1) & (UART_RX_BUFFER_SIZE - 1);
    return true;
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

    // Check for and clear hardware error flags to prevent the IRQ from re-firing
    if (USART6->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) 
        USART6->ICR |= USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;
    
}