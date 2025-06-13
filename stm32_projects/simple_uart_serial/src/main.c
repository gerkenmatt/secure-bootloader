#include "uart.h"
#include "stm32f7xx.h"

static void led_init(void) {
    // Enable GPIOD clock (LED on PD12)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    // Set PD12 as output
    GPIOB->MODER &= ~((3UL << (0 * 2)) | (3UL << (7 * 2)) | (3UL << (14 * 2)));
    GPIOB->MODER |= ((1UL << (0 * 2)) | (1UL << (7 * 2)) | (1UL << (14 * 2)));
}

static void led_blink_once(void) {
    // toggle green
    GPIOB->ODR ^= (1UL << 0);
    for (volatile int i = 0; i < 1000000; i++); // Simple delay
    // toggle green
    GPIOB->ODR ^= (1UL << 0);
}

int main(void) {
    char buffer[100];

    usart_init();
    led_init();
    //turn on all leds
    GPIOB->ODR |= (1UL << 0) | (1UL << 14); // Set PD12, PD13, PD14 high
    usart_puts("USART3 ready. Type something:\r\n");

    while (1) {
        usart_gets(buffer, sizeof(buffer));
        usart_puts("You typed: ");
        usart_puts(buffer);
        usart_puts("\r\n");
    }
}