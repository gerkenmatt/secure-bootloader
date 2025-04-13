#include <stdbool.h>
#include <string.h>
#include "bootloader.h"
#include "ota.h"
#include "stm32f7xx.h"
#include "uart.h"

// -----------------------------------------------------------------------------
// Function Prototypes 
// -----------------------------------------------------------------------------
static void system_init(void);
static void log_state_transition(bootloader_state_t new_state);


// -----------------------------------------------------------------------------
// Main Entry Point
// -----------------------------------------------------------------------------
int main(void)
{
    bootloader_state_t current_state = BL_STATE_IDLE;
    bool state_switched = false;

    system_init();               // Basic chip setup: FPU, cache, GPIO, etc.
    usart_init();                // Set up USART3 for serial debug output
    __enable_irq();
    usart_puts("\n\nBootloader started.\r\n");

    bootloader_init();           // Prepare internal state, verify memory aliasing
    validate_boot_environment(); // Confirm VTOR and aliasing are valid



    // Bootloader main loop: handle command mode or jump to app
    while (1)
    {
        process_bootloader_command(); 

        state_switched = (bootloader_get_state() != current_state);
        current_state = bootloader_get_state();
        if (state_switched) {
            log_state_transition(current_state);
            state_switched = false;
        }
    }
    return 0;
}

/**
 * Logs bootloader state transitions to UART
 * Prints human readable state description
 *
 * @param new_state New bootloader state being transitioned to
 */
static void log_state_transition(bootloader_state_t new_state) {
    switch (new_state) {
        case BL_STATE_READY:      usart_puts("Bootloader ready. Waiting for command (run/ota)...\r\n"); break;
        case BL_STATE_RECEIVING: usart_puts("State: Receiving firmware\r\n"); break;
        case BL_STATE_PROGRAMMING: usart_puts("State: Programming flash\r\n"); break;
        case BL_STATE_VERIFY:    usart_puts("State: Verifying firmware\r\n"); break;
        case BL_STATE_ERROR:     usart_puts("State: Error occurred\r\n"); break;
        default:                 usart_puts("State: Unknown\r\n"); break;
    }
}

/**
 * Initializes system hardware and peripherals
 * Configures FPU, caches, flash latency, GPIO
 */
static void system_init(void)
{
    //TODO: get rid of these prints since uart is not set up yet
    usart_puts("Initializing system...\r\n");

    // Enable FPU (floating point unit)
    SCB->CPACR |= ((3UL << 20) | (3UL << 22));
    usart_puts("FPU enabled\r\n");

    // Enable instruction and data caches
    SCB_EnableICache();
    SCB_EnableDCache();
    usart_puts("Caches enabled\r\n");

    // Configure flash latency for high-speed operation (216 MHz)
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_7WS;
    usart_puts("Flash latency configured\r\n");

    // Enable GPIOA and GPIOC peripheral clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    usart_puts("GPIO clocks enabled\r\n");

    // Set PA5 to output (was used for LED blinking, optional now)
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER5) | GPIO_MODER_MODER5_0;

    // Set PC13 (BOOT0 pin) as input with pull-down
    GPIOC->MODER &= ~GPIO_MODER_MODER13;
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR13_1;

    /* Enable clock for GPIOB */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* Configure PB0, PB7, and PB14 as outputs for LEDs*/
    GPIOB->MODER &= ~((3UL << (0 * 2)) | (3UL << (7 * 2)) | (3UL << (14 * 2)));
    GPIOB->MODER |= ((1UL << (0 * 2)) | (1UL << (7 * 2)) | (1UL << (14 * 2)));
    GPIOB->ODR &= ~(1UL << 0); //clear_green
    GPIOB->ODR &= ~(1UL << 7); //clear_blue
    GPIOB->ODR &= ~(1UL << 14); //clear_red
    usart_puts("GPIO configured\r\n");
}