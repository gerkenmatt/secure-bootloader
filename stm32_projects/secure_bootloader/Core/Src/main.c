#include <stdbool.h>
#include <string.h>
#include "bootloader.h"
#include "ota.h"
#include "stm32f7xx.h"
#include "uart.h"

// --- Function Prototypes ---
static void system_init(void);
static void log_state_transition(bootloader_state_t new_state);


// --- Main Entry Point ---
int main(void)
{

    system_init();               // Basic chip setup: FPU, cache, GPIO, etc.
    uart_init();                // Set up USART3 for serial debug output
    __enable_irq();
    uart_puts("\n\nBootloader started.\r\n");

    bootloader_init();           // Prepare internal state, verify memory aliasing
    validate_boot_environment(); // Confirm VTOR and aliasing are valid

    log_state_transition(bootloader_get_state()); // Log initial state

    // Bootloader main loop: handle command mode or jump to app
    while (1)
    {
        bootloader_run_state_machine();

        //TODO: add watchdog? 
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
        case BL_STATE_READY:        uart_puts("Bootloader ready. Waiting for command (run/update)...\r\n"); break;
        case BL_STATE_RECEIVING:    uart_puts("State: Receiving firmware\r\n"); break;
        case BL_STATE_PROGRAMMING:  uart_puts("State: Programming flash\r\n"); break;
        case BL_STATE_VERIFY:       uart_puts("State: Verifying firmware\r\n"); break;
        case BL_STATE_ERROR:        uart_puts("State: Error occurred\r\n"); break;
        default:                    uart_puts("State: Unknown\r\n"); break;
    }
}

/**
 * Initializes system hardware and peripherals
 * Configures FPU, caches, flash latency, GPIO
 */
static void system_init(void)
{
    // --- Clock Configuration ---
    uint32_t hclk_freq = 16000000; // Assume 16MHz HSI

    // --- SysTick Configuration ---
    // Configure the SysTick to fire an interrupt every 1 millisecond.
    if (SysTick_Config(hclk_freq / 1000))
    {
        while(1); // Trap in an infinite loop on error.
    }

    // Enable FPU (floating point unit)
    SCB->CPACR |= ((3UL << 20) | (3UL << 22));

    // Enable instruction and data caches
    SCB_EnableICache();
    SCB_EnableDCache();

    // Configure flash latency for high-speed operation (216 MHz)
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_7WS;

    // Enable GPIOA and GPIOC peripheral clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;

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

    // Enable all fault handlers
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;

}
