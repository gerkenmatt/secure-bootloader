#include <stdbool.h>
#include "bootloader.h"
#include "stm32f7xx.h"
#include "uart.h"

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------
static void system_init(void);

// -----------------------------------------------------------------------------
// Main Entry Point
// -----------------------------------------------------------------------------
int main(void)
{
    bootloader_state_t prev_state;
    bootloader_state_t current_state;
    bool state_switched = false;

    system_init();               // Basic chip setup: FPU, cache, GPIO, etc.
    usart_init();                // Set up USART3 for serial debug output
    usart_puts("\n\nBootloader started.\r\n");

    bootloader_init();           // Prepare internal state, verify memory aliasing
    validate_boot_environment(); // Confirm VTOR and aliasing are valid

    // Bootloader main loop: handle command mode or jump to app
    while (1)
    {
        prev_state = current_state;
        current_state = bootloader_get_state();
        if (prev_state != current_state)
        {
            state_switched = true;
        }
        switch (current_state)
        {
            case BL_STATE_READY:
                if (state_switched)
                {
                    usart_puts("Bootloader ready.\r\n");
                    if (bootloader_get_config()->boot_flags & BOOT_FLAG_STAY_IN_BL)
                    {
                        usart_puts("Boot flag: Stay in bootloader mode\r\n");
                    }
                    else
                    {
                        usart_puts("Boot flag: Jump to application\r\n");
                        usart_puts("Checking application at 0x08040000...\r\n");
                        bootloader_jump_to_application(0x08040000);
                    }
                    state_switched = false;
                }
                break;

            case BL_STATE_RECEIVING:
                if (state_switched)
                {
                    usart_puts("State: Receiving firmware\r\n");
                    state_switched = false;
                }
                break;

            case BL_STATE_PROGRAMMING:
                if (state_switched)
                {
                    usart_puts("State: Programming flash\r\n");
                    state_switched = false;
                }
                break;

            case BL_STATE_VERIFY:
                if (state_switched)
                {
                    usart_puts("State: Verifying firmware\r\n");
                    state_switched = false;
                }
                break;

            case BL_STATE_ERROR:
                if (state_switched) 
                {
                    usart_puts("State: Error occurred\r\n");
                    state_switched = false;
                }
                break;

            default:
                if (state_switched)
                {
                    usart_puts("State: Unknown\r\n");
                    state_switched = false;
                }
                break;
        }
    }
}

// -----------------------------------------------------------------------------
// System Initialization: clocks, GPIO, flash, etc.
// -----------------------------------------------------------------------------
static void system_init(void)
{
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
    usart_puts("GPIO configured\r\n");
}
