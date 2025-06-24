
#include "bootloader.h"
#include "ota.h"
#include "stm32f767xx.h"
#include "uart.h"
#include "systick.h"
#include "logger.h"
#include "led.h"

#include <stdbool.h>
#include <string.h>

// -----------------------------------------------------------------------------
// Static Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Initializes the system hardware and peripherals.
 */
static void system_init(void);

/**
 * @brief Logs bootloader state transitions to UART.
 * @param new_state New bootloader state being transitioned to
 */
static void log_state_transition(bootloader_state_t new_state);


// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------

int main(void)
{
    system_init();               // Basic chip setup: FPU, cache, GPIO, etc.
    uart_init();                 // Set up USART6 for serial communication
    __enable_irq();
    uart_puts("\n\nBootloader started.\r\n");

    bootloader_init();           // Prepare internal state, verify memory aliasing
    validate_boot_environment(); // Confirm VTOR and aliasing are valid

    log_state_transition(bootloader_get_state()); // Log initial state

    // Bootloader main loop: handle command mode or jump to app
    while (1)
    {
        bootloader_run_state_machine();
    }
    return 0;
}

// -----------------------------------------------------------------------------
// Static Function Implementations
// -----------------------------------------------------------------------------

static void log_state_transition(bootloader_state_t new_state) 
{
    switch (new_state) 
    {
        case BL_STATE_READY:        uart_puts("Bootloader ready. Waiting for command (run/update)...\r\n"); break;
        case BL_STATE_RECEIVING:    uart_puts("State: Receiving firmware\r\n"); break;
        case BL_STATE_VERIFY:       uart_puts("State: Verifying firmware\r\n"); break;
        case BL_STATE_ERROR:        uart_puts("State: Error occurred\r\n"); break;
        default:                    uart_puts("State: Unknown\r\n"); break;
    }
}

static void system_init(void)
{

    systick_init();

    // Enable FPU (floating point unit)
    SCB->CPACR |= ((3UL << 20) | (3UL << 22));

    // Enable instruction and data caches
    SCB_EnableICache();
    SCB_EnableDCache();

    // Configure flash latency for high-speed operation (216 MHz)
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_7WS;

    led_init();

    // Enable all fault handlers
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;

}
