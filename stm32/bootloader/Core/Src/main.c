
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
    // Perform essential low-level hardware setup for the microcontroller.
    system_init();
    // Initialize the UART peripheral for serial communication, crucial for CLI and logging.
    uart_init();
    // Enable global interrupts to allow peripherals (like UART) to function.
    __enable_irq();

    // Send an initial message to the console to indicate bootloader startup.
    uart_puts("\n\nBootloader started.\r\n");

    // Initialize the bootloader's internal state and read/initialize configuration from flash.
    bootloader_init();
    // Validate critical bootloader environment settings, such as Vector Table Offset (VTOR)
    // and memory aliasing, to ensure correct operation.
    validate_boot_environment();

    // Log the initial operating state of the bootloader to the serial console.
    log_state_transition(bootloader_get_state());

    // This is the main operational loop of the bootloader. It continuously runs the
    // bootloader's state machine to handle various tasks:
    // - Processing commands (CLI)
    // - Managing OTA firmware updates
    // - Deciding whether to jump to an application or stay in bootloader mode.
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
    // Print a descriptive message to the UART console based on the current bootloader state.
    // This helps in debugging and understanding the bootloader's behavior.
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
    // Initialize the SysTick timer. This provides a high-resolution time base
    // for various bootloader operations, such as timeouts.
    systick_init();

    // Enable the Floating Point Unit (FPU). This is necessary if any floating-point
    // arithmetic is used in the bootloader or the application it jumps to.
    // CPACR bits 20 and 22 control access to CP10 and CP11 (FPU).
    SCB->CPACR |= ((3UL << 20) | (3UL << 22));

    // Enable the instruction and data caches. This significantly improves performance
    // by reducing the latency of flash memory accesses, especially at high clock speeds.
    SCB_EnableICache();
    SCB_EnableDCache();

    // Configure Flash memory access latency. This is crucial for stable operation
    // at higher CPU clock frequencies (e.g., 216 MHz for STM32F767).
    // The value `FLASH_ACR_LATENCY_7WS` specifies 7 wait states for reliable data reads.
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_7WS;

    // Initialize LED indicators. These provide visual feedback on the bootloader's state,
    // which is helpful when serial output is not available.
    led_init();

    // Enable various system fault handlers (UsageFault, BusFault, MemManageFault).
    // This helps in catching and debugging unexpected memory access violations or
    // instruction execution errors, making the system more robust during development.
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;
}