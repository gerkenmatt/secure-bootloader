#include "bootloader.h"
#include "stm32f7xx.h"
#include "uart.h"

// -----------------------------------------------------------------------------
// Static State and Configuration
// -----------------------------------------------------------------------------
static bootloader_state_t current_state = BL_STATE_IDLE;
static bootloader_config_t* const boot_config =
    (bootloader_config_t*)(BOOTLOADER_START_PHYS + BOOTLOADER_SIZE - sizeof(bootloader_config_t));

// -----------------------------------------------------------------------------
// Get Bootloader Configuration From Flash
// -----------------------------------------------------------------------------
const bootloader_config_t* bootloader_get_config(void)
{
    return boot_config;
}

// -----------------------------------------------------------------------------
// Return Current Bootloader State
// -----------------------------------------------------------------------------
bootloader_state_t bootloader_get_state(void)
{
    return current_state;
}

// -----------------------------------------------------------------------------
// Initialize Bootloader (state setup)
// -----------------------------------------------------------------------------
void bootloader_init(void)
{
    // Initialize configuration if not already set
    if (boot_config->magic != BOOT_CONFIG_MAGIC)
    {
        boot_config->magic = BOOT_CONFIG_MAGIC;
        boot_config->boot_flags = 0;  // Clear all flags by default
        boot_config->app_version = 0;
        boot_config->app_size = 0;
        boot_config->app_crc = 0;
    }

    // Set state to READY
    current_state = BL_STATE_READY;
}

// -----------------------------------------------------------------------------
// Handle Bootloader Command (stub for now)
// -----------------------------------------------------------------------------
bootloader_status_t bootloader_handle_command(void)
{
    usart_puts("Handling bootloader command...\r\n");
    return BL_OK;
}

// -----------------------------------------------------------------------------
// Compare AXI and ITCM memory regions to verify alias
// -----------------------------------------------------------------------------
bootloader_status_t bootloader_verify_memory_aliasing(void)
{
    uint32_t *flash = (uint32_t *)BOOTLOADER_START_PHYS;
    uint32_t *itcm  = (uint32_t *)BOOTLOADER_START_ALIAS;

    for (int i = 0; i < 256; i++)
    {
        if (flash[i] != itcm[i])
            return BL_ERROR_INVALID_BOOT_MODE;
    }
    return BL_OK;
}

// -----------------------------------------------------------------------------
// Read and return the state of BOOT0 pin (PC13)
// -----------------------------------------------------------------------------
uint32_t bootloader_get_boot_mode(void)
{
    return (GPIOC->IDR & GPIO_IDR_ID13) ? 1 : 0;
}

// -----------------------------------------------------------------------------
// Transition to user application in flash
// -----------------------------------------------------------------------------
void bootloader_jump_to_application(uint32_t app_addr)
{
    if (app_addr < APPLICATION_START_ADDR || app_addr >= FLASH_END_ADDR)
    {
        usart_puts("Invalid application address.\r\n");
        current_state = BL_STATE_ERROR;
        return;
    }

    uint32_t *vec_tab = (uint32_t *)app_addr;
    uint32_t sp = vec_tab[0];  // Application initial stack pointer
    uint32_t entry = vec_tab[1];  // Application reset handler

    usart_puts("Vector Table at 0x08040000:\r\n");
    
    // Print first 4 entries of vector table
    for(int i = 0; i < 10; i++) {
        usart_puts("Entry ");
        char idx = '0' + i;
        usart_putc(idx);
        usart_puts(": 0x");
        
        uint32_t value = vec_tab[i];
        // Convert to hex string and print
        char hex[9];
        for(int j = 7; j >= 0; j--) {
            int digit = (value >> (j * 4)) & 0xF;
            hex[7-j] = digit < 10 ? '0' + digit : 'A' + (digit - 10);
        }
        hex[8] = '\0';
        usart_puts(hex);
        usart_puts("\r\n");
    }

    // Verify that sp is a valid RAM address (starting at 0x20000000)
    if ((sp < RAM_START) || (sp > RAM_END))
    //if ((sp & 0x2FFE0000) != 0x20000000)
    {
        usart_puts("Invalid stack pointer: 0x");
        // Convert sp to hex string and print it
        char hex[9];  // 8 chars for 32-bit hex + null terminator
        for(int i = 7; i >= 0; i--) {
            int digit = (sp >> (i * 4)) & 0xF;
            hex[7-i] = digit < 10 ? '0' + digit : 'A' + (digit - 10);
        }
        hex[8] = '\0';
        usart_puts(hex);
        usart_puts("\r\n");
        current_state = BL_STATE_ERROR;
        return;
    }
    usart_puts("Jumping to application...\r\n");

    // Disable interrupts
    __disable_irq();

    // Reset system clocks and peripherals
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));
    RCC->CFGR = 0;
    
    // Disable SysTick
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // Set vector table location
    SCB->VTOR = app_addr;

    // Set main stack pointer
    __set_MSP(sp);

    // Jump to application
    void (*app_reset_handler)(void) = (void*)entry;
    app_reset_handler();

    // Should never reach here
    while(1);
}

// -----------------------------------------------------------------------------
// Boot Environment Validation: check aliasing and VTOR config
// -----------------------------------------------------------------------------
void validate_boot_environment(void)
{
    usart_puts("Validating boot environment...\r\n");

    // Make sure the vector table is remapped to the correct ITCM alias
    if ((SCB->VTOR & 0xFFF00000) != BOOTLOADER_START_ALIAS)
    {
        usart_puts("\tError: Unexpected VTOR address.\r\n");
        while (1) { for (volatile int i = 0; i < 50000; i++); }
    }
    usart_puts("\tVTOR configuration OK\r\n");

    // Make sure aliasing between ITCM and AXI flash matches
    if (bootloader_verify_memory_aliasing() != BL_OK)
    {
        usart_puts("\tError: Memory aliasing failed.\r\n");
        current_state = BL_STATE_ERROR;
        return;
    }
    usart_puts("\tMemory aliasing verified\r\n");
}
