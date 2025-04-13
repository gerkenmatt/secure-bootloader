#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bootloader.h"
#include "ota.h"
#include "stm32f7xx.h"
#include "uart.h"
#include "utilities.h"
#include "flash.h"

// -----------------------------------------------------------------------------
// Static State and Configuration
// -----------------------------------------------------------------------------
static bootloader_state_t current_state = BL_STATE_IDLE;


bootloader_state_t bootloader_get_state(void)
{
    return current_state;
}

void bootloader_set_state(bootloader_state_t new_state)
{
    current_state = new_state;
}

void bootloader_init(void)
{
    // Initialize configuration if not already set
    const bootloader_config_t* config = read_boot_config();
    if (config->magic != BOOT_CONFIG_MAGIC) {
        // Initialize config defaults and write to flash
        init_bootloader_config();
    }

    // Set state to READY
    current_state = BL_STATE_READY;
}

void init_bootloader_config(void) {
    bootloader_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.magic = BOOT_CONFIG_MAGIC;
    cfg.reboot_cause = 0;  // 0 = NORMAL_BOOT

    for (int i = 0; i < 2; i++) {
        cfg.slot[i].fw_crc      = 0xFFFFFFFF;
        cfg.slot[i].fw_size     = 0;
        cfg.slot[i].is_valid    = 0;
        cfg.slot[i].is_active   = 0;
        cfg.slot[i].should_run  = 0;
    }

    // Make slot 0 the default active slot
    cfg.slot[0].is_active = 1;
    cfg.slot[0].is_valid = 1;

    write_boot_config(&cfg);
}

void process_bootloader_command(void)  
{
    if (!(USART6->ISR & USART_ISR_RXNE))
        return;

    char cmd_buf[16] = {0};
    usart_gets(cmd_buf, sizeof(cmd_buf));

    if (strcmp(cmd_buf, "run") == 0 || strcmp(cmd_buf, "r") == 0)
    {
        usart_puts("Checking for firmware update...\r\n");
        load_new_app();  // apply update from slot 1 if needed
        usart_puts("Run application...\r\n");
        bootloader_jump_to_application(0x08040000);
    }
    else if (strcmp(cmd_buf, "ota") == 0)
    {
        usart_puts("Entering OTA mode...\r\n");
        bootloader_set_state(BL_STATE_RECEIVING);
        handle_ota_session();
        bootloader_set_state(BL_STATE_READY);
    }
    else if (strcmp(cmd_buf, "help") == 0 || strcmp(cmd_buf, "h") == 0)
    {
        usart_puts("Available commands:\r\n");
        usart_puts("  run  - Jump to application\r\n");
        usart_puts("  ota  - Start OTA update mode\r\n");
        usart_puts("  p    - Print first 10 words of application flash\r\n");
        usart_puts("  help - Show this message\r\n");
    }
    else if (strcmp(cmd_buf, "p") == 0)
    {
        volatile uint32_t* flash_addr = (volatile uint32_t*)0x08040000;
        usart_puts("First 10 words at 0x08040000:\r\n");
        for (int i = 0; i < 10; i++)  // Read 10 32-bit words
        {
            print_uint32_hex(flash_addr[i]);
            usart_puts(" ");
        }
        usart_puts("\r\n");
    }
    else
    {
        usart_puts("Unknown command.\r\n");
    }
}

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

uint32_t bootloader_get_boot_mode(void)
{
    return (GPIOC->IDR & GPIO_IDR_ID13) ? 1 : 0;
}

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

    // Verify that sp is a valid RAM address (starting at 0x20000000)
    if ((sp < RAM_START) || (sp > RAM_END))
    {
        usart_puts("Invalid stack pointer: 0x"); print_uint32_hex(sp); usart_puts("\r\n");
        current_state = BL_STATE_ERROR;
        return;
    }

    const bootloader_config_t* config = read_boot_config();
    const uint32_t size = config->slot[0].fw_size;
    const uint32_t crc  = config->slot[0].fw_crc;

    if (!verify_crc(APPLICATION_START_ADDR, size, crc)) {
        usart_puts("CRC check failed for active app. Aborting jump.\r\n");
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

bool write_boot_config(const bootloader_config_t* new_config) {
    if (!new_config) return false;

    unlock_flash();

    // Configure flash access control and program size
    FLASH->ACR |= (1 << 8) | (1 << 9); // Enable instruction and data cache
    FLASH->CR |= FLASH_CR_PSIZE_1;      // Set program size to 32-bit


    // Erase sector of bootloader
    FLASH->CR &= ~FLASH_CR_SNB;
    FLASH->CR |= FLASH_CR_SER | (CONFIG_SECTOR << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_SER;

    // Write config struct to flash word-by-word
    uint32_t* flash_ptr = (uint32_t*)read_boot_config();
    const uint32_t* data = (const uint32_t*)new_config;
    size_t word_count = sizeof(bootloader_config_t) / 4;

    FLASH->CR |= FLASH_CR_PG;
    for (size_t i = 0; i < word_count; i++) {
        flash_ptr[i] = data[i];
        __DSB(); __ISB();
        while (FLASH->SR & FLASH_SR_BSY);
    }
    FLASH->CR &= ~FLASH_CR_PG;

    // Lock flash
    FLASH->CR |= FLASH_CR_LOCK;

    return true;
}

void update_config_for_slot0(uint32_t fw_size, uint32_t fw_crc) {
    // Copy the current configuration from flash
    log("Updating config for slot 0\r\n");
    bootloader_config_t cfg = *read_boot_config();

    cfg.slot[0].fw_size     = fw_size;
    cfg.slot[0].fw_crc      = fw_crc;
    cfg.slot[0].is_valid    = 1;
    cfg.slot[0].is_active   = 1;
    cfg.slot[0].should_run  = 0;

    cfg.slot[1].should_run  = 0;  // Cancel attempt to run new firmware
    cfg.reboot_cause        = 0;  // NORMAL_BOOT

    // Write updated configuration to flash
    write_boot_config(&cfg);
}

void load_new_app(void) {
    const bootloader_config_t* cfg = read_boot_config();

    // Check if slot 1 has a new app we should run
    if (cfg->slot[1].should_run && cfg->slot[1].is_valid) {
        usart_puts("New firmware available in slot 1. Restoring...\r\n");

        if (verify_crc(SLOT1_ADDR, cfg->slot[1].fw_size, cfg->slot[1].fw_crc)) {
            if (copy_firmware(SLOT1_ADDR, APPLICATION_START_ADDR, cfg->slot[1].fw_size)) {
                usart_puts("Firmware restored to app area.\r\n");
                update_config_for_slot0(cfg->slot[1].fw_size, cfg->slot[1].fw_crc);
                return;
            }
            else {
                usart_puts("Error: failed to copy firmware from slot 1.\r\n");
            }
        } else {
            usart_puts("Error: slot 1 CRC check failed.\r\n");
        }
    }

    usart_puts("No new firmware update to load.\r\n");
}

const bootloader_config_t* read_boot_config(void) {
    return (const bootloader_config_t*)CONFIG_ADDR;
}