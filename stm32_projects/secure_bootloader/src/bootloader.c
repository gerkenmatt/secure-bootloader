#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bootloader.h"
#include "ota.h"
#include "stm32f7xx.h"
#include "uart.h"
#include "utilities.h"
#include "flash.h"
#include "mbedtls/platform.h"

// --- Static State ---
static bootloader_state_t current_state = BL_STATE_IDLE;

// --- Function Prototypes ---
static void bootloader_jump_to(uint32_t app_address);
static void handle_erase_command(const char* cmd_arg); 
static void handle_print_command(const char* cmd_arg); 
static void handle_bootloader_info(void);
static void handle_activate_command(const char* cmd_arg);

// --- Public Functions ---

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
    log("*****magic: "); print_uint32_hex(config->magic); log("\r\n");
    if (config->magic != BOOT_CONFIG_MAGIC) {
        // Initialize config defaults and write to flash
        log("******************Initializing bootloader configuration******************\r\n");
        init_bootloader_config();
    }

    // Needed for mbedTLS memory allocation
    mbedtls_platform_set_calloc_free(calloc, free);

    // Set state to READY
    current_state = BL_STATE_READY;
}

void init_bootloader_config(void) 
{
    bootloader_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.magic = BOOT_CONFIG_MAGIC;
    cfg.active_slot = SLOTA; // Default to booting from Slot A

    // Configure Slot A (assumed to hold a valid factory image)
    cfg.slot[0].is_valid = 1; 
    cfg.slot[0].boot_attempts_remaining = BOOT_ATTEMPT_COUNT; 
    cfg.slot[0].fw_size = 0; // Should be updated after initial flashing
    cfg.slot[0].fw_crc = 0xFFFFFFFF; // Should be updated after initial flashing

    // Configure Slot B as initially empty/invalid
    cfg.slot[1].is_valid = 0;
    cfg.slot[1].boot_attempts_remaining = 0;
    cfg.slot[1].fw_size = 0;
    cfg.slot[1].fw_crc = 0xFFFFFFFF;

    unlock_flash();
    program_flash_word(CONFIG_ADDR, 0xdeadbeef);
    lock_flash();
    if (!write_boot_config(&cfg))
    {
        log("Failed to write initial bootloader configuration!\r\n");
        current_state = BL_STATE_ERROR;
    }
    else
    {
        log("Bootloader configuration initialized successfully.\r\n");
    }
}

void process_bootloader_command(void)  
{
    if (!(USART6->ISR & USART_ISR_RXNE))
        return;

    char cmd_buf[16] = {0};
    usart_gets(cmd_buf, sizeof(cmd_buf));       // TODO: Ensure usart_gets is buffer-safe

    if (strcmp(cmd_buf, "run") == 0 || strcmp(cmd_buf, "r") == 0)
    {
        log("'run' command received. Attempting to boot active application...\r\n");
        bootloader_jump_to_active_application();
    }
    else if (strcmp(cmd_buf, "update") == 0)
    {
        usart_puts("Entering OTA mode...\r\n");
        bootloader_set_state(BL_STATE_RECEIVING);
        handle_ota_session();
        bootloader_set_state(BL_STATE_READY);
    }
    else if (strcmp(cmd_buf, "reboot") == 0)
    {
        usart_puts("Rebooting...\r\n");
        SCB_CleanDCache();          // TODO: is this needed?
        NVIC_SystemReset();
    }
    else if (strcmp(cmd_buf, "info") == 0) 
    {
        handle_bootloader_info();
    }
    else if (strncmp(cmd_buf, "erase ", 6) == 0) 
    {
        // Handle "erase <slot_num>"
        handle_erase_command(cmd_buf + 6);
    }
    else if (strncmp(cmd_buf, "p ", 2) == 0) {
        // Handle "p <slot_num>"
        handle_print_command(cmd_buf + 2);
        // volatile uint32_t* flash_addr = (volatile uint32_t*)APPLICATION_START_ADDR;
        // usart_puts("First 10 words at APPLICATION_START_ADDR:\r\n");
        // for (int i = 0; i < 10; i++)  // Read 10 32-bit words
        // {
        //     print_uint32_hex(flash_addr[i]);
        //     usart_puts(" ");
        // }
    }
    else if (strncmp(cmd_buf, "activate ", 9) == 0) 
    {
        handle_activate_command(cmd_buf + 9);
    }
    else if (strcmp(cmd_buf, "help") == 0 || strcmp(cmd_buf, "h") == 0)
    {
        usart_puts("Available commands:\r\n");
        usart_puts("  run  - Jump to application\r\n");
        usart_puts("  update <firmare_path>  - Start OTA update mode\r\n");
        usart_puts("  p <0|1> - Print first 10 words of slot flash\r\n");
        usart_puts("  erase <0|1> - Erase sectors of specified slot\r\n");
        usart_puts("  info - Show bootloader information\r\n");
        usart_puts("  help - Show this message\r\n");
        usart_puts("  reboot - Reboot the device\r\n");
    }
    else if (strcmp(cmd_buf, "status") == 0)
    {
        const bootloader_config_t* cfg = read_boot_config();
        usart_puts("Active slot: ");
        print_uint32_hex(cfg->active_slot);
        usart_puts("\r\n");
    }
    else
    {
        usart_puts("Unknown command.\r\n");
    }
}

void bootloader_jump_to_active_application(void) 
{
    const bootloader_config_t* cfg = read_boot_config();
    uint32_t active_slot_idx = cfg->active_slot;

    // Boundary check
    if (active_slot_idx > 1) 
    {
        log("Invalid active_slot index \r\n");
        bootloader_set_state(BL_STATE_ERROR);
        return;
    }

    // --- Rollback Logic ---
    if (cfg->slot[active_slot_idx].boot_attempts_remaining == 0) {
        log("!!! Boot attempts failed for slot. Rolling back... !!!\r\n");
        
        uint32_t fallback_slot_idx = (active_slot_idx == SLOTA) ? SLOTB : SLOTA;
        
        // Roll back to fallback slot if it is valid
        if (cfg->slot[fallback_slot_idx].is_valid) 
        {
            bootloader_config_t new_cfg;
            memcpy(&new_cfg, cfg, sizeof(bootloader_config_t));
            new_cfg.active_slot = fallback_slot_idx; // Swap to the other slot
            // Restore boot attempts for the slot we are rolling back TO
            new_cfg.slot[fallback_slot_idx].boot_attempts_remaining = BOOT_ATTEMPT_COUNT; 
            write_boot_config(&new_cfg);
            
            log("Rolled back to valid slot. Rebooting...\r\n");
            NVIC_SystemReset();
        } 
        else 
        {
            log("!!! Fallback slot is not valid. Cannot roll back. Halting. !!!\r\n");
            bootloader_set_state(BL_STATE_ERROR);
            return;
        }
    }

    // --- Decrement Boot Counter ---
    // If we are about to boot a valid slot with attempts remaining, use one attempt.
    if (cfg->slot[active_slot_idx].is_valid) 
    {
        bootloader_config_t new_cfg;
        memcpy(&new_cfg, cfg, sizeof(bootloader_config_t));
        new_cfg.slot[active_slot_idx].boot_attempts_remaining--;
        write_boot_config(&new_cfg);
    }

    // --- Determine Jump Address ---
    uint32_t jump_address = (active_slot_idx == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;
    
    // --- Verify and Jump ---
    const slot_metadata_t* active_slot_meta = &cfg->slot[active_slot_idx];

    if (!active_slot_meta->is_valid) {
        log("Attempting to boot invalid slot. Aborting.\r\n");
        bootloader_set_state(BL_STATE_ERROR);
        return;
    }
    
    if (!verify_crc(jump_address, active_slot_meta->fw_size, active_slot_meta->fw_crc)) {
        log("CRC check failed for active slot. Aborting jump.\r\n");
        // This boot attempt failed. The counter was already decremented. On next boot, it will try again or roll back.
        log("Rebooting to re-evaluate boot state...\r\n");
        NVIC_SystemReset();
        return;
    }

    bootloader_jump_to(jump_address);
}

static void bootloader_jump_to(uint32_t app_address) 
{
    log("Jumping to application at: "); print_uint32_hex(app_address); usart_puts("\r\n");

    // De-initialize peripherals and disable interrupts before jumping
    __disable_irq();
    // It's good practice to de-init peripherals used by bootloader, e.g., usart_deinit();
    
    // Reset system clocks to default state before handing over
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));
    RCC->CFGR = 0;
    
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // Set the application's vector table address
    SCB->VTOR = app_address;

    // Get the application's stack pointer and entry point
    uint32_t* vec_tab = (uint32_t*)app_address;
    uint32_t app_sp = vec_tab[0];
    uint32_t app_entry = vec_tab[1];

    // Set the main stack pointer
    __set_MSP(app_sp);

    // Create a function pointer to the application's reset handler and jump
    void (*app_reset_handler)(void) = (void*)app_entry;
    app_reset_handler();

    // This part should never be reached
    while(1);
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

// uint32_t bootloader_get_boot_mode(void)
// {
//     return (GPIOC->IDR & GPIO_IDR_ID13) ? 1 : 0;
// }


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

bool write_boot_config(const bootloader_config_t* new_config) 
{
    if (!new_config) return false;


    //---------TODO: can I move this to function? 
    clear_flash_errors();
    unlock_flash();

    // SCB_DisableDCache();

    // Configure flash access control and program size
    FLASH->ACR |= (1 << 8) | (1 << 9); // Enable instruction and data cache
    FLASH->CR |= FLASH_CR_PSIZE_1;      // Set program size to 32-bit
    //--------------------------------------------

    uint32_t errs = FLASH->SR;
    // Erase the config sector
    if (!erase_flash_sectors(CONFIG_SECTOR, CONFIG_SECTOR, CONFIG_ADDR, 0x40000)) 
    {
        log("Failed to eraseb config sector!\r\n");
        log("Flash errors before: "); print_uint32_hex(errs); log("\r\n");
        lock_flash(); // Always re-lock flash
        return false;
    }

    // Write the new config struct to flash
    if (!program_flash(CONFIG_ADDR, (const uint32_t*)new_config, sizeof(bootloader_config_t))) 
    {
        log("Failed to write new config to flash!\r\n");
        lock_flash();
        return false;
    }
    // SCB_EnableDCache();

    lock_flash();

    return true;

}

/**
 * @brief Handles the 'erase' command to wipe an application slot.
 * @param cmd_arg The argument string following "erase ", e.g., "0" or "1".
 */
static void handle_erase_command(const char* cmd_arg) 
{
    // Convert the argument string to an integer
    int slot_to_erase = atoi(cmd_arg);

    if (slot_to_erase != 0 && slot_to_erase != 1) 
    {
        log("Invalid slot specified. Use 'erase 0' or 'erase 1'.\r\n");
        return;
    }

    const bootloader_config_t* cfg = read_boot_config();
    
    // // --- SAFETY CHECK ---
    // // Prevent erasing the currently active slot.
    // if (slot_to_erase == cfg->active_slot) 
    // {
    //     log("Cannot erase the currently active slot\r\n");
    //     return;
    // }

    unlock_flash();
    clear_flash_errors();

    // Configure flash access control and program size
    FLASH->ACR |= (1 << 8) | (1 << 9);  // Enable instruction and data cache
    FLASH->CR |= FLASH_CR_PSIZE_1;      // Set program size to 32-bit


    log("Erasing slot: "); print_uint8_hex(slot_to_erase); log("\r\n");

    uint8_t sector_to_erase = (slot_to_erase == SLOTA) ? SLOTA_SECTOR : SLOTB_SECTOR;
    uint32_t erase_slot_addr = (slot_to_erase == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;


    // Perform the erase
    if (erase_flash_sectors(
        sector_to_erase, 
        sector_to_erase + SLOT_SECTOR_COUNT -1, 
        erase_slot_addr, 
        SLOT_SIZE)) 
    {
        log("Successfully erased slot\r\n");
        
        // Update the bootloader config to mark the slot as invalid
        bootloader_config_t new_cfg;
        memcpy(&new_cfg, cfg, sizeof(bootloader_config_t));
        
        new_cfg.slot[slot_to_erase].is_valid = 0;
        new_cfg.slot[slot_to_erase].boot_attempts_remaining = 0;
        new_cfg.slot[slot_to_erase].fw_size = 0;
        new_cfg.slot[slot_to_erase].fw_crc = 0xFFFFFFFF;
        
        if (!write_boot_config(&new_cfg)) 
        {
            log("Failed to update boot config after erase.\r\n");
        } 
        else 
        {
            log("Boot config updated to reflect erased slot.\r\n");
        }
    } 
    else 
    {
        log("Failed to erase slot\r\n");
    }
}

/**
 * @brief Handles the 'p' (print) command to show memory contents of a slot.
 * @param cmd_arg The argument string following "p ", e.g., "0" or "1".
 */
static void handle_print_command(const char* cmd_arg) 
{
    int slot_to_print = atoi(cmd_arg);

    if (slot_to_print != 0 && slot_to_print != 1) 
    {
        log("Invalid slot specified. Use 'p 0' or 'p 1'.\r\n");
        return;
    }

    uint32_t base_addr = (slot_to_print == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;
    volatile uint32_t* flash_addr = (volatile uint32_t*)base_addr;

    usart_puts("First 10 words at SLOT_ADDR:\r\n");
    for (int i = 0; i < 10; i++)  // Read 10 32-bit words
    {
        print_uint32_hex(flash_addr[i]);
        usart_puts(" ");
    }
    usart_puts("\r\n");
}

static void handle_bootloader_info(void) 
{
    const bootloader_config_t* cfg = read_boot_config();
    usart_puts("Bootloader Configuration:\r\n");
    usart_puts("|  Magic: 0x"); print_uint32_hex(cfg->magic); usart_puts("\r\n");
    usart_puts("|  Active Slot: "); print_uint32_hex(cfg->active_slot); usart_puts("\r\n");
    for (int i = 0; i < 2; i++) {
        usart_puts("|  Slot "); print_uint32_hex(i); usart_puts(":\r\n");
        usart_puts("|    is_valid: "); print_uint32_hex(cfg->slot[i].is_valid); usart_puts("\r\n");
        usart_puts("|    boot_attempts_remaining: "); print_uint32_hex(cfg->slot[i].boot_attempts_remaining); usart_puts("\r\n");
        usart_puts("|    fw_size: 0x"); print_uint32_hex(cfg->slot[i].fw_size); usart_puts("\r\n");
        usart_puts("|    fw_crc: 0x"); print_uint32_hex(cfg->slot[i].fw_crc); usart_puts("\r\n");
    }
}



const bootloader_config_t* read_boot_config(void) 
{
    return (const bootloader_config_t*)CONFIG_ADDR;
}

/**
 * @brief Handles the 'activate' command to switch the active slot.
 * @param cmd_arg The argument string following "activate ", e.g., "0" or "1".
 */
static void handle_activate_command(const char* cmd_arg) 
{
    int slot_to_activate = atoi(cmd_arg);
    if (slot_to_activate != 0 && slot_to_activate != 1) 
    {
        log("Invalid slot specified. Use 'activate 0' or 'activate 1'.\r\n");
        return;
    }
    const bootloader_config_t* cfg = read_boot_config();
    // if (!cfg->slot[slot_to_activate].is_valid) 
    // {
    //     log("Cannot activate an invalid slot.\r\n");
    //     return;
    // }
    bootloader_config_t new_cfg;
    memcpy(&new_cfg, cfg, sizeof(bootloader_config_t));
    new_cfg.active_slot = slot_to_activate;
    if (!write_boot_config(&new_cfg)) 
    {
        log("Failed to update boot config for slot activation.\r\n");
    } 
    else 
    {
        log("Active slot switched to: "); print_uint32_hex(slot_to_activate); log("\r\n");
    }
}