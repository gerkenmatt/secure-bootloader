#include "cli.h"
#include "bootloader.h"
#include "ota.h"
#include "stm32f767xx.h"
#include "uart.h"
#include "utilities.h"
#include "flash.h"
#include <string.h> 
#include <stdint.h>
#include <stdlib.h>
#include "boot_config.h"
#include "logger.h"

typedef void (*cmd_executor_t)(const char* cmd);

// Prototypes for internal CLI cmd handlers
static void execute_full_cmd_set(const char* cmd);
static void execute_recovery_cmd_set(const char* cmd);
static void handle_cmd_line_input(cmd_executor_t execute_func);

// --- Function Prototypes ---
static void handle_erase_cmd(const char* cmd_arg); 
static void handle_print_cmd(const char* cmd_arg); 
static void handle_info_cmd(void);
static void handle_activate_cmd(const char* cmd_arg);

void cli_init(void) 
{
    // Any CLI-specific initialization, e.g., print a prompt
    uart_puts("Bootloader CLI Ready.\r\n");
}

void cli_process_input(bootloader_state_t current_bl_state) 
{
    cmd_executor_t executor;
    if (current_bl_state == BL_STATE_ERROR) 
    {
        executor = execute_recovery_cmd_set;
    } 
    else 
    {
        executor = execute_full_cmd_set;
    }
    handle_cmd_line_input(executor);
}

// Renamed to avoid confusion with the public cli_process_input
static void handle_cmd_line_input(cmd_executor_t execute_func) {
    // Static variables to preserve state across calls
    static char cmd_buffer[64];
    static uint32_t buffer_index = 0;

    uint8_t byte;

    // Check if a character is available from the ring buffer
    if (uart_getc(&byte)) 
    {
        
        // --- Handle special characters ---
        if (byte == '\r' || byte == '\n') 
        { 
            // End of cmd (Enter key)
            uart_puts("\r\n"); // Echo newline for user
            
            if (buffer_index > 0) 
            {
                cmd_buffer[buffer_index] = '\0'; // Null-terminate the string
                execute_func(cmd_buffer); // Call the provided executor
            }
            buffer_index = 0; // Reset for the next cmd
        }
        else if (byte == 127 || byte == '\b') 
        { 
            // Backspace
            if (buffer_index > 0) 
            {
                buffer_index--;
                uart_puts("\b \b"); // Erase character on terminal
            }
        }
        // --- Handle printable characters ---
        else if (byte >= ' ' && byte <= '~') 
        {
            if (buffer_index < (sizeof(cmd_buffer) - 1)) 
            {
                cmd_buffer[buffer_index++] = byte;
                uart_putc(byte); // Echo character back to the user
            }
        }
    }
}

/**
 * @brief This function processes the actual cmds once a full line is received.
 * * @param cmd The null-terminated cmd string to process.
 */
static void execute_full_cmd_set(const char* cmd)
{
    if (strcmp(cmd, "run") == 0 || strcmp(cmd, "r") == 0)
    {
        LOG_INFO("'run' command received. Attempting to boot active application...\r\n");
        bootloader_jump_to_active_application();
    }
    else if (strcmp(cmd, "update") == 0)
    {
        LOG_INFO("Entering OTA mode...\r\n");
        ota_reset_timeout(); // Start the OTA timeout timer
        bootloader_set_state(BL_STATE_RECEIVING);
    }
    else if (strcmp(cmd, "reboot") == 0)
    {
        uart_puts("Rebooting...\r\n");
        SCB_CleanDCache();          // TODO: is this needed?
        NVIC_SystemReset();
    }
    else if (strcmp(cmd, "info") == 0) 
    {
        handle_info_cmd();
    }
    else if (strncmp(cmd, "erase ", 6) == 0) 
    {
        // Handle "erase <slot_num>"
        handle_erase_cmd(cmd + 6);
    }
    else if (strncmp(cmd, "p ", 2) == 0) {
        // Handle "p <slot_num>"
        handle_print_cmd(cmd + 2);
    }
    else if (strncmp(cmd, "activate ", 9) == 0) 
    {
        handle_activate_cmd(cmd + 9);
    }
    else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0)
    {
        uart_puts("Available commands:\r\n");
        uart_puts("  run  - Jump to application\r\n");
        uart_puts("  update <firmare_path>  - Start OTA update mode\r\n");
        uart_puts("  p <0|1> - Print first 10 words of slot flash\r\n");
        uart_puts("  erase <0|1> - Erase sectors of specified slot\r\n");
        uart_puts("  info - Show bootloader information\r\n");
        uart_puts("  help - Show this message\r\n");
        uart_puts("  reboot - Reboot the device\r\n");
    }
    else if (strcmp(cmd, "status") == 0)
    {
        const bootloader_config_t* cfg = read_boot_config();
        uart_printf("Active slot: %d\r\n", cfg->active_slot);
    }
    else
    {
        uart_puts("Unknown command.\r\n");
    }
}

/**
 * @brief Processes a limited, safe set of commands for recovery mode.
 */
static void execute_recovery_cmd_set(const char* cmd)
{
    if (strcmp(cmd, "reboot") == 0) 
    {
        uart_puts("Rebooting from error state...\r\n");
        NVIC_SystemReset();
    } 
    else if (strcmp(cmd, "info") == 0 || strcmp(cmd, "status") == 0) 
    {
        uart_puts("--- Recovery Mode Status ---\r\n");
        handle_info_cmd();
    } 
    else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0) 
    {
        uart_puts("--- Recovery Mode Commands ---\r\n");
        uart_puts("  reboot   - Restart the device\r\n");
        uart_puts("  info     - Show bootloader status\r\n");
        uart_puts("  help     - Show this message\r\n");
    } 
    else 
    {
        uart_puts("Unknown or disallowed command. Allowed: 'reboot', 'info', 'help'.\r\n");
    }
}


/**
 * @brief Handles the 'activate' command to switch the active slot.
 * @param cmd_arg The argument string following "activate ", e.g., "0" or "1".
 */
static void handle_activate_cmd(const char* cmd_arg) 
{
    int slot_to_activate = atoi(cmd_arg);
    if (slot_to_activate != 0 && slot_to_activate != 1) 
    {
        LOG_ERROR("Invalid slot specified. Use 'activate 0' or 'activate 1'.\r\n");
        return;
    }
    const bootloader_config_t* cfg = read_boot_config();
    // if (!cfg->slot[slot_to_activate].is_valid) 
    // {
    //     LOG_ERROR("Cannot activate an invalid slot.\r\n");
    //     return;
    // }
    bootloader_config_t new_cfg;
    memcpy(&new_cfg, cfg, sizeof(bootloader_config_t));
    new_cfg.active_slot = slot_to_activate;
    if (!write_boot_config(&new_cfg)) 
    {
        LOG_ERROR("Failed to update boot config for slot activation.\r\n");
    } 
    else 
    {
        LOG_INFO("Active slot switched to: %d\r\n", slot_to_activate);
    }
}


static void handle_info_cmd(void) 
{
    const bootloader_config_t* cfg = read_boot_config();
    uart_puts("Bootloader Configuration:\r\n");
    uart_printf("|  Magic: 0x%08X\r\n", cfg->magic);
    uart_printf("|  Active Slot: %d\r\n", cfg->active_slot);
    for (int i = 0; i < 2; i++) 
    {
        uart_printf("|  Slot %d:\r\n", i);
        uart_printf("|    is_valid: %d\r\n", cfg->slot[i].is_valid);
        uart_printf("|    boot_attempts_remaining: %d\r\n", cfg->slot[i].boot_attempts_remaining);
        uart_printf("|    fw_size: 0x%08X\r\n", cfg->slot[i].fw_size);
        uart_printf("|    fw_crc: 0x%08X\r\n", cfg->slot[i].fw_crc);
    }
}

/**
 * @brief Handles the 'p' (print) command to show memory contents of a slot.
 * @param cmd_arg The argument string following "p ", e.g., "0" or "1".
 */
static void handle_print_cmd(const char* cmd_arg) 
{
    int slot_to_print = atoi(cmd_arg);

    if (slot_to_print != 0 && slot_to_print != 1) 
    {
        uart_puts("Invalid slot specified. Use 'p 0' or 'p 1'.\r\n");
        return;
    }

    uint32_t base_addr = (slot_to_print == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;
    volatile uint32_t* flash_addr = (volatile uint32_t*)base_addr;

    uart_puts("First 10 words at SLOT_ADDR:\r\n");
    for (int i = 0; i < 10; i++)  // Read 10 32-bit words
    {
        uart_printf("0x%08X ", flash_addr[i]);
    }
    uart_puts("\r\n");
}

/**
 * @brief Handles the 'erase' command to wipe an application slot.
 * @param cmd_arg The argument string following "erase ", e.g., "0" or "1".
 */
static void handle_erase_cmd(const char* cmd_arg) 
{
    // Convert the argument string to an integer
    int slot_to_erase = atoi(cmd_arg);

    if (slot_to_erase != 0 && slot_to_erase != 1) 
    {
        uart_puts("Invalid slot specified. Use 'erase 0' or 'erase 1'.\r\n");
        return;
    }

    const bootloader_config_t* cfg = read_boot_config();
    
    // // --- SAFETY CHECK ---
    // // Prevent erasing the currently active slot.
    // if (slot_to_erase == cfg->active_slot) 
    // {
    //     uart_puts("Cannot erase the currently active slot\r\n");
    //     return;
    // }

    flash_prepare_for_write();

    uart_printf("Erasing slot: %d\r\n", slot_to_erase);

    uint8_t sector_to_erase = (slot_to_erase == SLOTA) ? SLOTA_SECTOR : SLOTB_SECTOR;

    // Perform the erase
    if (erase_flash_sectors(sector_to_erase, sector_to_erase + SLOT_SECTOR_COUNT -1))
    {
        uart_puts("Successfully erased slot\r\n");
        
        // Update the bootloader config to mark the slot as invalid
        bootloader_config_t new_cfg;
        memcpy(&new_cfg, cfg, sizeof(bootloader_config_t));
        
        new_cfg.slot[slot_to_erase].is_valid = 0;
        new_cfg.slot[slot_to_erase].boot_attempts_remaining = 0;
        new_cfg.slot[slot_to_erase].fw_size = 0;
        new_cfg.slot[slot_to_erase].fw_crc = 0xFFFFFFFF;
        
        if (!write_boot_config(&new_cfg)) 
        {
            uart_puts("Failed to update boot config after erase.\r\n");
        } 
        else 
        {
            uart_puts("Boot config updated to reflect erased slot.\r\n");
        }
    } 
    else 
    {
        uart_puts("Failed to erase slot\r\n");
    }
}