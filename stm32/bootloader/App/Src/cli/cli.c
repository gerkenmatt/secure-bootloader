#include "cli.h"
#include "bootloader.h"
#include "ota.h"
#include "stm32f767xx.h"
#include "uart.h"
#include "flash.h"
#include "boot_config.h"
#include "logger.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

// -----------------------------------------------------------------------------
// Private Constants
// -----------------------------------------------------------------------------

#define CLI_CMD_BUFFER_SIZE 64

// -----------------------------------------------------------------------------
// Type Definitions
// -----------------------------------------------------------------------------

// Forward declaration of the command structure
typedef struct cli_command_s cli_command_t;

// A consistent function pointer type for all command handlers
typedef void (*cmd_handler_t)(const char* args);

// The command definition structure
struct cli_command_s {
    const char* name;
    cmd_handler_t handler;
    const char* help;
};

// -----------------------------------------------------------------------------
// Static Function Prototypes
// -----------------------------------------------------------------------------

// --- Prototypes for internal CLI cmd handlers ---

/**
 * @brief Handles the command line input and executes the appropriate command.
 * @param args The command line input string.
 */
static void handle_run_cmd(const char* args);

/**
 * @brief Handles the "update" command to start the OTA update process.
 * @param args The command line input string.
 */
static void handle_update_cmd(const char* args);

/**
 * @brief Handles the "reboot" command to reset the device.
 * @param args The command line input string.
 */
static void handle_reboot_cmd(const char* args);

/**
 * @brief Handles the "info" command to display bootloader information.
 * @param args The command line input string.
 */
static void handle_info_cmd(const char* args);

/**
 * @brief Handles the "status" command to display the current bootloader status.
 * @param args The command line input string.
 */
static void handle_status_cmd(const char* args);

/**
 * @brief Handles the "erase <slot_num>" command to erase sectors of the specified slot.
 * @param args The argument string following "erase ", e.g., "0" or "1".
 */
static void handle_erase_cmd(const char* args); 

/**
 * @brief Handles the "p <slot_num>" command to print the first 10 words of the specified slot's flash.
 * @param args The argument string following "p ", e.g., "0" or "1".
 */
static void handle_print_cmd(const char* args); 

/**
 * @brief Handles the "activate <slot_num>" command to switch the active slot.
 * @param cmd_arg The argument string following "activate ", e.g., "0" or
 */
static void handle_activate_cmd(const char* cmd_arg);

/**
 * @brief Handles the "help" command to display available commands.
 * @param args The command line input string.
 */
static void handle_help_cmd(const char* args);

// --- Internal Helpers ---

/**
 * @brief Executes a command based on the provided command line input.
 * @param cmd_line The command line input string.
 * @param command_table The table of available commands.
 * @param table_size The size of the command table.
 */
static void execute_command(const char* cmd_line, const cli_command_t* command_table, size_t table_size);

/**
 * @brief Parses a slot argument from the command line input.
 * @param command_table The table of available commands.
 * @param table_size The size of the command table.
 */
static void handle_cmd_line_input(const cli_command_t* command_table, size_t table_size);

/**
 * @brief Parses a slot argument from the command line input.
 * @param arg The argument string to parse.
 * @param slot Pointer to the slot index to store the parsed value.
 * @return true if the argument was successfully parsed, false otherwise.
 */
static bool parse_slot_arg(const char* arg, slot_index_t* slot);


// -----------------------------------------------------------------------------
// Command Dispatch Tables
// -----------------------------------------------------------------------------

// Full command set for normal bootloader operation
static const cli_command_t g_full_commands[] = {
    { "run",      handle_run_cmd,      "Jump to active application" },
    { "update",   handle_update_cmd,   "Enter OTA update mode" },
    { "reboot",   handle_reboot_cmd,   "Reboot the device" },
    { "info",     handle_info_cmd,     "Show detailed bootloader config" },
    { "status",   handle_status_cmd,   "Show active slot status" },
    { "erase",    handle_erase_cmd,    "erase <0|1> - Erase a slot" },
    { "print",    handle_print_cmd,    "print <0|1> - Print 10 words of a slot" },
    { "activate", handle_activate_cmd, "activate <0|1> - Set active slot" },
    { "help",     handle_help_cmd,     "Help" }
};

// Limited command set available when the bootloader is in an error state, for recovery.
static const cli_command_t g_recovery_commands[] = {
    { "reboot", handle_reboot_cmd, "Reboot the device" },
    { "info",   handle_info_cmd,   "Show detailed bootloader config" },
    { "status", handle_status_cmd, "Show active slot status" },
    { "help",   handle_help_cmd,   "Help" }
};


// -----------------------------------------------------------------------------
// Public Function Implementations
// -----------------------------------------------------------------------------

void cli_process_input(bootloader_state_t current_bl_state)
{
    // Select the appropriate command table based on the current bootloader state.
    if (current_bl_state == BL_STATE_ERROR) 
    {
        handle_cmd_line_input(g_recovery_commands, sizeof(g_recovery_commands) / sizeof(cli_command_t));
    } 
    else 
    {
        handle_cmd_line_input(g_full_commands, sizeof(g_full_commands) / sizeof(cli_command_t));
    }
}

// -----------------------------------------------------------------------------
// Static Function Implementations
// -----------------------------------------------------------------------------

static void handle_cmd_line_input(const cli_command_t* command_table, size_t table_size)
{
    static char cmd_buffer[CLI_CMD_BUFFER_SIZE];
    static uint32_t buffer_index = 0;
    uint8_t byte;

    // Attempt to read a character from UART.
    if (!uart_getc(&byte))
        return; // No new character available, exit.

    if (byte == '\r' || byte == '\n')
    {
        // Newline or carriage return indicates end of command.
        uart_puts("\r\n"); // Echo newline to terminal.
        if (buffer_index > 0)
        {
            cmd_buffer[buffer_index] = '\0'; // Null-terminate the command string.
            execute_command(cmd_buffer, command_table, table_size);
        }
        buffer_index = 0; // Reset buffer for the next command.
    }
    else if (byte == 127 || byte == '\b')
    {
        // Handle backspace or delete key.
        if (buffer_index > 0)
        {
            buffer_index--;
            uart_puts("\b \b"); // Erase character from terminal: backspace, space, backspace.
        }
    }
    else if (isprint(byte) && buffer_index < (CLI_CMD_BUFFER_SIZE - 1))
    {
        // Store printable characters in the buffer, preventing overflow.
        cmd_buffer[buffer_index++] = byte;
        uart_putc(byte); // Echo character back to the terminal.
    }
}
static void execute_command(const char* cmd_line, const cli_command_t* command_table, size_t table_size)
{
    char cmd_name[CLI_CMD_BUFFER_SIZE];
    const char* args = ""; // Pointer to hold command arguments, defaults to empty.

    // Find the first space to delineate the command name from its arguments.
    const char* first_space = strchr(cmd_line, ' ');
    if (first_space != NULL)
    {
        size_t cmd_len = first_space - cmd_line;
        // Copy the command name, ensuring it fits the buffer and is null-terminated.
        if (cmd_len < sizeof(cmd_name))
        {
            memcpy(cmd_name, cmd_line, cmd_len);
            cmd_name[cmd_len] = '\0';
            args = first_space + 1; // Arguments start immediately after the space.
        }
    }
    else
    {
        // If no space is found, the entire line is the command name.
        strncpy(cmd_name, cmd_line, sizeof(cmd_name) - 1);
        cmd_name[sizeof(cmd_name) - 1] = '\0'; // Ensure null-termination.
    }

    // Iterate through the command table to find a matching command name.
    for (size_t i = 0; i < table_size; i++)
    {
        if (strcmp(cmd_name, command_table[i].name) == 0)
        {
            command_table[i].handler(args); // Execute the handler for the matched command.
            return; // Command found and executed, exit.
        }
    }

    // If no command was found after checking the entire table.
    uart_puts("Unknown command. Type 'help'.\r\n"); // Add newline for better terminal output.
}

static bool parse_slot_arg(const char* arg, slot_index_t* slot)
{
    char* endptr;
    // Convert string argument to a long integer, specifying base 10.
    long val = strtol(arg, &endptr, 10);

    // Check for conversion errors:
    // 1. `endptr == arg`: No digits were found, `strtol` didn't convert anything.
    // 2. `*endptr != '\0'`: There are non-numeric characters after the number, meaning input was not purely numeric.
    if (endptr == arg || *endptr != '\0')
    {
        uart_puts("ERROR: Argument must be a number.\r\n");
        return false;
    }

    // Validate the parsed number against expected slot values.
    if (val != SLOTA && val != SLOTB)
    {
        uart_printf("ERROR: Invalid slot number '%ld'. Use 0 for SLOT_A or 1 for SLOT_B.\r\n", val);
        return false;
    }

    *slot = (slot_index_t)val; // Cast and store the valid slot index.
    return true;
}

static void handle_run_cmd(const char* args)
{
    LOG_INFO("'run' command received. Attempting to boot active application...");
    bootloader_jump_to_active_application();
}

static void handle_update_cmd(const char* args)
{
    LOG_INFO("Entering OTA mode...");
    ota_reset_timeout(); // Reset OTA timeout to allow time for new firmware reception.
    bootloader_set_state(BL_STATE_RECEIVING); // Change bootloader state to enable OTA processing.
}

static void handle_reboot_cmd(const char* args)
{
    uart_puts("Rebooting...\r\n");
    NVIC_SystemReset(); // Trigger a system reset.
}

static void handle_status_cmd(const char* args)
{
    const bootloader_config_t* cfg = read_boot_config();
    // Display the currently active slot number.
    uart_printf("Active slot: %d\r\n", cfg->active_slot);
}

static void handle_activate_cmd(const char* args)
{
    slot_index_t slot_to_activate;
    // Attempt to parse the slot number from the command argument.
    if (!parse_slot_arg(args, &slot_to_activate))
        return; // Exit if argument is invalid.

    const bootloader_config_t* cfg = read_boot_config();
    // Check if the target slot contains valid firmware before attempting to activate.
    if (!cfg->slot[slot_to_activate].is_valid)
    {
        LOG_ERROR("Cannot activate an invalid slot.");
        return;
    }

    // Create a mutable copy of the bootloader configuration to modify the active slot.
    bootloader_config_t new_cfg;
    memcpy(&new_cfg, cfg, sizeof(bootloader_config_t));
    new_cfg.active_slot = slot_to_activate;

    // Attempt to write the updated configuration back to flash.
    if (write_boot_config(&new_cfg))
        LOG_INFO("Active slot switched to: %d", slot_to_activate);
    else
        LOG_ERROR("Failed to update boot config for slot activation.");
}

static void handle_info_cmd(const char* args)
{
    const bootloader_config_t* cfg = read_boot_config();
    uart_puts("--- Bootloader Configuration ---\r\n");
    // Display global configuration details, including magic number validity.
    uart_printf("| Magic: 0x%08X %s\r\n", cfg->magic, (cfg->magic == BOOT_CONFIG_MAGIC) ? "(Valid)" : "(INVALID!)");
    uart_printf("| Active Slot: %d\r\n", cfg->active_slot);
    // Iterate through all slots to display individual slot metadata.
    for (int i = 0; i < NUM_SLOTS; i++)
    {
        uart_printf("| Slot %c:\r\n", 'A' + i);
        uart_printf("|    is_valid: %s\r\n", cfg->slot[i].is_valid ? "true" : "false");
        uart_printf("|    boot_attempts: %d\r\n", cfg->slot[i].boot_attempts_remaining);
        uart_printf("|    fw_size: %u bytes\r\n", cfg->slot[i].fw_size);
        uart_printf("|    fw_crc: 0x%08X\r\n", cfg->slot[i].fw_crc);
    }
}

static void handle_print_cmd(const char* args)
{
    slot_index_t slot_to_print;
    // Parse the slot number from the command argument.
    if (!parse_slot_arg(args, &slot_to_print)) return;

    // Determine the base address of the selected slot in flash memory.
    uint32_t base_addr = (slot_to_print == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;
    // Cast to volatile uint32_t pointer for reading flash memory word by word.
    volatile uint32_t* flash_ptr = (volatile uint32_t*)base_addr;

    uart_printf("First 10 words of Slot %d (Address 0x%08X):\r\n", slot_to_print, base_addr);
    // Print the first 10 32-bit words from the specified slot.
    for (int i = 0; i < 10; i++) {
        uart_printf("0x%08X ", flash_ptr[i]);
    }
    uart_puts("\r\n");
}


static void handle_erase_cmd(const char* args)
{
    slot_index_t slot_to_erase;
    // Parse the slot number from the command argument.
    if (!parse_slot_arg(args, &slot_to_erase))
        return;

    const bootloader_config_t* cfg = read_boot_config();
    // Prevent erasing the currently active boot slot to avoid bricking.
    if (slot_to_erase == cfg->active_slot)
    {
        LOG_ERROR("Cannot erase the currently active slot.");
        return;
    }

    LOG_INFO("Erasing slot: %d", slot_to_erase);
    flash_prepare_for_write(); // Prepare flash for erase operations.
    // Determine the starting sector for the target slot.
    uint8_t sector_start = (slot_to_erase == SLOTA) ? SLOTA_SECTOR : SLOTB_SECTOR;

    // Erase the relevant flash sectors for the chosen slot.
    if (erase_flash_sectors(sector_start, sector_start + SLOT_SECTOR_COUNT - 1) == FLASH_OK)
    {
        LOG_INFO("Successfully erased slot flash.");

        // Update the boot configuration to mark the erased slot as invalid and clear its metadata.
        bootloader_config_t new_cfg;
        memcpy(&new_cfg, cfg, sizeof(bootloader_config_t));

        new_cfg.slot[slot_to_erase].is_valid = false;
        new_cfg.slot[slot_to_erase].boot_attempts_remaining = 0;
        new_cfg.slot[slot_to_erase].fw_size = 0;
        new_cfg.slot[slot_to_erase].fw_crc = 0xFFFFFFFF; // Reset CRC to an invalid value.

        // Write the updated configuration to flash.
        if (write_boot_config(&new_cfg))
            LOG_INFO("Boot config updated to mark slot as invalid.");
        else
            LOG_ERROR("Failed to update boot config after erase.");
    }
    else
    {
        LOG_ERROR("Failed to erase slot flash.");
    }

    lock_flash(); // Always lock flash after operations.
}

static void handle_help_cmd(const char* args)
{
    const cli_command_t* command_table;
    size_t table_size;

    // Select the appropriate command table and print header based on current bootloader state.
    if (bootloader_get_state() == BL_STATE_ERROR)
    {
        command_table = g_recovery_commands;
        table_size = sizeof(g_recovery_commands) / sizeof(cli_command_t);
        uart_puts("--- Recovery Mode Commands ---\r\n");
    }
    else
    {
        command_table = g_full_commands;
        table_size = sizeof(g_full_commands) / sizeof(cli_command_t);
        uart_puts("--- Available Commands ---\r\n");
    }

    // Print each command name and its associated help message.
    for (size_t i = 0; i < table_size; i++)
    {
        uart_printf("  %-10s - %s\r\n", command_table[i].name, command_table[i].help);
    }
}