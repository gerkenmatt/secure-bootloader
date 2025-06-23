/**
 * @file cli.h
 * @brief Command Line Interface (CLI) for STM32 bootloader.
 */
#ifndef CLI_H
#define CLI_H

#include <stdbool.h>
#include "bootloader.h"

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Initializes the CLI module (e.g., UART for input/output).
 */
void cli_init(void);

/**
 * @brief Processes incoming characters and executes commands.
 * @param current_bootloader_state The current state of the bootloader, 
 * to determine which command set to use.
 */
void cli_process_input(bootloader_state_t current_bootloader_state);

#endif // CLI_H