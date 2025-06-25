/**
 * @file boot_config.h
 * @brief Bootloader configuration structure and management for A/B swap.
 */
#ifndef BOOT_CONFIG_H
#define BOOT_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "bootloader.h"

// -----------------------------------------------------------------------------
// Constants and Macros
// -----------------------------------------------------------------------------

#define BOOT_CONFIG_MAGIC        0xB007CF60  ///< Magic number to identify valid config
#define BOOT_ATTEMPT_COUNT       7           ///< Number of boot attempts before rollback

// -----------------------------------------------------------------------------
// Data Structures
// -----------------------------------------------------------------------------

/**
 * @brief Metadata for a firmware slot.
 */
typedef struct {
    uint32_t fw_size;                  ///< Firmware size in bytes
    uint32_t fw_crc;                   ///< CRC32 checksum
    bool     is_valid;                 ///< 1 if valid, 0 if not
    uint8_t  boot_attempts_remaining;  ///< For rollback
} slot_metadata_t;

/**
 * @brief Bootloader configuration structure for A/B swap.
 */
typedef struct {
    uint32_t        magic;             ///< Magic number to identify valid config
    slot_index_t    active_slot;       ///< 0 for Slot A, 1 for Slot B
    slot_metadata_t slot[2];            ///< Metadata for Slot A and B
} bootloader_config_t;

// -----------------------------------------------------------------------------
// Public Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Writes the bootloader configuration to flash memory.
 * @param p_new_config Pointer to the new bootloader configuration structure
 * @return true if the configuration was successfully written, false otherwise
 */
bool write_boot_config(const bootloader_config_t* p_new_config);

/**
 * @brief Initializes bootloader configuration.
 * Sets up default config values if magic number not present.
 */
void init_bootloader_config(void);

/**
 * @brief Reads the bootloader configuration from flash memory.
 * @return Pointer to the bootloader configuration structure
 */
const bootloader_config_t* read_boot_config(void);

#endif // BOOT_CONFIG_H