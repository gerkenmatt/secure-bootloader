#ifndef BOOT_CONFIG_H
#define BOOT_CONFIG_H   

#include <stdint.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// Boot Configuration
// -----------------------------------------------------------------------------

#define BOOT_CONFIG_MAGIC        0xB007CF60
#define BOOT_ATTEMPT_COUNT       7            // Number of boot attempts before rollback


typedef struct {
    uint32_t fw_size;
    uint32_t fw_crc;
    uint8_t  is_valid;
    uint8_t  boot_attempts_remaining; // For rollback
} slot_metadata_t;

// Bootloader configuration structure for A/B swap
typedef struct {
    uint32_t magic;             // Magic number to identify valid config
    uint32_t active_slot;       // 0 for Slot A, 1 for Slot B
    slot_metadata_t slot[2];    // Metadata for Slot A and B
} bootloader_config_t;

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------


/**
 * Writes the bootloader configuration to flash memory
 * 
 * This function unlocks the flash memory, erases the sector where the 
 * bootloader configuration is stored, and writes the new configuration 
 * data to flash. It then locks the flash memory again.
 * 
 * @param new_config Pointer to the new bootloader configuration structure
 * @return true if the configuration was successfully written, false otherwise
 */
bool write_boot_config(const bootloader_config_t* new_config);

/**
 * Initializes bootloader configuration
 * Sets up default config values if magic number not present
 */
void init_bootloader_config(void); 

/**
 * Reads the bootloader configuration from flash memory
 * 
 * @return Pointer to bootloader_config_t structure
 */
const bootloader_config_t* read_boot_config(void);

#endif // BOOT_CONFIG_H