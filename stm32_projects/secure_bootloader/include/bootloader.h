#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include <stdint.h>

// -----------------------------------------------------------------------------
// Enumerations
// -----------------------------------------------------------------------------

/// Bootloader states
typedef enum {
    BL_STATE_IDLE = 0,
    BL_STATE_READY,
    BL_STATE_RECEIVING,
    BL_STATE_PROGRAMMING,
    BL_STATE_VERIFY,
    BL_STATE_ERROR
} bootloader_state_t;

/// Bootloader status codes
typedef enum {
    BL_OK = 0,
    BL_ERROR_INVALID_ADDRESS = -1,
    BL_ERROR_FLASH_ERASE = -2,
    BL_ERROR_FLASH_WRITE = -3,
    BL_ERROR_VERIFICATION = -4,
    BL_ERROR_INVALID_STATE = -5,
    BL_ERROR_INVALID_BOOT_MODE = -6
} bootloader_status_t;

// -----------------------------------------------------------------------------
// Memory Map Definitions (A/B Partition Swap Layout)
// -----------------------------------------------------------------------------

#define FLASH_BASE_ADDR          0x08000000  // Main flash memory (AXI bus)
#define ITCM_FLASH_ADDR          0x00200000  // Aliased flash memory (ITCM bus)
#define FLASH_END_ADDR           0x08200000  // 2MB total flash size
#define CR_PSIZE_MASK            ((uint32_t)0xFFFFFCFFU)

#define RAM_START                0x20000000
#define RAM_SIZE                 (512 * 1024)  // 512KB RAM
#define RAM_END                  (RAM_START + RAM_SIZE)

// --- Bootloader Layout ---
#define BOOTLOADER_START_PHYS     FLASH_BASE_ADDR
#define BOOTLOADER_START_ALIAS    ITCM_FLASH_ADDR
#define CONFIG_SECTOR             11
#define CONFIG_ADDR               0x081C0000  // Placed within the bootloader's flash region

// --- Application Slot Layout ---
#define SLOTA                     0
#define SLOTB                     1

#define SLOTA_ADDR                0x08040000  // Slot A starts at Sector 5
#define SLOTA_SECTOR              5
#define SLOTB_ADDR                0x08100000  // Slot B starts at Sector 8
#define SLOTB_SECTOR              8

#define SLOT_SIZE                 0x000C0000  // 768KB per slot
#define SLOT_SECTOR_COUNT         3           // Each slot occupies 3 x 256KB sectors

// -----------------------------------------------------------------------------
// Boot Configuration
// -----------------------------------------------------------------------------

#define BOOT_CONFIG_MAGIC        0xB007CF60
#define BOOT_ATTEMPT_COUNT       3             // Number of boot attempts before rollback


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
 * Initializes bootloader configuration and state
 * Sets up default config values if magic number not present
 */
void bootloader_init(void);

/**
 * Processes commands received from host
 * Handles commands like OTA update, configuration, etc.
 */
void process_bootloader_command(void);  

/**
 * Sets bootloader state to new value
 * 
 * @param new_state New state to set
 */
void bootloader_set_state(bootloader_state_t new_state);

/**
 * Returns current bootloader state
 * 
 * @return Current bootloader_state_t value
 */
bootloader_state_t bootloader_get_state(void);


/**
 * Jumps to the active application based on bootloader configuration
 * 
 * This function reads the bootloader configuration, determines the active slot,
 * verifies the slot's validity and CRC, and then jumps to the application.
 */
void bootloader_jump_to_active_application(void);

/**
 * Verifies memory aliasing between AXI and ITCM regions
 * Compares first 256 words to ensure proper mapping
 * 
 * @return BL_OK if aliasing verified, BL_ERROR_INVALID_BOOT_MODE if mismatch
 */
bootloader_status_t bootloader_verify_memory_aliasing(void);

/**
 * Reads and returns state of BOOT0 pin (PC13)
 * 
 * @return 1 if pin high, 0 if pin low
 */
uint32_t bootloader_get_boot_mode(void);

/**
 * Returns pointer to bootloader configuration
 * 
 * @return Pointer to bootloader_config_t structure
 */
const bootloader_config_t* bootloader_get_config(void);

/**
 * Validates boot environment configuration
 * Checks VTOR settings and memory aliasing
 */
void validate_boot_environment(void);

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

#endif // BOOTLOADER_H
