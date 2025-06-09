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
// Memory Map Definitions
// -----------------------------------------------------------------------------

#define FLASH_BASE_ADDR          0x08000000  // Main flash memory (AXI bus)
#define ITCM_FLASH_ADDR          0x00200000  // Aliased flash memory (ITCM bus)
#define SYSTEM_BOOTLOADER_ADDR   0x00100000  // System bootloader (ROM)

#define RAM_START                0x20000000
#define RAM_SIZE                 (512 * 1024)  // 512KB RAM
#define RAM_END                  (RAM_START + RAM_SIZE)

#define FLASH_END_ADDR           0x08200000  // 2MB total flash size
#define CR_PSIZE_MASK            ((uint32_t)0xFFFFFCFFU)

// -----------------------------------------------------------------------------
// Flash Layout
// -----------------------------------------------------------------------------

#define BOOTLOADER_SIZE          (256 * 1024)   // 256KB
#define BOOTLOADER_START_PHYS    FLASH_BASE_ADDR
#define BOOTLOADER_START_ALIAS   ITCM_FLASH_ADDR
#define APPLICATION_START_ADDR   0x08040000

#define SLOT0_ADDR               0x08080000     // Starts at Sector 6
#define SLOT1_ADDR               0x08100000     // Starts at Sector 8
#define SLOT_SIZE                0x80000        // 512KB

#define CONFIG_SECTOR            3
#define CONFIG_ADDR              0x08018000     // 32 KB sector for bootloader config
#define SLOT0_SECTOR             6
#define SLOT1_SECTOR             8

// -----------------------------------------------------------------------------
// Boot Configuration
// -----------------------------------------------------------------------------

#define BOOT_CONFIG_MAGIC        0xB007CF60
#define BOOT_FLAG_STAY_IN_BL     0x01
#define BOOT_FLAG_UPDATE_APP     0x02

/// Bootloader configuration structure (stored in last page of bootloader section)
typedef struct {
    uint32_t magic;         // Magic number to identify valid config
    uint32_t reboot_cause;  // Reason for reset: NORMAL_BOOT, OTA_REQUEST, etc.

    struct {
        uint32_t fw_crc;      // CRC32 of the firmware in this slot
        uint32_t fw_size;     // Size in bytes of the firmware
        uint8_t  is_valid;    // 1 = CRC passed, 0 = invalid or empty
        uint8_t  is_active;   // 1 = this slot is currently active
        uint8_t  should_run;  // 1 = try this firmware on next boot
        uint8_t  reserved;    // Padding for alignment
    } slot[2];               // slot[0] = primary, slot[1] = backup/new
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
 * Transitions execution to user application at specified address
 * Validates address, stack pointer and sets up environment before jumping
 * 
 * @param app_addr Starting address of application in flash
 */
void bootloader_jump_to_application(uint32_t app_address);

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
 * Updates the bootloader configuration for slot 0 (primary firmware slot)
 * 
 * This function updates the bootloader configuration with new firmware details
 * for slot 0, marking it as valid and active. It also ensures slot 1 won't run
 * on next boot by clearing its should_run flag.
 * 
 * @param fw_size Size of the firmware in bytes
 * @param fw_crc CRC32 checksum of the firmware for validation
 */
void update_config_for_slot0(uint32_t fw_size, uint32_t fw_crc);

/**
 * Loads new application from slot 1 if available. Copies firmware from slot 1 to application area.
 */
void load_new_app(void);

/**
 * Reads the bootloader configuration from flash memory
 * 
 * @return Pointer to bootloader_config_t structure
 */
const bootloader_config_t* read_boot_config(void);

#endif // BOOTLOADER_H
