/**
 * @file bootloader.h
 * @brief Bootloader core logic and configuration interface.
 */
#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include <stdint.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// Constants and Macros
// -----------------------------------------------------------------------------

#define FLASH_BASE_ADDR          0x08000000                 ///< Main flash memory (AXI bus)
#define ITCM_FLASH_ADDR          0x00200000                 ///< Aliased flash memory (ITCM bus)
#define FLASH_END_ADDR           0x08200000                 ///< 2MB total flash size
#define CR_PSIZE_MASK            ((uint32_t)0xFFFFFCFFU)    ///< Mask for flash programming size bits

#define RAM_START                0x20000000
#define RAM_SIZE                 (512 * 1024)  
#define RAM_END                  (RAM_START + RAM_SIZE)

#define BOOTLOADER_START_PHYS     FLASH_BASE_ADDR
#define BOOTLOADER_START_ALIAS    ITCM_FLASH_ADDR
#define CONFIG_SECTOR             11                   
#define CONFIG_ADDR               0x081C0000  

// --- Application Slot Layout ---

#define SLOTA_ADDR                0x08040000  ///< Slot A starts at Sector 5
#define SLOTA_SECTOR              5
#define SLOTB_ADDR                0x08100000  ///< Slot B starts at Sector 8
#define SLOTB_SECTOR              8

#define SLOT_SIZE                 0x000C0000  ///< 768KB per slot
#define SLOT_SECTOR_COUNT         3           ///< Each slot occupies 3 x 256KB sectors

#define NUM_SLOTS                 2           ///< Total number of application slots

// -----------------------------------------------------------------------------
// Enumerations
// -----------------------------------------------------------------------------

/**
 * @brief Bootloader slot indices.
 * Used to identify which application slot is being referenced.
 */
typedef enum {
    SLOTA = 0,
    SLOTB = 1
} slot_index_t;

/**
 * @brief Bootloader states.
 */
typedef enum {
    BL_STATE_IDLE = 0,
    BL_STATE_READY,
    BL_STATE_RECEIVING,
    BL_STATE_VERIFY,
    BL_STATE_ERROR
} bootloader_state_t;

/**
 * @brief Bootloader status codes.
 */
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
// Public Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Initializes the bootloader.
 */
void bootloader_init(void);

/**
 * @brief Runs one iteration of the bootloader's main state machine.
 */
void bootloader_run_state_machine(void); 

/**
 * @brief Sets bootloader state to new value
 * @param new_state New state to set
 */
void bootloader_set_state(bootloader_state_t new_state);

/**
 * @brief Gets the current bootloader state.
 * @return The current bootloader_state_t value.
 */
bootloader_state_t bootloader_get_state(void);


/**
 * @brief Jumps to the active application based on bootloader configuration
 */
void bootloader_jump_to_active_application(void);

/**
 * @brief Validates the boot environment configuration.
 */
void validate_boot_environment(void);

#endif // BOOTLOADER_H
