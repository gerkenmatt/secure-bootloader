#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include <stdint.h>

// Bootloader states
typedef enum {
    BL_STATE_IDLE = 0,
    BL_STATE_READY,
    BL_STATE_RECEIVING,
    BL_STATE_PROGRAMMING,
    BL_STATE_VERIFY,
    BL_STATE_ERROR
} bootloader_state_t;

// Error codes
typedef enum {
    BL_OK = 0,
    BL_ERROR_INVALID_ADDRESS = -1,
    BL_ERROR_FLASH_ERASE = -2,
    BL_ERROR_FLASH_WRITE = -3,
    BL_ERROR_VERIFICATION = -4,
    BL_ERROR_INVALID_STATE = -5,
    BL_ERROR_INVALID_BOOT_MODE = -6
} bootloader_status_t;

// Memory map definitions for STM32F767ZI
// Physical addresses
#define FLASH_BASE_ADDR          0x08000000  // Main flash memory (AXI bus)
#define ITCM_FLASH_ADDR          0x00200000  // Aliased flash memory (ITCM bus)
#define SYSTEM_BOOTLOADER_ADDR   0x00100000  // System bootloader (ROM)
#define RAM_START 0x20000000
#define RAM_SIZE  (512 * 1024)  // 512KB
#define RAM_END   (RAM_START + RAM_SIZE)

// Bootloader configuration
#define BOOTLOADER_SIZE          (256 * 1024)    // 256KB
#define BOOTLOADER_START_PHYS    FLASH_BASE_ADDR
#define BOOTLOADER_START_ALIAS   ITCM_FLASH_ADDR
#define APPLICATION_START_ADDR   0x08040000     // where the application is loaded
#define FLASH_END_ADDR           0x08200000     // 2MB total flash

// Boot configuration
#define BOOT_CONFIG_MAGIC       0xB007CF60      // TODO: add security
#define BOOT_FLAG_STAY_IN_BL    0x01
#define BOOT_FLAG_UPDATE_APP    0x02

// Bootloader configuration structure (stored in last page of bootloader section)
typedef struct {
    uint32_t magic;           // Magic number to identify valid config
    uint32_t boot_flags;      // Boot configuration flags
    uint32_t app_version;     // Current application version
    uint32_t app_size;        // Size of application
    uint32_t app_crc;         // CRC of application
} bootloader_config_t;

// Function prototypes
void bootloader_init(void);
bootloader_status_t bootloader_handle_command(void);
bootloader_state_t bootloader_get_state(void);
void bootloader_jump_to_application(uint32_t app_address);
bootloader_status_t bootloader_verify_memory_aliasing(void);
uint32_t bootloader_get_boot_mode(void);
const bootloader_config_t* bootloader_get_config(void);
void validate_boot_environment(void);

#endif // BOOTLOADER_H
