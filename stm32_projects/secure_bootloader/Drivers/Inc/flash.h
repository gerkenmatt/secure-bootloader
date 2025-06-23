#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// Type Definitions
// -----------------------------------------------------------------------------

typedef enum 
{
    FLASH_OK = 0,
    FLASH_ERROR,
    FLASH_ERROR_ALIGNMENT,
    FLASH_ERROR_WRITE_PROTECTED,
    FLASH_ERROR_BUSY,
    FLASH_ERROR_VERIFY,
    FLASH_ERROR_ERASE,          
    FLASH_ERROR_ERASE_VERIFY
} flash_status_t;

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------

flash_status_t program_flash(uint32_t addr, const uint32_t* data, uint32_t length);



/**
 * Erases a range of flash sectors
 * 
 * @param start_sector The first sector to erase
 * @param end_sector The last sector to erase
 * @param start_address The starting address of the range to erase
 * @param size The size of the range to erase
 * @return 
 */
flash_status_t erase_flash_sectors(uint8_t start_sector, uint8_t end_sector);

/**
 * Unlocks the flash memory
 */
void unlock_flash(void);

/**
 * Locks the flash memory after programming
 */
void lock_flash(void);


/**
 * Clears any previous flash errors
 */
void clear_flash_errors(void);

void flash_prepare_for_write(void);

#endif // FLASH_H