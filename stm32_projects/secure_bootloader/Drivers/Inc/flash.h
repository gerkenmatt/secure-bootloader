/**
 * @file flash.h
 * @brief Flash memory programming and erase interface for STM32 bootloader.
 */
#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// Type Definitions
// -----------------------------------------------------------------------------

/**
 * @brief Flash operation status codes.
 */
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

/**
 * @brief Programs a region of flash memory.
 * @param addr Start address to program
 * @param data Pointer to data to write
 * @param length Number of bytes to write
 * @return Status code
 */
flash_status_t program_flash(uint32_t addr, const uint32_t* data, uint32_t length);

/**
 * @brief Erases a range of flash sectors.
 * @param start_sector The first sector to erase
 * @param end_sector The last sector to erase
 * @return Status code
 */
flash_status_t erase_flash_sectors(uint8_t start_sector, uint8_t end_sector);

/**
 * @brief Unlocks the flash memory for programming.
 */
void unlock_flash(void);

/**
 * @brief Locks the flash memory after programming.
 */
void lock_flash(void);

/**
 * @brief Clears any flash error flags.
 */
void clear_flash_errors(void);

/**
 * @brief Prepares the flash memory for writing.
 * 
 * This function unlocks the flash, clears any error flags,
 * sets the programming size to 32 bits
 */
void flash_prepare_for_write(void);

#endif // FLASH_H