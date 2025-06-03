#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------

/**
 * Programs a single 32-bit word to flash memory
 * 
 * @param addr The flash address to program (must be 4-byte aligned)
 * @param word The 32-bit word to write
 */
void program_flash_word(uint32_t addr, uint32_t word);

/**
 * Erases a range of flash sectors
 * 
 * @param start_sector The first sector to erase
 * @param end_sector The last sector to erase
 * @param start_address The starting address of the range to erase
 * @param size The size of the range to erase
 * @return true if the sectors were erased successfully, false otherwise
 */
bool erase_flash_sectors(uint8_t start_sector, uint8_t end_sector, uint32_t start_address, uint32_t size);

/**
 * Unlocks the flash memory
 */
void unlock_flash(void);

/**
 * Gets the sector number from a given flash address
 * 
 * @param addr The flash address to get the sector number for
 * @return The sector number of the given address
 */
uint8_t get_sector_from_addr(uint32_t addr);

/**
 * Clears any previous flash errors
 */
void clear_flash_errors(void);

/**
 * Copies firmware from one address to another
 * 
 * @param src_addr The source address to copy from
 * @param dst_addr The destination address to copy to
 * @param size The size of the range to copy
 * @return true if the copy was successful, false otherwise
 */
bool copy_firmware(uint32_t src_addr, uint32_t dst_addr, uint32_t size);

#endif // FLASH_H