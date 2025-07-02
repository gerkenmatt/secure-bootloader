#include "flash.h"
#include "utilities.h"  
#include "stm32f767xx.h"
#include "ota.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "logger.h"


// -----------------------------------------------------------------------------
// Module-Private Constants
// -----------------------------------------------------------------------------

// Calculate the number of defined flash sectors from the `sector_map` array.
#define NUM_SECTORS (sizeof(sector_map) / sizeof(flash_sector_info_t))

// Mask for the PSIZE bits in the FLASH_CR register, used to set programming parallelism.
#define CR_PSIZE_MASK (~(FLASH_CR_PSIZE))

// -----------------------------------------------------------------------------
// Type Definitions
// -----------------------------------------------------------------------------

/**
 * @brief Flash memory sector information structure.
 */
typedef struct 
{
    uint32_t address; // The starting address of the sector
    uint32_t size;    // The size of the sector in bytes
} flash_sector_info_t;


// -----------------------------------------------------------------------------
// Module-Private Data (Static Globals)
// -----------------------------------------------------------------------------

// Defines the memory layout of the STM32F767ZI flash sectors.
static const flash_sector_info_t sector_map[] = {
    // Sector, Address,     Size
    /* 0 */ { 0x08000000, (32 * 1024)   }, // 32 KB
    /* 1 */ { 0x08008000, (32 * 1024)   }, // 32 KB
    /* 2 */ { 0x08010000, (32 * 1024)   }, // 32 KB
    /* 3 */ { 0x08018000, (32 * 1024)   }, // 32 KB
    /* 4 */ { 0x08020000, (128 * 1024)  }, // 128 KB
    /* 5 */ { 0x08040000, (256 * 1024)  }, // 256 KB
    /* 6 */ { 0x08080000, (256 * 1024)  }, // 256 KB
    /* 7 */ { 0x080C0000, (256 * 1024)  }, // 256 KB
    /* 8 */ { 0x08100000, (256 * 1024)  }, // 256 KB
    /* 9 */ { 0x08140000, (256 * 1024)  }, // 256 KB
    /* 10 */ { 0x08180000, (256 * 1024)  }, // 256 KB
    /* 11 */ { 0x081C0000, (256 * 1024)  }, // 256 KB
};

// -----------------------------------------------------------------------------
// Static Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Programs a single 32-bit word to flash memory.
 * 
 * @param addr The flash address to program (must be 4-byte aligned)
 * @param word The 32-bit word to write
 * @return true if the word was programmed successfully, false otherwise
 */
static bool program_flash_word(uint32_t addr, uint32_t word); 

/**
 * @brief Gets the starting address for a given flash sector.
 * @param sector The sector number (0-11)
 * @return The starting address of the sector in flash memory
 */
static uint32_t get_address_for_sector(uint8_t sector);

/**
 * @brief Gets the size in bytes for a range of flash sectors.
 * @param start_sector The first sector number (0-11)
 * @param end_sector The last sector number (0-11)
 * @return The total size in bytes for the specified sectors
 */
static uint32_t get_size_for_sectors(uint8_t start_sector, uint8_t end_sector);


// -----------------------------------------------------------------------------
// Public Function Implementations
// -----------------------------------------------------------------------------

flash_status_t program_flash(uint32_t addr, const uint32_t* p_data, uint32_t length_bytes)
{
    // Calculate the number of 32-bit words to program.
    // Adds 3 and integer divides by 4 to correctly handle lengths not perfectly divisible by 4.
    uint32_t num_words = (length_bytes + 3) / 4;

    // Loop through each 32-bit word in the input data.
    for (uint32_t i = 0; i < num_words; i++)
    {
        uint32_t current_addr = addr + (i * 4);    // Calculate the flash address for the current word.
        uint32_t word_to_write = p_data[i];        // Get the 32-bit word to write.

        // Program the current word to flash.
        flash_status_t write_status = program_flash_word(current_addr, word_to_write);

        // Check the return status immediately. If any error occurs during programming,
        // stop and return that error.
        if (write_status != FLASH_OK)
        {
            return write_status;
        }

        // Verify the written data by reading it back from the flash memory.
        // This ensures the data was programmed correctly.
        uint32_t verified_word = *(volatile uint32_t *)current_addr;
        if (verified_word != word_to_write)
        {
            // If the read-back data does not match, return a verification error.
            return FLASH_ERROR_VERIFY;
        }
    }

    // If all words were programmed and verified successfully, return OK.
    return FLASH_OK;
}


flash_status_t erase_flash_sectors(uint8_t start_sector, uint8_t end_sector)
{
    // Get the actual physical start address and total size for the given sector range.
    uint32_t start_address = get_address_for_sector(start_sector);
    uint32_t size = get_size_for_sectors(start_sector, end_sector);

    // Set the flash program/erase parallelism to 32-bit (word access).
    // This setting dictates how many bits are written/erased in a single operation.
    FLASH->CR &= CR_PSIZE_MASK; // Clear current PSIZE bits.
    FLASH->CR |= (0x2 << FLASH_CR_PSIZE_Pos); // Set PSIZE to 32-bit.

    // Iterate through each sector to be erased.
    for (uint8_t sector = start_sector; sector <= end_sector; sector++)
    {
        // Clear any pending error flags in the Flash Status Register (FLASH->SR)
        // before starting a new erase operation. This prevents old errors from
        // affecting the current operation's status check.
        clear_flash_errors();

        // Clear the Programming (PG) bit and Sector Number (SNB) bits, then
        // set the Sector Erase (SER) bit and specify the current sector to erase.
        FLASH->CR &= ~FLASH_CR_PG; // Ensure programming mode is off.
        FLASH->CR &= ~FLASH_CR_SNB; // Clear previous sector number selection.
        FLASH->CR |= FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos); // Enable sector erase and select sector.
        FLASH->CR |= FLASH_CR_STRT; // Start the erase operation.

        // Wait for the erase operation to complete. The Busy (BSY) flag clears when done.
        while (FLASH->SR & FLASH_SR_BSY);

        // IMPORTANT: Check for hardware errors after the erase operation.
        // These flags indicate various types of programming/erase failures.
        if (FLASH->SR & (FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR | FLASH_SR_PGPERR | FLASH_SR_ERSERR))
        {
            FLASH->CR &= ~FLASH_CR_SER; // Disable sector erase mode.
            return FLASH_ERROR_ERASE;   // Return a generic erase error.
        }

        // Disable the Sector Erase (SER) bit after successful erase of the current sector.
        FLASH->CR &= ~FLASH_CR_SER;
    }

    // Invalidate the Data Cache to ensure that subsequent reads from flash
    // retrieve the newly erased (0xFFFFFFFF) data directly from flash,
    // not from a stale cache entry.
    SCB_CleanDCache();
    SCB_InvalidateDCache();
    __DSB(); // Data Synchronization Barrier: Ensures all outstanding memory accesses complete.
    __ISB(); // Instruction Synchronization Barrier: Flushes the pipeline, ensuring new instructions fetch from updated memory.

    // Verify that the erased sectors are indeed filled with 0xFFFFFFFF.
    // This is a post-erase integrity check.
    for (uint32_t i = 0; i < size / 4; i++)
    {
        if (((volatile uint32_t*)start_address)[i] != 0xFFFFFFFF)
            return FLASH_ERROR_ERASE_VERIFY; // Return an error if verification fails.
    }

    return FLASH_OK; // Return OK if all sectors were successfully erased and verified.
}

void unlock_flash(void)
{
    // Check if the Flash control register is currently locked.
    // If it is, sequence the unlock key values to the FLASH_KEYR register.
    // These specific key values are required by the STM32F7 for flash unlock.
    if (FLASH->CR & FLASH_CR_LOCK)
    {
        FLASH->KEYR = 0x45670123; // First key.
        FLASH->KEYR = 0xCDEF89AB; // Second key.
    }
}

void lock_flash(void)
{
    // Set the LOCK bit in the Flash Control Register (FLASH_CR).
    // This prevents accidental or unauthorized flash operations until explicitly unlocked.
    FLASH->CR |= FLASH_CR_LOCK;
}

void flash_prepare_for_write(void)
{
    // Unlock the flash controller to enable write and erase operations.
    unlock_flash();
    // Clear any pending error flags from previous flash operations.
    // This ensures a clean slate before starting new operations.
    clear_flash_errors();

    // Configure the programming parallelism for flash writes to 32-bit (word access).
    // This typically optimizes write speed for 32-bit microcontrollers.
    FLASH->CR &= CR_PSIZE_MASK; // Clear current PSIZE setting.
    FLASH->CR |= (0x2 << FLASH_CR_PSIZE_Pos); // Set PSIZE to 32-bit.
    // Enable Instruction and Data Caches (if not already enabled by system_init).
    // These bits are set to ensure efficient data transfer to flash.
    FLASH->ACR |= (1 << 8) | (1 << 9); // ICEN (Instruction Cache Enable) and DCEN (Data Cache Enable).
}

void clear_flash_errors(void)
{
    // Clear all pending error flags in the Flash Status Register (FLASH->SR) by writing 1 to them.
    // This is a necessary step before initiating new flash operations to accurately check for new errors.
    FLASH->SR |= FLASH_SR_PGPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_OPERR | FLASH_SR_ERSERR;
}

// -----------------------------------------------------------------------------
// Static Function Implementations
// -----------------------------------------------------------------------------

static bool program_flash_word(uint32_t addr, uint32_t word)
{
    // Ensure the target address for programming is 4-byte aligned.
    // Flash programming on STM32F7 usually requires word (32-bit) alignment.
    if (addr & 0x3) // Check if the last two bits are non-zero.
    {
        LOG_ERROR("Error: Flash address 0x%08X not 4-byte aligned for programming.\r\n", (unsigned int)addr);
        return FLASH_ERROR_ALIGNMENT;
    }

    // Wait until any ongoing flash operations (erase or program) are complete.
    // The BSY (Busy) flag indicates if the flash controller is active.
    while (FLASH->SR & FLASH_SR_BSY);

    // Clear any previously set error flags in the Flash Status Register.
    clear_flash_errors();

    // Set the programming size to 32-bit (word programming).
    // This configuration must match the size of data being written.
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= (0x2 << FLASH_CR_PSIZE_Pos);

    // Enable the Programming (PG) bit in the Flash Control Register.
    // This puts the flash controller into programming mode.
    FLASH->CR |= FLASH_CR_PG;

    // Write the 32-bit word to the specified flash address.
    // The `__IO` (volatile) cast ensures the compiler performs a direct memory write.
    *(__IO uint32_t*)addr = word;

    // Data Synchronization Barrier (DSB) and Instruction Synchronization Barrier (ISB)
    // ensure that the write operation completes and the CPU's instruction pipeline
    // is flushed, guaranteeing visibility of the new flash content.
    __DSB();
    __ISB();

    // Wait again for the programming operation to complete.
    while (FLASH->SR & FLASH_SR_BSY);

    // After the operation, check for any hardware programming errors.
    // These flags indicate issues like write protection errors, alignment errors, etc.
    if (FLASH->SR & (FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR | FLASH_SR_PGPERR | FLASH_SR_ERSERR))
    {
        LOG_ERROR("Error: Flash programming failed at address 0x%08X. SR: 0x%08X\r\n",
                  (unsigned int)addr, (unsigned int)FLASH->SR);
        // Disable programming mode after an error.
        FLASH->CR &= ~FLASH_CR_PG;
        // Return a generic FLASH_ERROR status.
        return FLASH_ERROR;
    }

    // Disable programming mode after successful operation.
    FLASH->CR &= ~FLASH_CR_PG;
    return FLASH_OK; // Return OK if the word was programmed successfully.
}

static uint32_t get_address_for_sector(uint8_t sector)
{
    // Perform a bounds check to ensure the requested sector number is valid.
    if (sector >= NUM_SECTORS)
    {
        LOG_ERROR("Error: Invalid flash sector number requested: %u\r\n", (unsigned int)sector);
        return 0; // Return 0 for an invalid sector, which is not a valid flash address.
    }

    // Return the starting address of the specified sector from the pre-defined map.
    return sector_map[sector].address;
}

static uint32_t get_size_for_sectors(uint8_t start_sector, uint8_t end_sector)
{
    uint32_t total_size = 0;
    // Basic validation: ensure end sector is not before start sector.
    if (end_sector < start_sector)
    {
        LOG_ERROR("Error: End sector (%u) is less than start sector (%u).\r\n", (unsigned int)end_sector, (unsigned int)start_sector);
        return 0;
    }
    // Also check if sectors are within bounds of the defined map.
    if (start_sector >= NUM_SECTORS || end_sector >= NUM_SECTORS) {
        LOG_ERROR("Error: Sector range [%u, %u] out of bounds (max %u).\r\n", (unsigned int)start_sector, (unsigned int)end_sector, (unsigned int)(NUM_SECTORS - 1));
        return 0;
    }


    // Iterate from the start sector to the end sector (inclusive) to sum their sizes.
    for (uint8_t i = start_sector; i <= end_sector; i++)
    {
        total_size += sector_map[i].size; // Add the size of each individual sector.
    }
    return total_size;
}