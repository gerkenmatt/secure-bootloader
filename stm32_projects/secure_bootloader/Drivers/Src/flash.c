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

// A struct to hold the properties of a single flash sector
typedef struct 
{
    uint32_t address; // The starting address of the sector
    uint32_t size;    // The size of the sector in bytes
} flash_sector_info_t;

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

#define NUM_SECTORS (sizeof(sector_map) / sizeof(flash_sector_info_t))

static bool program_flash_word(uint32_t addr, uint32_t word); 
static uint32_t get_address_for_sector(uint8_t sector);
static uint32_t get_size_for_sectors(uint8_t start_sector, uint8_t end_sector);


flash_status_t program_flash(uint32_t addr, const uint32_t* data, uint32_t length_bytes)
{
    // The number of 32-bit words to program.
    uint32_t num_words = (length_bytes + 3) / 4;

    for (uint32_t i = 0; i < num_words; i++) 
    {
        uint32_t current_addr = addr + (i * 4);
        uint32_t word_to_write = data[i];

        // Program the word to flash
        flash_status_t write_status = program_flash_word(current_addr, word_to_write);

        // Check the return status immediately.
        if (write_status != FLASH_OK) 
        {
            return write_status;
        }

        // Verify the written data by reading it back.
        uint32_t verified_word = *(volatile uint32_t *)current_addr;
        if (verified_word != word_to_write) 
        {
            return FLASH_ERROR_VERIFY;
        }
    }

    return FLASH_OK;
}

/**
 * Programs a single 32-bit word to flash memory
 * 
 * @param addr The flash address to program (must be 4-byte aligned)
 * @param word The 32-bit word to write
 * @return true if the word was programmed successfully, false otherwise
 */
static bool program_flash_word(uint32_t addr, uint32_t word) 
{
    // Check if address is 4-byte aligned
    if (addr & 0x3) 
    {
        LOG_ERROR("Error: Address not 4-byte aligned\r\n");
        return FLASH_ERROR_ALIGNMENT;
    }

    // Wait for any ongoing flash operations to complete
    while (FLASH->SR & FLASH_SR_BSY);
    
    // Clear any previous error flags
    clear_flash_errors();

    // Configure flash programming size to 32-bit
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= (0x2 << FLASH_CR_PSIZE_Pos);
    
    // Enable flash programming mode
    FLASH->CR |= FLASH_CR_PG;

    // Write the word to flash
    *(__IO uint32_t*)addr = word;
    
    // Data/Instruction barriers to ensure write completes
    __DSB();
    __ISB();

    // Wait for programming to complete
    while (FLASH->SR & FLASH_SR_BSY);

    // Check for any programming errors
    if (FLASH->SR & (FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR | FLASH_SR_PGPERR | FLASH_SR_ERSERR)) 
    {
        LOG_ERROR("Error: Flash programming failed\r\n");
        FLASH->CR &= ~FLASH_CR_PG;
        // Report a generic write error
        return FLASH_ERROR;
    }

    // Disable programming mode
    FLASH->CR &= ~FLASH_CR_PG;
    return FLASH_OK;
}

flash_status_t erase_flash_sectors(uint8_t start_sector, uint8_t end_sector) 
{
    // These could be looked up from a table based on the start_sector
    uint32_t start_address = get_address_for_sector(start_sector);
    uint32_t size = get_size_for_sectors(start_sector, end_sector);

    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= (0x2 << FLASH_CR_PSIZE_Pos); // Set program size to 32-bit

    for (uint8_t sector = start_sector; sector <= end_sector; sector++) 
    {
        // Clear any previous error flags before starting
        clear_flash_errors();

        FLASH->CR &= ~FLASH_CR_PG;
        FLASH->CR &= ~FLASH_CR_SNB;
        FLASH->CR |= FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos);
        FLASH->CR |= FLASH_CR_STRT;

        while (FLASH->SR & FLASH_SR_BSY);

        // IMPORTANT: Check for hardware errors after the operation
        if (FLASH->SR & (FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR | FLASH_SR_PGPERR | FLASH_SR_ERSERR)) 
        {
            FLASH->CR &= ~FLASH_CR_SER; // Disable sector erase mode
            return FLASH_ERROR_ERASE; // Return specific hardware erase error
        }

        FLASH->CR &= ~FLASH_CR_SER;
    }

    // Invalidate the Data Cache to ensure we read fresh data from flash
    SCB_CleanDCache();
    SCB_InvalidateDCache();
    __DSB();
    __ISB();

    // Verify sectors are fully erased (all 0xFF)
    for (uint32_t i = 0; i < size / 4; i++) 
    {
        if (((volatile uint32_t*)start_address)[i] != 0xFFFFFFFF) 
            return FLASH_ERROR_ERASE_VERIFY;
    }

    return FLASH_OK;
}

void unlock_flash(void) 
{
    if (FLASH->CR & FLASH_CR_LOCK) 
    {
        FLASH->KEYR = 0x45670123;
        FLASH->KEYR = 0xCDEF89AB;
    }
}

void lock_flash(void) 
{
    FLASH->CR |= FLASH_CR_LOCK; 
}

void flash_prepare_for_write(void) 
{
    unlock_flash();
    clear_flash_errors();

    // Set the programming size to 32 bits
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= (0x2 << FLASH_CR_PSIZE_Pos); // 32-bit programming size
    FLASH->ACR |= (1 << 8) | (1 << 9); 
}

// --- Static Helper Functions ---

/**
 * @brief Gets the starting memory address for a given flash sector.
 * @param sector The sector number (0-7 for the first 1MB).
 * @return The 32-bit starting address of the sector. Returns 0 for an invalid sector.
 */
static uint32_t get_address_for_sector(uint8_t sector)
{
    // Bounds check the input against the size of our map
    if (sector >= NUM_SECTORS) 
        return 0; 
    
    return sector_map[sector].address;
}


/**
 * @brief Calculates the TOTAL size of a RANGE of sectors.
 */
static uint32_t get_size_for_sectors(uint8_t start_sector, uint8_t end_sector)
{
    uint32_t total_size = 0;
    if (end_sector < start_sector) 
        return 0;

    // Loop from the start to the end sector (inclusive)
    for (uint8_t i = start_sector; i <= end_sector; i++) 
    {
        // Add the size of each individual sector to the total
        total_size +=  sector_map[i].size;
    }
    return total_size;
}

// /**
//  * Gets the sector number from a given flash address
//  * 
//  * @param addr The flash address to get the sector number for
//  * @return The sector number of the given address
//  */
// static uint8_t get_sector_from_addr(uint32_t addr) 
// {
//     if (addr < 0x08000000 || addr >= 0x08200000)
//         return -1;  // Invalid address

//     if (addr < 0x08008000) return 0;
//     if (addr < 0x08010000) return 1;
//     if (addr < 0x08018000) return 2;
//     if (addr < 0x08020000) return 3;
//     if (addr < 0x08040000) return 4;
//     if (addr < 0x08080000) return 5;
//     if (addr < 0x080C0000) return 6;
//     if (addr < 0x08100000) return 7;
//     if (addr < 0x08140000) return 8;
//     if (addr < 0x08180000) return 9;
//     if (addr < 0x081C0000) return 10;
//     if (addr < 0x08200000) return 11;

//     return -1;  // Outside known flash
// }

void clear_flash_errors(void) 
{
    FLASH->SR |= FLASH_SR_PGPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_OPERR | FLASH_SR_ERSERR;
}