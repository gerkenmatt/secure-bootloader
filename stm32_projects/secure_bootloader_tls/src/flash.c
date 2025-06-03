#include "flash.h"
#include "utilities.h"  
#include "stm32f7xx.h"
#include "ota.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

void program_flash_word(uint32_t addr, uint32_t word) {
    // Check if address is 4-byte aligned
    if (addr & 0x3) {
        ota_send_response(RESP_NACK);
        log("Error: Address not 4-byte aligned\r\n");
        return;
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
    if (FLASH->SR & (FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR | FLASH_SR_PGPERR | FLASH_SR_ERSERR)) {
        ota_send_response(RESP_NACK);
        log("Error: Flash programming failed\r\n");
        FLASH->CR &= ~FLASH_CR_PG;
        return;
    }

    // Disable programming mode
    FLASH->CR &= ~FLASH_CR_PG;
}


bool erase_flash_sectors(uint8_t start_sector, uint8_t end_sector, uint32_t start_address, uint32_t size) { 
    
    for (uint8_t sector = start_sector; sector <= end_sector; sector++) {
        FLASH->CR &= ~FLASH_CR_PG;  // Disable programming mode
        FLASH->CR &= ~FLASH_CR_SNB; // Clear sector number
        FLASH->CR |= FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos); // Set sector erase and number
        FLASH->CR |= FLASH_CR_STRT; // Start erase
        while (FLASH->SR & FLASH_SR_BSY); // Wait for completion
        FLASH->CR &= ~FLASH_CR_SER; // Clear sector erase flag
    }

    // Verify sectors are fully erased (all 0xFF)
    volatile uint32_t* check = (uint32_t*)start_address;
    for (int i = 0; i < size/4; i++) {
        if (check[i] != 0xFFFFFFFF) {
            ota_send_response(RESP_NACK);
            log("Flash erase check failed at address 0x");
            print_uint32_hex((uint32_t)&check[i]);
            log("\r\n");
            return false;
        }
    }
    return true;
}

bool copy_firmware(uint32_t src_addr, uint32_t dst_addr, uint32_t size) {
    unlock_flash();
    clear_flash_errors();

    log("Erasing destination sectors before programming\r\n");
    // Erase destination sectors before programming
    if (!erase_flash_sectors(get_sector_from_addr(dst_addr), get_sector_from_addr(dst_addr), dst_addr, size)) {
        return false;
    }

    // Set up pointers for copying
    const uint32_t* src = (const uint32_t*)src_addr;
    uint32_t* dst = (uint32_t*)dst_addr;
    size_t words = size / 4;

    // Copy word by word
    for (size_t i = 0; i < words; i++) {
        program_flash_word((uint32_t)&dst[i], src[i]);
        // Verify written data matches source
        if (dst[i] != src[i]) return false;
    }

    return true;
}

void unlock_flash(void) {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123;
        FLASH->KEYR = 0xCDEF89AB;
    }
}

uint8_t get_sector_from_addr(uint32_t addr) {
    if (addr < 0x08000000 || addr >= 0x08200000)
        return -1;  // Invalid address

    if (addr < 0x08008000) return 0;
    if (addr < 0x08010000) return 1;
    if (addr < 0x08018000) return 2;
    if (addr < 0x08020000) return 3;
    if (addr < 0x08040000) return 4;
    if (addr < 0x08080000) return 5;
    if (addr < 0x080C0000) return 6;
    if (addr < 0x08100000) return 7;
    if (addr < 0x08140000) return 8;
    if (addr < 0x08180000) return 9;
    if (addr < 0x081C0000) return 10;
    if (addr < 0x08200000) return 11;

    return -1;  // Outside known flash
}

void clear_flash_errors(void) {
    FLASH->SR |= FLASH_SR_PGPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR;
}