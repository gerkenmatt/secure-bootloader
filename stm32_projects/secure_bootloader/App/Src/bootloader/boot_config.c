#include "boot_config.h"
#include "bootloader.h"
#include "flash.h"
#include "logger.h"
#include <string.h> 


// -----------------------------------------------------------------------------
// Public Function Implementations
// -----------------------------------------------------------------------------


void init_bootloader_config(void) 
{
    bootloader_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.magic = BOOT_CONFIG_MAGIC;
    cfg.active_slot = SLOTA; // Default to booting from Slot A

    // Configure Slot A (assumed to hold a valid factory image)
    cfg.slot[0].is_valid = 1; 
    cfg.slot[0].boot_attempts_remaining = BOOT_ATTEMPT_COUNT; 
    cfg.slot[0].fw_size = 0; // Should be updated after initial flashing
    cfg.slot[0].fw_crc = 0xFFFFFFFF; // Should be updated after initial flashing

    // Configure Slot B as initially empty/invalid
    cfg.slot[1].is_valid = 0;
    cfg.slot[1].boot_attempts_remaining = 0;
    cfg.slot[1].fw_size = 0;
    cfg.slot[1].fw_crc = 0xFFFFFFFF;

    if (!write_boot_config(&cfg))
    {
        LOG_ERROR("Failed to write initial bootloader configuration!\r\n");
        bootloader_set_state(BL_STATE_ERROR);
    }
    else
    {
        LOG_INFO("Bootloader configuration initialized successfully.\r\n");
    }
}

bool write_boot_config(const bootloader_config_t* p_new_config) 
{
    if (!p_new_config) return false;

    flash_prepare_for_write();

    // Erase the config sector
    if (erase_flash_sectors(CONFIG_SECTOR, CONFIG_SECTOR) != FLASH_OK) 
    
    {
        LOG_ERROR("Failed to erase config sector!\r\n");
        lock_flash();
        return false;
    }

    // Write the new config struct to flash
    if (program_flash(CONFIG_ADDR, (const uint32_t*)p_new_config, sizeof(bootloader_config_t)) != FLASH_OK) 
    {
        LOG_ERROR("Failed to write new config to flash!\r\n");
        lock_flash();
        return false;
    }

    lock_flash();
    return true;

}

const bootloader_config_t* read_boot_config(void) 
{
    return (const bootloader_config_t*)CONFIG_ADDR;
}

