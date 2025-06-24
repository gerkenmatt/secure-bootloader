/**
 * @file bootloader.c
 * @brief Implementation of the bootloader core logic and state management.
 */
#include "bootloader.h"

// Other project-specific includes.
#include "boot_config.h"
#include "cli.h"
#include "flash.h"
#include "logger.h"
#include "ota.h"
#include "utilities.h"

#include "stm32f767xx.h"
#include "mbedtls/platform.h"

#include <string.h>

// -----------------------------------------------------------------------------
// Module-Private Data (Static Globals)
// -----------------------------------------------------------------------------

static bootloader_state_t current_state = BL_STATE_IDLE;

// -----------------------------------------------------------------------------
// Static Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Verifies memory aliasing between AXI and ITCM regions.
 * @return BL_OK if aliasing verified, BL_ERROR_INVALID_BOOT_MODE if mismatch.
 */
static bootloader_status_t bootloader_verify_memory_aliasing(void);

/**
 * @brief Jumps to the specified application address.   
 * @param app_address The address of the application to jump to.
 */
static void bootloader_jump_to(uint32_t app_address);

// -----------------------------------------------------------------------------
// Public Function Implementations
// -----------------------------------------------------------------------------

void bootloader_init(void)
{
    // Initialize configuration if not already set
    const bootloader_config_t* config = read_boot_config();
    if (config->magic != BOOT_CONFIG_MAGIC) 
    {
        // Initialize config defaults and write to flash
        LOG_INFO("***Initializing bootloader configuration***");
        init_bootloader_config();
    }

    // Needed for mbedTLS memory allocation
    mbedtls_platform_set_calloc_free(calloc, free);

    // Set state to READY
    current_state = BL_STATE_READY;
}

bootloader_state_t bootloader_get_state(void) 
{
    return current_state;
}

void bootloader_set_state(bootloader_state_t new_state) 
{
    if (current_state == new_state) 
        return; // No change
    
    if (new_state == BL_STATE_RECEIVING) 
    {
        LOG_INFO("Entering OTA mode. Initializing OTA module...");
        ota_init();
    }
    current_state = new_state;
}

void bootloader_run_state_machine(void)
{
    switch (current_state)
    {
        case BL_STATE_READY:
            // In the READY state, handle input using the full command set.
            cli_process_input(current_state);
            break;

        case BL_STATE_RECEIVING:
            ota_process_non_blocking();
            break;

        case BL_STATE_VERIFY:
            LOG_INFO("Verifying received firmware...");
            if (ota_finalize_and_verify()) 
            {
                LOG_INFO("Verification successful. Rebooting to new firmware.");
                NVIC_SystemReset();
            } 
            else 
            {
                LOG_ERROR("Verification FAILED. Entering error state.");
                bootloader_set_state(BL_STATE_ERROR);
            }
            break;

        case BL_STATE_ERROR:
        {
            // In the ERROR state, handle input using the limited recovery command set.
            static bool error_indicated = false;
            if (!error_indicated) 
            {
                GPIOB->ODR |= (1UL << 14); // Set red LED
                LOG_ERROR("\r\n!!! An error occurred. Entering recovery mode. Type 'help'. !!!");
                error_indicated = true;
            }
            cli_process_input(current_state);
            break;
        }

        default:
            break;
    }
}
void bootloader_jump_to_active_application(void)
{
    const bootloader_config_t* p_cfg = read_boot_config();
    bootloader_config_t new_cfg;
    memcpy(&new_cfg, p_cfg, sizeof(bootloader_config_t)); // Work with a mutable copy

    bool config_changed = false;
    slot_index_t current_slot = new_cfg.active_slot;

    // 1. --- DECIDE ---
    // Perform all logic checks first before writing to flash.
    if (current_slot >= NUM_SLOTS || !new_cfg.slot[current_slot].is_valid)
    {
        LOG_ERROR("Active slot (%d) is invalid. Halting.", current_slot);
        bootloader_set_state(BL_STATE_ERROR);
        return;
    }

    // Check for rollback condition
    if (new_cfg.slot[current_slot].boot_attempts_remaining == 0)
    {
        LOG_ERROR("!!! No boot attempts left for slot %d. Attempting rollback. !!!", current_slot);
        slot_index_t fallback_slot = (current_slot == SLOTA) ? SLOTB : SLOTA;

        if (new_cfg.slot[fallback_slot].is_valid)
        {
            new_cfg.active_slot = fallback_slot;
            new_cfg.slot[fallback_slot].boot_attempts_remaining = BOOT_ATTEMPT_COUNT; // Restore attempts
            config_changed = true;
            
            // Write the single change and reboot to try the new config cleanly.
            LOG_INFO("Rolling back to slot %d. Writing new config and rebooting.", fallback_slot);
            write_boot_config(&new_cfg);
            NVIC_SystemReset();
            return; // Should not be reached
        }
        else
        {
            LOG_ERROR("!!! Fallback slot %d is not valid. Cannot roll back. Halting. !!!", fallback_slot);
            bootloader_set_state(BL_STATE_ERROR);
            return;
        }
    }

    // 2. --- PREPARE FOR JUMP ---
    // If we're here, we are committed to booting the current slot. Decrement the counter.
    new_cfg.slot[current_slot].boot_attempts_remaining--;
    config_changed = true;

    // 3. --- WRITE ONCE ---
    // Write the updated config (with decremented counter) to flash.
    if (config_changed)
    {
        if (!write_boot_config(&new_cfg))
        {
            LOG_ERROR("Failed to write updated boot config. Halting.");
            bootloader_set_state(BL_STATE_ERROR);
            return;
        }
    }

    // 4. --- EXECUTE ---
    // Verify CRC and jump.
    uint32_t jump_address = (current_slot == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;
    const slot_metadata_t* active_slot_meta = &p_cfg->slot[current_slot];

    if (!verify_crc(jump_address, active_slot_meta->fw_size, active_slot_meta->fw_crc))
    {
        LOG_ERROR("CRC check failed for active slot. Rebooting.");
        // The boot attempt was already consumed. The next boot will either try again or roll back.
        NVIC_SystemReset();
        return; // Should not be reached
    }

    LOG_INFO("Boot counter decremented. CRC OK. Jumping to application in slot %d.", current_slot);
    bootloader_jump_to(jump_address);
}

void validate_boot_environment(void)
{
    LOG_INFO("Validating boot environment...");

    // Make sure the vector table is remapped to the correct ITCM alias
    if ((SCB->VTOR & 0xFFF00000) != BOOTLOADER_START_ALIAS)
    {
        LOG_ERROR("\tError: Unexpected VTOR address.");
        while (1) { for (volatile int i = 0; i < 50000; i++); }
    }
    LOG_INFO("\tVTOR configuration OK");

    // Make sure aliasing between ITCM and AXI flash matches
    if (bootloader_verify_memory_aliasing() != BL_OK)
    {
        LOG_ERROR("\tError: Memory aliasing failed.");
        current_state = BL_STATE_ERROR;
        return;
    }
    LOG_INFO("\tMemory aliasing verified");
}


// -----------------------------------------------------------------------------
// Static Function Implementations
// -----------------------------------------------------------------------------

static bootloader_status_t bootloader_verify_memory_aliasing(void)
{
    uint32_t *flash = (uint32_t *)BOOTLOADER_START_PHYS;
    uint32_t *itcm  = (uint32_t *)BOOTLOADER_START_ALIAS;

    for (int i = 0; i < 256; i++)
    {
        if (flash[i] != itcm[i])
            return BL_ERROR_INVALID_BOOT_MODE;
    }
    return BL_OK;
}

static void bootloader_jump_to(uint32_t app_address) 
{
    LOG_INFO("Jumping to application at: 0x%08X\r\n"); 

    // De-initialize peripherals and disable interrupts before jumping
    __disable_irq();
    // It's good practice to de-init peripherals used by bootloader, e.g., uart_deinit();
    
    // Reset system clocks to default state before handing over
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));
    RCC->CFGR = 0;
    
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // Set the application's vector table address
    SCB->VTOR = app_address;

    // Get the application's stack pointer and entry point
    uint32_t* vec_tab = (uint32_t*)app_address;
    uint32_t app_sp = vec_tab[0];
    uint32_t app_entry = vec_tab[1];

    // Set the main stack pointer
    __set_MSP(app_sp);

    // Create a function pointer to the application's reset handler and jump
    void (*app_reset_handler)(void) = (void*)app_entry;
    app_reset_handler();

    // This part should never be reached
    while(1);
}