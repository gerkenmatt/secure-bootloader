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
#include "led.h"

#include "stm32f767xx.h"
#include "mbedtls/platform.h"

#include <string.h>

// Magic value written to a backup register upon successful application boot.
#define BOOT_SUCCESS_MAGIC      0xDEADBEEF

// -----------------------------------------------------------------------------
// Module-Private Data (Static Globals)
// -----------------------------------------------------------------------------

// Tracks the current operational state of the bootloader.
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
    // Check if bootloader configuration exists and is valid.
    const bootloader_config_t* config = read_boot_config();
    if (config->magic != BOOT_CONFIG_MAGIC)
    {
        // Initialize default configuration if no valid config is found (first boot or corrupted).
        LOG_INFO("***Initializing bootloader configuration***");
        init_bootloader_config();
    }

    // Configure mbedTLS platform memory allocation functions.
    mbedtls_platform_set_calloc_free(calloc, free);

    // Transition to the READY state, allowing CLI interaction.
    current_state = BL_STATE_READY;
}

bootloader_state_t bootloader_get_state(void)
{
    return current_state;
}

void bootloader_set_state(bootloader_state_t new_state)
{
    if (current_state == new_state)
        return; // No state change needed.

    if (new_state == BL_STATE_RECEIVING)
    {
        // Initialize OTA module when entering the receiving state.
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
            // Indicate ready state with green LED and process CLI commands.
            led_on(LED_GREEN);
            cli_process_input(current_state);
            break;

        case BL_STATE_RECEIVING:
            // Indicate receiving state with blue LED and process incoming OTA data.
            led_on(LED_BLUE);
            led_off(LED_GREEN);
            ota_process_non_blocking(); // Non-blocking to allow other tasks.
            break;

        case BL_STATE_VERIFY:
            LOG_INFO("Verifying received firmware...");
            if (ota_finalize_and_verify())
            {
                // If verification is successful, reboot to trigger the new firmware boot.
                LOG_INFO("Verification successful. Rebooting to new firmware.");
                NVIC_SystemReset();
            }
            else
            {
                // If verification fails, enter an error state.
                LOG_ERROR("Verification FAILED. Entering error state.");
                bootloader_set_state(BL_STATE_ERROR);
            }
            break;

        case BL_STATE_ERROR:
        {
            // Indicate error state with red LED and provide limited CLI for recovery.
            static bool error_indicated = false;
            if (!error_indicated)
            {
                led_on(LED_RED);
                led_off(LED_GREEN);
                led_off(LED_BLUE);
                LOG_ERROR("\r\n!!! An error occurred. Entering recovery mode. Type 'help'. !!!");
                error_indicated = true;
            }
            cli_process_input(current_state);
            break;
        }

        default:
            // No action for other states, or states not explicitly handled.
            break;
    }
}

void bootloader_jump_to_active_application(void)
{
    const bootloader_config_t* p_cfg = read_boot_config();
    // Create a mutable copy of the configuration to modify boot attempts or active slot.
    bootloader_config_t new_cfg;
    memcpy(&new_cfg, p_cfg, sizeof(bootloader_config_t));

    bool config_changed = false;
    slot_index_t current_slot = new_cfg.active_slot;

    // --- CHECK FOR SUCCESSFUL BOOT INDICATION ---
    // Enable PWR clock and disable backup protection to access RTC backup registers.
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR1 |= PWR_CR1_DBP;
    while ((PWR->CR1 & PWR_CR1_DBP) == 0); // Wait for DBP to be set.

    // Check if the BOOT_SUCCESS_MAGIC was written by the previously booted application.
    if (RTC->BKP1R == BOOT_SUCCESS_MAGIC)
    {
        LOG_INFO("Success signal from previous boot detected.");
        // Reset boot attempts for the current active slot, indicating a good boot.
        new_cfg.slot[current_slot].boot_attempts_remaining = BOOT_ATTEMPT_COUNT;
        config_changed = true;
        // Clear the magic number to ensure it's a one-time signal.
        RTC->BKP1R = 0x00000000;
    }

    // Re-enable backup protection.
    PWR->CR1 &= ~PWR_CR1_DBP;

    // 1. --- DECIDE WHICH FIRMWARE TO BOOT ---
    // Validate the currently active slot.
    if (current_slot >= NUM_SLOTS || !new_cfg.slot[current_slot].is_valid)
    {
        LOG_ERROR("Active slot (%d) is invalid. Halting.", current_slot);
        bootloader_set_state(BL_STATE_ERROR);
        return;
    }

    // Check if boot attempts for the current slot are exhausted, triggering a rollback.
    if (new_cfg.slot[current_slot].boot_attempts_remaining == 0)
    {
        LOG_ERROR("!!! No boot attempts left for slot %d. Attempting rollback. !!!", current_slot);
        // Determine the fallback slot (the other slot).
        slot_index_t fallback_slot = (current_slot == SLOTA) ? SLOTB : SLOTA;

        if (new_cfg.slot[fallback_slot].is_valid)
        {
            // Switch active slot to the fallback and reset its boot attempts.
            new_cfg.active_slot = fallback_slot;
            new_cfg.slot[fallback_slot].boot_attempts_remaining = BOOT_ATTEMPT_COUNT;
            config_changed = true;

            // Write the updated configuration and then reset to boot from the new active slot.
            LOG_INFO("Rolling back to slot %d. Writing new config and resetting.", fallback_slot);
            write_boot_config(&new_cfg);
            // After writing config, return to the main loop to re-evaluate and jump.
            // This ensures a clean state transition.
            return;
        }
        else
        {
            // If fallback slot is also invalid, then there's no valid firmware to boot.
            LOG_ERROR("!!! Fallback slot %d is not valid. Cannot roll back. Halting. !!!", fallback_slot);
            bootloader_set_state(BL_STATE_ERROR);
            return;
        }
    }

    // 2. --- PREPARE FOR JUMP ---
    // Decrement the boot attempt counter for the current active slot.
    // This happens even if the boot is ultimately successful to track attempts.
    new_cfg.slot[current_slot].boot_attempts_remaining--;
    config_changed = true;

    // 3. --- WRITE ONCE (if needed) ---
    // Persist any changes to the boot configuration to flash before jumping.
    if (config_changed)
    {
        if (!write_boot_config(&new_cfg))
        {
            LOG_ERROR("Failed to write updated boot config. Halting.");
            bootloader_set_state(BL_STATE_ERROR);
            return;
        }
    }

    // 4. --- EXECUTE JUMP ---
    // Determine the physical address of the active firmware.
    uint32_t jump_address = (current_slot == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;
    const slot_metadata_t* active_slot_meta = &p_cfg->slot[current_slot]; // Use original config for metadata

    // Perform CRC verification of the firmware image.
    if (!verify_crc(jump_address, active_slot_meta->fw_size, active_slot_meta->fw_crc))
    {
        LOG_ERROR("CRC check failed for active slot. Cannot jump.");
        bootloader_set_state(BL_STATE_ERROR);
        return;
    }

    LOG_INFO("Boot counter decremented. CRC OK. Jumping to application in slot %d.", current_slot);
    bootloader_jump_to(jump_address);
}

void validate_boot_environment(void)
{
    LOG_INFO("Validating boot environment...");

    // Verify that the Vector Table Offset Register (VTOR) is correctly remapped.
    if ((SCB->VTOR & 0xFFF00000) != BOOTLOADER_START_ALIAS)
    {
        LOG_ERROR("\tError: Unexpected VTOR address. Halting.");
        // Infinite loop to indicate a critical unrecoverable error.
        while (1) { for (volatile int i = 0; i < 50000; i++); }
    }
    LOG_INFO("\tVTOR configuration OK");

    // Verify memory aliasing between the AXI bus and ITCM (Instruction Tightly-Coupled Memory).
    // This is crucial for correct bootloader execution on some architectures.
    if (bootloader_verify_memory_aliasing() != BL_OK)
    {
        LOG_ERROR("\tError: Memory aliasing failed.");
        current_state = BL_STATE_ERROR; // Transition to error state if aliasing is incorrect.
        return;
    }
    LOG_INFO("\tMemory aliasing verified");
}

// -----------------------------------------------------------------------------
// Static Function Implementations
// -----------------------------------------------------------------------------

static bootloader_status_t bootloader_verify_memory_aliasing(void)
{
    // Compare content of physical flash address with its ITCM alias.
    // They should be identical if aliasing is correctly configured.
    uint32_t *flash = (uint32_t *)BOOTLOADER_START_PHYS;
    uint32_t *itcm  = (uint32_t *)BOOTLOADER_START_ALIAS;

    // Check a small region for consistency.
    for (int i = 0; i < 256; i++)
    {
        if (flash[i] != itcm[i])
            return BL_ERROR_INVALID_BOOT_MODE; // Mismatch indicates an aliasing issue.
    }
    return BL_OK;
}

static void bootloader_jump_to(uint32_t app_address)
{
    LOG_INFO("Jumping to application at: 0x%08X\r\n", app_address); // Added app_address to log

    // Disable all interrupts to prevent interference during the jump.
    __disable_irq();
    // It's good practice to de-initialize peripherals used by bootloader (e.g., UART)
    // to ensure the application starts with a clean slate.

    // Reset system clocks to their default (HSI) state before handing over control.
    // This ensures the application starts from a known clock configuration.
    RCC->CR |= RCC_CR_HSION;            // Enable HSI (High-Speed Internal) oscillator
    while(!(RCC->CR & RCC_CR_HSIRDY)); // Wait for HSI to be ready
    RCC->CFGR = 0;                      // Reset clock configuration registers

    // Disable SysTick timer to prevent unexpected interrupts in the application.
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // Set the application's vector table address. This is critical for the application's exception handling.
    SCB->VTOR = app_address;

    // Retrieve the application's initial stack pointer and entry point from its vector table.
    uint32_t* vec_tab = (uint32_t*)app_address;
    uint32_t app_sp = vec_tab[0];    // First entry in vector table is initial stack pointer
    uint32_t app_entry = vec_tab[1]; // Second entry is the reset handler (entry point)

    // Set the Main Stack Pointer (MSP) to the application's initial stack pointer.
    __set_MSP(app_sp);

    // Create a function pointer to the application's reset handler and execute the jump.
    // The cast to `void*` followed by `(void(*)(void))` ensures proper function pointer type.
    void (*app_reset_handler)(void) = (void*)app_entry;
    app_reset_handler();

    // This part of the code should technically never be reached, as control is transferred to the application.
    while(1);
}