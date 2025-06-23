#ifndef OTA_H
#define OTA_H

#include <stdint.h>
#include <stdbool.h>
#include "bootloader.h"
#include "mbedtls/sha256.h"
#include "mbedtls/pk.h"
#include "mbedtls/error.h"

/// Total size of fixed fields in OTA frame
#define FRAME_TOTAL_LEN   10

/// Size of sector 5 (128KB)
#define SECTOR5_SIZE      (128 * 1024)

#define SIG_MAX_LEN       512

// -----------------------------------------------------------------------------
// Data Structures
// -----------------------------------------------------------------------------

/// Structure representing firmware metadata header
typedef struct {
    uint32_t fw_size;                    // Firmware size in bytes
    uint32_t fw_crc;                     // CRC32 checksum of firmware
    uint32_t version;                    // Firmware version number
    uint32_t reserved;                   // Reserved for future use
} ota_header_info_t;

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Initializes the OTA module.
 * This function should be called when the bootloader enters the OTA state.
 */
void ota_init(void);


/**
 * @brief Processes the OTA data stream in a non-blocking manner.
 * This is the main engine for the OTA module during an update.
 */
void ota_process_non_blocking(void);

/**
 * @brief Resets the OTA session timeout timer.
 * Should be called when an OTA session begins.
 */
void ota_reset_timeout(void);

/**
 * @brief Performs the final verification of the downloaded firmware.
 * This includes checking the signature and, upon success, updating
 * the bootloader configuration to make the new slot active.
 *
 * @return true if verification is successful and config is written, false otherwise.
 */
bool ota_finalize_and_verify(void);




#endif // OTA_H
