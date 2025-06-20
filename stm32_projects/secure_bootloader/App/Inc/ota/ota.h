#ifndef OTA_H
#define OTA_H

#include <stdint.h>
#include <stdbool.h>
#include "bootloader.h"
#include "mbedtls/sha256.h"
#include "mbedtls/pk.h"
#include "mbedtls/error.h"

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

/// OTA packet types
#define PACKET_CMD        0x01
#define PACKET_HEADER     0x02
#define PACKET_DATA       0x03
#define PACKET_RESP       0x04
#define PACKET_SIG        0x05

/// OTA command types
#define CMD_START         0xA0
#define CMD_END           0xA1

/// OTA response types
#define RESP_ACK          0xAB
#define RESP_NACK         0xCD

/// OTA frame markers
#define OTA_SOF           0xA5
#define OTA_EOF           0xB6

/// Maximum data size per OTA packet
#define OTA_MAX_DATA      256

/// Total size of fixed fields in OTA frame
#define FRAME_TOTAL_LEN   10

/// Logging macro
#define log(msg)          uart_puts(msg)

/// Size of sector 5 (128KB)
#define SECTOR5_SIZE      (128 * 1024)

#define SIG_MAX_LEN       512

// -----------------------------------------------------------------------------
// Data Structures
// -----------------------------------------------------------------------------

/// Structure representing a received OTA frame
typedef struct {
    uint8_t  type;                       // Packet type
    uint16_t length;                     // Length of data field
    uint8_t  data[OTA_MAX_DATA];         // Payload data
    uint32_t crc;                        // CRC32 of payload
} ota_frame_t;

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
 * @brief Sends a response packet back to the host.
 * 
 * Frame format: [SOF][PACKET_RESP][LEN=1][0x00][STATUS][0x00 x4][EOF]
 *
 * @param status Response status code (RESP_ACK or RESP_NACK)
 */
void ota_send_response(uint8_t status);

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
