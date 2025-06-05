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
#define log(msg)          usart_puts(msg)

/// Size of sector 5 (128KB)
#define SECTOR5_SIZE      (128 * 1024)

#define SIG_MAX_LEN       512

// -----------------------------------------------------------------------------
// Data Structures
// -----------------------------------------------------------------------------

/// Structure representing a received OTA frame
typedef struct {
    uint8_t  type;                        ///< Packet type
    uint16_t length;                     ///< Length of data field
    uint8_t  data[OTA_MAX_DATA];         ///< Payload data
    uint32_t crc;                        ///< CRC32 of payload
} ota_frame_t;

/// Structure representing firmware metadata header
typedef struct {
    uint32_t fw_size;                    ///< Firmware size in bytes
    uint32_t fw_crc;                     ///< CRC32 checksum of firmware
    uint32_t version;                    ///< Firmware version number
    uint32_t reserved;                   ///< Reserved for future use
} ota_header_info_t;

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Main OTA update handler function that processes incoming OTA packets.
 * 
 * Runs in an infinite loop waiting for and handling OTA frames.
 * Frame types supported:
 * - PACKET_CMD: Control commands like start/end update
 * - PACKET_HEADER: Contains firmware metadata
 * - PACKET_DATA: Contains firmware binary data
 * @return true if OTA session completed successfully, false if interrupted
 */
bool handle_ota_session(void);

/**
 * @brief Receives and validates an OTA frame from UART.
 * 
 * Frame format:
 * [SOF][TYPE][LEN_LO][LEN_HI][DATA...][CRC0-3][EOF]
 *
 * @param frame Pointer to frame structure to populate
 * @return true if frame received and validated successfully, false otherwise
 */
bool ota_receive_frame(ota_frame_t* frame);

/**
 * @brief Handles OTA command frames received during firmware update.
 * 
 * Commands supported:
 * - CMD_START: Prepares flash for firmware update by erasing sectors
 * - CMD_END: Completes update, locks flash and reboots system
 *
 * @param frame Pointer to received OTA frame containing command
 * @return true if command handled successfully, false if session should end
 */
bool handle_ota_command(const ota_frame_t* frame);

/**
 * @brief Handles incoming OTA header packet containing firmware metadata.
 * 
 * Validates header length and copies metadata to global ota_header struct.
 *
 * @param frame Pointer to received OTA frame
 */
void handle_ota_header(const ota_frame_t* frame);

/**
 * @brief Handles incoming OTA data packets containing firmware binary.
 * 
 * Validates data length, programs data to flash in 32-bit words,
 * and verifies written data.
 *
 * @param frame Pointer to received OTA frame
 */
void handle_ota_data(const ota_frame_t* frame);

/**
 * @brief Sends a response packet back to the host.
 * 
 * Frame format: [SOF][PACKET_RESP][LEN=1][0x00][STATUS][0x00 x4][EOF]
 *
 * @param status Response status code (RESP_ACK or RESP_NACK)
 */
void ota_send_response(uint8_t status);

/**
 * @brief Handles incoming OTA signature packet containing firmware signature.
 * 
 * Validates signature length and copies signature to global ota_signature struct.
 *
 * @param frame Pointer to received OTA frame
 */
void handle_ota_signature(const ota_frame_t* frame);

/**
 * @brief Verifies the signature of a firmware image.
 * 
 * @param data Pointer to the firmware image data
 * @param data_len Length of the firmware image data
 * @param sig Pointer to the signature data
 * @param sig_len Length of the signature data
 * @return true if signature is valid, false otherwise
 */
bool verify_signature(const uint8_t *data, uint32_t data_len, const uint8_t  *sig, uint16_t sig_len);


#endif // OTA_H
