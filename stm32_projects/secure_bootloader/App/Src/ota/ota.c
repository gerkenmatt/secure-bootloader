#include "ota.h"
#include "bootloader.h" // Needed for bootloader_set_state and config structs
#include "uart.h"       // For logging and comms
#include "utilities.h"  // For crc32, etc.
#include "flash.h"      // For writing new config
#include "string.h"     // For memcpy
#include "mbedtls/platform.h"
#include "mbedtls/sha256.h"
#include "mbedtls/pk.h"
#include "mbedtls/md.h"
#include "mbedtls/error.h"
#include "stm32f767xx.h"

#define CHUNK_SIZE 256
#define PUBKEY_DER_LEN 91

typedef enum {
    OTA_PARSE_STATE_WAIT_SOF,
    OTA_PARSE_STATE_GET_TYPE,
    OTA_PARSE_STATE_GET_LEN_LO,
    OTA_PARSE_STATE_GET_LEN_HI,
    OTA_PARSE_STATE_GET_DATA,
    OTA_PARSE_STATE_GET_CRC,
    OTA_PARSE_STATE_WAIT_EOF
} ota_parse_state_t;

// --- Timeout Management Data (at top of file) ---
#define OTA_TIMEOUT_MS 5000 // 5 seconds
static uint32_t ota_last_byte_timestamp = 0;

// --- OTA Session State ---
// This struct holds the context for the current OTA session.
typedef struct {
    ota_header_info_t header;         // Firmware size and CRC from the header packet
    uint32_t flash_write_address;     // The starting address of the slot being written to
    uint8_t  inactive_slot_index;     // The index (0 or 1) of the slot being updated
    uint8_t  signature[SIG_MAX_LEN];  // Buffer for the received firmware signature
    uint16_t signature_length;        // Length of the received signature
} ota_session_t;

static ota_session_t ota_session; // A single instance for the OTA process

// --- Public Key ---
// The public key is embedded in the bootloader to verify firmware signatures.
// For production, this key should be protected against tampering.
static const unsigned char pubkey_der[] = {
  0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x02,
  0x01, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x03, 0x01, 0x07, 0x03,
  0x42, 0x00, 0x04, 0x04, 0xb8, 0x40, 0x1e, 0xa6, 0x42, 0xa4, 0xf8, 0xef,
  0xeb, 0x1a, 0x1d, 0xc0, 0xe7, 0x12, 0x6f, 0x13, 0x31, 0x72, 0xfb, 0x8d,
  0x13, 0xc2, 0xd8, 0xc2, 0xc8, 0xb6, 0x99, 0x5e, 0xb8, 0xa2, 0x4c, 0x58,
  0x7c, 0x72, 0x0f, 0x4a, 0x21, 0x5b, 0x56, 0x76, 0xcf, 0xe5, 0xba, 0xd9,
  0x6d, 0x3e, 0xc0, 0x6c, 0xb3, 0xd4, 0xad, 0x48, 0xf4, 0x07, 0xa7, 0xdc,
  0x29, 0x46, 0x41, 0xef, 0x57, 0x38, 0x74
};

// --- Static Function Prototypes ---

static bool verify_signature(const uint8_t *data, uint32_t data_len, const uint8_t *sig, uint16_t sig_len);
static void handle_ota_command(const ota_frame_t* frame);
static void handle_ota_header(const ota_frame_t* frame);
static void handle_ota_data(const ota_frame_t* frame);
static void handle_ota_signature(const ota_frame_t* frame);
static bool ota_receive_frame(ota_frame_t* frame);
static void ota_check_timeout(void);
static bool process_cmd_start(void); 

// --- Public Functions ---

void ota_process_non_blocking(void) 
{
    ota_frame_t frame;

    if (ota_receive_frame(&frame)) 
    {
        // Process the single, valid frame that was just received
        switch (frame.type) 
        {
            case PACKET_CMD:
                // If the command is CMD_END, it will trigger the state transition
                handle_ota_command(&frame); 
                break;
            case PACKET_HEADER:  handle_ota_header(&frame);      break;
            case PACKET_DATA:    handle_ota_data(&frame);        break;
            case PACKET_SIG:     handle_ota_signature(&frame);   break;
            default:
                ota_send_response(RESP_NACK);
                log("Unknown packet type\r\n");
                break;
        }
    }
    
    ota_check_timeout(); // Check for session timeout
}

// --- Frame Reception and Command Handling ---

/**
 * @brief Attempts to receive a complete OTA frame in a non-blocking manner.
 * This function is a state machine that processes one byte at a time from the UART buffer.
 *
 * @param frame Pointer to an ota_frame_t struct that will be filled if a frame is complete.
 * @return true if a complete, valid frame was received, false otherwise.
 */
bool ota_receive_frame(ota_frame_t* frame) 
{
    // --- Static variables to hold the state between calls ---
    static ota_parse_state_t parser_state = OTA_PARSE_STATE_WAIT_SOF;
    static ota_frame_t internal_frame; // We build the frame internally first
    static uint16_t data_index = 0;
    static uint8_t crc_buffer[4];
    static uint8_t crc_index = 0;
    
    uint8_t byte;

    // Process all bytes currently in the ring buffer
    while (uart_getc(&byte)) 
    {
        ota_reset_timeout(); // A byte was received, reset the timeout

        switch (parser_state) 
        {
            case OTA_PARSE_STATE_WAIT_SOF:
                if (byte == OTA_SOF) 
                {
                    parser_state = OTA_PARSE_STATE_GET_TYPE;
                }
                break;

            case OTA_PARSE_STATE_GET_TYPE:
                internal_frame.type = byte;
                parser_state = OTA_PARSE_STATE_GET_LEN_LO;
                break;

            case OTA_PARSE_STATE_GET_LEN_LO:
                internal_frame.length = byte;
                parser_state = OTA_PARSE_STATE_GET_LEN_HI;
                break;

            case OTA_PARSE_STATE_GET_LEN_HI:
                internal_frame.length |= ((uint16_t)byte << 8);
                if (internal_frame.length > OTA_MAX_DATA) 
                {
                    parser_state = OTA_PARSE_STATE_WAIT_SOF; // Reset on error
                } 
                else 
                {
                    data_index = 0;
                    parser_state = (internal_frame.length > 0) ? OTA_PARSE_STATE_GET_DATA : OTA_PARSE_STATE_GET_CRC;
                }
                break;

            case OTA_PARSE_STATE_GET_DATA:
                internal_frame.data[data_index++] = byte;
                if (data_index >= internal_frame.length) 
                {
                    crc_index = 0;
                    parser_state = OTA_PARSE_STATE_GET_CRC;
                }
                break;

            case OTA_PARSE_STATE_GET_CRC:
                crc_buffer[crc_index++] = byte;
                if (crc_index >= 4) 
                {
                    internal_frame.crc = extract_crc(crc_buffer);
                    parser_state = OTA_PARSE_STATE_WAIT_EOF;
                }
                break;
            
            case OTA_PARSE_STATE_WAIT_EOF:
            {
                bool success = false;
                if (byte == OTA_EOF) 
                {
                    uint32_t calc_crc = crc32(internal_frame.data, internal_frame.length);
                    if (calc_crc == internal_frame.crc) 
                    {
                        // SUCCESS! Frame is complete and valid.
                        // Copy the internal frame to the user's provided pointer.
                        memcpy(frame, &internal_frame, sizeof(ota_frame_t));
                        success = true;
                    }
                }
                // Always reset the state machine for the next frame
                parser_state = OTA_PARSE_STATE_WAIT_SOF;
                
                if (success) 
                {
                    // Return true only on this specific path
                    return true; 
                }
                break;
            }
        }
    }
    
    // If we reach here, it means we ran out of bytes to process
    // and a full frame has not yet been completed.
    return false;
}

void handle_ota_command(const ota_frame_t* frame) 
{
    // Validate frame has at least 1 byte for command
    if (frame->length < 1) 
    {
        log("Command packet has no payload\r\n");
        ota_send_response(RESP_NACK);
        return; 
    }

    switch (frame->data[0]) 
    {
        case CMD_START:
            process_cmd_start();
            break;
        case CMD_END:
            log("CMD_END received. Proceeding to verification.\r\n");
            ota_send_response(RESP_ACK);
            bootloader_set_state(BL_STATE_VERIFY);
            break;
        default:
            // Unknown command received
            ota_send_response(RESP_NACK);
            log("Unknown CMD: "); uart_print_hex32(frame->data[0]); log("\r\n");
            break;
    }
}

bool ota_finalize_and_verify(void) 
{
     log("Finalizing update...\r\n");

     uint32_t inactive_slot_addr = (ota_session.inactive_slot_index == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;

     // 1. Verify the signature of the newly downloaded firmware
     log("Verifying signature...\r\n");
     if (!verify_signature(
         (uint8_t*)inactive_slot_addr, 
         ota_session.header.fw_size, 
         ota_session.signature, 
         ota_session.signature_length))
     {
         log("Signature verification FAILED. Aborting update.\r\n");
         ota_send_response(RESP_NACK);
         return false;
     }
     log("Signature verified\r\n");

     // 2. Prepare the new configuration with the atomic swap
     bootloader_config_t new_cfg;
     memcpy(&new_cfg, read_boot_config(), sizeof(bootloader_config_t)); // Make a mutable copy

     // Update metadata for the new firmware slot
     new_cfg.slot[ota_session.inactive_slot_index].fw_size = ota_session.header.fw_size;
     new_cfg.slot[ota_session.inactive_slot_index].fw_crc = ota_session.header.fw_crc;
     new_cfg.slot[ota_session.inactive_slot_index].is_valid = 1;
     new_cfg.slot[ota_session.inactive_slot_index].boot_attempts_remaining = BOOT_ATTEMPT_COUNT;

     // Perform the atomic swap by changing the active slot index
     new_cfg.active_slot = ota_session.inactive_slot_index;

     // 3. Write the new configuration back to flash
     log("Writing boot config to activate slot\r\n");
     if (!write_boot_config(&new_cfg)) 
     {
         ota_send_response(RESP_NACK);
         log("Failed to write boot config\r\n");
         return false; 
     }
     log("Boot config written\r\n");
     ota_send_response(RESP_ACK);

     return true; // Success
}

// --- Static Helper Functions ---

/**
 * @brief Processes the CMD_START command to prepare for OTA update.
 * 
 * This function clears any previous flash error flags, sets the green LED,
 * determines the inactive slot for writing, erases the necessary flash sectors,
 * and sends an ACK response if successful.
 *
 * @return true if successful, false if flash erase failed
 */
static bool process_cmd_start(void) 
{
    // Clear any previous flash error flags
    clear_flash_errors();

    // Determine which slot is inactive and prepare it
    const bootloader_config_t* cfg = read_boot_config();
    ota_session.inactive_slot_index = (cfg->active_slot == SLOTA) ? SLOTB : SLOTA;
    
    uint32_t inactive_slot_addr = (ota_session.inactive_slot_index == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;
    uint8_t  inactive_slot_sector = (ota_session.inactive_slot_index == SLOTA) ? SLOTA_SECTOR : SLOTB_SECTOR;

    // Initialize the session context for this OTA
    memset(&ota_session.header, 0, sizeof(ota_session.header));
    ota_session.flash_write_address = inactive_slot_addr;
    ota_session.signature_length = 0;

    unlock_flash();
    clear_flash_errors();

    // Configure flash access control and program size
    FLASH->ACR |= (1 << 8) | (1 << 9);  // Enable instruction and data cache
    FLASH->CR |= FLASH_CR_PSIZE_1;      // Set program size to 32-bit

    // Erase inactive slot sectors
    if (!erase_flash_sectors(
        inactive_slot_sector, 
        inactive_slot_sector + SLOT_SECTOR_COUNT -1, 
        inactive_slot_addr, 
        SLOT_SIZE)) 
    {
        ota_send_response(RESP_NACK);
        log("Flash erase failed\r\n");
        bootloader_set_state(BL_STATE_ERROR);
        return false;
    }
    // Erase successful, send ACK
    ota_send_response(RESP_ACK);
    lock_flash();
    return true;
}

/**
 * @brief Handles incoming OTA header packet containing firmware metadata.
 * 
 * Validates header length and copies metadata to global ota_header struct.
 *
 * @param frame Pointer to received OTA frame
 */
static void handle_ota_header(const ota_frame_t* frame) 
{
    // Verify header length matches expected size
    if (frame->length != sizeof(ota_header_info_t)) 
    {
        ota_send_response(RESP_NACK);
        log("Invalid header length\r\n");
        return;
    }

    // Copy header data to global struct
    memcpy(&ota_session.header, frame->data, sizeof(ota_header_info_t));

    // Sanity check firmware size against slot size
    if (ota_session.header.fw_size > SLOT_SIZE) 
    {
        log("Firmware size exceeds slot size\r\n");
        ota_send_response(RESP_NACK);
        bootloader_set_state(BL_STATE_ERROR);
        return;
    }

    ota_send_response(RESP_ACK);
}

/**
 * @brief Handles incoming OTA data packets containing firmware binary.
 * 
 * Validates data length, programs data to flash in 32-bit words,
 * and verifies written data.
 *
 * @param frame Pointer to received OTA frame
 */
static void handle_ota_data(const ota_frame_t* frame) 
{
    // Validate data length is within bounds
    if (frame->length == 0 || frame->length > OTA_MAX_DATA) 
    {
        ota_send_response(RESP_NACK);
        log("Invalid data length\r\n");
        return;
    }

    unlock_flash();
    // Write data chunk to the correct address in the inactive slot
    if (!program_flash(ota_session.flash_write_address, (uint32_t*)frame->data, frame->length)) 
    {
        ota_send_response(RESP_NACK);
        log("Flash write failed at address: "); uart_print_hex32(ota_session.flash_write_address); log("\r\n");
        bootloader_set_state(BL_STATE_ERROR);
    }
    else
    {
        // Update write address for the next chunk
        ota_session.flash_write_address += frame->length;
        ota_send_response(RESP_ACK);
    }
    lock_flash();
}

/**
 * @brief Handles incoming OTA signature packet containing firmware signature.
 * 
 * Validates signature length and copies signature to global ota_signature struct.
 *
 * @param frame Pointer to received OTA frame
 */
static void handle_ota_signature(const ota_frame_t* frame) 
{
    // sanity check
    if (frame->length > SIG_MAX_LEN) 
    {
        ota_send_response(RESP_NACK);
        log("Signature too large\r\n");
        return;
    }
    // copy it into RAM
    memcpy(ota_session.signature, frame->data, frame->length);
    ota_session.signature_length = frame->length;
    ota_send_response(RESP_ACK);
}

/**
 * @brief Verifies the signature of a firmware image.
 * 
 * @param data Pointer to the firmware image data
 * @param data_len Length of the firmware image data
 * @param sig Pointer to the signature data
 * @param sig_len Length of the signature data
 * @return true if signature is valid, false otherwise
 */
static bool verify_signature(const uint8_t *data, uint32_t data_len, const uint8_t *sig, uint16_t sig_len) 
{
    int ret;
    uint8_t hash[32];
    mbedtls_pk_context pk_ctx;
    mbedtls_sha256_context sha_ctx;

    mbedtls_sha256_init(&sha_ctx);
    if (mbedtls_sha256_starts(&sha_ctx, 0) != 0 ||
        mbedtls_sha256_update(&sha_ctx, data, data_len) != 0 ||
        mbedtls_sha256_finish(&sha_ctx, hash) != 0) 
    {
        mbedtls_sha256_free(&sha_ctx);
        return false;
    }
    mbedtls_sha256_free(&sha_ctx);

    mbedtls_pk_init(&pk_ctx);
    if (mbedtls_pk_parse_public_key(&pk_ctx, pubkey_der, PUBKEY_DER_LEN) != 0) 
    {
        mbedtls_pk_free(&pk_ctx);
        return false;
    }

    ret = mbedtls_pk_verify(&pk_ctx, MBEDTLS_MD_SHA256, hash, sizeof(hash), sig, sig_len);
    mbedtls_pk_free(&pk_ctx);
    return (ret == 0);
}


void ota_send_response(uint8_t status) 
{
    uint8_t payload[] = {status};
    uint32_t crc = crc32(payload, sizeof(payload)); // Assuming crc32 function is available

    uint8_t frame[] = {
        OTA_SOF, PACKET_RESP, 
        (uint8_t)(sizeof(payload) & 0xFF), // len_lo
        (uint8_t)((sizeof(payload) >> 8) & 0xFF), // len_hi
        status,
        (uint8_t)(crc & 0xFF),
        (uint8_t)((crc >> 8) & 0xFF),
        (uint8_t)((crc >> 16) & 0xFF),
        (uint8_t)((crc >> 24) & 0xFF),
        OTA_EOF
    };
    // Send frame via uart_putc loop
    for (size_t i = 0; i < sizeof(frame); i++) 
    {
        uart_putc(frame[i]);
    }
}

void ota_reset_timeout(void) 
{
    ota_last_byte_timestamp = get_systick(); // Assumes a get_systick() function
}

void ota_check_timeout(void) 
{
    if (ota_last_byte_timestamp != 0 && (get_systick() - ota_last_byte_timestamp) > OTA_TIMEOUT_MS) 
    {
        log("OTA session timed out.\r\n");
        bootloader_set_state(BL_STATE_ERROR);
        ota_last_byte_timestamp = 0; // Stop checking
    }
}
