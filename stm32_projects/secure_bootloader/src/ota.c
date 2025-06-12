#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ota.h"
#include "stm32f7xx.h"
#include "uart.h"
#include "utilities.h"
#include "flash.h"
#include "bootloader.h"

// mbed TLS Headers
#include "mbedtls/platform.h" // If using mbedtls_platform_set_calloc_free
#include "mbedtls/sha256.h"
#include "mbedtls/pk.h"
#include "mbedtls/md.h"       // For MBEDTLS_MD_SHA256 enum
#include "mbedtls/error.h"

#define CHUNK_SIZE 256
#define PUBKEY_DER_LEN 91

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

static bool process_cmd_start(void);
static bool process_cmd_end(void);
static void handle_ota_header(const ota_frame_t* frame);
static void handle_ota_data(const ota_frame_t* frame);
static void handle_ota_signature(const ota_frame_t* frame);
static bool verify_signature(const uint8_t *data, uint32_t data_len, const uint8_t *sig, uint16_t sig_len);

// --- Public Functions ---

bool handle_ota_session(void) {
    log("Waiting for OTA packets...\r\n");

    while (1) {

        ota_frame_t frame;
        if (!ota_receive_frame(&frame)) {
            log("Invalid frame received.\r\n");
            continue; 
        }

        // Process a valid frame based on its type
        switch (frame.type) {
            case PACKET_CMD:
                if (!handle_ota_command(&frame)) {
                    // This indicates a command was received that should terminate the session (e.g., failed CMD_END)
                    log("Exiting OTA session due to command handler result.\r\n");
                    return false;
                }
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
    return true;
}

// --- Frame Reception and Command Handling ---

bool ota_receive_frame(ota_frame_t* frame) {

    // Validate input parameter
    if (!frame) {
        log("Frame pointer is NULL\r\n");
        return false;
    }

    // Wait for start of frame marker
    //TODO: add timeout
    uint8_t byte = 0;
    while (byte != OTA_SOF){
        byte = usart_getc();
    }

    // Read frame type and length
    frame->type = usart_getc();
    uint8_t len_lo = usart_getc();
    uint8_t len_hi = usart_getc();
    frame->length = (uint16_t)((len_hi << 8) | len_lo);

    // Validate frame length
    if (frame->length > OTA_MAX_DATA) {
        log("Frame too large\r\n");
        return false;
    }

    // Read frame payload data
    for (uint16_t i = 0; i < frame->length; i++) {
        frame->data[i] = usart_getc();
    }

    // Read and extract 32-bit CRC
    uint8_t crc_bytes[4];
    for (int i = 0; i < 4; i++) crc_bytes[i] = usart_getc();
    frame->crc = extract_crc(crc_bytes);

    // Verify end of frame marker
    if (usart_getc() != OTA_EOF) {
        log("Invalid EOF\r\n");
        return false;
    }

    // Validate CRC
    uint32_t calc_crc = crc32(frame->data, frame->length);
    if (calc_crc != frame->crc) {
        log("CRC mismatch: calc=0x"); print_uint32_hex(calc_crc);
        log(" frame=0x"); print_uint32_hex(frame->crc); log("\r\n");
        return false;
    }

    return true;
}

bool handle_ota_command(const ota_frame_t* frame) 
{
    // Validate frame has at least 1 byte for command
    if (frame->length < 1) 
    {
        log("Command packet has no payload\r\n");
        ota_send_response(RESP_NACK);
        return true; // Stay in OTA session
    }

    switch (frame->data[0]) 
    {
        case CMD_START:
            return process_cmd_start();
        case CMD_END:
            return process_cmd_end();
        default:
            // Unknown command received
            ota_send_response(RESP_NACK);
            log("Unknown CMD: "); print_uint32_hex(frame->data[0]); log("\r\n");
            break;
    }
    return true;
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

    // uint32_t errs = FLASH->SR;
    // // Erase the config sector
    // // program_flash_word(SLOTA_ADDR, 0xbeefb00b);
    // // log("slota first word: "); print_uint32_hex(*(volatile uint32_t*)SLOTA_ADDR); log("\r\n");
    // if (!erase_flash_sectors(CONFIG_SECTOR, CONFIG_SECTOR, CONFIG_ADDR, 0x40000)) 
    // {
    //     log("Failed to eraseb config sector!\r\n");
    //     log("Flash errors before: "); print_uint32_hex(errs); log("\r\n");
    //     lock_flash(); // Always re-lock flash
    //     return false;
    // }

    // Erase inactive slot sectors
    if (!erase_flash_sectors(
        inactive_slot_sector, 
        inactive_slot_sector + SLOT_SECTOR_COUNT -1, 
        inactive_slot_addr, 
        SLOT_SIZE)) 
    {
        ota_send_response(RESP_NACK);
        log("Flash erase failed\r\n");
        return true;
    }
    // Erase successful, send ACK
    ota_send_response(RESP_ACK);
    return true;
}

/**
 * @brief Processes the CMD_END command to finalize the OTA update.
 * 
 * Verifies the firmware signature, updates bootloader configuration,
 * locks the flash memory, and reboots the system.
 *
 * @return true if successful, false if signature verification failed
 */
static bool process_cmd_end(void) {
    log("CMD_END received. Finalizing update...\r\n");

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
    lock_flash(); // for debugging

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
        return false; // Terminate OTA session, something is wrong with flash config
    }
    log("Boot config written\r\n");
    ota_send_response(RESP_ACK);

    // OTA update complete, lock flash and reboot
    FLASH->CR |= FLASH_CR_LOCK;
    log("Flash locked\r\nRebooting...\r\n");
    SCB_CleanDCache();        // Clean data cache ----- TODO: is this needed? 
    NVIC_SystemReset();       // Reset system

    return false; // Code should not reach here
}

/**
 * @brief Handles incoming OTA header packet containing firmware metadata.
 * 
 * Validates header length and copies metadata to global ota_header struct.
 *
 * @param frame Pointer to received OTA frame
 */
static void handle_ota_header(const ota_frame_t* frame) {
    // Verify header length matches expected size
    if (frame->length != sizeof(ota_header_info_t)) {
        ota_send_response(RESP_NACK);
        log("Invalid header length\r\n");
        return;
    }

    // Copy header data to global struct
    memcpy(&ota_session.header, frame->data, sizeof(ota_header_info_t));

    // Sanity check firmware size against slot size
    if (ota_session.header.fw_size > SLOT_SIZE) {
        log("Firmware size exceeds slot size\r\n");
        ota_send_response(RESP_NACK);
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
static void handle_ota_data(const ota_frame_t* frame) {
    // Validate data length is within bounds
    if (frame->length == 0 || frame->length > OTA_MAX_DATA) {
        ota_send_response(RESP_NACK);
        log("Invalid data length\r\n");
        return;
    }

    // Write data chunk to the correct address in the inactive slot
    if (!program_flash(ota_session.flash_write_address, (uint32_t*)frame->data, frame->length)) {
        ota_send_response(RESP_NACK);
        log("Flash write failed at address: "); print_uint32_hex(ota_session.flash_write_address); log("\r\n");
        return;
    }

    // Update write address for the next chunk
    ota_session.flash_write_address += frame->length;
    ota_send_response(RESP_ACK);
}

/**
 * @brief Handles incoming OTA signature packet containing firmware signature.
 * 
 * Validates signature length and copies signature to global ota_signature struct.
 *
 * @param frame Pointer to received OTA frame
 */
static void handle_ota_signature(const ota_frame_t* frame) {
    // sanity check
    if (frame->length > SIG_MAX_LEN) {
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
    uint8_t hash[32]; // SHA-256 output is 32 bytes
    mbedtls_pk_context pk_ctx;
    mbedtls_sha256_context sha_ctx;

    // --- 1. Hash the firmware image ---
    log("Hashing firmware image (0x"); print_uint32_hex(data_len); log(" bytes)...\r\n");
    mbedtls_sha256_init(&sha_ctx);

    ret = mbedtls_sha256_starts(&sha_ctx, 0); // 0 for SHA-256
    if (ret != 0) {
        log("SHA256 starts failed\r\n");
        mbedtls_sha256_free(&sha_ctx);
        return false;
    }

    ret = mbedtls_sha256_update(&sha_ctx, data, data_len);
    if (ret != 0) {
        log("SHA256 update failed/r/n");
        mbedtls_sha256_free(&sha_ctx);
        return false;
    }

    ret = mbedtls_sha256_finish(&sha_ctx, hash);
    if (ret != 0) {
        log("SHA256 finish failed\r\n"); 
        mbedtls_sha256_free(&sha_ctx);
        return false;
    }
    mbedtls_sha256_free(&sha_ctx);
    log("Firmware hashing complete.\r\n");

    // --- 2. Initialize and parse the public key ---
    log("Parsing public key...\r\n");
    mbedtls_pk_init(&pk_ctx);
    ret = mbedtls_pk_parse_public_key(&pk_ctx, pubkey_der, PUBKEY_DER_LEN);
    if (ret != 0) {
        log("PK parse public key failed\r\n");
        mbedtls_pk_free(&pk_ctx);
        return false;
    }

    if (!mbedtls_pk_can_do(&pk_ctx, MBEDTLS_PK_ECKEY)) { // Ensure MBEDTLS_PK_ECKEY is known from your mbedTLS config
        log("Error: Parsed key is not an EC key.\r\n");
        mbedtls_pk_free(&pk_ctx);
        return false;
    }
    log("Public key parsed successfully.\r\n");

    // --- 3. Verify the signature ---
    log("Verifying signature against hash (signature len: 0x"); print_uint16_hex(sig_len); log(")...\r\n");
    ret = mbedtls_pk_verify(&pk_ctx, MBEDTLS_MD_SHA256, hash, sizeof(hash), sig, sig_len);
    
    if (ret != 0) {
        log("!!! Signature verification FAILED: ");
    } else {
        log(">>> Signature VERIFIED successfully! <<<\r\n");
    }

    mbedtls_pk_free(&pk_ctx);

    return (ret == 0);
}


void ota_send_response(uint8_t status) {
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
    // Send frame via usart_putc loop
    for (size_t i = 0; i < sizeof(frame); i++) {
        usart_putc(frame[i]);
    }
}