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

// OTA session state
static ota_header_info_t ota_header;
static uint32_t flash_write_addr = SLOT1_ADDR;

// storage for the incoming signature:
static uint8_t  ota_signature[SIG_MAX_LEN];
static uint16_t ota_sig_len;

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


bool handle_ota_session(void) {
    log("Waiting for OTA packets...\r\n");

    while (1) {
        ota_frame_t frame;

        // Try to receive a valid frame
        if (ota_receive_frame(&frame)) {
            // Process frame based on its type
            switch (frame.type) {
                case PACKET_CMD:    
                    if (!handle_ota_command(&frame)) {
                        // Exit OTA session if handler returns false
                        return false;
                    }
                    break;
                case PACKET_HEADER: 
                    handle_ota_header(&frame);  // Process firmware metadata
                    break;
                case PACKET_DATA:   
                    handle_ota_data(&frame);    // Handle firmware data chunks
                    break;
                case PACKET_SIG:
                    handle_ota_signature(&frame); // Handle firmware signature
                    break;
                default:
                    // Invalid/unknown packet type
                    ota_send_response(RESP_NACK);
                    log("Unknown packet type\r\n");
                    break;
            }
        } 
        else {
            // Frame validation failed
            ota_send_response(RESP_NACK);
            log("Invalid frame\r\n");
        }
    }
    return true;
}

bool ota_receive_frame(ota_frame_t* frame) {
    // Validate input parameter
    if (!frame) {
        ota_send_response(RESP_NACK);
        log("Frame pointer is NULL\r\n");
        return false;
    }

    // Wait for start of frame marker
    uint8_t byte = 0;
    while (byte != OTA_SOF){
        byte = usart_getc();
    }

    // Read frame type and length
    frame->type = usart_getc();
    uint8_t len_lo = usart_getc();
    uint8_t len_hi = usart_getc();
    frame->length = (len_hi << 8) | len_lo;

    // Validate frame length
    if (frame->length > OTA_MAX_DATA) {
        ota_send_response(RESP_NACK);
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
        ota_send_response(RESP_NACK);
        log("Invalid EOF\r\n");
        return false;
    }

    // Validate CRC
    uint32_t calc_crc = crc32(frame->data, frame->length);
    if (calc_crc != frame->crc) {
        ota_send_response(RESP_NACK);
        log("CRC mismatch: calc=0x"); print_uint32_hex(calc_crc);
        log(" frame=0x"); print_uint32_hex(frame->crc); log("\r\n");
        return false;
    }

    return true;
}

bool handle_ota_command(const ota_frame_t* frame) {
    // Validate frame has at least 1 byte for command
    if (frame->length < 1) return true;

    switch (frame->data[0]) {
        case CMD_START:
            //TODO: should I put each case in its own function?
            // Clear any previous flash error flags
            FLASH->SR |= FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_OPERR | 
                         FLASH_SR_PGPERR | FLASH_SR_ERSERR;

        
            GPIOB->ODR |= (1UL << 0); //set_green
            flash_write_addr = SLOT1_ADDR;
            
            // Unlock flash if locked
            unlock_flash();

            // Configure flash access control and program size
            FLASH->ACR |= (1 << 8) | (1 << 9); // Enable instruction and data cache
            FLASH->CR |= FLASH_CR_PSIZE_1;      // Set program size to 32-bit

            // Erase slot1 sector (sector for new firmware)
            if (!erase_flash_sectors(SLOT1_SECTOR, SLOT1_SECTOR, flash_write_addr, ota_header.fw_size)) {
                ota_send_response(RESP_NACK);
                log("Flash erase failed\r\n");
                return true;
            }

            // Erase successful, send ACK
            ota_send_response(RESP_ACK);

            break;

        case CMD_END:
            log("Verifying signature...\r\n");
            if (!verify_signature(
                    (uint8_t*)SLOT1_ADDR,
                    ota_header.fw_size,
                    ota_signature,
                    ota_sig_len))
            {
                // ota_send_response(RESP_NACK);
                log("Signature verification failed\r\n");
                // Exit OTA session and return to bootloader mode
                ota_send_response(RESP_NACK);
                return false;
            }


            log("Signature verified\r\n");

            bootloader_config_t cfg = *read_boot_config();  // Read current config

            // Update slot 1 metadata
            cfg.slot[1].fw_size     = ota_header.fw_size;
            cfg.slot[1].fw_crc      = ota_header.fw_crc;
            cfg.slot[1].is_valid    = 1;
            cfg.slot[1].should_run  = 1;

            // Clear run flag from slot 0 (optional)
            cfg.slot[0].should_run = 0;

            // Keep magic + reboot_cause (or set to OTA_REQUEST if used)
            cfg.magic = BOOT_CONFIG_MAGIC;
            cfg.reboot_cause = 1;  // e.g., OTA_REQUEST (if defined)

            log("Writing boot config...\r\n");

            if (!write_boot_config(&cfg)) {
                ota_send_response(RESP_NACK);
                log("Failed to write boot config\r\n");
                return true;
            }

            log("Boot config written\r\n");

            ota_send_response(RESP_ACK);

            // OTA update complete, lock flash and reboot
            FLASH->CR |= FLASH_CR_LOCK;
            log("Flash locked\r\n"); log("Rebooting...\r\n");
            SCB_CleanDCache();        // Clean data cache
            NVIC_SystemReset();       // Reset system
            break;

        default:
            // Unknown command received
            ota_send_response(RESP_NACK);
            log("Unknown CMD: "); print_uint32_hex(frame->data[0]); log("\r\n");
            break;
    }
    return true;
}


void ota_send_response(uint8_t status) {
    uint8_t frame[] = {
        OTA_SOF, PACKET_RESP, 0x01, 0x00, status,  // Header + status
        0x00, 0x00, 0x00, 0x00, OTA_EOF            // Padding + EOF
    };
    for (int i = 0; i < sizeof(frame); i++) usart_putc(frame[i]);
}

void handle_ota_header(const ota_frame_t* frame) {
    // Verify header length matches expected size
    if (frame->length != sizeof(ota_header_info_t)) {
        ota_send_response(RESP_NACK);
        log("Invalid header length\r\n");
        return;
    }

    // Copy header data to global struct
    memcpy(&ota_header, frame->data, sizeof(ota_header_info_t));
    ota_send_response(RESP_ACK);
}


void handle_ota_data(const ota_frame_t* frame) {
    // Validate data length is within bounds
    if (frame->length == 0 || frame->length > OTA_MAX_DATA) {
        ota_send_response(RESP_NACK);
        log("Invalid data length\r\n");
        return;
    }

    // Process data in 32-bit word chunks
    for (uint16_t i = 0; i < frame->length; i += 4) {
        uint32_t word = 0xFFFFFFFF;
        // Handle partial words at end of frame
        uint16_t len = (frame->length - i >= 4) ? 4 : (frame->length - i);
        memcpy(&word, &frame->data[i], len);

        // Program word to flash
        program_flash_word(flash_write_addr, word);

        // Verify written data
        uint32_t verify = *(volatile uint32_t *)flash_write_addr;
        if (verify != word) {
            ota_send_response(RESP_NACK);
            log("Flash verify failed\r\n");
            log("Expected: "); print_uint32_hex(word);
            log("\r\nReadback: "); print_uint32_hex(verify); log("\r\n");
            return;
        }

        flash_write_addr += 4;
    }

    ota_send_response(RESP_ACK);
}

void handle_ota_signature(const ota_frame_t* frame) {
    // sanity check
    if (frame->length > SIG_MAX_LEN) {
        ota_send_response(RESP_NACK);
        log("Signature too large\r\n");
        return;
    }
    // copy it into RAM
    memcpy(ota_signature, frame->data, frame->length);
    ota_sig_len = frame->length;
    ota_send_response(RESP_ACK);
}

bool verify_signature(const uint8_t *data,
                      uint32_t      data_len,
                      const uint8_t *sig,
                      uint16_t      sig_len)
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
        log("SHA256 starts failed/r/n");
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


