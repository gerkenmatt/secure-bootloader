#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ota.h"
#include "stm32f7xx.h"
#include "uart.h"
#include "utilities.h"
#include "flash.h"
#include "bootloader.h"

// OTA session state
static ota_header_info_t ota_header;
static uint32_t flash_write_addr = SLOT1_ADDR;

// storage for the incoming signature:
static uint8_t  ota_signature[SIG_MAX_LEN];
static uint16_t ota_sig_len;

static const char firmware_pub_pem[] =
"-----BEGIN PUBLIC KEY-----\n"
"MFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEf+4s6HcLDTeocFcerXXR0FqdirYf\n"
"8sCC+Nub9LtYSXLNu02/F7LXbVpf+c5h1Nv3C4PYYkhmiD2tmqLkl1wYmQ==\n"
"-----END PUBLIC KEY-----\n";

void handle_ota_session(void) {
    log("Waiting for OTA packets...\r\n");

    while (1) {
        ota_frame_t frame;

        // Try to receive a valid frame
        if (ota_receive_frame(&frame)) {
            // Process frame based on its type
            switch (frame.type) {
                case PACKET_CMD:    
                    handle_ota_command(&frame); // Handle control commands
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
        log("CRC mismatch: calc=0x");
        print_uint32_hex(calc_crc);
        log(" frame=0x"); 
        print_uint32_hex(frame->crc);
        log("\r\n");
        return false;
    }

    return true;
}

void handle_ota_command(const ota_frame_t* frame) {
    // Validate frame has at least 1 byte for command
    if (frame->length < 1) return;

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
                return;
            }

            // Erase successful, send ACK
            ota_send_response(RESP_ACK);

            break;

        case CMD_END:
            log("Received OTA END\r\n");

            // verify the signature on the new image in flash
            if (!verify_signature(
                    (uint8_t*)SLOT1_ADDR,
                    ota_header.fw_size,
                    ota_signature,
                    ota_sig_len))
            {
                log("Signature verification failed\r\n");
                ota_send_response(RESP_NACK);
                return;
            }
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

            if (!write_boot_config(&cfg)) {
                log("Failed to write boot config\r\n");
                ota_send_response(RESP_NACK);
                return;
            }

            // OTA update complete, lock flash and reboot
            FLASH->CR |= FLASH_CR_LOCK;
            log("Flash locked\r\n");
            log("Rebooting...\r\n");
            SCB_CleanDCache();        // Clean data cache
            NVIC_SystemReset();       // Reset system
            break;

        default:
            // Unknown command received
            ota_send_response(RESP_NACK);
            log("Unknown CMD: "); print_uint32_hex(frame->data[0]); log("\r\n");
            break;
    }
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
        //TODO: we are failing here now
        uint32_t verify = *(volatile uint32_t *)flash_write_addr;
        if (verify != word) {
            ota_send_response(RESP_NACK);
            log("Flash verify failed\r\n");
            log("Expected: "); print_uint32_hex(word);
            log("\r\nReadback: "); print_uint32_hex(verify);
            log("\r\n");
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
                      uint32_t        data_len,
                      const uint8_t  *sig,
                      uint16_t        sig_len)
{
    int ret;
    uint8_t hash[32];
    mbedtls_pk_context pk;

    // 1) Hash the firmware image with SHA-256
    mbedtls_sha256_context sha_ctx;
    mbedtls_sha256_init(&sha_ctx);
    mbedtls_sha256_starts_ret(&sha_ctx, 0);
    mbedtls_sha256_update_ret(&sha_ctx, data, data_len);
    mbedtls_sha256_finish_ret(&sha_ctx, hash);
    mbedtls_sha256_free(&sha_ctx);

    // 2) Parse the public key
    mbedtls_pk_init(&pk);
    ret = mbedtls_pk_parse_public_key(&pk,
                                      (const unsigned char*)firmware_pub_pem,
                                      strlen(firmware_pub_pem) + 1);
    if (ret != 0) {
        // Optional: call mbedtls_strerror(ret, buf, len) to log the error
        mbedtls_pk_free(&pk);
        return false;
    }

    // 3) Verify the signature (DER-encoded) over the hash
    ret = mbedtls_pk_verify(&pk,
                            MBEDTLS_MD_SHA256,
                            hash, sizeof(hash),
                            sig, sig_len);

    mbedtls_pk_free(&pk);
    return (ret == 0);
}