#include "ota.h"
#include "ota_crypto.h" 
#include "ota_protocol.h" 

#include "boot_config.h"
#include "bootloader.h"
#include "logger.h"
#include "uart.h"
#include "utilities.h"
#include "flash.h"
#include "string.h"
#include "systick.h"
#include "stm32f767xx.h"


#define CHUNK_SIZE 256


// --- Timeout Management Data ---
#define OTA_TIMEOUT_MS 5000
static uint32_t ota_last_byte_timestamp = 0;

// --- OTA Session State ---
typedef struct 
{
    ota_header_info_t header;
    uint32_t flash_write_address;
    uint8_t  inactive_slot_index;
    uint8_t  signature[SIG_MAX_LEN];
    uint16_t signature_length;
} ota_session_t;

static ota_session_t ota_session; 
static ota_parser_t  ota_parser;  

// --- Static Function Prototypes ---

static void ota_send_response(uint8_t status);
static void handle_ota_command(const ota_frame_t* frame);
static void handle_ota_header(const ota_frame_t* frame);
static void handle_ota_data(const ota_frame_t* frame);
static void handle_ota_signature(const ota_frame_t* frame);
static void ota_check_timeout(void);
static bool process_cmd_start(void);

// --- Public Functions ---

void ota_init(void) 
{
    ota_protocol_parser_init(&ota_parser);
    ota_reset_timeout();
}


void ota_process_non_blocking(void) 
{
    uint8_t byte;
    ota_frame_t received_frame;

    while (uart_getc(&byte)) 
    {
        ota_reset_timeout(); // A byte was received, reset the session timeout.

        // If the parser returns true, a complete, CRC-valid frame has arrived.
        if (ota_protocol_parse_byte(&ota_parser, byte, &received_frame)) 
        {
            // Now we process the *meaning* of the valid frame.
            switch (received_frame.type) 
            {
                case PACKET_CMD:    handle_ota_command(&received_frame);   break;
                case PACKET_HEADER: handle_ota_header(&received_frame);    break;
                case PACKET_DATA:   handle_ota_data(&received_frame);      break;
                case PACKET_SIG:    handle_ota_signature(&received_frame); break;
                default:
                    ota_send_response(RESP_NACK);
                    LOG_ERROR("Unknown packet type\r\n");
                    break;
            }
        }
    }

    // After processing all available bytes, check if the session has timed out.
    ota_check_timeout();
}

// --- Frame Reception and Command Handling ---

void handle_ota_command(const ota_frame_t* frame) 
{
    // Validate frame has at least 1 byte for command
    if (frame->length < 1) 
    {
        LOG_ERROR("Command packet has no payload\r\n");
        ota_send_response(RESP_NACK);
        return; 
    }

    switch (frame->data[0]) 
    {
        case CMD_START:
            process_cmd_start();
            break;
        case CMD_END:
            LOG_INFO("CMD_END received. Proceeding to verification.\r\n");
            ota_send_response(RESP_ACK);
            bootloader_set_state(BL_STATE_VERIFY);
            break;
        default:
            // Unknown command received
            ota_send_response(RESP_NACK);
            LOG_ERROR("Unknown CMD: 0x%08X\r\n", frame->data[0]); 
            break;
    }
}

bool ota_finalize_and_verify(void) 
{
    LOG_INFO("Finalizing update...\r\n");

     uint32_t inactive_slot_addr = (ota_session.inactive_slot_index == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;

    // 1. Verify the signature of the newly downloaded firmware
    LOG_INFO("Verifying signature...\r\n");
    if (!ota_crypto_verify_signature(
            (uint8_t*)inactive_slot_addr,
            ota_session.header.fw_size,
            ota_session.signature,
            ota_session.signature_length))
    {
        LOG_ERROR("Signature verification FAILED. Aborting update.\r\n");
        ota_send_response(RESP_NACK);
        return false;
    }
    LOG_INFO("Signature verified\r\n");

     // 2. Prepare the new configuration with the atomic swap
     bootloader_config_t new_cfg;
     memcpy(&new_cfg, read_boot_config(), sizeof(bootloader_config_t)); 

     // Update metadata for the new firmware slot
     new_cfg.slot[ota_session.inactive_slot_index].fw_size = ota_session.header.fw_size;
     new_cfg.slot[ota_session.inactive_slot_index].fw_crc = ota_session.header.fw_crc;
     new_cfg.slot[ota_session.inactive_slot_index].is_valid = 1;
     new_cfg.slot[ota_session.inactive_slot_index].boot_attempts_remaining = BOOT_ATTEMPT_COUNT;
     new_cfg.active_slot = ota_session.inactive_slot_index;

     // 3. Write the new configuration back to flash
     LOG_INFO("Writing boot config to activate slot\r\n");
     if (!write_boot_config(&new_cfg)) 
     {
         ota_send_response(RESP_NACK);
         LOG_ERROR("Failed to write boot config\r\n");
         return false; 
     }
     LOG_INFO("Boot config written\r\n");
     ota_send_response(RESP_ACK);

     return true; // Success
}

void ota_reset_timeout(void) 
{
    ota_last_byte_timestamp = get_systick(); // Assumes a get_systick() function
}

// --- Static Helper Functions ---

static void ota_send_response(uint8_t status) 
{
    uint8_t frame_buffer[16]; 
    uint16_t frame_len = 0;

    ota_protocol_create_response(status, frame_buffer, &frame_len);

    for (uint16_t i = 0; i < frame_len; i++) 
    {
        uart_putc(frame_buffer[i]);
    }
}

void ota_check_timeout(void) 
{
    if (ota_last_byte_timestamp != 0 && (get_systick() - ota_last_byte_timestamp) > OTA_TIMEOUT_MS) 
    {
        LOG_ERROR("OTA session timed out.\r\n");
        bootloader_set_state(BL_STATE_ERROR);
        ota_last_byte_timestamp = 0; 
    }
}


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
    const bootloader_config_t* cfg = read_boot_config();
    ota_session.inactive_slot_index = (cfg->active_slot == SLOTA) ? SLOTB : SLOTA;
    
    uint32_t inactive_slot_addr = (ota_session.inactive_slot_index == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;
    uint8_t  inactive_slot_sector = (ota_session.inactive_slot_index == SLOTA) ? SLOTA_SECTOR : SLOTB_SECTOR;

    memset(&ota_session.header, 0, sizeof(ota_session.header));
    ota_session.flash_write_address = inactive_slot_addr;
    ota_session.signature_length = 0;

    flash_prepare_for_write();

    flash_status_t status = erase_flash_sectors(inactive_slot_sector, inactive_slot_sector + SLOT_SECTOR_COUNT - 1);
    if (status != FLASH_OK) 
    {
        // This is the failure path
        ota_send_response(RESP_NACK);
        LOG_ERROR("Flash erase failed\r\n");
        bootloader_set_state(BL_STATE_ERROR);
        lock_flash();
        return false;
    }

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
        LOG_ERROR("Invalid header length\r\n");
        return;
    }

    // Copy header data to global struct
    memcpy(&ota_session.header, frame->data, sizeof(ota_header_info_t));

    // Sanity check firmware size against slot size
    if (ota_session.header.fw_size > SLOT_SIZE) 
    {
        LOG_ERROR("Firmware size exceeds slot size\r\n");
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
        LOG_ERROR("Invalid data length\r\n");
        return;
    }

    unlock_flash();

    flash_status_t status = program_flash(ota_session.flash_write_address, (uint32_t*)frame->data, frame->length);

    if (status != FLASH_OK) 
    {
        // This is an OTA-specific action based on a generic driver status.
        ota_send_response(RESP_NACK);
        
        LOG_ERROR("Flash write failed: ");
        switch(status) 
        {
            case FLASH_ERROR_ALIGNMENT: LOG_ERROR("Alignment Error\r\n"); break;
            case FLASH_ERROR:           LOG_ERROR("Programming Error\r\n"); break;
            case FLASH_ERROR_VERIFY:    LOG_ERROR("Programming Verification Error\r\n"); break;
            default:                    LOG_ERROR("Unknown Error\r\n"); break;
        }
        
        bootloader_set_state(BL_STATE_ERROR);
    } 
    else 
    {
        // Flash write was successful
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
        LOG_ERROR("Signature too large\r\n");
        return;
    }
    // copy it into RAM
    memcpy(ota_session.signature, frame->data, frame->length);
    ota_session.signature_length = frame->length;
    ota_send_response(RESP_ACK);
}


