/**
 * @file ota.c
 * @brief OTA update protocol implementation and firmware verification logic.
 */
#include "ota.h"
#include "ota_crypto.h"
#include "ota_protocol.h"
#include "boot_config.h"
#include "bootloader.h"
#include "logger.h"
#include "uart.h"
#include "utilities.h"
#include "flash.h"
#include "systick.h"
#include "stm32f767xx.h"

#include "mbedtls/sha256.h" 
#include "mbedtls/pk.h"     
#include "mbedtls/error.h"
#include <string.h> 

// -----------------------------------------------------------------------------
// Module-Private Constants
// -----------------------------------------------------------------------------

#define CHUNK_SIZE 256
#define OTA_TIMEOUT_MS 5000

// -----------------------------------------------------------------------------
// Enumerations
// -----------------------------------------------------------------------------

typedef enum {
    OTA_STATE_IDLE,
    OTA_STATE_STARTED, 
    OTA_STATE_RECEIVING_DATA, 
    OTA_STATE_AWAITING_VERIFICATION 
} ota_process_state_t;

// -----------------------------------------------------------------------------
// Data Structures
// -----------------------------------------------------------------------------
typedef struct {
    ota_header_info_t header;           ///< Firmware metadata header
    uint32_t flash_write_address;       ///< Current flash write address
    uint8_t  inactive_slot_index;       ///< Index of inactive slot for update
    uint8_t  signature[SIG_MAX_LEN];    ///< Firmware signature buffer
    uint16_t signature_length;          ///< Length of signature
    ota_process_state_t state;          ///< Current OTA Session state
} ota_session_t;

// -----------------------------------------------------------------------------
// Module-Private Data (Static Globals)
// -----------------------------------------------------------------------------

static uint32_t ota_last_byte_timestamp = 0;

static ota_session_t ota_session;
static ota_parser_t  ota_parser;

// -----------------------------------------------------------------------------
// Static Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Sends an OTA protocol response frame with the given status.
 * @param status Response status code
 */
static void ota_send_response(uint8_t status);

/**
 * @brief Checks for OTA session timeout and sets error state if timed out.
 * @param p_frame Pointer to the received OTA frame
 */
static void handle_ota_command(const ota_frame_t* p_frame);

/**
 * @brief Handles the OTA header packet containing firmware metadata.
 * Validates the header and prepares for data reception.
 * @param p_frame Pointer to the received OTA frame
 * @param p_session Pointer to the OTA session to handle
 */
static void handle_ota_header(const ota_frame_t* p_frame, ota_session_t* p_session);

/**
 * @brief Handles incoming OTA data packets containing firmware binary.
 * Validates data length, programs data to flash in 32-bit words,
 * and verifies written data.
 * @param p_frame Pointer to the received OTA frame
 * @param p_session Pointer to the OTA session to handle
 */
static void handle_ota_data(const ota_frame_t* p_frame, ota_session_t* p_session);

/**
 * @brief Handles incoming OTA signature packet containing firmware signature.
 * Validates signature length and copies signature to global ota_signature struct.
 * @param p_frame Pointer to the received OTA frame
 * @param p_session Pointer to the OTA session to handle
 */
static void handle_ota_signature(const ota_frame_t* p_frame, ota_session_t* p_session);

/**
 * @brief Resets the OTA session timeout timer.
 * Should be called when an OTA session begins or a byte is received.
 */
static void ota_check_timeout(void);

/**
 * @brief Processes the CMD_START command to prepare for OTA update.
 * Erases the inactive slot and prepares for data reception.
 * @param p_session Pointer to the OTA session to handle
 */
static void process_cmd_start(ota_session_t* p_session);

// -----------------------------------------------------------------------------
// Public Function Implementations
// -----------------------------------------------------------------------------

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
            // Process the meaning of the valid frame.
            switch (received_frame.type)
            {
                case PACKET_CMD:    handle_ota_command(&received_frame);   break;
                case PACKET_HEADER: handle_ota_header(&received_frame, &ota_session);    break;
                case PACKET_DATA:   handle_ota_data(&received_frame, &ota_session);      break;
                case PACKET_SIG:    handle_ota_signature(&received_frame, &ota_session); break;
                default:
                    ota_send_response(RESP_NACK);
                    LOG_ERROR("Unknown packet type");
                    break;
            }
        }
    }
    // After processing all available bytes, check if the session has timed out.
    ota_check_timeout();
}

void handle_ota_command(const ota_frame_t* p_frame) 
{
    // Validate frame has at least 1 byte for command
    if (p_frame->length < 1)
    {
        LOG_ERROR("Command packet has no payload");
        ota_send_response(RESP_NACK);
        return;
    }

    switch (p_frame->data[0])
    {
        case CMD_START:
            process_cmd_start(&ota_session);
            break;
        case CMD_END:
            LOG_INFO("CMD_END received. Proceeding to verification.");
            ota_send_response(RESP_ACK);
            bootloader_set_state(BL_STATE_VERIFY);
            break;
        default:
            ota_send_response(RESP_NACK);
            LOG_ERROR("Unknown CMD: 0x%08X", p_frame->data[0]);
            break;
    }
}

bool ota_finalize_and_verify(void) 
{
    LOG_INFO("Finalizing update...");
    uint32_t inactive_slot_addr = (ota_session.inactive_slot_index == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;

    // 1. Verify the signature of the newly downloaded firmware
    LOG_INFO("Verifying signature...");
    if (!ota_crypto_verify_signature(
            (uint8_t*)inactive_slot_addr,
            ota_session.header.fw_size,
            ota_session.signature,
            ota_session.signature_length))
    {
        LOG_ERROR("Signature verification FAILED. Aborting update.");
        ota_send_response(RESP_NACK);
        return false;
    }
    LOG_INFO("Signature verified");

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
    LOG_INFO("Writing boot config to activate slot");
    if (!write_boot_config(&new_cfg))
    {
        ota_send_response(RESP_NACK);
        LOG_ERROR("Failed to write boot config");
        return false;
    }
    LOG_INFO("Boot config written");
    ota_send_response(RESP_ACK);
    return true;
}

void ota_reset_timeout(void) 
{
    ota_last_byte_timestamp = get_systick();
}

// -----------------------------------------------------------------------------
// Static Function Implementations
// -----------------------------------------------------------------------------

static void ota_send_response(uint8_t status)
{
    uint8_t frame_buffer[OTA_MAX_RESPONSE_FRAME_LEN];
    uint16_t frame_len = 0;
    ota_protocol_create_response(status, frame_buffer, &frame_len);
    for (uint16_t i = 0; i < frame_len; i++)
    {
        uart_putc(frame_buffer[i]);
    }
}

static void ota_check_timeout(void)
{
    if (ota_last_byte_timestamp != 0 && (get_systick() - ota_last_byte_timestamp) > OTA_TIMEOUT_MS)
    {
        LOG_ERROR("OTA session timed out.");
        bootloader_set_state(BL_STATE_ERROR);
        ota_last_byte_timestamp = 0;
    }
}

static void process_cmd_start(ota_session_t* p_session)
{
    p_session->state = OTA_STATE_STARTED;

    const bootloader_config_t* cfg = read_boot_config();
    p_session->inactive_slot_index = (cfg->active_slot == SLOTA) ? SLOTB : SLOTA;
    uint32_t inactive_slot_addr = (p_session->inactive_slot_index == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;
    uint8_t  inactive_slot_sector = (p_session->inactive_slot_index == SLOTA) ? SLOTA_SECTOR : SLOTB_SECTOR;

    memset(&p_session->header, 0, sizeof(p_session->header));

    p_session->flash_write_address = inactive_slot_addr;
    p_session->signature_length = 0;

    flash_prepare_for_write();
    flash_status_t status = erase_flash_sectors(inactive_slot_sector, inactive_slot_sector + SLOT_SECTOR_COUNT - 1);
    if (status != FLASH_OK)
    {
        ota_send_response(RESP_NACK);
        LOG_ERROR("Flash erase failed");
        bootloader_set_state(BL_STATE_ERROR);
        lock_flash();
        return;
    }
    ota_send_response(RESP_ACK);
    lock_flash();
}

static void handle_ota_header(const ota_frame_t* p_frame, ota_session_t* p_session)
{
    if (p_session->state != OTA_STATE_STARTED)
    {
        LOG_ERROR("Received HEADER packet before START command.");
        ota_send_response(RESP_NACK);
        return;
    }
    p_session->state = OTA_STATE_RECEIVING_DATA;

    if (p_frame->length != sizeof(ota_header_info_t))
    {
        ota_send_response(RESP_NACK);
        LOG_ERROR("Invalid header length");
        return;
    }

    // Copy header data to global struct
    memcpy(&p_session->header, p_frame->data, sizeof(ota_header_info_t));

    // Sanity check firmware size against slot size
    if (p_session->header.fw_size > SLOT_SIZE) 
    {
        LOG_ERROR("Firmware size exceeds slot size");
        ota_send_response(RESP_NACK);
        bootloader_set_state(BL_STATE_ERROR);
        return;
    }
    ota_send_response(RESP_ACK);
}

static void handle_ota_data(const ota_frame_t* p_frame, ota_session_t* p_session)
{
    if (p_session->state != OTA_STATE_RECEIVING_DATA)
    {
        ota_send_response(RESP_NACK);
        LOG_ERROR("Received DATA packet in unexpected state.");
        return;
    }

    if (p_frame->length == 0 || p_frame->length > OTA_MAX_DATA)
    {
        ota_send_response(RESP_NACK);
        LOG_ERROR("Invalid data length");
        return;
    }
    flash_prepare_for_write();
    flash_status_t status = program_flash(p_session->flash_write_address, (uint32_t*)p_frame->data, p_frame->length);
    if (status != FLASH_OK)
    {
        ota_send_response(RESP_NACK);
        LOG_ERROR("Flash write failed: ");
        switch(status)
        {
            case FLASH_ERROR_ALIGNMENT: LOG_ERROR("Alignment Error"); break;
            case FLASH_ERROR:           LOG_ERROR("Programming Error"); break;
            case FLASH_ERROR_VERIFY:    LOG_ERROR("Programming Verification Error"); break;
            default:                    LOG_ERROR("Unknown Error"); break;
        }
        bootloader_set_state(BL_STATE_ERROR);
    }
    else
    {
        p_session->flash_write_address += p_frame->length;
        ota_send_response(RESP_ACK);
    }
    lock_flash();
}

static void handle_ota_signature(const ota_frame_t* p_frame, ota_session_t* p_session)
{
    // sanity check
    if (p_frame->length > SIG_MAX_LEN) 
    {
        ota_send_response(RESP_NACK);
        LOG_ERROR("Signature too large");
        return;
    }
    // copy it into RAM
    memcpy(p_session->signature, p_frame->data, p_frame->length);
    p_session->signature_length = p_frame->length;
    ota_send_response(RESP_ACK);
}


