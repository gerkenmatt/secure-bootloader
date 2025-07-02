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
    // Initialize the protocol parser to its default state.
    ota_protocol_parser_init(&ota_parser);
    // Reset the OTA session timeout counter.
    ota_reset_timeout();
}

void ota_process_non_blocking(void)
{
    uint8_t byte;
    ota_frame_t received_frame;

    // Continuously read bytes from UART until no more are available.
    while (uart_getc(&byte))
    {
        // Reset the timeout timer whenever a new byte is successfully received.
        ota_reset_timeout();

        // Feed the byte to the OTA protocol parser. If a complete, valid frame is parsed,
        // `ota_protocol_parse_byte` returns true and populates `received_frame`.
        if (ota_protocol_parse_byte(&ota_parser, byte, &received_frame))
        {
            // Dispatch the received frame to the appropriate handler based on its type.
            switch (received_frame.type)
            {
                case PACKET_CMD:    handle_ota_command(&received_frame);    break;
                case PACKET_HEADER: handle_ota_header(&received_frame, &ota_session);     break;
                case PACKET_DATA:   handle_ota_data(&received_frame, &ota_session);       break;
                case PACKET_SIG:    handle_ota_signature(&received_frame, &ota_session); break;
                default:
                    // Send NACK for unknown packet types and log an error.
                    ota_send_response(RESP_NACK);
                    LOG_ERROR("Unknown packet type received during OTA.");
                    break;
            }
        }
    }
    // After processing all currently available UART bytes, check if the OTA session has timed out.
    ota_check_timeout();
}

void handle_ota_command(const ota_frame_t* p_frame)
{
    // Validate that the command packet contains at least one byte for the command code.
    if (p_frame->length < 1)
    {
        LOG_ERROR("Command packet received with no payload.");
        ota_send_response(RESP_NACK);
        return;
    }

    // Process the specific command based on the first byte of the payload.
    switch (p_frame->data[0])
    {
        case CMD_START:
            // Initiate the OTA session setup.
            process_cmd_start(&ota_session);
            break;
        case CMD_END:
            // Signal the end of data transfer and transition to firmware verification.
            LOG_INFO("CMD_END received. Proceeding to verification.");
            ota_send_response(RESP_ACK);
            bootloader_set_state(BL_STATE_VERIFY); // Change bootloader state to trigger verification.
            break;
        default:
            // Respond with NACK for unknown commands and log the error.
            ota_send_response(RESP_NACK);
            LOG_ERROR("Unknown command received: 0x%02X", p_frame->data[0]);
            break;
    }
}

bool ota_finalize_and_verify(void)
{
    LOG_INFO("Finalizing update and performing verification...");
    // Determine the flash address of the slot where the new firmware was downloaded.
    uint32_t inactive_slot_addr = (ota_session.inactive_slot_index == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;

    // 1. Verify the digital signature of the newly downloaded firmware.
    LOG_INFO("Verifying firmware signature...");
    if (!ota_crypto_verify_signature(
                (uint8_t*)inactive_slot_addr,   // Pointer to the start of the downloaded firmware.
                ota_session.header.fw_size,     // Size of the downloaded firmware.
                ota_session.signature,          // Pointer to the received signature.
                ota_session.signature_length))  // Length of the received signature.
    {
        LOG_ERROR("Signature verification FAILED. Aborting update.");
        ota_send_response(RESP_NACK);
        return false;
    }
    LOG_INFO("Firmware signature verified successfully.");

    // 2. Prepare the new bootloader configuration for an atomic slot swap.
    // Read the current configuration to modify it.
    bootloader_config_t new_cfg;
    memcpy(&new_cfg, read_boot_config(), sizeof(bootloader_config_t));

    // Update the metadata for the slot that just received the new firmware.
    new_cfg.slot[ota_session.inactive_slot_index].fw_size = ota_session.header.fw_size;
    new_cfg.slot[ota_session.inactive_slot_index].fw_crc = ota_session.header.fw_crc;
    new_cfg.slot[ota_session.inactive_slot_index].is_valid = 1; // Mark the new slot as valid.
    // Reset boot attempts for the newly updated slot.
    new_cfg.slot[ota_session.inactive_slot_index].boot_attempts_remaining = BOOT_ATTEMPT_COUNT;
    // Atomically switch the active slot to the one with the new firmware.
    new_cfg.active_slot = ota_session.inactive_slot_index;

    // 3. Write the new configuration back to flash.
    LOG_INFO("Writing updated boot configuration to activate the new slot.");
    if (!write_boot_config(&new_cfg))
    {
        ota_send_response(RESP_NACK);
        LOG_ERROR("Failed to write updated boot configuration. Update failed.");
        return false;
    }
    LOG_INFO("Boot configuration updated successfully.");
    ota_send_response(RESP_ACK); // Acknowledge successful finalization.
    return true;
}

void ota_reset_timeout(void)
{
    // Update the timestamp of the last received byte to the current system tick count.
    // This resets the inactivity timer for the OTA session.
    ota_last_byte_timestamp = get_systick();
}

// -----------------------------------------------------------------------------
// Static Function Implementations
// -----------------------------------------------------------------------------

static void ota_send_response(uint8_t status)
{
    uint8_t frame_buffer[OTA_MAX_RESPONSE_FRAME_LEN]; // Buffer to construct the response frame.
    uint16_t frame_len = 0;

    // Create the OTA response frame with the given status.
    ota_protocol_create_response(status, frame_buffer, &frame_len);

    // Transmit each byte of the constructed response frame over UART.
    for (uint16_t i = 0; i < frame_len; i++)
    {
        uart_putc(frame_buffer[i]);
    }
}

static void ota_check_timeout(void)
{
    // Check if an OTA session is active (timestamp is not 0) and if the elapsed time
    // since the last received byte exceeds the defined timeout.
    if (ota_last_byte_timestamp != 0 && (get_systick() - ota_last_byte_timestamp) > OTA_TIMEOUT_MS)
    {
        LOG_ERROR("OTA session timed out due to inactivity.");
        // Transition the bootloader to an error state upon timeout.
        bootloader_set_state(BL_STATE_ERROR);
        // Reset the timestamp to 0 to indicate no active timeout is running.
        ota_last_byte_timestamp = 0;
    }
}

static void process_cmd_start(ota_session_t* p_session)
{
    // Set the OTA session state to STARTED, indicating that the update process has officially begun.
    p_session->state = OTA_STATE_STARTED;

    // Determine which flash slot is currently inactive, as this will be the target for the new firmware.
    const bootloader_config_t* cfg = read_boot_config();
    p_session->inactive_slot_index = (cfg->active_slot == SLOTA) ? SLOTB : SLOTA;

    // Get the base address and starting sector of the inactive slot.
    uint32_t inactive_slot_addr = (p_session->inactive_slot_index == SLOTA) ? SLOTA_ADDR : SLOTB_ADDR;
    uint8_t  inactive_slot_sector = (p_session->inactive_slot_index == SLOTA) ? SLOTA_SECTOR : SLOTB_SECTOR;

    // Clear the firmware header information in the session, preparing for new header reception.
    memset(&p_session->header, 0, sizeof(p_session->header));

    // Initialize the flash write address to the beginning of the inactive slot.
    p_session->flash_write_address = inactive_slot_addr;
    // Reset signature length, as a new signature will be received.
    p_session->signature_length = 0;

    flash_prepare_for_write();
    
    // Erase the entire inactive slot to clear any old firmware before writing new data.
    flash_status_t status = erase_flash_sectors(inactive_slot_sector, inactive_slot_sector + SLOT_SECTOR_COUNT - 1);
    if (status != FLASH_OK)
    {
        // If flash erase fails, send NACK, log error, and transition bootloader to error state.
        ota_send_response(RESP_NACK);
        LOG_ERROR("Flash erase failed during OTA start.");
        bootloader_set_state(BL_STATE_ERROR);
        lock_flash(); // Ensure flash is locked even on error.
        return;
    }
    // If erase is successful, send ACK to the host.
    ota_send_response(RESP_ACK);
    lock_flash(); // Lock flash after erase operation.
}

static void handle_ota_header(const ota_frame_t* p_frame, ota_session_t* p_session)
{
    // Ensure that a START command was received before a HEADER packet.
    if (p_session->state != OTA_STATE_STARTED)
    {
        LOG_ERROR("Received HEADER packet before START command. Ignoring.");
        ota_send_response(RESP_NACK);
        return;
    }
    // Transition the session state to indicate that data reception is expected next.
    p_session->state = OTA_STATE_RECEIVING_DATA;

    // Validate the length of the received header payload. It must match the expected header structure size.
    if (p_frame->length != sizeof(ota_header_info_t))
    {
        ota_send_response(RESP_NACK);
        LOG_ERROR("Invalid OTA header length received.");
        return;
    }

    // Copy the received header data into the session's header structure.
    memcpy(&p_session->header, p_frame->data, sizeof(ota_header_info_t));

    // Perform a sanity check: ensure the reported firmware size does not exceed the available slot size.
    if (p_session->header.fw_size > SLOT_SIZE)
    {
        LOG_ERROR("Firmware size (%u bytes) exceeds allocated slot size (%u bytes). Aborting.",
                  (unsigned int)p_session->header.fw_size, (unsigned int)SLOT_SIZE);
        ota_send_response(RESP_NACK);
        bootloader_set_state(BL_STATE_ERROR); // Critical error, transition to bootloader error state.
        return;
    }
    // Acknowledge successful header reception.
    ota_send_response(RESP_ACK);
}

static void handle_ota_data(const ota_frame_t* p_frame, ota_session_t* p_session)
{
    // Ensure that data packets are only received when the session is in the data reception state.
    if (p_session->state != OTA_STATE_RECEIVING_DATA)
    {
        ota_send_response(RESP_NACK);
        LOG_ERROR("Received DATA packet in unexpected OTA state.");
        return;
    }

    // Validate the length of the incoming data chunk.
    if (p_frame->length == 0 || p_frame->length > OTA_MAX_DATA)
    {
        ota_send_response(RESP_NACK);
        LOG_ERROR("Invalid data length in OTA DATA packet.");
        return;
    }

    flash_prepare_for_write();

    // Program the received data chunk to the current flash write address.
    flash_status_t status = program_flash(p_session->flash_write_address, (uint32_t*)p_frame->data, p_frame->length);
    if (status != FLASH_OK)
    {
        // If flash write fails, send NACK and log detailed error information.
        ota_send_response(RESP_NACK);
        LOG_ERROR("Flash write failed during OTA data transfer: ");
        switch(status)
        {
            case FLASH_ERROR_ALIGNMENT: LOG_ERROR("Alignment Error"); break;
            case FLASH_ERROR:           LOG_ERROR("Programming Error"); break;
            case FLASH_ERROR_VERIFY:    LOG_ERROR("Programming Verification Error (data mismatch after write)"); break;
            default:                    LOG_ERROR("Unknown Error"); break;
        }
        // Transition bootloader to error state due to critical flash write failure.
        bootloader_set_state(BL_STATE_ERROR);
    }
    else
    {
        // If write is successful, increment the flash write address for the next data chunk.
        p_session->flash_write_address += p_frame->length;
        ota_send_response(RESP_ACK); // Acknowledge successful data write.
    }
    lock_flash(); // Always lock flash after write operations.
}

static void handle_ota_signature(const ota_frame_t* p_frame, ota_session_t* p_session)
{
    // Sanity check: ensure the received signature length does not exceed the buffer capacity.
    if (p_frame->length > SIG_MAX_LEN)
    {
        ota_send_response(RESP_NACK);
        LOG_ERROR("Received signature is too large (%u bytes). Max allowed: %u bytes.",
                  (unsigned int)p_frame->length, (unsigned int)SIG_MAX_LEN);
        return;
    }
    // Copy the received signature bytes into the session's signature buffer in RAM.
    memcpy(p_session->signature, p_frame->data, p_frame->length);
    // Store the actual length of the received signature.
    p_session->signature_length = p_frame->length;
    ota_send_response(RESP_ACK); // Acknowledge successful signature reception.
}