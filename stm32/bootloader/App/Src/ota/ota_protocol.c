#include "ota_protocol.h"
#include "utilities.h"

#include <string.h> 


// -----------------------------------------------------------------------------
// Public Function Implementations
// -----------------------------------------------------------------------------

void ota_protocol_parser_init(ota_parser_t* p_parser)
{
    if (!p_parser)
        return;

    // Reset the parser state machine to its initial state, ready for a new frame.
    p_parser->state = OTA_PARSE_STATE_WAIT_SOF;
    // Reset data and CRC buffer indices.
    p_parser->data_index = 0;
    p_parser->crc_index = 0;
    // Clear the internal frame buffer to ensure no stale data from previous parses.
    memset(&p_parser->internal_frame, 0, sizeof(ota_frame_t));
}

bool ota_protocol_parse_byte(ota_parser_t* p_parser, uint8_t byte, ota_frame_t* p_out_frame)
{
    // State machine to parse incoming bytes based on the expected frame structure.
    switch (p_parser->state)
    {
        case OTA_PARSE_STATE_WAIT_SOF:
            // Look for the Start of Frame (SOF) marker.
            if (byte == OTA_SOF)
                p_parser->state = OTA_PARSE_STATE_GET_TYPE; // Transition to get frame type.
            // If not SOF, stay in this state, discarding the byte.
            break;

        case OTA_PARSE_STATE_GET_TYPE:
            // Store the frame type and move to getting the payload length.
            p_parser->internal_frame.type = byte;
            p_parser->state = OTA_PARSE_STATE_GET_LEN_LO;
            break;

        case OTA_PARSE_STATE_GET_LEN_LO:
            // Store the lower 8 bits of the payload length.
            p_parser->internal_frame.length = byte;
            p_parser->state = OTA_PARSE_STATE_GET_LEN_HI;
            break;

        case OTA_PARSE_STATE_GET_LEN_HI:
            // Store the upper 8 bits of the payload length and combine with the lower bits.
            p_parser->internal_frame.length |= ((uint16_t)byte << 8);

            // Validate the received length against the maximum allowed data size.
            if (p_parser->internal_frame.length > OTA_MAX_DATA)
            {
                // If length is invalid, reset the parser to discard the corrupt frame.
                ota_protocol_parser_init(p_parser);
            }
            else
            {
                p_parser->data_index = 0; // Reset data index for payload reception.
                // If the frame has a payload, transition to GET_DATA; otherwise, skip directly to GET_CRC.
                p_parser->state = (p_parser->internal_frame.length > 0) ? OTA_PARSE_STATE_GET_DATA : OTA_PARSE_STATE_GET_CRC;
            }
            break;

        case OTA_PARSE_STATE_GET_DATA:
            // Store the incoming byte into the frame's data payload buffer.
            p_parser->internal_frame.data[p_parser->data_index++] = byte;
            // Check if all payload bytes have been received.
            if (p_parser->data_index >= p_parser->internal_frame.length)
            {
                p_parser->crc_index = 0; // Reset CRC index for CRC reception.
                p_parser->state = OTA_PARSE_STATE_GET_CRC; // Transition to get CRC bytes.
            }
            break;

        case OTA_PARSE_STATE_GET_CRC:
            // Store the incoming byte into the temporary CRC buffer.
            p_parser->crc_buffer[p_parser->crc_index++] = byte;
            // Check if all 4 CRC bytes have been received.
            if (p_parser->crc_index >= 4)
            {
                // Extract the full 32-bit CRC from the received bytes.
                p_parser->internal_frame.crc = extract_crc(p_parser->crc_buffer);
                p_parser->state = OTA_PARSE_STATE_WAIT_EOF; // Transition to wait for End of Frame.
            }
            break;

        case OTA_PARSE_STATE_WAIT_EOF:
        {
            bool success = false;
            // Check for the End of Frame (EOF) marker.
            if (byte == OTA_EOF)
            {
                // If EOF is received, the entire frame structure is complete.
                // Now, verify the data integrity by calculating the CRC of the received payload.
                uint32_t calc_crc = crc32(p_parser->internal_frame.data, p_parser->internal_frame.length);
                if (calc_crc == p_parser->internal_frame.crc)
                {
                    // If calculated CRC matches the received CRC, the frame is valid.
                    // Copy the fully parsed internal frame to the output parameter.
                    memcpy(p_out_frame, &p_parser->internal_frame, sizeof(ota_frame_t));
                    success = true; // Indicate successful frame parsing.
                }
            }
            // Always reset the parser after attempting to process a complete frame (valid or invalid EOF).
            ota_protocol_parser_init(p_parser);
            // Return true if the frame was successfully parsed and validated.
            if (success)
                return true;
            break;
        }
    }
    return false; // Return false if no complete, valid frame was parsed in this step.
}

void ota_protocol_create_response(uint8_t status, uint8_t* p_out_buffer, uint16_t* p_out_len)
{
    // Define the payload for the response frame, which is just the status byte.
    uint8_t payload[] = {status};
    // Calculate the CRC32 for the payload.
    uint32_t crc = crc32(payload, sizeof(payload));

    int i = 0;
    // Assemble the frame in the output buffer according to the protocol.
    p_out_buffer[i++] = OTA_SOF;        // Start of Frame marker.
    p_out_buffer[i++] = PACKET_RESP;    // Frame type: Response.
    // Length of the payload (low byte then high byte).
    p_out_buffer[i++] = (uint8_t)(sizeof(payload) & 0xFF);
    p_out_buffer[i++] = (uint8_t)((sizeof(payload) >> 8) & 0xFF);

    // Copy the actual payload (status byte) into the buffer.
    memcpy(&p_out_buffer[i], payload, sizeof(payload));
    i += sizeof(payload);

    // Copy the 32-bit CRC into the buffer, in little-endian format (byte by byte).
    p_out_buffer[i++] = (uint8_t)(crc & 0xFF);
    p_out_buffer[i++] = (uint8_t)((crc >> 8) & 0xFF);
    p_out_buffer[i++] = (uint8_t)((crc >> 16) & 0xFF);
    p_out_buffer[i++] = (uint8_t)((crc >> 24) & 0xFF);

    p_out_buffer[i++] = OTA_EOF;        // End of Frame marker.

    // Set the total length of the created frame.
    *p_out_len = i;
}