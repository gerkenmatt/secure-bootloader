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

    // Set the parser to its initial, known state.
    p_parser->state = OTA_PARSE_STATE_WAIT_SOF;
    p_parser->data_index = 0;
    p_parser->crc_index = 0;
    memset(&p_parser->internal_frame, 0, sizeof(ota_frame_t));
}

bool ota_protocol_parse_byte(ota_parser_t* p_parser, uint8_t byte, ota_frame_t* p_out_frame) 
{
    switch (p_parser->state) 
    {
        case OTA_PARSE_STATE_WAIT_SOF:
            if (byte == OTA_SOF) 
                p_parser->state = OTA_PARSE_STATE_GET_TYPE;
            break;

        case OTA_PARSE_STATE_GET_TYPE:
            p_parser->internal_frame.type = byte;
            p_parser->state = OTA_PARSE_STATE_GET_LEN_LO;
            break;

        case OTA_PARSE_STATE_GET_LEN_LO:
            p_parser->internal_frame.length = byte;
            p_parser->state = OTA_PARSE_STATE_GET_LEN_HI;
            break;

        case OTA_PARSE_STATE_GET_LEN_HI:
            p_parser->internal_frame.length |= ((uint16_t)byte << 8);
            if (p_parser->internal_frame.length > OTA_MAX_DATA) 
            {
                // Invalid length, reset the parser to wait for the next frame
                ota_protocol_parser_init(p_parser);
            } 
            else 
            {
                p_parser->data_index = 0;
                // If there's no data payload, skip straight to getting the CRC
                p_parser->state = (p_parser->internal_frame.length > 0) ? OTA_PARSE_STATE_GET_DATA : OTA_PARSE_STATE_GET_CRC;
            }
            break;

        case OTA_PARSE_STATE_GET_DATA:
            p_parser->internal_frame.data[p_parser->data_index++] = byte;
            if (p_parser->data_index >= p_parser->internal_frame.length) 
            {
                p_parser->crc_index = 0;
                p_parser->state = OTA_PARSE_STATE_GET_CRC;
            }
            break;

        case OTA_PARSE_STATE_GET_CRC:
            p_parser->crc_buffer[p_parser->crc_index++] = byte;
            if (p_parser->crc_index >= 4) 
            {
                p_parser->internal_frame.crc = extract_crc(p_parser->crc_buffer);
                p_parser->state = OTA_PARSE_STATE_WAIT_EOF;
            }
            break;

        case OTA_PARSE_STATE_WAIT_EOF: 
        {
            bool success = false;
            if (byte == OTA_EOF) 
            {
                // We have a complete frame, now verify its integrity
                uint32_t calc_crc = crc32(p_parser->internal_frame.data, p_parser->internal_frame.length);
                if (calc_crc == p_parser->internal_frame.crc) 
                {
                    // SUCCESS! The frame is valid. Copy it to the output parameter.
                    memcpy(p_out_frame, &p_parser->internal_frame, sizeof(ota_frame_t));
                    success = true;
                }
            }
            ota_protocol_parser_init(p_parser);
            if (success) 
                return true;
            break;
        }
    }
    return false;
}


void ota_protocol_create_response(uint8_t status, uint8_t* p_out_buffer, uint16_t* p_out_len) 
{
    uint8_t payload[] = {status};
    uint32_t crc = crc32(payload, sizeof(payload));

    int i = 0;
    p_out_buffer[i++] = OTA_SOF;
    p_out_buffer[i++] = PACKET_RESP;
    p_out_buffer[i++] = (uint8_t)(sizeof(payload) & 0xFF);     // len_lo
    p_out_buffer[i++] = (uint8_t)((sizeof(payload) >> 8) & 0xFF); // len_hi
    
    // Copy payload
    memcpy(&p_out_buffer[i], payload, sizeof(payload));
    i += sizeof(payload);

    // Copy CRC
    p_out_buffer[i++] = (uint8_t)(crc & 0xFF);
    p_out_buffer[i++] = (uint8_t)((crc >> 8) & 0xFF);
    p_out_buffer[i++] = (uint8_t)((crc >> 16) & 0xFF);
    p_out_buffer[i++] = (uint8_t)((crc >> 24) & 0xFF);
    
    p_out_buffer[i++] = OTA_EOF;

    *p_out_len = i;
}