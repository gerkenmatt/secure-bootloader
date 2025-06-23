#include "ota_protocol.h"
#include <string.h> 
#include "utilities.h"


void ota_protocol_parser_init(ota_parser_t* parser) 
{
    if (!parser) 
    {
        return;
    }
    // Set the parser to its initial, known state.
    parser->state = OTA_PARSE_STATE_WAIT_SOF;
    parser->data_index = 0;
    parser->crc_index = 0;
    memset(&parser->internal_frame, 0, sizeof(ota_frame_t));
}

bool ota_protocol_parse_byte(ota_parser_t* parser, uint8_t byte, ota_frame_t* out_frame) 
{

    switch (parser->state) 
    {
        case OTA_PARSE_STATE_WAIT_SOF:
            if (byte == OTA_SOF) 
            {
                parser->state = OTA_PARSE_STATE_GET_TYPE;
            }
            break;

        case OTA_PARSE_STATE_GET_TYPE:
            parser->internal_frame.type = byte;
            parser->state = OTA_PARSE_STATE_GET_LEN_LO;
            break;

        case OTA_PARSE_STATE_GET_LEN_LO:
            parser->internal_frame.length = byte;
            parser->state = OTA_PARSE_STATE_GET_LEN_HI;
            break;

        case OTA_PARSE_STATE_GET_LEN_HI:
            parser->internal_frame.length |= ((uint16_t)byte << 8);
            if (parser->internal_frame.length > OTA_MAX_DATA) 
            {
                // Invalid length, reset the parser to wait for the next frame
                ota_protocol_parser_init(parser);
            } 
            else 
            {
                parser->data_index = 0;
                // If there's no data payload, skip straight to getting the CRC
                parser->state = (parser->internal_frame.length > 0) ? OTA_PARSE_STATE_GET_DATA : OTA_PARSE_STATE_GET_CRC;
            }
            break;

        case OTA_PARSE_STATE_GET_DATA:
            parser->internal_frame.data[parser->data_index++] = byte;
            if (parser->data_index >= parser->internal_frame.length) 
            {
                parser->crc_index = 0;
                parser->state = OTA_PARSE_STATE_GET_CRC;
            }
            break;

        case OTA_PARSE_STATE_GET_CRC:
            parser->crc_buffer[parser->crc_index++] = byte;
            if (parser->crc_index >= 4) 
            {
                parser->internal_frame.crc = extract_crc(parser->crc_buffer);
                parser->state = OTA_PARSE_STATE_WAIT_EOF;
            }
            break;

        case OTA_PARSE_STATE_WAIT_EOF: 
        {
            bool success = false;
            if (byte == OTA_EOF) 
            {
                // We have a complete frame, now verify its integrity
                uint32_t calc_crc = crc32(parser->internal_frame.data, parser->internal_frame.length);
                if (calc_crc == parser->internal_frame.crc) 
                {
                    // SUCCESS! The frame is valid.
                    // Copy it to the output parameter.
                    memcpy(out_frame, &parser->internal_frame, sizeof(ota_frame_t));
                    success = true;
                }
                // NOTE: If the CRC or EOF is bad, 'success' remains false,
                // and we just fall through to re-initialize the parser.
            }

            // No matter if it was a success or failure, the frame is complete,
            // so we must reset the parser for the next one.
            ota_protocol_parser_init(parser);

            // IMPORTANT: We only return true on the exact byte that successfully
            // completes a valid frame.
            if (success) 
            {
                return true;
            }
            break;
        }
    }

    // If we haven't returned true by now, it means this byte did not
    // complete a frame, so we return false.
    return false;
}


void ota_protocol_create_response(uint8_t status, uint8_t* out_buffer, uint16_t* out_len) 
{
    uint8_t payload[] = {status};
    uint32_t crc = crc32(payload, sizeof(payload));

    int i = 0;
    out_buffer[i++] = OTA_SOF;
    out_buffer[i++] = PACKET_RESP;
    out_buffer[i++] = (uint8_t)(sizeof(payload) & 0xFF);     // len_lo
    out_buffer[i++] = (uint8_t)((sizeof(payload) >> 8) & 0xFF); // len_hi
    
    // Copy payload
    memcpy(&out_buffer[i], payload, sizeof(payload));
    i += sizeof(payload);

    // Copy CRC
    out_buffer[i++] = (uint8_t)(crc & 0xFF);
    out_buffer[i++] = (uint8_t)((crc >> 8) & 0xFF);
    out_buffer[i++] = (uint8_t)((crc >> 16) & 0xFF);
    out_buffer[i++] = (uint8_t)((crc >> 24) & 0xFF);
    
    out_buffer[i++] = OTA_EOF;

    *out_len = i;
}