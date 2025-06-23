#ifndef OTA_PROTOCOL_H
#define OTA_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// --- Protocol Constants (Moved from ota.h) ---
/// OTA packet types
#define PACKET_CMD        0x01
#define PACKET_HEADER     0x02
#define PACKET_DATA       0x03
#define PACKET_RESP       0x04
#define PACKET_SIG        0x05

/// OTA command types
#define CMD_START         0xA0
#define CMD_END           0xA1

/// OTA response types
#define RESP_ACK          0xAB
#define RESP_NACK         0xCD

/// OTA frame markers
#define OTA_SOF           0xA5
#define OTA_EOF           0xB6

#define OTA_MAX_DATA        256

// --- Protocol Data Structures ---
typedef struct {
    uint8_t  type;
    uint16_t length;
    uint8_t  data[OTA_MAX_DATA];
    uint32_t crc;
} ota_frame_t;

// --- Parser State Machine Context ---
typedef enum {
    OTA_PARSE_STATE_WAIT_SOF,
    OTA_PARSE_STATE_GET_TYPE,
    OTA_PARSE_STATE_GET_LEN_LO,
    OTA_PARSE_STATE_GET_LEN_HI,
    OTA_PARSE_STATE_GET_DATA,
    OTA_PARSE_STATE_GET_CRC,
    OTA_PARSE_STATE_WAIT_EOF
} ota_parse_state_t;

typedef struct {
    ota_parse_state_t state;
    ota_frame_t       internal_frame;
    uint16_t          data_index;
    uint8_t           crc_buffer[4];
    uint8_t           crc_index;
} ota_parser_t;

// --- Public Functions ---

/** @brief Initializes the OTA protocol parser state. */
void ota_protocol_parser_init(ota_parser_t* parser);

/**
 * @brief Processes a single byte from the input stream.
 * @param parser The parser context.
 * @param byte The byte to process.
 * @param out_frame If a complete frame is received, it is copied here.
 * @return true if a complete, valid frame was parsed, false otherwise.
 */
bool ota_protocol_parse_byte(ota_parser_t* parser, uint8_t byte, ota_frame_t* out_frame);

/** 
 * @brief Creates a serialized response frame
 * @param status The response status (RESP_ACK or RESP_NACK)
 * @param out_buffer Pointer to the output buffer where the response will be written.
 * @param out_len Pointer to the length of the output buffer. Will be set to the length of the response. 
 */
void ota_protocol_create_response(uint8_t status, uint8_t* out_buffer, uint16_t* out_len);

#endif // OTA_PROTOCOL_H