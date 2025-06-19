#include "utilities.h"
#include "stm32f7xx.h"
#include "ota.h"
#include "uart.h"   
#include "flash.h"

uint32_t crc32(const uint8_t* data, uint16_t len) {
    // Initialize CRC to all Fs
    uint32_t crc = 0xFFFFFFFF;

    // Process each byte of data
    for (uint16_t i = 0; i < len; i++) {
        // XOR next byte into CRC
        crc ^= data[i];

        // Process each bit
        for (int j = 0; j < 8; j++) {
            // If LSB is 1, shift right and XOR with polynomial
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320; // Reversed polynomial
            else
                crc >>= 1; // Just shift if LSB is 0
        }
    }

    // Final complement gives CRC32 value
    return ~crc;
}

uint32_t extract_crc(const uint8_t *bytes) {
    return bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (bytes[3] << 24);
}

bool verify_crc(uint32_t address, uint32_t size, uint32_t expected_crc) {
    uint32_t actual_crc = crc32((const uint8_t*)address, size);
    return (actual_crc == expected_crc);
}