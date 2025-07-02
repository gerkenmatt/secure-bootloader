#include "utilities.h"
#include "stm32f7xx.h"

// -----------------------------------------------------------------------------
// Public Function Implementations
// -----------------------------------------------------------------------------

//TODO: use the built-in CRC peripheral
uint32_t crc32(const uint8_t* p_data, size_t len)
{
    // Initialize the CRC register to all ones (0xFFFFFFFF) as per the CRC-32 standard.
    uint32_t crc = 0xFFFFFFFF;

    // Process each byte of the input data.
    for (uint16_t i = 0; i < len; i++)
    {
        // XOR the current data byte with the least significant byte of the current CRC value.
        crc ^= p_data[i];

        // Process each bit of the current byte (8 bits).
        for (int j = 0; j < 8; j++)
        {
            // Check if the least significant bit of the CRC is 1.
            if (crc & 1)
                // If LSB is 1, shift CRC right and XOR with the reversed polynomial (0xEDB88320).
                // This polynomial is specific to CRC-32-IEEE 802.3.
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                // If LSB is 0, just shift CRC right by one bit.
                crc >>= 1;
        }
    }

    // The final CRC-32 value is the bitwise complement of the accumulated CRC.
    return ~crc;
}

uint32_t extract_crc(const uint8_t *bytes)
{
    // Reconstruct a 32-bit CRC value from an array of 4 bytes.
    // This assumes the CRC is stored in little-endian format (LSB first).
    return bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (bytes[3] << 24);
}

bool verify_crc(uint32_t address, uint32_t size, uint32_t expected_crc)
{
    // Calculate the CRC of the data block located at 'address' with 'size' bytes.
    uint32_t actual_crc = crc32((const uint8_t*)address, size);
    // Compare the calculated CRC with the provided expected CRC.
    return (actual_crc == expected_crc);
}