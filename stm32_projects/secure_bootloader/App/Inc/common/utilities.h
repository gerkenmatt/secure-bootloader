/**
 * @file utilities.h
 * @brief Utility functions for CRC and data verification.
 */
#ifndef UTILITIES_H
#define UTILITIES_H

#include <stdint.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// Public Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Calculates CRC32 checksum using standard polynomial 0x04C11DB7 (reversed).
 * Implements standard CRC32 algorithm used in Ethernet, zip, etc.
 * @param p_data Pointer to data buffer to calculate CRC over
 * @param len Length of data buffer in bytes
 * @return Calculated 32-bit CRC value
 */
uint32_t crc32(const uint8_t* p_data, uint16_t len);

/**
 * @brief Extracts a 32-bit CRC value from a byte array.
 * @param p_bytes Pointer to byte array containing CRC value
 * @return Extracted 32-bit CRC value
 */
uint32_t extract_crc(const uint8_t* p_bytes);

/**
 * @brief Verifies CRC32 checksum of data in flash memory.
 * @param address Start address of data to verify
 * @param size Size of data to verify in bytes
 * @param expected_crc Expected CRC value
 * @return true if CRC matches, false otherwise
 */
bool verify_crc(uint32_t address, uint32_t size, uint32_t expected_crc);

#endif // UTILITIES_H
