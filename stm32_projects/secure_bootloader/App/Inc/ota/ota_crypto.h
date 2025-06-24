/**
 * @file ota_crypto.h
 * @brief OTA firmware signature verification interface.
 */
#ifndef OTA_CRYPTO_H
#define OTA_CRYPTO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// -----------------------------------------------------------------------------
// Public Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Verifies the ECDSA-SHA256 signature of a firmware image.
 * @param p_data Pointer to the firmware image in memory.
 * @param data_len Length of the firmware image.
 * @param p_sig Pointer to the received signature.
 * @param sig_len Length of the signature.
 * @return true if the signature is valid, false otherwise.
 */
bool ota_crypto_verify_signature(const uint8_t* p_data, uint32_t data_len, const uint8_t* p_sig, uint16_t sig_len);

/**
 * @brief Zeroizes a buffer in memory (mbedTLS compatibility).
 * @param p_buf Pointer to buffer
 * @param len Length of buffer
 */
void mbedtls_platform_zeroize(void *p_buf, size_t len);

#endif // OTA_CRYPTO_H