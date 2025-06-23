/**
 * @file ota_crypto.h
 * @brief OTA firmware signature verification interface.
 */
#ifndef OTA_CRYPTO_H
#define OTA_CRYPTO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Verifies the ECDSA-SHA256 signature of a firmware image.
 * @param fw_data Pointer to the firmware image in memory.
 * @param fw_len Length of the firmware image.
 * @param signature Pointer to the received signature.
 * @param sig_len Length of the signature.
 * @return true if the signature is valid, false otherwise.
 */
bool ota_crypto_verify_signature(const uint8_t* fw_data, uint32_t fw_len, const uint8_t* signature, uint16_t sig_len);

/**
 * @brief Zeroizes a buffer in memory (mbedTLS compatibility).
 * @param buf Pointer to buffer
 * @param len Length of buffer
 */
void mbedtls_platform_zeroize(void *buf, size_t len);

#endif // OTA_CRYPTO_H