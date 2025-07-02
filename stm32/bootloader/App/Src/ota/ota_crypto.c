#include "ota_crypto.h"
#include "uart.h"

#include "mbedtls/pk.h"
#include "mbedtls/md.h"
#include "mbedtls/error.h"
#include "mbedtls/sha256.h"
#include "mbedtls/platform.h"  

// -----------------------------------------------------------------------------
// Module-Private Constants
// -----------------------------------------------------------------------------

#define PUBKEY_DER_LEN 91

// -----------------------------------------------------------------------------
// Module-Private Data (Static Globals)
// -----------------------------------------------------------------------------

// --- Public Key ---
// The public key is embedded in the bootloader to verify firmware signatures.
// For now, it is embedded directly into the bootloader binary.
// TODO: this key should be protected against tampering.
static const unsigned char pubkey_der[] = {
  0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x02,
  0x01, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x03, 0x01, 0x07, 0x03,
  0x42, 0x00, 0x04, 0x04, 0xb8, 0x40, 0x1e, 0xa6, 0x42, 0xa4, 0xf8, 0xef,
  0xeb, 0x1a, 0x1d, 0xc0, 0xe7, 0x12, 0x6f, 0x13, 0x31, 0x72, 0xfb, 0x8d,
  0x13, 0xc2, 0xd8, 0xc2, 0xc8, 0xb6, 0x99, 0x5e, 0xb8, 0xa2, 0x4c, 0x58,
  0x7c, 0x72, 0x0f, 0x4a, 0x21, 0x5b, 0x56, 0x76, 0xcf, 0xe5, 0xba, 0xd9,
  0x6d, 0x3e, 0xc0, 0x6c, 0xb3, 0xd4, 0xad, 0x48, 0xf4, 0x07, 0xa7, 0xdc,
  0x29, 0x46, 0x41, 0xef, 0x57, 0x38, 0x74
};

// -----------------------------------------------------------------------------
// Public Function Implementations
// -----------------------------------------------------------------------------

bool ota_crypto_verify_signature(const uint8_t *p_data, uint32_t data_len, const uint8_t *p_sig, uint16_t sig_len)
{
    int ret;
    uint8_t hash[32]; // Buffer to store the SHA256 hash of the data.
    mbedtls_pk_context pk_ctx; // Context for the public key.
    mbedtls_sha256_context sha_ctx; // Context for the SHA256 hash calculation.

    // 1. Compute SHA256 hash of the input data.
    mbedtls_sha256_init(&sha_ctx);
    if (mbedtls_sha256_starts(&sha_ctx, 0) != 0 ||      // Initialize SHA256 context (0 for SHA256).
        mbedtls_sha256_update(&sha_ctx, p_data, data_len) != 0 || // Process the data to be hashed.
        mbedtls_sha256_finish(&sha_ctx, hash) != 0)     // Finalize hash computation and store result.
    {
        // If any hashing step fails, free resources and return false.
        mbedtls_sha256_free(&sha_ctx);
        return false;
    }
    mbedtls_sha256_free(&sha_ctx); // Free the SHA256 context after use.

    // 2. Parse the embedded public key.
    mbedtls_pk_init(&pk_ctx); // Initialize the public key context.
    if (mbedtls_pk_parse_public_key(&pk_ctx, pubkey_der, PUBKEY_DER_LEN) != 0)
    {
        // If public key parsing fails, free resources and return false.
        mbedtls_pk_free(&pk_ctx);
        return false;
    }

    // 3. Verify the signature against the computed hash using the parsed public key.
    // MBEDTLS_MD_SHA256 specifies the hash algorithm used for signing.
    ret = mbedtls_pk_verify(&pk_ctx, MBEDTLS_MD_SHA256, hash, sizeof(hash), p_sig, sig_len);
    mbedtls_pk_free(&pk_ctx); // Free the public key context after verification.

    // Return true if verification was successful (mbedtls_pk_verify returns 0 for success).
    return (ret == 0);
}

/* This function is a platform-specific implementation of mbedtls_platform_zeroize.
 * It is required by mbedTLS for securely clearing sensitive data from memory.
 * It uses a volatile pointer to prevent the compiler from optimizing away the memory write operations,
 * ensuring that the memory is truly overwritten with zeros.
 * It does not rely on any standard library functions or function pointers.
 */
void mbedtls_platform_zeroize( void *p_buf, size_t len )
{
    // Cast the buffer to a volatile unsigned char pointer to ensure byte-by-byte zeroing.
    volatile unsigned char *p = (volatile unsigned char*) p_buf;
    // Loop through each byte and set it to 0. The 'volatile' keyword prevents optimization.
    while( len-- )
        *p++ = 0;
}