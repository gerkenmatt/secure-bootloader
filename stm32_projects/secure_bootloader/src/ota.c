#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ota.h"
#include "stm32f7xx.h"
#include "uart.h"
#include "utilities.h"
#include "flash.h"
#include "bootloader.h"
#include "mbedtls/platform.h"
#include "mbedtls/asn1.h"
#include "mbedtls/ecp.h"

#define CHUNK_SIZE 256
#define PUBKEY_DER_LEN 91

// OTA session state
static ota_header_info_t ota_header;
static uint32_t flash_write_addr = SLOT1_ADDR;

// storage for the incoming signature:
static uint8_t  ota_signature[SIG_MAX_LEN];
static uint16_t ota_sig_len;

static mbedtls_pk_context pk;  // Avoid large stack allocation
static uint8_t temp[64] = {0};
__attribute__((aligned(4))) static uint8_t aligned_buf[64];


// static const char firmware_pub_pem[] =
// "-----BEGIN PUBLIC KEY-----\n"
// "MFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEBLhAHqZCpPjv6xodwOcSbxMxcvuN\n"
// "E8LYwsi2mV64okxYfHIPSiFbVnbP5brZbT7AbLPUrUj0B6fcKUZB71c4dA==\n"
// "-----END PUBLIC KEY-----\n";

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
// static unsigned int pubkey_der_len = 91;

static int normalize_ecp_point(const mbedtls_ecp_group *grp, mbedtls_ecp_point *pt) {
    int ret;
    mbedtls_mpi Zi, Zi2, Zi3;

    mbedtls_mpi_init(&Zi);
    mbedtls_mpi_init(&Zi2);
    mbedtls_mpi_init(&Zi3);

    if (mbedtls_mpi_cmp_int(&pt->Z, 1) == 0) {
        ret = 0;
        goto cleanup;
    }

    MBEDTLS_MPI_CHK(mbedtls_mpi_inv_mod(&Zi, &pt->Z, &grp->P));       // Zi  = 1/Z
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mpi(&Zi2, &Zi, &Zi));             // Zi2 = 1/Z^2
    MBEDTLS_MPI_CHK(mbedtls_mpi_mod_mpi(&Zi2, &Zi2, &grp->P));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mpi(&Zi3, &Zi2, &Zi));            // Zi3 = 1/Z^3
    MBEDTLS_MPI_CHK(mbedtls_mpi_mod_mpi(&Zi3, &Zi3, &grp->P));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mpi(&pt->X, &pt->X, &Zi2));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mod_mpi(&pt->X, &pt->X, &grp->P));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mpi(&pt->Y, &pt->Y, &Zi3));
    MBEDTLS_MPI_CHK(mbedtls_mpi_mod_mpi(&pt->Y, &pt->Y, &grp->P));
    MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&pt->Z, 1));

cleanup:
    mbedtls_mpi_free(&Zi);
    mbedtls_mpi_free(&Zi2);
    mbedtls_mpi_free(&Zi3);
    return ret;
}

void handle_ota_session(void) {
    log("Waiting for OTA packets...\r\n");

    while (1) {
        ota_frame_t frame;

        // Try to receive a valid frame
        if (ota_receive_frame(&frame)) {
            // Process frame based on its type
            switch (frame.type) {
                case PACKET_CMD:    
                    handle_ota_command(&frame); // Handle control commands
                    break;
                case PACKET_HEADER: 
                    handle_ota_header(&frame);  // Process firmware metadata
                    break;
                case PACKET_DATA:   
                    handle_ota_data(&frame);    // Handle firmware data chunks
                    break;
                case PACKET_SIG:
                    handle_ota_signature(&frame); // Handle firmware signature
                    break;
                default:
                    // Invalid/unknown packet type
                    ota_send_response(RESP_NACK);
                    log("Unknown packet type\r\n");
                    break;
            }
        } 
        else {
            // Frame validation failed
            ota_send_response(RESP_NACK);
            log("Invalid frame\r\n");
        }
    }
}

bool ota_receive_frame(ota_frame_t* frame) {
    // Validate input parameter
    if (!frame) {
        ota_send_response(RESP_NACK);
        log("Frame pointer is NULL\r\n");
        return false;
    }

    // Wait for start of frame marker
    uint8_t byte = 0;
    while (byte != OTA_SOF){
        byte = usart_getc();
    }

    // Read frame type and length
    frame->type = usart_getc();
    uint8_t len_lo = usart_getc();
    uint8_t len_hi = usart_getc();
    frame->length = (len_hi << 8) | len_lo;

    // Validate frame length
    if (frame->length > OTA_MAX_DATA) {
        ota_send_response(RESP_NACK);
        log("Frame too large\r\n");
        return false;
    }

    // Read frame payload data
    for (uint16_t i = 0; i < frame->length; i++) {
        frame->data[i] = usart_getc();
    }

    // Read and extract 32-bit CRC
    uint8_t crc_bytes[4];
    for (int i = 0; i < 4; i++) crc_bytes[i] = usart_getc();
    frame->crc = extract_crc(crc_bytes);

    // Verify end of frame marker
    if (usart_getc() != OTA_EOF) {
        ota_send_response(RESP_NACK);
        log("Invalid EOF\r\n");
        return false;
    }

    // Validate CRC
    uint32_t calc_crc = crc32(frame->data, frame->length);
    if (calc_crc != frame->crc) {
        ota_send_response(RESP_NACK);
        log("CRC mismatch: calc=0x");
        print_uint32_hex(calc_crc);
        log(" frame=0x"); 
        print_uint32_hex(frame->crc);
        log("\r\n");
        return false;
    }

    return true;
}

void handle_ota_command(const ota_frame_t* frame) {
    // Validate frame has at least 1 byte for command
    if (frame->length < 1) return;

    switch (frame->data[0]) {
        case CMD_START:
            //TODO: should I put each case in its own function?
            // Clear any previous flash error flags
            FLASH->SR |= FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_OPERR | 
                         FLASH_SR_PGPERR | FLASH_SR_ERSERR;

        
            GPIOB->ODR |= (1UL << 0); //set_green
            flash_write_addr = SLOT1_ADDR;
            
            // Unlock flash if locked
            unlock_flash();

            // Configure flash access control and program size
            FLASH->ACR |= (1 << 8) | (1 << 9); // Enable instruction and data cache
            FLASH->CR |= FLASH_CR_PSIZE_1;      // Set program size to 32-bit

            // Erase slot1 sector (sector for new firmware)
            if (!erase_flash_sectors(SLOT1_SECTOR, SLOT1_SECTOR, flash_write_addr, ota_header.fw_size)) {
                ota_send_response(RESP_NACK);
                log("Flash erase failed\r\n");
                return;
            }

            // Erase successful, send ACK
            ota_send_response(RESP_ACK);

            break;

        case CMD_END:
            // log("Received OTA END\r\n");

            log("Verifying signature...\r\n");

            // verify the signature on the new image in flash
            if (!verify_signature(
                    (uint8_t*)SLOT1_ADDR,
                    ota_header.fw_size,
                    ota_signature,
                    ota_sig_len))
            {
                ota_send_response(RESP_NACK);
                log("Signature verification failed\r\n");
                return;
            }


            log("Signature verified\r\n");

            bootloader_config_t cfg = *read_boot_config();  // Read current config

            // Update slot 1 metadata
            cfg.slot[1].fw_size     = ota_header.fw_size;
            cfg.slot[1].fw_crc      = ota_header.fw_crc;
            cfg.slot[1].is_valid    = 1;
            cfg.slot[1].should_run  = 1;

            // Clear run flag from slot 0 (optional)
            cfg.slot[0].should_run = 0;

            // Keep magic + reboot_cause (or set to OTA_REQUEST if used)
            cfg.magic = BOOT_CONFIG_MAGIC;
            cfg.reboot_cause = 1;  // e.g., OTA_REQUEST (if defined)

            log("Writing boot config...\r\n");

            if (!write_boot_config(&cfg)) {
                ota_send_response(RESP_NACK);
                log("Failed to write boot config\r\n");
                return;
            }

            log("Boot config written\r\n");

            ota_send_response(RESP_ACK);

            // OTA update complete, lock flash and reboot
            FLASH->CR |= FLASH_CR_LOCK;
            log("Flash locked\r\n");
            log("Rebooting...\r\n");
            SCB_CleanDCache();        // Clean data cache
            NVIC_SystemReset();       // Reset system
            break;

        default:
            // Unknown command received
            ota_send_response(RESP_NACK);
            log("Unknown CMD: "); print_uint32_hex(frame->data[0]); log("\r\n");
            break;
    }
}


void ota_send_response(uint8_t status) {
    uint8_t frame[] = {
        OTA_SOF, PACKET_RESP, 0x01, 0x00, status,  // Header + status
        0x00, 0x00, 0x00, 0x00, OTA_EOF            // Padding + EOF
    };
    for (int i = 0; i < sizeof(frame); i++) usart_putc(frame[i]);
}

void handle_ota_header(const ota_frame_t* frame) {
    // Verify header length matches expected size
    if (frame->length != sizeof(ota_header_info_t)) {
        ota_send_response(RESP_NACK);
        log("Invalid header length\r\n");
        return;
    }

    // Copy header data to global struct
    memcpy(&ota_header, frame->data, sizeof(ota_header_info_t));
    ota_send_response(RESP_ACK);
}


void handle_ota_data(const ota_frame_t* frame) {
    // Validate data length is within bounds
    if (frame->length == 0 || frame->length > OTA_MAX_DATA) {
        ota_send_response(RESP_NACK);
        log("Invalid data length\r\n");
        return;
    }

    // Process data in 32-bit word chunks
    for (uint16_t i = 0; i < frame->length; i += 4) {
        uint32_t word = 0xFFFFFFFF;
        // Handle partial words at end of frame
        uint16_t len = (frame->length - i >= 4) ? 4 : (frame->length - i);
        memcpy(&word, &frame->data[i], len);

        // Program word to flash
        program_flash_word(flash_write_addr, word);

        // Verify written data
        //TODO: we are failing here now
        uint32_t verify = *(volatile uint32_t *)flash_write_addr;
        if (verify != word) {
            ota_send_response(RESP_NACK);
            log("Flash verify failed\r\n");
            log("Expected: "); print_uint32_hex(word);
            log("\r\nReadback: "); print_uint32_hex(verify);
            log("\r\n");
            return;
        }

        flash_write_addr += 4;
    }

    ota_send_response(RESP_ACK);
}

void handle_ota_signature(const ota_frame_t* frame) {
    // sanity check
    if (frame->length > SIG_MAX_LEN) {
        ota_send_response(RESP_NACK);
        log("Signature too large\r\n");
        return;
    }
    // copy it into RAM
    memcpy(ota_signature, frame->data, frame->length);
    ota_sig_len = frame->length;
    ota_send_response(RESP_ACK);
}
bool verify_signature(const uint8_t *data,
                      uint32_t        data_len,
                      const uint8_t  *sig,
                      uint16_t        sig_len)
{
    int ret;
    uint8_t hash[32];
    mbedtls_pk_context pk;
    mbedtls_mpi r, s;
    unsigned char *p = (unsigned char *)sig;
    unsigned char *end = p + sig_len;
    size_t len;
    unsigned char tmp[32];

    SCB_DisableICache();
    SCB_DisableDCache();
    mbedtls_platform_set_calloc_free(calloc, free);

    log("Hashing firmware image...\r\n");
    mbedtls_sha256_context sha_ctx;
    mbedtls_sha256_init(&sha_ctx);
    mbedtls_sha256_starts_ret(&sha_ctx, 0);
    mbedtls_sha256_update_ret(&sha_ctx, data, data_len);
    mbedtls_sha256_finish_ret(&sha_ctx, hash);
    mbedtls_sha256_free(&sha_ctx);

    log("Hash: ");
    for (int i = 0; i < 32; i++) { print_uint8_hex(hash[i]); log(" "); }
    log("\r\n");

    mbedtls_pk_init(&pk);
    ret = mbedtls_pk_parse_public_key(&pk, pubkey_der, PUBKEY_DER_LEN);
    if (ret != 0) {
        char buf[100];
        mbedtls_strerror(ret, buf, sizeof(buf));
        log("DER parse failed: "); log(buf); log("\r\n");
        return false;
    }

    if (!mbedtls_pk_can_do(&pk, MBEDTLS_PK_ECKEY)) {
        log("Parsed key is not EC key\r\n");
        return false;
    }

    mbedtls_ecp_keypair *ec = mbedtls_pk_ec(pk);
    log("Curve ID: "); print_uint32_hex(ec->grp.id); log("\r\n");

    mbedtls_mpi_write_binary(&ec->Q.X, tmp, 32);
    log("Q.X = "); for (int i = 0; i < 32; i++) { print_uint8_hex(tmp[i]); log(" "); } log("\r\n");
    mbedtls_mpi_write_binary(&ec->Q.Y, tmp, 32);
    log("Q.Y = "); for (int i = 0; i < 32; i++) { print_uint8_hex(tmp[i]); log(" "); } log("\r\n");
    mbedtls_mpi_write_binary(&ec->Q.Z, tmp, 32);
    log("Q.Z = "); for (int i = 0; i < 32; i++) { print_uint8_hex(tmp[i]); log(" "); } log("\r\n");

    // Force Q.Z = 1
    mbedtls_mpi_lset(&ec->Q.Z, 1);
    mbedtls_mpi_lset(&ec->grp.G.Z, 1);

    mbedtls_mpi_write_binary(&ec->grp.P, tmp, 32);
    log("grp.P = "); for (int i = 0; i < 32; i++) { print_uint8_hex(tmp[i]); log(" "); } log("\r\n");
    mbedtls_mpi_write_binary(&ec->grp.A, tmp, 32);
    log("grp.A = "); for (int i = 0; i < 32; i++) { print_uint8_hex(tmp[i]); log(" "); } log("\r\n");
    mbedtls_mpi_write_binary(&ec->grp.B, tmp, 32);
    log("grp.B = "); for (int i = 0; i < 32; i++) { print_uint8_hex(tmp[i]); log(" "); } log("\r\n");

    mbedtls_mpi_write_binary(&ec->grp.N, tmp, 32);
    log("grp.N = "); for (int i = 0; i < 32; i++) { print_uint8_hex(tmp[i]); log(" "); } log("\r\n");

    mbedtls_mpi_write_binary(&ec->grp.G.X, tmp, 32);
    log("G.X = "); for (int i = 0; i < 32; i++) { print_uint8_hex(tmp[i]); log(" "); } log("\r\n");

    mbedtls_mpi_write_binary(&ec->grp.G.Y, tmp, 32);
    log("G.Y = "); for (int i = 0; i < 32; i++) { print_uint8_hex(tmp[i]); log(" "); } log("\r\n");

    mbedtls_mpi_write_binary(&ec->grp.G.Z, tmp, 32);
    log("G.Z = "); for (int i = 0; i < 32; i++) { print_uint8_hex(tmp[i]); log(" "); } log("\r\n");

    ret = mbedtls_ecp_check_pubkey(&ec->grp, &ec->Q);
    if (ret != 0) {
        char errbuf[100];
        mbedtls_strerror(ret, errbuf, sizeof(errbuf));
        log("check_pubkey failed: "); log(errbuf); log("\r\n");
        return false;
    }

    mbedtls_mpi_init(&r); mbedtls_mpi_init(&s);
    ret = mbedtls_asn1_get_tag(&p, end, &len, MBEDTLS_ASN1_CONSTRUCTED | MBEDTLS_ASN1_SEQUENCE);
    if (ret != 0) { log("ASN.1 tag read failed\r\n"); return false; }

    ret = mbedtls_asn1_get_mpi(&p, end, &r);
    if (ret != 0) { log("ASN.1 r decode failed\r\n"); return false; }
    mbedtls_mpi_write_binary(&r, tmp, 32);
    log("r = "); for (int i = 0; i < 32; i++) { print_uint8_hex(tmp[i]); log(" "); } log("\r\n");

    ret = mbedtls_asn1_get_mpi(&p, end, &s);
    if (ret != 0) { log("ASN.1 s decode failed\r\n"); return false; }
    mbedtls_mpi_write_binary(&s, tmp, 32);
    log("s = "); for (int i = 0; i < 32; i++) { print_uint8_hex(tmp[i]); log(" "); } log("\r\n");

    // Check r/s range
    if (mbedtls_mpi_cmp_int(&r, 1) < 0 || mbedtls_mpi_cmp_mpi(&r, &ec->grp.N) >= 0)
        log("⚠️  r not in valid range [1, N-1]\r\n");
    if (mbedtls_mpi_cmp_int(&s, 1) < 0 || mbedtls_mpi_cmp_mpi(&s, &ec->grp.N) >= 0)
        log("⚠️  s not in valid range [1, N-1]\r\n");

    // Check Q is not point at infinity
    if (mbedtls_ecp_is_zero(&ec->Q))
        log("⚠️  Q is the point at infinity!\r\n");

    // 1. Ensure affine
    if (mbedtls_mpi_cmp_int(&ec->Q.Z, 1) != 0) {
        log("❌ Q.Z != 1\r\n");
    }

    // 2. Check field bounds
    if (mbedtls_mpi_cmp_int(&ec->Q.X, 0) < 0 || mbedtls_mpi_cmp_mpi(&ec->Q.X, &ec->grp.P) >= 0) {
        log("❌ Q.X not in field\r\n");
    }
    if (mbedtls_mpi_cmp_int(&ec->Q.Y, 0) < 0 || mbedtls_mpi_cmp_mpi(&ec->Q.Y, &ec->grp.P) >= 0) {
        log("❌ Q.Y not in field\r\n");
    }


    log("Verifying with mbedtls_ecdsa_verify...\r\n");
    // ecp_normalize_jac( &ec->grp, &ec->grp.G );
    // normalize_ecp_point( &ec->grp, &ec->grp.G );
    // mbedtls_mpi_lset(&ec->grp.G.Z, 1);
    ret = mbedtls_ecdsa_verify(&ec->grp, hash, 32, &ec->Q, &r, &s);
    if (ret != 0) {
        char errbuf[100];
        mbedtls_strerror(ret, errbuf, sizeof(errbuf));
        log("ecdsa_verify failed: "); log(errbuf); log("\r\n");
    } else {
        log("✅ ECDSA verification succeeded\r\n");
    }

    mbedtls_mpi_free(&r); mbedtls_mpi_free(&s);
    mbedtls_pk_free(&pk);
    return (ret == 0);
}



// bool verify_signature(const uint8_t *data,
//                       uint32_t        data_len,
//                       const uint8_t  *sig,
//                       uint16_t        sig_len)
// {
//     int ret;
//     uint8_t hash[32];
//     // mbedtls_pk_context pk;

//     SCB_DisableICache();
//     SCB_DisableDCache();
//     mbedtls_platform_set_calloc_free(calloc, free);


//     // 1) Hash the firmware image with SHA-256
//     log("Hashing firmware image...\r\n");
//     mbedtls_sha256_context sha_ctx;
//     mbedtls_sha256_init(&sha_ctx);
//     mbedtls_sha256_starts_ret(&sha_ctx, 0);
//     mbedtls_sha256_update_ret(&sha_ctx, data, data_len);
//     mbedtls_sha256_finish_ret(&sha_ctx, hash);
//     mbedtls_sha256_free(&sha_ctx);

//     log("Hash: ");
//     for (int i = 0; i < 32; i++) {
//         print_uint8_hex(hash[i]);
//         log(" ");
//     }
//     log("\r\n");

//     log("DER key length: ");
//     print_uint32_hex(PUBKEY_DER_LEN); log("\r\n");

//     for (int i = 0; i < PUBKEY_DER_LEN; i++) {
//         print_uint8_hex(pubkey_der[i]); //log(" ");
//     }
//     log("\r\n");

//     //2) Parse the public key
//     log("Parsing public key...\r\n");
//     mbedtls_pk_init(&pk);
//     ret = mbedtls_pk_parse_public_key(&pk,
//                                     pubkey_der,
//                                     PUBKEY_DER_LEN);
//     if (ret != 0) {
//         char buf[100];
//         mbedtls_strerror(ret, buf, sizeof(buf));
//         log("DER parse failed: "); log(buf); log("\r\n");
//         return false;
//     }



//     // 3) Print Signature
//     log("Signature: ");
//     for (int i = 0; i < sig_len; i++) {
//         print_uint8_hex(sig[i]);
//         log(" ");
//     }
//     log("\r\n");
//     log("sig_len: "); print_uint32_hex(sig_len); log("\r\n");

//     log("hash size: "); print_uint32_hex(sizeof(hash)); log("\r\n");


//     // After parsing the key
//     size_t len;
//     // 1) Extract the EC keypair internals
//     mbedtls_ecp_keypair *ec = mbedtls_pk_ec(pk);

//     // 2) Print the full 32-byte X/Y, big-endian
//     unsigned char tmp[32];
//     mbedtls_mpi_write_binary(&ec->Q.X, tmp, 32);
//     log("Q.X = "); for(int i=0;i<32;i++){ print_uint8_hex(tmp[i]); log(" "); } log("\r\n");
//     mbedtls_mpi_write_binary(&ec->Q.Y, tmp, 32);
//     log("Q.Y = "); for(int i=0;i<32;i++){ print_uint8_hex(tmp[i]); log(" "); } log("\r\n");

//     // 3) ASN.1-decode your DER signature
//     unsigned char *p = sig, *end = sig + sig_len;
//     mbedtls_mpi r, s;
//     mbedtls_mpi_init(&r); mbedtls_mpi_init(&s);
//     mbedtls_asn1_get_tag(&p, end, &len, MBEDTLS_ASN1_SEQUENCE|MBEDTLS_ASN1_CONSTRUCTED);
//     mbedtls_asn1_get_mpi(&p, end, &r);
//     mbedtls_asn1_get_mpi(&p, end, &s);

//     // 4) Print r and s
//     mbedtls_mpi_write_binary(&r, tmp, 32);
//     log("r = "); for(int i=0;i<32;i++){ print_uint8_hex(tmp[i]); log(" "); } log("\r\n");
//     mbedtls_mpi_write_binary(&s, tmp, 32);
//     log("s = "); for(int i=0;i<32;i++){ print_uint8_hex(tmp[i]); log(" "); } log("\r\n");

//     // ret = mbedtls_ecdsa_verify(
//     //     &ec->grp,
//     //     hash,  
//     //     32,
//     //     &ec->Q,
//     //     &r,      
//     //     &s      
//     // );
//     // if( ret != 0 ) {
//     //     char errbuf[100];
//     //     mbedtls_strerror(ret, errbuf, sizeof(errbuf));
//     //     log("verify failed: ");
//     //     log(errbuf); 
//     //     log("\r\n");
//     // }
    
//     // 4) Verify
//     log("Verifying signature...\r\n");
//     ret = mbedtls_pk_verify(&pk,
//                             MBEDTLS_MD_SHA256,
//                             hash, sizeof(hash),
//                             sig, sig_len);
//     if (ret != 0) {
//         log("Signature verify failed, mbedtls_pk_verify returned: ");
//         print_uint32_hex(ret);
//         log("\r\n");
//     }

//     if( ret != 0 ) {
//         char errbuf[100];
//         mbedtls_strerror(ret, errbuf, sizeof(errbuf));
//         log("verify failed: ");
//         log(errbuf); 
//         log("\r\n");
//     }

//     mbedtls_pk_free(&pk);
//     return (ret == 0);
// }