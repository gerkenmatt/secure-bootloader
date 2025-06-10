# Secure Bootloader for STM32F767ZI

## Overview
This project implements a secure bootloader for the STM32F767ZI microcontroller. It provides robust firmware update capabilities, secure memory management, and a protected update protocol via UART and OTA (Over-The-Air) mechanisms.

## Memory Layout
- **Bootloader**: 0x08000000 - 0x0801FFFF (128KB)
- **Bootloader Config**: 0x08020000 (128KB sector for configuration)
- **Application**: 0x08040000 - 0x080BFFFF (512KB, starts at sector 5)
- **Slot 0**: 0x080C0000 - 0x0813FFFF (512KB, backup/alternate image)
- **Slot 1**: 0x08140000 - 0x081BFFFF (512KB, backup/OTA image)
- **RAM**: 0x20000000 - 0x2007FFFF (512KB)
- **Flash End**: 0x08200000 (2MB total flash)

## Flash Layout
- **BOOTLOADER_START_PHYS**: 0x08000000
- **APPLICATION_START_ADDR**: 0x08040000
- **SLOT0_ADDR**: 0x080C0000
- **SLOT1_ADDR**: 0x08140000
- **CONFIG_ADDR**: 0x08020000

## Flash/Sector Layout

| Region                | Start Address  | Size     | Sectors         | Description                      |
|-----------------------|---------------|----------|-----------------|----------------------------------|
| Bootloader            | 0x08000000    | 256 KB   | 0-3             | Bootloader code                  |
| Bootloader Config     | 0x08020000    | 128 KB   | 4               | Bootloader configuration         |
| Application           | 0x08040000    | 512 KB   | 5-6             | Main application image           |
| Slot 0 (Backup)       | 0x080C0000    | 512 KB   | 7-8             | Backup/alternate image           |
| Slot 1 (OTA/Backup)   | 0x08140000    | 512 KB   | 9-10            | OTA/backup image                 |
| (Unused/Reserved)     | 0x081C0000    | 512 KB   | 11-12           | Reserved/unused                  |
| Flash End             | 0x08200000    | -        | -               | End of flash (2MB total)         |

- **Sector size:** Sectors 0-3: 32 KB each, Sector 4: 128 KB, Sectors 5-12: 256 KB each

## Bootloader Logic
- **Startup and Validation:** On reset, the bootloader initializes core hardware and validates its own integrity to ensure it's running correctly from its protected memory region.
- **Command Driven:** The bootloader operates via a simple command-line interface over UART. It waits for user commands to either run the application or begin an OTA update.
- **Update and Launch:** When the `run` command is issued, it first checks for a pending firmware update. If one exists, it copies it to the active application slot. It then performs a `CRC32` check on the active application before jumping to it, ensuring it is not corrupt.
- **Configuration:** Key information, such as the application's size, CRC, and update status, is stored in a dedicated, isolated configuration sector in flash.

### Application Jump Process
When `run` is issued:
- Check `should_run` flag in Slot 1 configuration (TODO: fix this)
- If update is pending, copy firmware from Slot 1 to Slot 0
- Perform pre-flight checks:
  - Validate application's stack pointer is within valid RAM bounds
  - Perform a CRC32 check on the application image
- Upon success:
  - De-initialize MCU peripherals
  - Set the main stack pointer (`__set_MSP`) and vector table (`SCB->VTOR`) to the application's values
  - Jump to the application's reset handler

## OTA Update Logic

### Custom Frame Protocol
OTA updates are received via UART using a custom frame protocol:
- Frame structure:
  - Start-of-Frame (`OTA_SOF`) byte
  - Packet type
  - 16-bit payload length
  - Payload
  - 32-bit CRC of payload
  - End-of-Frame (`OTA_EOF`) byte
- Bootloader validates CRC and responds with `RESP_ACK` or `RESP_NACK`

### Session Flow
OTA session is initiated by `ota` command and proceeds with:
- `PACKET_CMD (CMD_START)`: Erase update flash sectors for Slot 1 (`SLOT1_SECTOR`)
- `PACKET_HEADER`: Receive metadata (`ota_header_info_t`), including firmware size and CRC32
- `PACKET_DATA`: Receive firmware in data packets, write to Slot 1 with read-back verification
- `PACKET_SIG`: Receive ECDSA signature of the firmware
- `PACKET_CMD (CMD_END)`: Finalize OTA process

### Update Finalization and Activation
Upon receiving `CMD_END`:
- **Signature Verification**: Verify ECDSA signature using mbedTLS (`mbedtls_pk_verify`) against SHA-256 hash of Slot 1 firmware
- **Configuration Update**: If valid:
  - Mark Slot 1 as valid
  - Set `should_run` flag to true
  - Store firmware size and CRC
- **Reboot**: Send final `RESP_ACK` and trigger system reboot

### Failure Handling
- If signature verification or any step fails, send `RESP_NACK` and abort OTA process
- Return to command prompt without modifying existing application in Slot 0

## Security Features

### Firmware Authenticity
- Firmware updates are cryptographically authenticated using ECDSA over SHA-256
- Verification implemented with mbedTLS (`mbedtls_pk_verify`)
- Hardcoded DER-encoded public key in bootloader (**TODO: add more protection**)

### Data Integrity
- **Transport-Level CRC32**: Every OTA packet includes a CRC32 checksum; corrupted frames are discarded
- **Firmware Image CRC32**: Before booting the main application, CRC32 of firmware image is validated against known-good value in boot configuration

### Secure Configuration Storage
- Critical boot parameters (firmware size, CRC checksums, update flags) stored in `bootloader_config_t` struct in dedicated flash sector (`CONFIG_SECTOR`)
- Prevents main application from overwriting bootloader's critical decision-making data

## UART Interface
- **USART**: Typically USART2 or USART3
- **Baud rate**: 115200
- **Data bits**: 8
- **Stop bits**: 1
- **Parity**: None
- **Flow control**: None

## Key Files
- `src/bootloader.c` / `include/bootloader.h`: Core bootloader logic and configuration
- `src/ota.c` / `include/ota.h`: OTA update protocol and handlers
- `src/flash.c` / `include/flash.h`: Flash memory operations
- `src/uart.c` / `include/uart.h`: UART communication
- `src/utilities.c` / `include/utilities.h`: CRC and helper functions
- `stm32f767z.ld`: Linker script defining memory layout

## Building & Flashing
- Build with `make` in the project root
- Flash using OpenOCD: `make flash`

## WROOM BLE Bridge Integration
TODO

## Typical Workflow
1. Power on or reset the device
2. Bootloader checks for valid application and update requests
3. If update is needed, enters OTA mode and waits for commands
4. Receives firmware and signature, verifies, and updates configuration
5. Reboots into new application if update is successful

