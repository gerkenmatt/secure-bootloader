# Secure Bootloader for STM32F767ZI

A secure bootloader implementation for the STM32F767ZI microcontroller that supports:
- Over-The-Air (OTA) firmware updates with CRC verification
- Memory aliasing verification between AXI and ITCM buses
- Application jumping with VTOR remapping
- Boot configuration storage with security features
- Serial communication for debugging and OTA updates
- Flash memory protection and verification

## Project Structure

```
secure_bootloader/
├── src/
│   ├── main.c           # Main bootloader logic and command processing
│   ├── bootloader.c     # Core bootloader functions and state management
│   ├── ota.c           # Over-The-Air update implementation
│   ├── flash.c         # Flash memory operations and protection
│   ├── uart.c          # UART communication interface
│   ├── utilities.c     # CRC calculation and helper functions
│   └── startup.c       # Vector table and startup code
├── include/
│   ├── bootloader.h    # Bootloader definitions and configurations
│   ├── ota.h          # OTA protocol definitions and structures
│   ├── flash.h        # Flash memory operation interfaces
│   ├── uart.h         # UART communication interfaces
│   └── utilities.h    # Utility function declarations
├── stm32f767z.ld      # Linker script for memory layout
└── Makefile          # Build configuration
```

## Memory Layout

The bootloader uses a secure memory layout with protected regions:
- Bootloader: 0x08000000 - 0x0803FFFF (256KB)
  - Protected from overwrite during updates
  - Stores bootloader code and configuration
- Boot Configuration: 0x08020000 (128KB, Sector 4)
  - Stores boot flags and firmware metadata
  - Contains slot validity information
- Application: 0x08040000 - 0x0807FFFF (512KB, Sector 5)
  - Region for currently running application
- Firmware Slots:
  - Slot 0 (Primary): 0x08080000 - 0x080FFFFF (512KB, Sector 6)
  - Slot 1 (Update): 0x080C0000 - 0x0813FFFF (512KB, Sector 7)
- RAM: 0x20000000 - 0x2007FFFF (512KB)

## Building

```bash
# Clean previous builds
make clean

# Build the project
make
```

Build outputs in `build/` directory:
- `secure_bootloader.elf` - ELF file with debug symbols
- `secure_bootloader.hex` - Intel HEX format
- `secure_bootloader.bin` - Raw binary for flashing

## Flashing

```bash
# Flash the bootloader using OpenOCD
make flash
```

## Features

### Over-The-Air Updates
- Secure firmware update protocol
- CRC32 verification of firmware
- Version control and compatibility checking
- Atomic updates with rollback capability
- Protected bootloader region

### Security Features
- Flash write protection
- Memory region verification
- CRC validation of firmware
- Boot configuration protection
- Secure update protocol

### Boot Modes

1. Normal Boot:
   - Verifies application integrity
   - Checks boot flags and configuration
   - Jumps to application if valid

2. Bootloader Mode:
   - Enters on invalid application or flag
   - Accepts OTA updates
   - Provides debug interface

### UART Interface

The bootloader uses USART2 for communication:
- Baud rate: 115200
- Data bits: 8
- Stop bits: 1
- Parity: None
- Flow control: None

Available commands:
- `run` or `r`: Jump to application
- `ota`: Enter OTA update mode
- `p`: Print flash contents
- `help` or `h`: Show command list

## Debugging

```bash
# Start OpenOCD and GDB
make debug
```

## Dependencies

- ARM GCC toolchain (arm-none-eabi-gcc)
- OpenOCD
- STM32F767ZI development board
- ST-Link programmer
