# Simple Bootloader for STM32F767ZI

A minimal bootloader implementation for the STM32F767ZI microcontroller that supports:
- Memory aliasing verification between AXI and ITCM buses
- Application jumping with VTOR remapping
- Basic boot configuration storage
- Serial communication for debugging

## Project Structure

```
simple_bootloader/
├── src/
│   ├── main.c           # Main bootloader logic
│   ├── bootloader.c     # Bootloader core functions
│   ├── startup.c        # Vector table and startup code
│   ├── system_stm32f7xx.c  # System initialization
│   └── uart.c          # UART communication
├── include/
│   ├── bootloader.h    # Bootloader definitions
│   └── uart.h         # UART function declarations
├── stm32f767z.ld      # Linker script
└── Makefile          # Build configuration
```

## Memory Layout

- Bootloader: 0x08000000 - 0x08007FFF (32KB)
- Application: 0x08008000 - 0x08200000 (2MB - 32KB)
- RAM: 0x20000000 - 0x2007FFFF (512KB)

## Building

```bash
# Clean any previous builds
make clean

# Build the project
make
```

This will generate:
- `build/simple_bootloader.elf` - Debug information and symbols
- `build/simple_bootloader.hex` - Intel HEX format file
- `build/simple_bootloader.bin` - Binary file for flashing

## Flashing

```bash
# Flash the bootloader using OpenOCD
make flash
```

## System Initialization

The bootloader performs the following initialization steps:
1. Enables FPU (Floating Point Unit)
2. Configures system clock to 216MHz
3. Sets up Flash latency for high-speed operation
4. Enables instruction and data caches
5. Configures GPIO for boot mode detection
6. Verifies memory aliasing between AXI and ITCM buses

## Boot Modes

1. Normal Boot:
   - If BOOT0 pin is low, jumps to application
   - If BOOT_FLAG_STAY_IN_BL is set, stays in bootloader

2. Bootloader Mode:
   - If BOOT0 pin is high, enters bootloader
   - Waits for commands via UART

## Debugging

```bash
# Start OpenOCD and GDB
make debug
```

## Serial Communication

The bootloader uses USART3 for debug output:
- Baud rate: 115200
- Data bits: 8
- Stop bits: 1
- Parity: None
- Flow control: None

## Dependencies

- ARM GCC toolchain (arm-none-eabi-gcc)
- OpenOCD
- STM32F767ZI development board
- ST-Link programmer
