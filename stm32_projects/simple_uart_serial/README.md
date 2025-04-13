# Simple UART Serial Application for STM32F767ZI

This is a minimal UART serial application for the STM32F767ZI microcontroller that implements a simple echo functionality using USART3.

## Prerequisites

- ARM GCC toolchain (arm-none-eabi-gcc)
- OpenOCD
- minicom (for serial communication)
- STM32F767ZI development board
- ST-Link programmer

## Building the Project

```bash
# Clean any previous builds
make clean

# Build the project
make
```

This will generate in the `build/` directory:
- `uart_print.elf` - Debug information and symbols
- `uart_print.hex` - Intel HEX format file
- `uart_print.bin` - Binary file for flashing

## Memory Layout

The application can be flashed at two different addresses:

1. Default address (0x08000000):
   - Full flash access
   - Used for standalone application
   - Command: `make flash`

2. Application address (0x08040000):
   - Starts after bootloader
   - Used when running with bootloader
   - Command: `make flash_app`

## Flashing the Program

1. Default address (0x08000000):
```bash
make flash
```

2. Application address (0x08040000):
```bash
make flash_app
```

## Reading Serial Output

The ST-Link/USB connector allows USART3 communication using PD8 as the TX pin and PD9 as the RX pin.
To read the serial output from the STM32F767ZI:

```bash
# Connect to the serial port using minicom
sudo minicom -D /dev/ttyACM0 -b 115200
```

## Project Structure

- `main.c` - Main application code with UART implementation
- `startup.c` - Startup code and vector table
- `stm32f767z.ld` - Linker script
- `Makefile` - Build configuration 