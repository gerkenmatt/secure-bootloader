# STM32F7 LED Blinky Project

A minimal LED cycling project for the STM32F767ZI microcontroller using CMake as the build system. The project demonstrates basic GPIO control by cycling through three LEDs with a 500ms delay between toggles.

## Hardware Requirements

- STM32F767ZI Nucleo board
- ST-LINK V2/V3 (included on Nucleo board)
- USB cable

## Software Requirements

- ARM GCC Toolchain (`arm-none-eabi-gcc`)
- CMake (version 3.16 or higher)
- OpenOCD
- STM32CubeF7 SDK


## Building

```bash
# Configure and build (debug)
cmake -B build --preset stm32f7-debug
cmake --build build
```

## Flashing

```bash
# Flash using OpenOCD
cmake --build build --target flash
```
## Debugging
### Using VS Code
1. Install the "Cortex-Debug" extension
2. The project is pre-configured for debugging in VS Code
3. Press F5 or use the Run and Debug panel to start debugging

### Using Command Line
To start a GDB debugging session manually:
```bash
# In one terminal, start OpenOCD:
openocd -f interface/stlink.cfg -f target/stm32f7x.cfg

# In another terminal, start GDB:
arm-none-eabi-gdb build/blinky.elf
(gdb) target remote :3333
```

## Project Structure

```
.
├── src/
│   ├── main.c          # Main application
│   ├── led_control.c   # LED control implementation
│   ├── startup.c       # Startup code
│   └── utils.c         # Utility functions
├── include/
│   ├── led_control.h   # LED control interface
│   ├── utils.h         # Utility functions
│   └── config.h        # Project configuration
├── cmake/
│   └── toolchain-arm-none-eabi.cmake  # Toolchain config
├── .vscode/
│   ├── launch.json     # Debug configuration
│   └── extensions.json # Recommended extensions
├── CMakeLists.txt      # Build configuration
├── CMakePresets.json   # Build presets
└── stm32f767z.ld      # Linker script
```


## Troubleshooting

1. **Build fails with "Could not find CMAKE_ROOT"**:
   ```bash
   hash -r
   ```

