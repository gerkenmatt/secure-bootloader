# STM32 Secure Bootloader with BLE OTA Updates

This project implements a secure bootloader for an STM32 microcontroller, enabling cryptographically signed firmware updates Over-the-Air (OTA) using Bluetooth Low Energy (BLE).

The system uses an ESP32-WROOM module as a BLE-to-UART bridge, allowing a host PC to securely connect and send new firmware to the STM32. The host communicates using Python scripts that provide a command-line interface for interaction and are accompanied by a full suite of pytest integration tests.



## Key Features

- **Secure Boot:** Firmware updates are authenticated using ECDSA signatures to ensure only authorized code is executed.
- **OTA Updates via BLE:** A robust update process managed by a Python host script, facilitated by an ESP32 for BLE communication.
- **Dual-MCU Architecture:** Utilizes the STM32 for real-time application processing and the ESP32-WROOM for efficient wireless connectivity.
- **Command-Line Host Tools:** Interactive Python scripts for managing the OTA process from a host computer.
- **Automated Integration Testing:** Includes a test suite using pytest to validate the entire OTA flow from the host to the target.

## System Architecture

The system consists of three main components that communicate in a chain:

- **Host PC:** Runs the Python scripts to initiate and manage the OTA update.
- **ESP32-WROOM:** Acts as a wireless bridge, receiving commands and firmware via BLE from the host and forwarding them over UART.
- **STM32:** Runs the secure bootloader, receives data from the ESP32 via UART, validates the firmware's signature, and performs the flash programming.

The communication flow is as follows: 
`[Host PC] <--- Bluetooth Low Energy (BLE) ---> [ESP32-WROOM] <--- UART ---> [STM32]`

## Development Toolchain
This project was developed with a focus on a lightweight, IDE-independent toolchain.
- **Editor:** All code was written in Visual Studio Code.
- **Build System:** The STM32 projects are built using CMake and Makefiles, not proprietary IDE project files.
- **Bare-Metal Approach:** The STM32 firmware is written directly at the **register access level**. It **does not use the STM32 HAL libraries**. This approach was chosen to gain a deeper understanding of the hardware and to produce more efficient, transparent code.

## Repository Structure

```
.
├── stm32_projects/        // Main STM32 projects and prototypes
│   ├── secure_bootloader/ // The primary secure bootloader application
│   └── ...                // Other stepping-stone and test projects
│
├── esp32_projects/        // Firmware for the ESP32-WROOM BLE-to-UART bridge
│
└── stm32_ota/             // Host-side Python scripts and related files
    ├── ble_ota_host.py    // Main Python script for BLE OTA updates
    ├── firmware/          // Pre-compiled firmware binaries, signatures, and keys
    └── integration_tests/ // Pytest integration tests for the OTA process
```

## Getting Started: System Setup

Follow these steps to set up the hardware and flash the necessary firmware.

### 1. Hardware Requirements

- An STM32 NUCLEO-F767ZI Development Board
- An ESP32-WROOM development board
- A host computer with a Bluetooth adapter
- Jumper wires

### 2. Hardware Connections

Connect the STM32 and ESP32 boards via UART. Ensure both boards share a common ground (GND) and are powered at a compatible voltage level (e.g., 3.3V).

| ESP32 Pin | STM32 Pin (example) | Description                  |
|-----------|--------------------|------------------------------|
| TX2       | RX (e.g., PA10)    | UART Transmit to Receive      |
| RX2       | TX (e.g., PA9)     | UART Receive to Transmit      |
| GND       | GND                | Common Ground                 |
| Vin       | 5V                 | 5V Power                      |


### 3. Firmware Flashing

You need to flash firmware onto both the ESP32 and the STM32.

#### Flash the ESP32 Bridge

1. Navigate to the `esp32_projects/` directory.
2. Open the BLE-to-UART bridge project.
3. Using the ESP-IDF toolchain, build and flash the project onto your ESP32-WROOM board.

#### Flash the STM32 Secure Bootloader

1. Navigate to `stm32_projects/secure_bootloader/`.
2. Open the project in your preferred STM32 development environment (e.g. VS-Code).
3. Build and flash the bootloader onto your STM32 board.

Once both boards are flashed and connected, the hardware setup is complete!

## Usage: Performing an OTA Update

All host-side operations are run from the `stm32_ota/` directory.

### 1. Setup Python Environment

It's recommended to use a virtual environment.

```bash
# Navigate to the host script directory
cd stm32_ota

# Create and activate a virtual environment
python -m venv venv
source venv/bin/activate

# Install required packages
pip install -r requirements.txt
```

### 2. Run the OTA Host Script
Execute the main host script to start the interactive command line. The script will automatically scan for and connect to the ESP32 BLE bridge.
```bash
python ble_ota_host.py
```
You will be presented with a command-line interface where you can issue commands like `update <firmware_path>`, `run`, `reboot`, and more to interact with the STM32 bootloader.

## Running Tests
The project includes integration tests to ensure the system works end-to-end.
To run the tests, simply execute `pytest` from the `stm32_ota/integration_tests` directory:

```bash
python -m pytest test_secure_ota_ble.py -s

The tests will automatically handle connecting to the device, sending firmware, and verifying the bootloader's responses.

