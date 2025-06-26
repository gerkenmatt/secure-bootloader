# STM32 Secure Bootloader with BLE OTA Updates

This project showcases a secure bootloader for an STM32 microcontroller, enabling cryptographically authenticated firmware updates Over-the-Air (OTA) via Bluetooth Low Energy (BLE).

The system uses an ESP32-WROOM module as a BLE-to-UART bridge, allowing a host PC to securely connect and send new firmware to the STM32. The host communicates using Python scripts that provide a command-line interface for interaction and are accompanied by a full suite of pytest integration tests.



## Key Features

- **Secure Firmware Update:** Implements ECDSA signature verification to ensure the authenticity and integrity of firmware before flashing, preventing unauthorized code execution and malicious tampering.
- **Dual-Bank (A/B) Updates:** Supports atomic and fault-tolerant OTA updates by writing new firmware to an inactive bank, minimizing downtime and providing rollback capability.
- **Over-the-Air (OTA) Capability:** Leverages BLE for wireless firmware delivery, bridging to UART for efficient communication with the STM32
- **Bare-Metal Embedded C:** The STM32 bootloader is developed in C at the register access level, showcasing a deep understanding of microcontroller internals and efficient resource management without relying on HAL libraries.
- **Embedded Cryptography:** Integrates mbedTLS for cryptographic operations, specifically for ECDSA signature verification.
- **Dual-MCU Architecture:** Utilizes the STM32 for real-time application processing and the ESP32-WROOM for efficient wireless connectivity.
- **Host-Side Automation & Testing:** Custom Python scripts provide a sophisticated command-line interface (CLI) for firmware management, accompanied by a full suite of Pytest integration tests for automated, end-to-end validation of the secure OTA flow.
- **Automated Integration Testing:** Includes a test suite using pytest to validate the entire 

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
- **Build System:** The STM32 projects are built using CMake and Makefiles.
- **Bare-Metal Approach:** The STM32 firmware is written directly at the **register access level**. It **does not use the STM32 HAL libraries**. This approach was chosen to gain a deeper understanding of the hardware and to produce more efficient, transparent code.

## Repository Structure

```
.
├── docs                               <- Project documentation, architecture diagrams, security insights, setup guides
├── host_pc                            <- All host-side Python tools, OTA clients, and automated tests
├── esp32                              <- ESP32 bridge firmware projects
│   ├── esp32_ble_bridge                 <- ESP32 BLE-to-UART bridge firmware (PlatformIO project)
│   └── esp32_bt_bridge                  <- ESP32 Bluetooth Classic bridge firmware 
├── stm32                              <- STM32 embedded firmware projects
│   ├── bootloader                       <- The core STM32 Secure Bootloader firmware project
│   └── applications                     <- Example application firmware for the bootloader (e.g., blinky, UART echo)
├── development_projects               <- Older iterations or proof-of-concept projects that were stepping stones
└── generated_artifacts                <- Output binaries (.bin, .sig) from the main build processes
```

## Getting Started: System Setup

Follow these steps to set up the hardware and flash the necessary firmware.

### 1. Hardware Requirements

- An STM32 NUCLEO-F767ZI Development Board
- An ESP32-WROOM development board
- A host computer with a Bluetooth adapter
- Jumper wires for UART connections

### 2. Hardware Connections

Connect the STM32 and ESP32 boards via UART. Ensure both boards share a common ground (GND) and are powered at a compatible voltage level (e.g., 5V).

| ESP32 Pin | STM32 Pin (example) | Description                  |
|-----------|--------------------|------------------------------|
| TX2       | RX (e.g., PA10)    | UART Transmit to Receive      |
| RX2       | TX (e.g., PA9)     | UART Receive to Transmit      |
| GND       | GND                | Common Ground                 |
| Vin       | 5V                 | 5V Power                      |


### 3. Firmware Flashing

You need to flash firmware onto both the ESP32 and the STM32.

#### Flash the ESP32 Bridge

1. Navigate to the `esp32/esp32_ble_bridge/` directory.
2. Follow the instructions in its `README.md` to build and flash the project onto your ESP32-WROOM board using the PlatformIO or ESP-IDF toolchain.

#### Flash the STM32 Secure Bootloader

1. Navigate to `stm32/bootloader/`.
2. Follow the instructions in its `README.md` to build and flash the secure bootloader onto your STM32 board.

Once both boards are flashed and connected, the hardware setup is complete!

## Usage: Performing an OTA Update

All host-side operations are run from the `host_pc/` directory.

### 1. Setup Python Environment

It's recommended to use a virtual environment.

```bash
# Navigate to the host_pc directory
cd host_pc

# Create and activate a virtual environment
python -m venv venv
source venv/bin/activate

# Install required Python packages
pip install -r requirements.txt
```

### 2. Run the OTA Host Script
Execute the main host script to start the interactive command line. The script will automatically scan for and connect to the ESP32 BLE bridge.
```bash
python ota_client/ble_ota_client.py
```
You will be presented with a command-line interface where you can issue commands like `update <firmware_path>`, `run`, `reboot`, and more to interact with the STM32 bootloader.

## Running Automated Tests
The project includes integration tests to ensure the system works end-to-end.
To run the tests, simply execute `pytest` from the `host_pc/integration_tests` directory:

```bash
cd host_pc/tests
python -m pytest -s
```

The tests will automatically handle connecting to the device, sending various firmware (including valid, invalid, and large files), and verifying the bootloader's responses and behavior on the STM32.

