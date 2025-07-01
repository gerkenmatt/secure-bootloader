# Project Setup Guide

This guide provides a comprehensive, step-by-step walkthrough to get the entire STM32 Secure Bootloader project up and running. Follow these instructions sequentially to configure your hardware, set up the development environment, and perform your first secure firmware update.

## 1. Environment Setup

These steps configure your host computer with the necessary tools to build, flash, and communicate with the embedded devices.

### Toolchain and Dependencies

First, install the required development tools for an Ubuntu/Debian-based Linux system.

```bash
# Update package list
sudo apt-get update

# Install Git for version control
sudo apt-get install -y git

# Install ARM GCC toolchain for cross-compiling STM32 firmware
sudo apt-get install -y gcc-arm-none-eabi

# Install Make for the build system
sudo apt-get install -y make

# Install OpenOCD for flashing and debugging the STM32
sudo apt-get install -y openocd
```

### Configure ST-Link `udev` Rules

To allow OpenOCD to access the ST-Link debugger without `sudo`, create a `udev` rule.

1.  **Create the rules file:**
    ```bash
    sudo nano /etc/udev/rules.d/99-stlink.rules
    ```

2.  **Add the following content:**
    ```
    # ST-Link/V2, ST-Link/V2-1, ST-Link/V3
    ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666"
    ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666"
    ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374d", MODE="0666"
    ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3752", MODE="0666"
    ```

3.  **Reload the udev rules:**
    ```bash
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
    Now, unplug and re-plug your ST-Link device.

### Python Environment

The host scripts require Python 3.8+ and several packages. It is highly recommended to use a virtual environment.

```bash
# Navigate to the host tools directory
cd host_pc/

# Create a Python virtual environment
python3 -m venv venv

# Activate the virtual environment
source venv/bin/activate

# Install all required packages
pip install -r requirements.txt
```

**Note:** Remember to activate the virtual environment (`source host_pc/venv/bin/activate`) in any new terminal session where you intend to run the host scripts.

### Clone the Repository

Finally, clone the project repository and initialize its submodules (like mbedTLS).

```bash
git clone https://github.com/gerkenmatt/secure-bootloader.git
cd secure-bootloader
git submodule update --init --recursive
```

---

## 2. Hardware Setup

This section details the physical hardware requirements and wiring connections.

### Hardware Requirements

* **STM32 Board:** NUCLEO-F767ZI Development Board
* **ESP32 Board:** An ESP32-WROOM development board (e.g., a generic dev kit)
* **Host PC:** A computer with a working Bluetooth adapter
* **Cables:** USB cables for power/flashing and jumper wires for UART

### Hardware Connections

Connect the ESP32 and STM32 boards via UART. This allows the ESP32 to act as a BLE-to-Serial bridge.

| ESP32 Pin | STM32 Pin ( USART3 ) | Description              |
| :-------- | :------------------ | :----------------------- |
| **GPIO17 (TX2)** | **PD8 (RX)** | ESP32 Transmit ➞ STM32 Receive |
| **GPIO16 (RX2)** | **PD9 (TX)** | ESP32 Receive ➞ STM32 Transmit |
| **GND** | **GND** | Common Ground            |
| **Vin** | **5V** | 5V Power (optional if both powered by USB) |

**Important:** Ensure both boards share a common ground (GND).

---

## 3. Initial Firmware Flashing

Before performing an OTA update, you must flash the initial firmware onto both microcontrollers using a USB connection.

### Flash the ESP32 BLE Bridge

The ESP32 must be programmed with the BLE-to-UART bridge firmware. This project is built using PlatformIO.

1.  **Navigate to the ESP32 project directory:**
    ```bash
    cd esp32/esp32_ble_bridge/
    ```

2.  **Build and Flash:**
    Follow the instructions in the `esp32/esp32_ble_bridge/README.md` to build and upload the firmware using the PlatformIO CLI or VSCode extension.
    ```bash
    # Example using PlatformIO Core CLI
    pio run --target upload
    ```

### Flash the STM32 Secure Bootloader

Next, flash the secure bootloader onto the STM32.

1.  **Navigate to the bootloader directory:**
    ```bash
    cd stm32/bootloader/
    ```

2.  **Build the bootloader:**
    ```bash
    make
    ```

3.  **Flash the bootloader:**
    Connect the ST-Link to your PC and run the flash command.
    ```bash
    make flash
    ```

At this point, the core bootloader is on the STM32 and the BLE bridge is on the ESP32. The system is now ready to receive its first signed application via a wireless OTA update.

---

## 4. First OTA Update: A Step-by-Step Workflow

This workflow guides you through preparing a signed application and sending it to the device wirelessly.

### Step 1: Build the Test Application and Copy to Firmware Directory

First, compile one of the sample applications (e.g., `blinky`). The `Makefile` is configured to build versions for each memory slot.

```bash
# Navigate to the blinky app directory
cd stm32/applications/blinky_app/

# Build the binaries for both Slot A and Slot B
make all

# Copy to the valid_firmware directory
cp build/blinky_slota.bin ../../../host_pc/utilities/valid_firmware
cp build/blinky_slotb.bin ../../../host_pc/utilities/valid_firmware
```

### Step 2 (Optional): Generate Cryptographic Keys
**There are already keys inside the `host_pc/test_data` directory, but if you want to regenerate new ones follow these steps**

The host utilities are used to create the ECDSA key pair for signing firmware. 

```bash
# Navigate to the host utilities directory
cd ../../../host_pc/utilities/

# Ensure your Python venv is active
source ../venv/bin/activate

# Generate a new key pair
python generate_keys.py --output-dir ../test_data/
```
**Note:** Generating new keys requires you to update the public key in the bootloader source (`stm32/bootloader/src/ota_crypto.c`) and re-flash the bootloader. For initial setup, use the keys provided or follow the project-specific guide on key rotation.

### Step 3: Sign the Application Firmware

Now, use the host utility script to sign the `blinky.bin` application you built earlier.

1.  **Copy the application binary to the utilities input folder:**
    ```bash
    # (From the host_pc/utilities/ directory)
    cp ../../stm32/applications/blinky_app/build/blinky_*.bin ./valid_firmware/
    ```
2.  **Run the signing script:**
    This script signs the binaries and places them in the `test_data` folder, which is used by both the test suite and the OTA client.
    ```bash
    # (From the host_pc/utilities/ directory)
    python generate_test_firmware.py --input-dir valid_firmware --output-dir ../test_data --key-dir ../test_data
    ```
    This creates files like `firmware_A_slota.bin` and `firmware_A_slota.sig` in the `host_pc/test_data/` directory.

### Step 4: Perform the OTA Update with the Client

Use the interactive client to send the newly signed firmware to the STM32.

1.  **Populate the client's firmware directory:**
    Copy the signed artifacts into the client's local firmware folder for convenience.
    ```bash
    # (From the host_pc/ directory)
    cp test_data/firmware_A_* ota_client/firmware/
    ```

2.  **Launch the OTA client:**
    ```bash
    # (From the host_pc/ota_client/ directory)
    # Ensure your Python venv is active
    python ble_ota_client.py
    ```

3.  **Use the client to update:**
    The client will scan for and connect to the ESP32. Once you see the `➜` prompt, you can issue commands.
    ```
    update firmware_A
    ```
    The client will query the device's active slot and send the correct firmware to the inactive slot.

4.  **Run the new application:**
    After the update completes, use the `run` command.
    ```
    run
    ```
    The device will reboot, and you should see the Blinky LEDs start, confirming a successful secure OTA update!

---

## 5. Running Automated Integration Tests

To verify the entire system is working correctly, you can run the `pytest` integration suite. These tests automatically handle key generation, signing, and OTA updates for various success and failure scenarios.

```bash
# Navigate to the integration tests directory
cd host_pc/integration_tests/

# Ensure your Python venv is active and dependencies are installed
# Run the full test suite
python -m pytest -s
```

The tests provide detailed, color-coded output to show the communication between the host and the device, making it easy to validate system behavior.
