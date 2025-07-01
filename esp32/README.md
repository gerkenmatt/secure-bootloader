# ESP32 Firmware Projects
This directory contains the embedded firmware projects for the ESP32 microcontroller, which serves a critical role in the `secure-bootloader` ecosystem.

## Project Overview
The `esp32` folder contains a single project:
- `esp32_ota_ble_bridge/`: This project turns an ESP32 into a dedicated Bluetooth Low Energy (BLE) to UART bridge.

## Purpose
The primary function of this bridge is to enable wireless communication between the host PC (running the test suite or OTA client) and the STM32 microcontroller (running the secure bootloader). The host communicates with the ESP32 over BLE, and the ESP32 then transparently forwards all data to the STM32's UART port, and vice-versa.
This setup allows for Over-the-Air (OTA) updates and interaction with the bootloader's command-line interface without requiring a physical USB connection to the STM32 board.

## Getting Started
All details regarding the secure-bootloader system functionality, features, hardware wiring, and build instructions are located [SETUP.md](../docs/SETUP.md)

### PlatformIO Setup 

PlatformIO is used to build, flash, and monitor the ESP32 bridge firmware. You only need to set it up using one of the two methods below.

---
#### Option 1: VS Code with PlatformIO IDE Extension (Recommended)

This approach provides a user-friendly, graphical interface within your code editor.

1.  **Install Visual Studio Code:** If you haven't already, download and install it. 
2.  **Install the Extension:**
    * Open VS Code.
    * Go to the **Extensions** view (click the square icon on the left sidebar or press `Ctrl+Shift+X`).
    * Search for `PlatformIO IDE` and click **Install** on the official extension published by PlatformIO.
    * After installation, reload VS Code if prompted. The extension will automatically download and set up all necessary components.

---
#### Option 2: PlatformIO Core (Command-Line Interface)

This option is for users who prefer to work entirely within the terminal.

1.  **Install with pip:** Make sure you have Python and pip installed. Then, run the following command in your terminal:
    ```bash
    pip install -U platformio
    ```
2.  **Verify Installation:** Check that the installation was successful by running:
    ```bash
    pio --version
    ```
    This should display the installed PlatformIO Core version number.
    
### ESP32-WROOM `udev` rules
1.  **Download Rules:**
    Open terminal and type: 
    ```bash
    curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
    ```
2.  **Reload the udev rules:**
    ```bash
    sudo service udev restart
    # or
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
    Now, unplug and re-plug your ESP32 device.

### Build and Flash the ESP32 BLE Bridge

The ESP32 must be programmed with the BLE-to-UART bridge firmware. This project is built using PlatformIO.

1.  **Navigate to the ESP32 project directory:**
    ```bash
    cd esp32/esp32_ble_bridge/
    ```

2.  **Build and Flash:**
    - **Using VS-Code Extension:**
        - click on the `Platform IO: Build` checkbox in the bottom bar
        - click on the `Platform IO: Upload` arrow in the bottom bar
    - **Using PlatformIO CLI:**
        - go to `esp32/esp32_ota_ble_bridge` project directory and run: 
        ```bash
        pio run --target upload
        ```
