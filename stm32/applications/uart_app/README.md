# UART Echo Test Application for Secure Bootloader

## 1. Overview

This directory contains a simple "UART Echo" application for the STM32F767ZI-Nucleo board. Its primary purpose is to serve as an alternative test firmware image for the main **Secure Bootloader** project.

It is a minimal, bare-metal application that demonstrates:
* How to build a firmware image that is correctly linked to run from one of the bootloader's application slots (Slot A or Slot B).
* How to provide a binary that can be signed and used for testing the Over-the-Air (OTA) update process.
* The behavior of the bootloader's automatic rollback feature.

## 2. Functionality

When running, the application performs a simple UART echo task:
1.  Initializes USART3 at 115200 baud.
2.  Turns on the Green and Red LEDs to indicate it is running.
3.  Prints a "USART3 ready. Type something:" message to the console.
4.  Waits for user input and echoes any received line back to the console.

## 3. Important Note for Bootloader Integration

This application is designed as a **basic functionality test**. It **intentionally does not contain the logic to reset the bootloader's boot attempt counter**.

**What does this mean?**
When you successfully update a slot with this firmware and `run` it, the bootloader will:
1.  Decrement the boot attempt counter for that slot.
2.  Jump to this application.
3.  The application will run correctly (echoing UART characters).
4.  If the device is reset, the bootloader will repeat this process.
5.  After 7 resets, the boot attempt counter will reach zero, and the bootloader will trigger its **automatic rollback mechanism**, reverting to the other known-good application slot.

This behavior is **expected** and demonstrates that the bootloader's fail-safe rollback feature is working correctly.

## 4. Building the Application

The provided `Makefile` is configured to build the application for different memory locations, which is essential for compatibility with the bootloader's memory map.

### Build Targets

* `make all` (or just `make`): Builds the `uart_print` binaries for both Slot A and Slot B.
    * `build/uart_print_slota.bin`: Linked to run from **Slot A** (`0x08040000`).
    * `build/uart_print_slotb.bin`: Linked to run from **Slot B** (`0x08100000`).
* `make app`: Builds a standalone version linked to run from `0x08000000`.
* `make clean`: Removes the `build/` directory.

## 5. Usage with the Secure Bootloader & Host Utilities

This is the primary use case for the UART application, using the provided host utility scripts to automate the signing and preparation process.

1.  **Build the Firmware**:
    First, build the application binaries for both slots. This ensures you have the latest version ready for signing.
    ```bash
    # From the 'uart_app' directory
    make all
    ```

2.  **Prepare Firmware for Host Scripts**:
    Copy the newly built binaries into the `valid_firmware` directory. The host scripts use this directory as the source for creating signed test files.
    ```bash
    # (From the 'uart_app' directory)
    cp build/uart_print_slota.bin ../host_pc/utilities/valid_firmware/
    cp build/uart_print_slotb.bin ../host_pc/utilities/valid_firmware/
    ```

3.  **Generate Keys (First-Time Setup)**:
    If you haven't already, navigate to the utilities directory and run the key generation script. This only needs to be done once.
    ```bash
    # Navigate to the utilities directory
    cd ../host_pc/utilities/

    # Generate keys
    python3 generate_keys.py --output-dir test_data
    ```

4.  **Generate Signed Test Firmware**:
    From the `utilities` directory, run the firmware generation script. This script takes the binaries from `valid_firmware`, signs them, renames them, and places the final artifacts in the `test_data` directory.
    ```bash
    # (From the 'host_pc/utilities' directory)
    python3 generate_test_firmware.py --input-dir valid_firmware --output-dir test_data --key-dir test_data
    ```
    This will create files like `firmware_B_slota.bin`, `firmware_B_slota.sig`, etc., inside the `test_data/` directory.

5.  **Perform the OTA Update**:
    The generated files in `test_data/` are now ready for an update. This can be done manually via the OTA client or automatically via the integration tests.

    **A) Using the Interactive OTA Client:**
    The `ota_client.py` script provides a command-line interface to interact with the bootloader and send the firmware.
    ```bash
    # (From the project root directory)
    python3 host_pc/ota_client.py 
    ```
    Once the client is running, you can use its internal commands (e.g., `update`, `send_file`) to perform the OTA update using one of the `firmware_B` files and its corresponding signature from the `host_pc/utilities/test_data/` directory.

    **B) Running Integration Tests:**
    The files generated in the `test_data` directory are primarily designed to be consumed by an automated test suite (e.g., `pytest`). Running these tests will programmatically perform the OTA updates to verify the entire system is working correctly.

6.  **Run the Application**:
    After the update is complete, connect to the bootloader's serial CLI and issue the `run` command. The bootloader will jump to the newly updated slot, and you should see the "USART3 ready" prompt.

## 6. Standalone Usage

While not its main purpose, you can also flash and run this application directly on the board without the bootloader for simple hardware tests.

1.  **Build the Standalone Version**:
    ```bash
    make app
    ```

2.  **Flash the Application**:
    Use the `flash_app` make target, which flashes the binary to the start of memory.
    ```bash
    make flash_app
    ```
    The device will reset, and you should see the "USART3 ready" prompt in your serial terminal.
