# Blinky Test Application for Secure Bootloader

## 1. Overview

This directory contains a simple "Blinky" application for the STM32F767ZI-Nucleo board. Its primary purpose is to serve as a test firmware image for the main **Secure Bootloader** project.

It is a minimal, bare-metal application that demonstrates:
* How to build a firmware image that is correctly linked to run from one of the bootloader's application slots (Slot A or Slot B).
* How to provide a binary that can be signed and used for testing the Over-the-Air (OTA) update process.
* The behavior of the bootloader's automatic rollback feature.

## 2. Functionality

When running, the application performs a very simple task: it sequentially toggles the three user LEDs on the Nucleo board (Green, Blue, and Red) with a 50ms delay between each toggle.

* **Green LED**: `PB0`
* **Blue LED**: `PB7`
* **Red LED**: `PB14`

## 3. Important Note for Bootloader Integration

This application is designed as a **basic functionality test**. It **intentionally does not contain the logic to reset the bootloader's boot attempt counter**.

**What does this mean?**
When you successfully update a slot with this firmware and `run` it, the bootloader will:
1.  Decrement the boot attempt counter for that slot.
2.  Jump to this application.
3.  The application will run correctly (blinking the LEDs).
4.  If the device is reset, the bootloader will repeat this process.
5.  After 7 resets, the boot attempt counter will reach zero, and the bootloader will trigger its **automatic rollback mechanism**, reverting to the other known-good application slot.

This behavior is **expected** and demonstrates that the bootloader's fail-safe rollback feature is working correctly.

## 4. Building the Application

The provided `Makefile` is configured to build the application for different memory locations, which is essential for compatibility with the bootloader's memory map.

### Build Targets

* `make all` (or just `make`): Builds three versions of the binary, one for each potential location.
    * `build/blinky_slota.bin`: Linked to run from **Slot A** (`0x08040000`).
    * `build/blinky_slotb.bin`: Linked to run from **Slot B** (`0x08100000`).
    * `build/blinky_app.bin`: Linked to run from the start of flash (`0x08000000`) for standalone testing.

* `make slota`: Builds only the Slot A version.
* `make slotb`: Builds only the Slot B version.
* `make clean`: Removes the `build/` directory.

## 5. Usage with the Secure Bootloader & Host Utilities

This is the primary use case for the blinky application, using the provided host utility scripts to automate the signing process.

1.  **Build the Firmware**:
    First, build the application binary for the desired slots. To build the binaries for both Slot A and Slot B:
    ```bash
    make
    ```
    Or to test an OTA update to a specific slot (Slot B), you will need `blinky_slotb.bin`:
    ```bash
    make slotb
    ```

3.  **Prepare Firmware for Host Scripts**:
    Copy the built binary into the `valid_firmware` directory used by the host scripts. Rename it to `blinky.bin`.
    ```bash
    # (From the 'application' directory)
    cp build/blinky_slotb.bin ../../../host_pc/utilities/valid_firmware/blinky.bin
    ```

4.  **Generate Keys (First-Time Setup)**:
    **Note:** Generating new keys will require updating the hardcoded public key value in the `stm32/bootloader` project. This can be found in the `ota_crypto.c` file. 
    If you haven't already, run the key generation script. This only needs to be done once.
    ```bash
    # Navigate to the utilities directory
    cd ../host_pc/utilities/

    # Generate keys
    python generate_keys.py --output-dir test_data
    ```

4.  **Generate Signed Test Firmware**:
    From the `utilities` directory, run the firmware generation script. This script takes the binaries from `valid_firmware`, signs them, renames them, and places the final artifacts in the `test_data` directory.
    ```bash
    # (From the 'host_pc/utilities' directory)
    python3 generate_test_firmware.py --input-dir valid_firmware --output-dir test_data --key-dir test_data
    ```
    This will create files like `firmware_A_slota.bin`, `firmware_A_slota.sig`, `firmware_A_slotb.bin`, etc., inside the `test_data/` directory.

5.  **Perform the OTA Update**:
    The generated files in `test_data/` are now ready for an update. This can be done manually via the OTA client or automatically via the integration tests.

    **A) Using the Interactive OTA Client:**
    The `ota_client.py` script provides a command-line interface to interact with the bootloader and send the firmware.
    ```bash
    # (From the project root directory)
    python3 host_pc/ota_client/ble_ota_client.py
    ```
    Once the client is running, you can use its internal commands (e.g., `update`, `run`) to perform the OTA update using the firmware and signature files from the `host_pc/utilities/test_data/` directory.

    **B) Running Integration Tests:**
    The files generated in the `test_data` directory are primarily designed to be consumed by an automated test suite (e.g., `pytest`). Running these tests will programmatically perform the OTA updates to verify the entire system is working correctly.

6.  **Run the Application**:
    After the update is complete, connect to the bootloader's serial CLI and issue the `run` command. The bootloader will jump to the newly updated slot, and you should see the LEDs begin to blink.


## 6. Standalone Usage

While not its main purpose, you can also flash and run this application directly on the board without the bootloader for simple hardware tests.

1.  **Build the Standalone Version**:
    This target is already built if you ran `make all`. The correct binary is `build/blinky_app.bin`.

2.  **Flash the Application**:
    Use the `flash_app` make target, which flashes the binary to the start of memory.
    ```bash
    make flash_app
    ```
    The device will reset, and the LEDs will start blinking immediately.
