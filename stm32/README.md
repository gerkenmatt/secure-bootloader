# STM32 Firmware Projects

This directory contains all the embedded firmware projects for the STM32F767ZI target used in the `secure-bootloader` system.

## Project Structure
```
.
├── applications/      # Test applications designed to run with the bootloader
│   ├── blinky/        # A simple LED blinking application
│   └── uart_print/    # A UART echo application
└── bootloader/        # The main secure bootloader project
```

## 1. Bootloader

The `./bootloader` directory contains the main secure bootloader project. Its primary responsibilities are:

* **Securely booting** an application from one of two memory slots (Slot A or Slot B).
* **Verifying firmware integrity and authenticity** using ECDSA signatures before launching.
* Providing an **Over-the-Air (OTA) update mechanism** via a custom serial protocol.
* Implementing a **fail-safe rollback mechanism** to prevent the device from being bricked by a faulty update.

For detailed information on its architecture, features, and setup, please see its dedicated README file:
* **[./bootloader/README.md](./bootloader/README.md)**

## 2. Test Applications

The `./applications` directory contains sample applications used to test the functionality of the secure bootloader. These applications are designed to be built for specific memory slots and do not contain the logic to reset the bootloader's boot counter, which allows for testing the automatic rollback feature.

### Blinky Application

* **Location**: `./applications/blinky`
* **Functionality**: Sequentially blinks the three user LEDs (Green, Blue, Red).
* **Purpose**: Serves as a basic "hello world" to confirm that the bootloader can successfully jump to an application in a given slot.
* **Details**: See the **[Blinky README](./applications/blinky_app/README.md)**.

### UART Echo Application

* **Location**: `./applications/uart_print`
* **Functionality**: Initializes a UART interface and echoes any received text back to the console.
* **Purpose**: Serves as an alternative firmware image to test updating from one application type to another. It's useful for verifying that a more complex application runs correctly.
* **Details**: See the **[UART App README](./applications/uart_app/README.md)**.

## Overall Workflow

The typical development and testing workflow involves all three projects:

1.  **Build the Bootloader**: Compile and flash the bootloader from the `./bootloader` directory. This only needs to be done once or when the bootloader code changes.
2.  **Build the Test Applications**: Compile one or both of the test applications from their respective directories (e.g., `make all` in `applications/blinky`).
3.  **Prepare Firmware**: Use the host utility scripts (located in `host_pc/utilities`) to prepare and sign the application binaries. This involves placing the compiled `.bin` files into the `valid_firmware` directory and running the provided Python scripts.
4.  **Perform OTA Update**: Use the `ota_client.py` or automated tests to upload the signed firmware to the device running the bootloader.
5.  **Verify**: Use the bootloader's CLI to `run` the newly updated application and verify its functionality
