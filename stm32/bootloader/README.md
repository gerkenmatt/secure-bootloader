# Secure Bootloader for STM32F767ZI

## Overview
This project provides a secure, robust bootloader for the STM32F767ZI platform, designed for reliability and security in field-updatable embedded systems. It features a dual-bank (A/B) update mechanism, fail-safe automatic rollback, and cryptographic signature verification for all firmware updates.

## Key Features
- **Secure Boot Process:** Ensures that only authenticated firmware is executed by verifying an ECDSA signature before activating a new application.
- **A/B Dual-Bank Updates:** A new firmware update is written to an inactive slot, ensuring the device remains fully operational and bootable from the last known-good image if the update process is interrupted.
- **Automatic Rollback:** A boot attempt counter prevents boot-loops. If a new application fails to boot successfully after a set number of attempts, the bootloader automatically reverts to the previous working version.
- **Robust OTA Protocol:** A custom, packet-based serial protocol with CRC32 checks ensures reliable and error-free transfer of firmware images.
- **Interactive CLI:** A command-line interface over UART provides control for launching the application, initiating updates, and viewing system status.
- **Modular Architecture:** Cleanly separated modules for boot logic, configuration management, OTA handling, and cryptographic services

## Core Concepts Explained
### A/B Partitioning
The flash memory is divided into two primary application slots: Slot A and Slot B. This allows an update to be written to the inactive slot without affecting the current, running application. Only after the new firmware is fully received and its signature is verified does the bootloader switch the "active" slot and reboot, providing an atomic and fail-safe update.

### Boot Counter & Rollback
To protect against faulty but correctly signed firmware (e.g., an app that crashes at runtime), the bootloader uses a boot attempt counter.
1. Before jumping to an application, the bootloader decrements its boot_attempts_remaining counter in flash.
2. It is the application's responsibility to confirm a successful boot by signaling the bootloader to reset this counter.
3. If the application crashes and the device reboots, the counter is not reset. After 7 failed attempts, the bootloader marks the slot as invalid and rolls back to the other slot.
   
## Memory Layout

| Region | Start Address | End Address | Size | Sectors | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Bootloader** | `0x08000000` | `0x0803FFFF` | 256KB | 0-3 | Contains the immutable bootloader code. Should be protected by readout/write protection fuses. |
| **Application Slot A** | `0x08040000` | `0x080FFFFF` | 768KB | 4-7 | New firmware is loaded here when Slot B is active|
| **Application Slot B** | `0x08100000` | `0x081BFFFF` | 768KB | 8-10 | New firmware is loaded here when Slot A is active |
| **Config Sector** | `0x081C0000` | `0x081FFFFF` | 256KB | 11 | Stores the boot configuration structure, which manages the A/B state and rollback mechanism. |
| **RAM** | `0x20000000` | `0x27FFFFFF` | 512KB | N/A | Volatile memory. | 

- **Sector size:**
  - **Sectors 0-3:** 32 KB each
  - **Sector 4:** 128 KB
  - **Sectors 5-12:** 256 KB each

## Getting Started

### Prerequisites
- **Hardware:** STM32F767ZI-Nucleo Board
- **Toolchain:** ARM GCC (arm-none-eabi-*)
- **Utilities:** make, st-flash, mbedtls-utils, python3, pyserial

### Basic Workflow
For a complete, step-by-step guide on setting up the environment, generating keys, and performing an update, please see the Setup Guide.

#### Building & Flashing
- Build with `make` in the project root
- Flash using OpenOCD: `make flash`

#### Typical Workflow
1. Power on or reset the device
2. Bootloader checks for valid application and update requests
3. If update is needed, enters OTA mode and waits for commands
4. Receives firmware and signature, verifies, and updates configuration
5. Reboots into new application if update is successful

