# Host PC Tools for Secure Bootloader

This directory contains all host-side Python scripts and tools required to interact with, test, and update the STM32 secure bootloader.

---

## Directory Structure & Purpose

This folder is organized into several components that work together in a data pipeline. The general flow is from `utilities` (which create test data) to `ota_client` or `integration_tests` (which consume that data).

---

### `utilities/`

**Purpose:** Starting point for preparing test firmware. Contains Python scripts to automate the generation of cryptographic keys and the signing of compiled application binaries.

**Key Contents:**
- `generate_keys.py`: Creates ECDSA key pairs for signing.
- `generate_test_firmware.py`: Signs firmware and packages it for testing.
- `valid_firmware/`: Input directory for compiled `.bin` files from STM32 projects.

---

### `test_data/`

**Purpose:** Output directory for the utility scripts. Stores all generated keys and signed firmware artifacts.

**Key Contents:**
- `ec_priv.pem`, `ec_pub.der`, etc.: Cryptographic keys.
- `firmware_A_slota.bin`, `firmware_A_slota.sig`, etc.: Final, signed firmware packages.

**Note:** Both the integration tests and OTA client consume data from this directory.

---

### `integration_tests/` 

**Purpose:** Contains the automated `pytest` suite for end-to-end validation of the bootloader.

**Key Contents:**
- `test_*.py`: Test files containing validation logic.
- `ota_host.py`: High-level API for communicating with the device over BLE.
- `conftest.py`: Pytest configuration and fixture setup.

---

### `ota_client/` 

**Purpose:** Interactive command-line client for manual OTA updates.

**Key Contents:**
- `ble_ota_client.py`: Script to launch the interactive client.
- `firmware/`: Convenience directory for artifacts from `test_data/`.
- `requirements.txt`: List of Python dependencies (`pytest`, `bleak`, `cryptography`, etc.).

---

## Getting Started

For detailed instructions, refer to the README files in each directory:

- **[./utilities/README.md](./utilities/README.md)**: How to prepare and sign your firmware.
- **[./ota_client/README.md](./ota_client/README.md)**: How to run manual OTA updates.
- **[./integration_tests/README.md](./integration_tests/README.md)**: How to run the automated test suite.
