# Secure Bootloader Integration Tests

This directory contains the automated integration test suite for the secure bootloader. These tests validate end-to-end functionality from a host's perspective, covering OTA protocol, firmware verification, and slot management.

---

## 1. Frameworks and Tools

The test suite uses the following Python libraries:

- **`pytest`** – Core test framework.
- **`pytest-asyncio`** – Enables testing of `asyncio` code (used for BLE).
- **`bleak`** – BLE communication backend used by `ota_host.py`.

---

## 2. Test Suite Architecture

The suite is structured into several key files:

### `conftest.py`

- Central configuration and fixtures for pytest.
- **Constants**: File paths, device names, color codes.
- **Helpers**: Reusable functions like `reboot_and_wait_for_ready()`, `get_bootloader_info()`, and `perform_simple_ota()`.
- **`ota_host_fixture`**: 
  - Connects to the device.
  - Reboots to a clean state.
  - Yields `OTAHost` object to tests.
  - Cleans up afterward.
- **Session Prerequisite Check**: 
  - Validates presence of firmware and signature files in `test_data/`.
  - If missing, the session fails immediately.

### `ota_host.py`

A high-level BLE interface to the bootloader.

**Responsibilities:**
- BLE device discovery and connection.
- Notification parsing (ACK/NACK, logs).
- OTA frame construction and transmission.
- Provides `perform_full_ota()` to run complete update sequences.

### `test_secure_ota_ble.py`

Tests for OTA update security and robustness.

**Scenarios:**
- Valid updates (standard, large firmware).
- Corrupted firmware/signatures.
- Firmware signed with an untrusted key.
- Signature/firmware mismatch.

### `test_slot_management.py`

Tests for bootloader CLI slot commands.

**Scenarios:**
- Manual slot activation via `activate <slot>`.
- Slot erasure via `erase <slot>`.
- Safety check: cannot erase the active slot.
- Memory validation with `print <slot>` (checks 0xFF fill and correct firmware writes).

---

## 3. Prerequisites

- **Hardware**:
  - STM32 device flashed with secure bootloader.
  - ESP32 WROOM device flashed with BLE bridge and connected over UART to the STM32 device. 
- **Test Data**: Run the scripts in `../utilities/` to generate keys and signed firmware in `../test_data/`.

---

## 4. How to Run the Tests

### Step 1: Create a Virtual Environment (Recommended)
It is highly recommended to use a Python virtual environment to manage project dependencies and avoid conflicts with system-wide packages. From the host_pc directory:
```bash
# Create the virtual environment
python3 -m venv .venv

# Activate it
source .venv/bin/activate
```
You will need to activate the virtual environment every time you open a new terminal to work on this project.

### Step 2: Install Dependencies

From the `host_pc/` directory:

```bash
pip install -r requirements.txt
```

### Step 3: Run the Test Suite
From the integration_tests/ directory:

```bash
# Run all tests 
python -m pytest -s 
```
To run a specific file:

```bash
python -m pytest -s  test_slot_management.py
```
To run a specific test:

```bash
python -m pytest -s -k test_manual_slot_activation
```

## 5. Understanding the Test Output
The test output uses a color-coded logging scheme to improve readability:

- **SETUP / TEARDOWN:** Indicates when the ota_host_fixture is connecting or disconnecting.
- **TEST START / END:** Marks the beginning and end of a specific test function.
- **DEV_LOG:** Decoded log messages received from the device's UART.
- **<-- ACK/NACK:** Parsed ACK/NACK responses from the device.
- **Waiting...:** Indicates the host is waiting for a specific response or log from the device.
- **ERROR / TIMEOUT:** Indicates an error condition, a failed assertion, or a timeout.
