# Host-Side Test Utilities

This directory contains Python scripts designed to automate the preparation of firmware for testing the secure bootloader. These utilities handle cryptographic key generation and the signing and packaging of firmware binaries into a structured format suitable for automated testing (pytest) or manual updates via the `ota_client`.

---

## Directory Structure
```
.
├── generate_keys.py # Script to create ECDSA key pairs.
├── generate_test_firmware.py # Script to sign and package firmware for testing.
├── valid_firmware/ # INPUT: Directory for placing compiled application binaries.
│ ├── blinky_slota.bin
│ ├── blinky_slotb.bin
│ └── ...
└── test_data/ # OUTPUT: Directory where all keys and signed firmware are generated.
├── ec_priv.pem
├── firmware_A_slota.bin
└── firmware_A_slota.sig
└── ...
```

---

## Scripts Overview

### `generate_keys.py`

This script creates the cryptographic keys required for signing and verification.

**Functionality:**
- Generates a primary `secp256r1` key pair: `ec_priv.pem`, `ec_pub.pem`, `ec_pub.der` (valid key).
- Generates a secondary private key: `ec_priv_bad.pem` (invalid key for negative test cases).
- Output: All keys are saved to the `--output-dir` (default: `test_data/`).

**Important:**  
The public key (`ec_pub.der`) must be converted to a C-array and embedded into the bootloader's `ota_crypto.c`. **If you generate new keys, you must recompile and re-flash the bootloader.**

---

### `generate_test_firmware.py`

This script signs and packages firmware binaries.

**Functionality:**
- Reads source binaries from `valid_firmware/`.
- Renames them into a consistent test format (e.g., `firmware_A_slota.bin`).
- Signs binaries using the primary "good" key.
- Creates special test cases, including:
  - Padded firmware
  - Firmware with invalid signatures
- Output: Signed `.bin` and `.sig` files saved to `--output-dir` (default: `test_data/`).

---

## Step-by-Step Workflow

Follow these steps to prepare your compiled applications for OTA updates or automated tests.

### Step 1: Compile and Place Firmware Binaries

Build the STM32 applications and copy the binaries into `valid_firmware/`.

```bash
# 1. Build applications
make -C stm32/applications/blinky all
make -C stm32/applications/uart_print all

# 2. Copy binaries into the utilities input directory
cp stm32/applications/blinky/build/blinky_slota.bin host_pc/utilities/valid_firmware/
cp stm32/applications/blinky/build/blinky_slotb.bin host_pc/utilities/valid_firmware/
cp stm32/applications/uart_print/build/uart_print_slota.bin host_pc/utilities/valid_firmware/
cp stm32/applications/uart_print/build/uart_print_slotb.bin host_pc/utilities/valid_firmware/
```
### Step 2: Generate Cryptographic Keys (First-Time Setup)
Run this once, or when generating a new key pair.
```bash
# From within host_pc/utilities/
python3 generate_keys.py --output-dir test_data
```
This creates key files in `test_data/`.

### Step 3: Generate Signed Test Firmware
Run the script to process all binaries in valid_firmware/.

```bash
# From within host_pc/utilities/
python3 generate_test_firmware.py --input-dir valid_firmware --output-dir test_data --key-dir test_data
```
### Step 4: Use the Generated Files
Artifacts in `test_data/` are now ready for use.

- **Manual Updates:** Use ota_client.py to send .bin and .sig files to the bootloader.
- **Automated Testing:** Run the pytest suite. It automatically uses files in test_data/ to validate the update process.

