# Security Overview

This document outlines the security features implemented in the STM32 secure bootloader, detailing the threats addressed and the cryptographic and architectural mitigations employed.

---

## 1. Security Goals

The primary security goals of this secure bootloader are:

* **Authenticity:** Ensure that firmware images originate only from a trusted source.
* **Integrity:** Guarantee that firmware images have not been tampered with during transit or storage.
* **Confidentiality:** *Not explicitly addressed in the current implementation, as firmware is transmitted in plaintext. Future work includes exploring code encryption, Memory Readout Protection (RDP), etc.*
* **Anti-Rollback:** Prevent malicious actors from downgrading the device to an older, potentially vulnerable firmware version.
* **Fault Tolerance:** Ensure that failed updates do not brick the device, maintaining a working system.

---

## 2. Threat Model & Mitigations

| Threat                                      | Description                                                                  | Mitigation Strategy                                                                     |
| :------------------------------------------ | :--------------------------------------------------------------------------- | :-------------------------------------------------------------------------------------- |
| **Unauthorized Firmware Installation** | An attacker attempts to load custom, unverified firmware onto the device.     | **ECDSA Signature Verification:** All incoming firmware images *must* be digitally signed with a private key known only to the firmware developer. The bootloader verifies this signature using an embedded public key. If the signature is invalid, the firmware is rejected. |
| **Firmware Tampering / Data Corruption** | Firmware data is altered during transmission or while stored in flash.        | **ECDSA Signature Verification:** The signature covers the entire firmware image, ensuring any alteration (even single bit flips) invalidates the signature, leading to rejection. <br> **CRC (does this count?):** -please update- |
| **Rollback Attacks** | An attacker forces the device to run an older, potentially vulnerable firmware version. | **Version Number Checks:** The bootloader compares the version of the new firmware with the currently active firmware. A new firmware must have a version number *greater than or equal to* the currently running version (or a specific policy for "same version" re-flashing). |
| **Device Bricking during Update** | Power loss or communication failure occurs during a firmware update.           | **Dual-Bank (A/B) Update:** New firmware is always written to the *inactive* flash slot. The currently running application remains untouched until the new firmware is fully received, verified, and marked as valid. Only then does the bootloader switch to the new slot. If an update fails, the device reverts to the last known good application. |
| **Side-Channel Attacks / Key Extraction** | Sophisticated attacks attempt to extract the embedded public key or internal cryptographic states. | *No specific hardware or software countermeasures against side-channel attacks are implemented in this project.*|
| **Physical Tampering** | Attacker gains physical access to the device (e.g., debugging ports).         | *No specific physical tamper detection or hardening (beyond software-based flash segregation and potential future RDP) is implemented in this project.*|

---

## 3. Cryptographic Primitives

* **Algorithm:** Elliptic Curve Digital Signature Algorithm (ECDSA)
* **Curve:** `secp256r1` (NIST P-256)
* **Hash Function:** SHA-256
* **Library:** `mbedTLS` (specifically `mbedtls_pk_verify` for signature verification)

The public key used for verification is directly embedded into the bootloader's flash memory as a C array. This static embedding provides a simple root of trust. *Future work includes evaluating the use of hardware-backed secure storage for the public key, and applying write protection to the bootloader's flash region to prevent its modification.*

---

## 4. Flash Protection & Memory Segregation

* **Bootloader Protection:** The bootloader code region in flash is logically separated from application regions. *While the current implementation relies on careful linker script configuration and software logic, future enhancements could include leveraging hardware features like Readout Protection (RDP) or Write Protection (WRP) (via STM32 option bytes) to prevent unauthorized access or modification of the bootloader itself, and Memory Protection Units (MPU) for finer-grained runtime access control.*
* **Application Slot Segregation:** Dedicated flash regions for Application Slot A and Slot B are defined within the linker script (.ld file). This logical separation ensures that each application's code and data are confined to its designated area. During an OTA update, new firmware is written exclusively to the inactive slot, preserving the integrity of the active application until the update is complete and validated. The bootloader's internal logic strictly enforces writes only to the intended inactive slot.
---

## 5. Future Security Enhancements (Considerations)

* **Root of Trust (RoT):** Integrate with hardware security modules (HSMs) or Secure Elements for true hardware-backed root of trust for key storage and cryptographic operations.
* **Secure Storage for Flags:** Utilize redundant non-volatile memory or hardware tamper-detection for bootloader flags (e.g., active slot, update status) to resist physical attacks.
* **Secure Debugging:** Disable JTAG/SWD or implement secure debugging protocols in production.
* **Firmware Encryption:** Encrypt firmware binaries for confidentiality during OTA transmission.
* **TLS/DTLS for BLE:** Implement a TLS/DTLS layer over BLE for enhanced transport security, especially if confidentiality is a concern.

---
