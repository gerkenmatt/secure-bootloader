# ESP32 Firmware Projects
This directory contains the embedded firmware projects for the ESP32 microcontroller, which serves a critical role in the `secure-bootloader` ecosystem.

## Project Overview
The `esp32` folder contains a single project:
- `esp32_ota_ble_bridge/`: This project turns an ESP32 into a dedicated Bluetooth Low Energy (BLE) to UART bridge.

## Purpose
The primary function of this bridge is to enable wireless communication between the host PC (running the test suite or OTA client) and the STM32 microcontroller (running the secure bootloader). The host communicates with the ESP32 over BLE, and the ESP32 then transparently forwards all data to the STM32's UART port, and vice-versa.
This setup allows for Over-the-Air (OTA) updates and interaction with the bootloader's command-line interface without requiring a physical USB connection to the STM32 board.

## Getting Started
All details regarding the ESP32 project's functionality, features, hardware wiring, and build instructions are located in its dedicated README file.
For full details, please see: [./esp32_ota_ble_bridge/README.md](./esp32_ota_ble_bridge/README.md)
