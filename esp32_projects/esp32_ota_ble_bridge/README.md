# ESP32 OTA BLE Bridge

This project implements a BLE-to-UART bridge using an ESP32 and the NimBLE-Arduino stack. It enables wireless Over-the-Air (OTA) firmware updates to an STM32 microcontroller via Bluetooth Low Energy (BLE), using a custom UART-based OTA protocol.

## Features

- Bridges BLE to UART for OTA firmware transfers
- Uses Nordic UART Service (NUS) UUIDs for BLE compatibility
- Sends BLE writes directly to STM32 over UART
- Relays STM32 UART responses back to BLE central (host)
- Detects end-of-frame marker (`OTA_EOF`) and forwards complete frames
- Blinks onboard LED to indicate bridge activity
- Debug logging over USB serial

## BLE Service

- **Service UUID**: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- **RX Characteristic (Write)**: `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
  - Host writes OTA frames here
- **TX Characteristic (Notify)**: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
  - ESP32 notifies host with UART responses

## Wiring

| ESP32 Pin | Function          | Connects To STM32 |
|-----------|-------------------|-------------------|
| GPIO16    | UART RX (input)   | TX                |
| GPIO17    | UART TX (output)  | RX                |
| GPIO2     | Onboard LED       | Status Indicator  |

## OTA Protocol Markers

- `OTA_SOF`: `0xA5` — Start of Frame
- `OTA_EOF`: `0xB6` — End of Frame

## Build Requirements

- PlatformIO or Arduino IDE
- NimBLE-Arduino library

## Usage

1. Flash this firmware to your ESP32.
2. Power up the STM32 and connect it via UART to the ESP32.
3. Connect to the ESP32 over BLE using a mobile app or Python script.
4. Send OTA commands to the RX characteristic.
5. Receive STM32 responses via the TX characteristic.

## Debugging

- USB serial logs available at 115200 baud.
- `[UART → BLE]` messages show forwarded responses from STM32.
- The onboard LED blinks every 500ms to indicate normal operation.
