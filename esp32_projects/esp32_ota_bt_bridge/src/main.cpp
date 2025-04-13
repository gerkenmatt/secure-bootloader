#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define STM32_RX 16  // ESP32 receives from STM32 TX
#define STM32_TX 17  // ESP32 transmits to STM32 RX

void setup() {
  Serial.begin(115200);  // Debug log over USB
  Serial2.begin(115200, SERIAL_8N1, STM32_RX, STM32_TX);

  SerialBT.begin("ESP32_OTA");  // Bluetooth device name
  Serial.println("Bluetooth OTA bridge started");
}

void loop() {
  // Forward from Bluetooth → STM32
  while (SerialBT.available()) {
    char c = SerialBT.read();
    Serial2.write(c);
  }

  // Forward from STM32 → Bluetooth
  while (Serial2.available()) {
    char c = Serial2.read();
    SerialBT.write(c);
  }
}