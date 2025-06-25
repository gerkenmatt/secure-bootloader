#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define STM32_RX 16  // ESP32 receives from STM32 TX
#define STM32_TX 17  // ESP32 transmits to STM32 RX
#define LED_PIN 2     // Onboard LED

unsigned long lastBlink = 0;
bool ledState = false;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, STM32_RX, STM32_TX);

  SerialBT.begin("ESP32_OTA");
  Serial.println("Bluetooth OTA bridge started");

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // UART bridge: Bluetooth → STM32
  while (SerialBT.available()) {
    Serial2.write(SerialBT.read());
  }

  // UART bridge: STM32 → Bluetooth
  while (Serial2.available()) {
    SerialBT.write(Serial2.read());
  }

  // Non-blocking LED blink every 500ms
  unsigned long now = millis();
  if (now - lastBlink >= 500) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }
}
