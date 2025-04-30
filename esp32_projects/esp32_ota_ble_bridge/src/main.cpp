#include <Arduino.h>
#include <NimBLEDevice.h>

// Define STM32 UART pins
#define STM32_RX 16  // ESP32 receives from STM32 TX
#define STM32_TX 17  // ESP32 transmits to STM32 RX
#define LED_PIN 2    // Onboard LED for status blinking
#define OTA_SOF 0xA5
#define OTA_EOF 0xB6

// BLE UART characteristic handles
NimBLECharacteristic *txChar;  // ESP32 → Host (Notify)
NimBLECharacteristic *rxChar;  // Host → ESP32 (Write)

unsigned long lastBlink = 0;
bool ledState = false;

// === Callback for BLE writes ===
// When the host writes to the RX characteristic, forward data to STM32
class RxCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pCharacteristic) override {
    std::string data = pCharacteristic->getValue();
    Serial2.write((uint8_t*)data.data(), data.length());
  }
};

void setup() {
  // Start USB serial for debugging
  Serial.begin(115200);

  // Start UART to STM32
  Serial2.begin(115200, SERIAL_8N1, STM32_RX, STM32_TX);

  // LED setup
  pinMode(LED_PIN, OUTPUT);

  // Initialize BLE
  NimBLEDevice::init("ESP32_OTA_BLE");  // BLE device name
  NimBLEServer *server = NimBLEDevice::createServer();

  // Create a custom BLE UART-like service (Nordic UART Service UUID)
  NimBLEService *uartService = server->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

  // RX characteristic: host writes commands here → ESP32 → STM32
  rxChar = uartService->createCharacteristic(
    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rxChar->setCallbacks(new RxCallback());

  // TX characteristic: ESP32 notifies host with data received from STM32
  txChar = uartService->createCharacteristic(
    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
    NIMBLE_PROPERTY::NOTIFY
  );

  // Start BLE service and advertising
  uartService->start();
  server->getAdvertising()->start();

  Serial.println("BLE OTA bridge started");
}

void loop() {
  static String buffer;

  while (Serial2.available()) {
    char c = Serial2.read();
    buffer += c;

    if (c == '\n') {
      txChar->setValue((uint8_t*)buffer.c_str(), buffer.length());
      txChar->notify();
      Serial.print("[UART → BLE] "); Serial.println(buffer);
      buffer = "";
    }
    if ((uint8_t)c == OTA_EOF) {
      txChar->setValue((uint8_t*)buffer.c_str(), buffer.length());
      txChar->notify();
      buffer = "";
    }
  }

  delay(1); // yield to BLE stack
}

// void loop() {
//   static unsigned long lastLog = 0;
//   static String buffer;

//   // Send dummy data every 2 seconds
//   if (millis() - lastLog >= 2000) {
//     lastLog = millis();
//     const char* fakeLog = "Fake log line from ESP32\r\n";
//     txChar->setValue((uint8_t*)fakeLog, strlen(fakeLog));
//     txChar->notify();
//     Serial.println("[DEBUG] Sent dummy log");
//   }

//   delay(10); // allow BLE and stack time
// }