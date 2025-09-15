
#include <Arduino.h>
#include <BLE_Wearable.h>

BLE_Wearable bleWearable("ESP32_Hello");

void setup() {
  Serial.begin(115200);
  bleWearable.begin();
  Serial.println("Waiting for a client to connect...");
}

void loop() {
  if (bleWearable.isDeviceConnected()) {
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 2000) {
      lastTime = millis();
      Serial.println("Device connected, sending notifications...");
      const char* msg = "Hello World";
      bleWearable.sendData((const uint8_t*)msg, strlen(msg));
      Serial.println("Sent: Hello World");
    }
  }
}
