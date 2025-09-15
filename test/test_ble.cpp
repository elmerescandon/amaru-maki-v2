#include <BLEDevice.h>
#include <BLEServer.h>

void setup() {
  BLEDevice::init("ESP32_Test");
  BLEServer *pServer = BLEDevice::createServer();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  pAdvertising->setScanResponse(true);           // Enable scan response
  pAdvertising->setMinPreferred(0x06);           // Helps macOS detect device
  pAdvertising->setMinPreferred(0x12);
  
  pAdvertising->start();                         // Start advertising
}

void loop() {
  // Keep running — ESP32 keeps advertising
}
