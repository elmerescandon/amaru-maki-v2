#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>

BLEServer *pServer;
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// Custom UUIDs (you can generate your own at https://www.uuidgenerator.net/)
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-5678-90ab-cdef-1234567890ab"

// Callback class to track connection state
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->startAdvertising(); // Restart advertising after disconnect
  }
};

void setup() {
  Serial.begin(115200);

  // 1. Initialize BLE and set device name
  BLEDevice::init("ESP32_Hello");

  // 2. Create BLE server and set callbacks
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // 3. Create service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // 4. Create characteristic with Notify property
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // 5. Add a descriptor to allow notifications
  pCharacteristic->addDescriptor(new BLE2902());

  // 6. Start service and advertising
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("Waiting for a client to connect...");
    // Print MAC address to Serial Monitor
  Serial.print("ESP32 BLE MAC Address: ");
  Serial.println(BLEDevice::getAddress().toString().c_str());
}

void loop() {
  if (deviceConnected) {
    // Send Hello World every 2 seconds
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 2000) {
      lastTime = millis();
      pCharacteristic->setValue("Hello World");
      pCharacteristic->notify(); // Send notification to client
      Serial.println("Sent: Hello World");
    }
  }
}
