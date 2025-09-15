
#ifndef BLE_WEARABLE_H
#define BLE_WEARABLE_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>

class BLE_Wearable {
public:
	BLE_Wearable(const char* deviceName = "ESP32_Wearable");
	void begin();
	void sendData(const uint8_t* data, size_t length);
	bool isDeviceConnected() const;

private:
	BLEServer* pServer;
	BLECharacteristic* pCharacteristic;
	bool deviceConnected;
	class ServerCallbacks;
	static constexpr const char* SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab";
	static constexpr const char* CHARACTERISTIC_UUID = "abcd1234-5678-90ab-cdef-1234567890ab";
	const char* _deviceName;
};

#endif // BLE_WEARABLE_H
