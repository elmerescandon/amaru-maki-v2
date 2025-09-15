#include "BLE_Wearable.h"

// Internal callback class to handle connection events
class BLE_Wearable::ServerCallbacks : public BLEServerCallbacks {
public:
	ServerCallbacks(BLE_Wearable* parent) : _parent(parent) {}
	void onConnect(BLEServer* pServer) override {
		_parent->deviceConnected = true;
	}
	void onDisconnect(BLEServer* pServer) override {
		_parent->deviceConnected = false;
		pServer->startAdvertising();
	}
private:
	BLE_Wearable* _parent;
};

BLE_Wearable::BLE_Wearable(const char* deviceName)
	: pServer(nullptr), pCharacteristic(nullptr), deviceConnected(false), _deviceName(deviceName) {}

void BLE_Wearable::begin() {
	BLEDevice::init(_deviceName);
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new ServerCallbacks(this));

	BLEService* pService = pServer->createService(SERVICE_UUID);
	pCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID,
		BLECharacteristic::PROPERTY_NOTIFY
	);
	pCharacteristic->addDescriptor(new BLE2902());
	pService->start();
	BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->setScanResponse(true);
	pAdvertising->start();
}

void BLE_Wearable::sendData(const uint8_t* data, size_t length) {
	if (deviceConnected && pCharacteristic) {
		std::string value(reinterpret_cast<const char*>(data), length);
		pCharacteristic->setValue(value);
		pCharacteristic->notify();
	}
}

bool BLE_Wearable::isDeviceConnected() const {
	return deviceConnected;
}
