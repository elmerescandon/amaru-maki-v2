#include <BLE_Wearable.h>
#include <Wire.h>
#include "i2c_mux.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

const int LED_PIN = 2;  // Most ESP32 dev boards use GPIO 2 for the onboard LED

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);
BLE_Wearable bleWearable("ESP32_Hello");
double lastQuat[4] = {0, 0, 0, 0};

void setup() {
  // Initialize Serial Monitor
  pinMode(LED_PIN, OUTPUT); // Set LED pin as output
  Serial.begin(115200);

  // Initialize BLE
  bleWearable.begin();
  Serial.println("1) BLE Wearable started!");
  delay(1000);

  // Initialize I2C Multiplexer
  Serial.println("Starting I2C MUX setup...");
  I2CMux mux(0x70);
  if (!mux.begin()) {
		while(1){
      digitalWrite(LED_PIN, LOW); // Turn off the built-in LED to indicate error
      delay(500);
      digitalWrite(LED_PIN, HIGH); // Turn on the built-in LED to indicate error
      delay(500);
    }
  }
  Serial.println("2) I2C MUX found!");
  delay(1000);


  // Initialize BNO055 Sensor
  Serial.println("Starting BNO055 setup...");
	if(!bno.begin()){
		Serial.print("BNO055 not found, please restart the device!");
		while(1){
      digitalWrite(LED_PIN, LOW); // Turn off the built-in LED to indicate error
      delay(500);
      digitalWrite(LED_PIN, HIGH); // Turn on the built-in LED to indicate error
      delay(500);
    }
	}
  bno.setExtCrystalUse(true);
	Serial.println("3) BNO055 initialized, let's read data!");
  digitalWrite(LED_PIN, HIGH); // Turn on the built-in LED to indicate successful setup
  delay(1000);

}

void loop() {
  if (bleWearable.isDeviceConnected()) {
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 100) {
      lastTime = millis();
      // Read Quaternion data from BNO055
      Serial.println("Device connected, sending quaternion data...");
      imu::Quaternion quat = bno.getQuat();
      lastQuat[0] = quat.w();
      lastQuat[1] = quat.x();
      lastQuat[2] = quat.y();
      lastQuat[3] = quat.z();
      bleWearable.sendData((const uint8_t*)lastQuat, sizeof(lastQuat));
      Serial.println("Sent quaternion data");
    }
  }
}
