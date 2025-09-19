#include <BLE_Wearable.h>
#include <Wire.h>
#include "i2c_mux.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (16) // ~60Hz
const int LED_PIN = 2;  // Most ESP32 dev boards use GPIO 2 for the onboard LED

// Only Initializations
BLE_Wearable bleWearable("ESP32_Hello");
I2CMux mux(0x70);


// Sensor Initializations
Adafruit_BNO055 bno_shoulder = Adafruit_BNO055(-1, 0x29, &Wire);
Adafruit_BNO055 bno_elbow = Adafruit_BNO055(-1, 0x28, &Wire);
Adafruit_BNO055 bno_wrist = Adafruit_BNO055(-1, 0x28, &Wire); 
Adafruit_BNO055 bno[3] = {bno_shoulder, bno_elbow, bno_wrist};

uint8_t SENSORS_ACTIVE = 2;

// Data initialization
double lastQuat[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static unsigned long lastTime = 0;
static unsigned long currentTime = 0;


// Utility function to handle errors
void throwError() {
    while(1){
        digitalWrite(LED_PIN, LOW); // Turn off the built-in LED to indicate error
        delay(500);
        digitalWrite(LED_PIN, HIGH); // Turn on the built-in LED to indicate error
        delay(500);
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT); // Set LED pin as output
    Serial.begin(115200); // Initialize Serial Monitor

    // Initialize BLE
    bleWearable.begin();
    Serial.println("[INIT] BLE Wearable started!");
    delay(1000);

    // Initialize I2C Multiplexer
    Serial.println("[INIT] Starting I2C MUX setup...");
    if (!mux.begin()) {
        Serial.println("[ERROR] MUX not found, please check wiring! \n");
        throwError();
    }
    Serial.printf("[INIT] %d Sensors will be initialized \n", SENSORS_ACTIVE);
    for (uint8_t i = 0; i < SENSORS_ACTIVE; i++) {
        Serial.printf("[INIT] Setting up BNO055 sensor %d \n", i);
        uint8_t channel = i; // Assuming channels 0, 1, 2 for three sensors
        if (channel > 0) {
            channel = i - 1; // Skip channel 1 if not used
        }
        Serial.printf("[INIT] Selecting MUX channel %d \n", channel);
        if (!mux.selectChannel(channel)) {
            Serial.printf("[ERROR] Failed to select MUX channel %d \n", channel);
            throwError();
        }
        delay(100); // Short delay to ensure channel is set

        if (!bno[i].begin()) {
            Serial.printf("[INIT] BNO055 sensor %d not found, please restart the device! \n", i);
            throwError();
        }
        Serial.printf("[INIT] BNO055 sensor %d found! \n", i);
        bno[i].setExtCrystalUse(true);
        delay(1000);
    }
    Serial.println("[INIT] IMU Sensors initialized, let's read data! \n");
    digitalWrite(LED_PIN, HIGH); // Turn on the built-in LED to indicate successful setup

}

void loop() {
  lastTime = millis();
    for (uint8_t i = 0; i < SENSORS_ACTIVE; i++) {
      uint8_t channel = i; // Assuming channels 0, 1, 2 for three sensors
      if (channel > 0) {
          channel = i - 1; // Skip channel 1 if not used
      }
      if (!mux.selectChannel(channel)) {
          Serial.printf("[ERROR] Failed to select MUX channel %d \n", channel);
          throwError();
      }
      imu::Quaternion quat = bno[i].getQuat();
      lastQuat[i*4 + 0] = quat.w();
      lastQuat[i*4 + 1] = quat.x();
      lastQuat[i*4 + 2] = quat.y();
      lastQuat[i*4 + 3] = quat.z();
  }
  currentTime = millis();
  // Serial.printf("Loop Time: %lu ms\n", currentTime - lastTime);

  // Send quaternion data over BLE
  lastTime = millis();
  bleWearable.sendData((const uint8_t*)lastQuat, sizeof(lastQuat));
  currentTime = millis();
  // Serial.printf("BLE Send Time: %lu ms\n", currentTime - lastTime);
  // Serial.println("Sent quaternion data");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
