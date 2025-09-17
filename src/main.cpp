#include <BLE_Wearable.h>
#include <Wire.h>
#include "i2c_mux.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
const int LED_PIN = 2;  // Most ESP32 dev boards use GPIO 2 for the onboard LED

// Only Initializations
BLE_Wearable bleWearable("ESP32_Hello");
I2CMux mux(0x70);


// Sensor Initializations
Adafruit_BNO055 bno_shoulder = Adafruit_BNO055(-1, 0x29, &Wire);
Adafruit_BNO055 bno_elbow = Adafruit_BNO055(-1, 0x28, &Wire);
Adafruit_BNO055 bno_wrist = Adafruit_BNO055(-1, 0x28, &Wire); 
Adafruit_BNO055 bno[2] = {bno_shoulder, bno_elbow }; // bno_wrist};

// Data initialization
double lastQuat[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


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
    throwError();
    }

    for (uint8_t i = 0; i < 3; i++) {
        Serial.printf("[INIT] Setting up BNO055 sensor %d \n", i);
        uint8_t channel = i; // Assuming channels 0, 1, 2 for three sensors
        if (channel > 0) {
            channel = i - 1; // Skip channel 1 if not used
        }
        if (!mux.selectChannel(channel)) {
            Serial.printf("[ERROR] Failed to select MUX channel %d \n", channel);
            throwError();
        }
        Serial.printf("[INIT] Selected MUX channel %d \n", i);
        delay(100); // Short delay to ensure channel is set

        if (!bno[i].begin()) {
            Serial.printf("[INIT] BNO055 sensor %d not found, please restart the device! \n", i);
            throwError();
        }
        bno[i].setExtCrystalUse(true);
        Serial.printf("[INIT] BNO055 sensor %d initialized. \n", i);
        delay(1000);
    }
    Serial.println("[INIT] IMU Sensors initialized, let's read data! \n");
    digitalWrite(LED_PIN, HIGH); // Turn on the built-in LED to indicate successful setup

}

void loop() {
//   if (bleWearable.isDeviceConnected()) {
//     static unsigned long lastTime = 0;
//     if (millis() - lastTime > 16) { // Send data at ~60Hz
//       lastTime = millis();
//       // Read Quaternion data from BNO055
//       Serial.println("Device connected, sending quaternion data...");
//       imu::Quaternion quat = bno.getQuat();
//       lastQuat[0] = quat.w();
//       lastQuat[1] = quat.x();
//       lastQuat[2] = quat.y();
//       lastQuat[3] = quat.z();
//       bleWearable.sendData((const uint8_t*)lastQuat, sizeof(lastQuat));
//       Serial.println("Sent quaternion data");
//     }
//   }
}
