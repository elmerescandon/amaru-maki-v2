#include <Wire.h>
#include "i2c_mux.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

void setup() //This code is executed once
{
	Serial.begin(115200);
	Serial.println("Starting I2C MUX setup...");
    // Initialize I2C MUX (PCA9548A) at default address 0x70
    I2CMux mux(0x70);
    if (!mux.begin()) {
        Serial.begin(115200);
        Serial.println("I2C MUX not found!");
        while (1);
    }
    // Select channel 0 for the first BNO055
    if (!mux.selectChannel(0)) {
        Serial.println("Failed to select MUX channel 0");
        while (1);
    }

	Serial.println("I2C MUX initialized.");
	/* Initialise the sensor */
	if(!bno.begin()){
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}

	delay(3000);

	/* Display the current temperature */
	int8_t temp = bno.getTemp();
	Serial.print("Current Temperature: ");
	Serial.print(temp);
	Serial.println(" C");
	Serial.println("");

	bno.setExtCrystalUse(true);

	Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

}

void loop() //This code is looped forever
{
  //Blank
}