#include "i2c_mux.h"
#include <Wire.h>

I2CMux::I2CMux(uint8_t address) : _address(address) {}

bool I2CMux::begin() {
    Wire.begin();
    // Optionally, check if device responds
    Wire.beginTransmission(_address);
    return (Wire.endTransmission() == 0);
}

bool I2CMux::selectChannel(uint8_t channel) {
    if (channel > 7) return false; // PCA9548A has 8 channels (0-7)
    Wire.beginTransmission(_address);
    Wire.write(1 << channel); // Select channel
    return (Wire.endTransmission() == 0);
}
