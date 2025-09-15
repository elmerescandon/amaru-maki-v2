#ifndef I2C_MUX_H
#define I2C_MUX_H

#include <Arduino.h>

class I2CMux {
public:
    I2CMux(uint8_t address);
    bool begin();
    bool selectChannel(uint8_t channel);
private:
    uint8_t _address;
};

#endif // I2C_MUX_H
