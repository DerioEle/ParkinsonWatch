#include "utils.h"
#include <Wire.h>

bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    int got = Wire.requestFrom((int)addr, (int)len);
    if (got != (int)len) return false;
    for (int i = 0; i < (int)len; i++) buf[i] = Wire.read();
    return true;
}

bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t &val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)addr, 1) != 1) return false;
    val = Wire.read();
    return true;
}

int16_t bma12(int16_t msb, int16_t lsb) {
    int16_t v = (int16_t)((msb << 8) | lsb);
    v >>= 4;
    if (v & 0x0800) v |= 0xF000;
    return v;
}
