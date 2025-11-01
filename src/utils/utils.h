#pragma once
#include <Arduino.h>

bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t &val);
int16_t bma12(int16_t msb, int16_t lsb);
