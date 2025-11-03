#pragma once
#include <Arduino.h>

void bleSetup();
void bleSendData(float mag, bool tremorDetected);
