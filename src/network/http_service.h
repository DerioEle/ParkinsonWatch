#pragma once
#include <Arduino.h>

void wifiSetup(const char *ssid, const char *password);
void httpSendData(uint32_t timestamp, float mag, float freq, int battPercent, bool tremorDetected);
bool isWiFiConnected();
