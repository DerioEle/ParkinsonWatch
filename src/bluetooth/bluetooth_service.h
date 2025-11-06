#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <LilyGoWatch.h>

void bleSetup();
void bleSendData(TTGOClass *watch, float mag, float freq, bool tremorDetected);
extern NimBLEServer *g_bleServer;
extern bool g_bleConnected;
