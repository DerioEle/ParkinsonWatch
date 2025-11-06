#pragma once
#include <Arduino.h>
#include <LilyGoWatch.h>

void initDisplay(TTGOClass *watch);
void updateDisplay(TTGOClass *watch, bool tremorDetected, float freq, int battPercent, bool bleConnected, bool useFakeData);
void addToGraph(float mag);
