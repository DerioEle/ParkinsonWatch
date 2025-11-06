#pragma once
#include <Arduino.h>
#include <LilyGoWatch.h>
#include "calibration/calibration.h"
#include "interface/interface_display.h"

extern TTGOClass *watch;
extern AccelCalibration calib;

void initSerial();
void initWatch();
void showStartupScreen();
void initSensorsAndCalibration();
void startBLE();