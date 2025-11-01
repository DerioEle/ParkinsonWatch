#pragma once
#include <Arduino.h>
#include <LilyGoWatch.h>

struct AccelCalibration {
    float offsetX;
    float offsetY;
    float offsetZ;
    bool valid;
};

void calibrateBMA423(AccelCalibration &calib);
void applyCalibration(float &ax, float &ay, float &az, const AccelCalibration &calib);
bool loadCalibrationFromNVS(AccelCalibration &calib);
void saveCalibrationToNVS(const AccelCalibration &calib);
void clearCalibrationNVS();

void handleRecalibration(TTGOClass *watch, AccelCalibration &calib);
