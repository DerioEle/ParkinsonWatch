#pragma once
#include <Arduino.h>
#include <LilyGoWatch.h>
#include "calibration/calibration.h"

// Declarações das funções de controle do loop
void handleButtonAndRecalibration();
bool handleSamplingTiming();
void runFakeDataMode();
void runRealDataMode();
