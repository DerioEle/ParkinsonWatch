#pragma once
#include <Arduino.h>
#include <LilyGoWatch.h>

bool bma423_init();
void imu_readAndPrint(uint32_t t0, TTGOClass *watch);
