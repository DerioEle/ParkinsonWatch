#pragma once
#include <Arduino.h>
#include <LilyGoWatch.h>

void tremorAddSample(float mag);
void tremorProcess(TTGOClass *watch, float sampleRate);
