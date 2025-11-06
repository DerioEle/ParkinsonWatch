#include "system/system_init.h"
#include "system/system_loop.h"
#include "interface/interface_display.h"

bool useFakeData = false;

void setup() {
    initSerial();
    initWatch();
    initDisplay(watch);
    showStartupScreen();
    initSensorsAndCalibration();

    Serial.println("[BLE DEBUG] Aguardando inicialização do T-Watch...");
    Serial.println("[BLE DEBUG] Esperando hardware estabilizar antes do BLE...");
    uint32_t startTime = millis();
    while (millis() - startTime < 5000) {
        if (watch && watch->tft && watch->power) {
            delay(100);
        }
    }
    startBLE();
}

void loop() {
    handleButtonAndRecalibration();
    if (!handleSamplingTiming()) return;

    if (useFakeData)
        runFakeDataMode();
    else
        runRealDataMode();
}
