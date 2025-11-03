#include "system_init.h"
#include "imu/imu.h"
#include "filters/filters.h"
#include "power/power.h"
#include "bluetooth/bluetooth_service.h"
#include "calibration/calibration.h"

TTGOClass *watch = nullptr;
AccelCalibration calib;

void initSerial() {
    Serial.begin(115200);
    delay(200);
    Serial.println("[SYS] Serial iniciado @115200");
}

void initWatch() {
    watch = TTGOClass::getWatch();
    watch->begin();
    watch->openBL();

    auto power = watch->power;
    power->setPowerOutPut(AXP202_LDO2, AXP202_ON);
    power->enableIRQ(AXP202_PEK_SHORTPRESS_IRQ | AXP202_PEK_LONGPRESS_IRQ, true);
    power->clearIRQ();
    Serial.println("[AXP] IRQ do botão habilitada");
}

void showStartupScreen() {
    if (!watch || !watch->tft) return;

    watch->tft->fillScreen(TFT_BLACK);
    watch->tft->setTextColor(TFT_GREEN, TFT_BLACK);
    watch->tft->setTextSize(2);
    watch->tft->setCursor(10, 10);
    watch->tft->println("T-Watch 2020 V2");
    watch->tft->setTextSize(1);
    watch->tft->setTextColor(TFT_WHITE, TFT_BLACK);
    watch->tft->println("Inicializando...");
}

void initSensorsAndCalibration() {
    Wire.begin();
    bma423_init();
    printAXP(watch);

    if (loadCalibrationFromNVS(calib)) {
        Serial.println("[SYS] Calibração carregada da NVS.");
    } else {
        if (watch->tft) {
            watch->tft->fillScreen(TFT_BLACK);
            watch->tft->setTextColor(TFT_YELLOW, TFT_BLACK);
            watch->tft->setTextSize(2);
            watch->tft->setCursor(20, 40);
            watch->tft->println("Calibrando...");
            watch->tft->setTextSize(1);
            watch->tft->setCursor(10, 70);
            watch->tft->println("Mantenha o relogio parado");
        }
        calibrateBMA423(calib);
    }

    if (watch->tft) {
        watch->tft->fillScreen(TFT_BLACK);
        watch->tft->setTextColor(TFT_GREEN, TFT_BLACK);
        watch->tft->setCursor(10, 40);
        watch->tft->println("Calibracao concluida!");
        delay(1000);
        watch->tft->fillScreen(TFT_BLACK);
        watch->tft->setCursor(10, 10);
        watch->tft->setTextColor(TFT_WHITE, TFT_BLACK);
        watch->tft->println("Streaming CSV iniciado");
        watch->tft->println("Abra o Serial @115200");
    }

    Serial.println("t_ms,ax_g,ay_g,az_g,mag_g");
}

void startBLE() {
    bleSetup();
    Serial.println("[BLE] Servidor inicializado e em modo de anúncio.");
}
