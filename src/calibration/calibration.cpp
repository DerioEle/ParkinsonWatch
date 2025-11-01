#include "calibration.h"
#include "utils/utils.h"
#include <Wire.h>
#include <Preferences.h>
#define BTN_PIN 36  // GPIO36 = botão lateral

static const uint8_t BMA_ADDR = 0x19;
static const uint8_t BMA_DATA_START = 0x12;
static const float LSB_TO_G = 2.0f / 2048.0f;

Preferences prefs;

void saveCalibrationToNVS(const AccelCalibration &calib) {
    prefs.begin("calib", false);
    prefs.putFloat("offx", calib.offsetX);
    prefs.putFloat("offy", calib.offsetY);
    prefs.putFloat("offz", calib.offsetZ);
    prefs.putBool("valid", true);
    prefs.end();
    Serial.println("[NVS] Calibração salva na memória flash.");
}

bool loadCalibrationFromNVS(AccelCalibration &calib) {
    prefs.begin("calib", true);
    bool valid = prefs.getBool("valid", false);
    if (valid) {
        calib.offsetX = prefs.getFloat("offx", 0.0f);
        calib.offsetY = prefs.getFloat("offy", 0.0f);
        calib.offsetZ = prefs.getFloat("offz", 0.0f);
        calib.valid = true;
        Serial.printf("[NVS] Calibração carregada: X=%.4f Y=%.4f Z=%.4f\n",
                      calib.offsetX, calib.offsetY, calib.offsetZ);
    } else {
        calib.valid = false;
        Serial.println("[NVS] Nenhuma calibração salva encontrada.");
    }
    prefs.end();
    return calib.valid;
}

void clearCalibrationNVS() {
    prefs.begin("calib", false);
    prefs.clear();
    prefs.end();
    Serial.println("[NVS] Calibração apagada manualmente!");
}

void calibrateBMA423(AccelCalibration &calib) {
    Serial.println("[CAL] Iniciando calibração...");
    const int samples = 200;
    float sumX = 0, sumY = 0, sumZ = 0;

    for (int i = 0; i < samples; i++) {
        uint8_t buf[6];
        if (i2cReadBytes(BMA_ADDR, BMA_DATA_START, buf, 6)) {
            int16_t rx = (int16_t)((buf[1] << 8) | buf[0]) >> 4;
            if (rx & 0x0800) rx |= 0xF000;
            int16_t ry = (int16_t)((buf[3] << 8) | buf[2]) >> 4;
            if (ry & 0x0800) ry |= 0xF000;
            int16_t rz = (int16_t)((buf[5] << 8) | buf[4]) >> 4;
            if (rz & 0x0800) rz |= 0xF000;

            float ax = rx * LSB_TO_G;
            float ay = ry * LSB_TO_G;
            float az = rz * LSB_TO_G;

            sumX += ax;
            sumY += ay;
            sumZ += az;
        }
        delay(10);
    }

    calib.offsetX = sumX / samples;
    calib.offsetY = sumY / samples;
    calib.offsetZ = (sumZ / samples) - 1.0f; // remove gravidade
    calib.valid = true;

    Serial.printf("[CAL] OffsetX=%.4f, OffsetY=%.4f, OffsetZ=%.4f\n",
                  calib.offsetX, calib.offsetY, calib.offsetZ);
    Serial.println("[CAL] Calibração concluída!");
    saveCalibrationToNVS(calib);
}

void applyCalibration(float &ax, float &ay, float &az, const AccelCalibration &calib) {
    if (!calib.valid) return;
    ax -= calib.offsetX;
    ay -= calib.offsetY;
    az -= calib.offsetZ;
}

void handleRecalibration(TTGOClass *watch, AccelCalibration &calib) {
    static bool waitingConfirm = false;
    static uint32_t confirmStart = 0;
    const uint32_t CONFIRM_TIMEOUT = 5000; // 5 s

    // --- Detecta o toque curto do botão via AXP202 ---
    if (watch->power->isPEKShortPressIRQ()) {
        Serial.println("[USER] Botão detectado via AXP202!");
        watch->power->clearIRQ();  // limpa a flag

        if (!waitingConfirm) {
            // Primeira pressão → pede confirmação
            waitingConfirm = true;
            confirmStart = millis();

            if (watch->tft) {
                watch->tft->fillScreen(TFT_BLACK);
                watch->tft->setTextColor(TFT_YELLOW, TFT_BLACK);
                watch->tft->setTextSize(2);
                watch->tft->setCursor(10, 20);
                watch->tft->println("Recalibrar?");
                watch->tft->setTextSize(1);
                watch->tft->setCursor(10, 60);
                watch->tft->println("Pressione o botao");
                watch->tft->setCursor(10, 75);
                watch->tft->println("novamente p/ confirmar");
            }
            Serial.println("[USER] Pressione novamente para confirmar recalibracao.");
        } 
        else if (millis() - confirmStart < CONFIRM_TIMEOUT) {
            // Segunda pressão dentro do tempo → recalibra
            waitingConfirm = false;
            clearCalibrationNVS();

            if (watch->tft) {
                watch->tft->fillScreen(TFT_BLACK);
                watch->tft->setTextColor(TFT_YELLOW, TFT_BLACK);
                watch->tft->setTextSize(2);
                watch->tft->setCursor(10, 40);
                watch->tft->println("Recalibrando...");
                watch->tft->setTextSize(1);
                watch->tft->setCursor(10, 70);
                watch->tft->println("Mantenha o relogio parado");
            }

            calibrateBMA423(calib);

            if (watch->tft) {
                watch->tft->fillScreen(TFT_BLACK);
                watch->tft->setTextColor(TFT_GREEN, TFT_BLACK);
                watch->tft->setTextSize(2);
                watch->tft->setCursor(10, 40);
                watch->tft->println("Recalibrado!");
                delay(1500);
                watch->tft->fillScreen(TFT_BLACK);
            }

            Serial.println("[USER] Recalibracao confirmada e concluida!");
        }
    }

    // --- Cancela se o usuário não confirmar em 5 s ---
    if (waitingConfirm && millis() - confirmStart > CONFIRM_TIMEOUT) {
        waitingConfirm = false;
        if (watch->tft) {
            watch->tft->fillScreen(TFT_BLACK);
            watch->tft->setTextColor(TFT_RED, TFT_BLACK);
            watch->tft->setTextSize(2);
            watch->tft->setCursor(10, 40);
            watch->tft->println("Recalibracao");
            watch->tft->setCursor(10, 65);
            watch->tft->println("cancelada!");
            delay(1000);
            watch->tft->fillScreen(TFT_BLACK);
        }
        Serial.println("[USER] Recalibracao cancelada (sem confirmacao).");
    }
}