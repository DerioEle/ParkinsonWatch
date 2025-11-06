#include "system_loop.h"
#include "imu/imu.h"
#include "tremor/tremor.h"
#include "bluetooth/bluetooth_service.h"
#include "filters/filters.h"
#include "power/power.h"
#include "utils/utils.h"
#include "interface/interface_display.h"

extern TTGOClass *watch;
extern AccelCalibration calib;
extern bool useFakeData;

// Controle de amostragem
static uint32_t t0 = millis();
static uint32_t nextSample = 0;
static const uint32_t SAMPLE_MS = 10; // 100 Hz

DCRemover dcx(0.01f), dcy(0.01f), dcz(0.01f);
OnePoleLPF lpx(0.20f), lpy(0.20f), lpz(0.20f);

// -----------------------------------------------------------------------------
// 1. Lida com botões e recalibração
// -----------------------------------------------------------------------------
void handleButtonAndRecalibration() {
    watch->power->readIRQ();
    handleRecalibration(watch, calib);

    if (watch->power->isPEKLongPressIRQ()) {
        useFakeData = !useFakeData;
        watch->power->clearIRQ();

        if (watch->tft) {
            watch->tft->fillScreen(useFakeData ? TFT_BLUE : TFT_GREEN);
            watch->tft->setTextColor(TFT_WHITE, useFakeData ? TFT_BLUE : TFT_GREEN);
            watch->tft->setTextSize(2);
            watch->tft->setCursor(10, 60);
            watch->tft->println(useFakeData ? "MODO TESTE ATIVO" : "MODO REAL ATIVO");
            delay(1200);
            watch->tft->fillScreen(TFT_BLACK);
        }

        Serial.printf("[MODE] Modo alterado: %s\n", useFakeData ? "SIMULADO" : "REAL");
    }

    watch->power->clearIRQ();
}

// -----------------------------------------------------------------------------
// 2. Controla a taxa de amostragem (~100 Hz)
// -----------------------------------------------------------------------------
bool handleSamplingTiming() {
    static uint32_t next = 0;
    uint32_t now = millis();

    if ((int32_t)(now - next) < 0) {
        return false;
    }
    next = now + SAMPLE_MS;

    // --- DEBUG: mostra estado BLE ---
    bool bleConnected = (NimBLEDevice::getServer() && NimBLEDevice::getServer()->getConnectedCount() > 0);

    return true;
}


// -----------------------------------------------------------------------------
// 3. Modo de teste (simulação de tremor Parkinsoniano 5 Hz)
// -----------------------------------------------------------------------------
void runFakeDataMode() {
    static float t = 0.0f;
    const float f = 5.0f;   // frequência do tremor
    const float dt = 0.01f; // 100 Hz

    float ax = 0.7f * sinf(2 * PI * f * t);
    float ay = 0.7f * cosf(2 * PI * f * t);
    float az = -1.0f + 0.2f * sinf(2 * PI * f * t);

    t += dt;
    if (t > 1.0f / f) t = 0.0f;

    // Magnitude simulada com tremor forte + ruído leve
    float mag = 1.0f + 0.8f * sinf(2 * PI * 5.0f * t)
                      + 0.05f * ((float)random(-100, 100) / 100.0f);

    extern float g_lastFreq;       // crie variável global em tremor.cpp
    extern bool g_lastTremor;      // mesma coisa
    static float lastFreq = 0.0f;
    static bool lastTremor = false;
    lastFreq = g_lastFreq;
    lastTremor = g_lastTremor;

    tremorAddSample(mag);
    tremorProcess(watch, 100.0f);

    // Atualiza gráfico e display
    addToGraph(mag);
    bool bleConnected = (NimBLEDevice::getServer() && NimBLEDevice::getServer()->getConnectedCount() > 0);

    updateDisplay(
        watch,
        true,               // em modo simulado, sempre exibe "tremor"
        lastFreq,               // frequência simulada
        watch->power->getBattPercentage(),
        bleConnected,
        useFakeData
    );

    delay(SAMPLE_MS);
}


// -----------------------------------------------------------------------------
// 4. Modo real (leitura do BMA423)
// -----------------------------------------------------------------------------
void runRealDataMode() {
    uint8_t buf[6];
    if (!i2cReadBytes(0x19, 0x12, buf, 6)) {
        Serial.println("[IMU] Falha na leitura I2C");
        return;
    }

    // Conversão de 12 bits
    auto bma12 = [](uint8_t msb, uint8_t lsb) -> int16_t {
        int16_t raw = ((int16_t)msb << 8) | lsb;
        raw >>= 4;
        if (raw & 0x0800) raw |= 0xF000;
        return raw;
    };

    int16_t rx = bma12(buf[1], buf[0]);
    int16_t ry = bma12(buf[3], buf[2]);
    int16_t rz = bma12(buf[5], buf[4]);

    const float LSB_TO_G = 2.0f / 2048.0f;
    float ax = rx * LSB_TO_G;
    float ay = ry * LSB_TO_G;
    float az = rz * LSB_TO_G;

    applyCalibration(ax, ay, az, calib);

    float ax_hp = lpx.apply(dcx.apply(ax));
    float ay_hp = lpy.apply(dcy.apply(ay));
    float az_hp = lpz.apply(dcz.apply(az));
    float mag = sqrtf(ax_hp * ax_hp + ay_hp * ay_hp + az_hp * az_hp);

    tremorAddSample(mag);
    tremorProcess(watch, 100.0f);

    // Adiciona ao gráfico
    addToGraph(mag);

    // Obtém dados de estado para exibição
    static float lastFreq = 0.0f;
    static bool lastTremor = false;

    // Últimos valores vêm da análise FFT no tremorProcess
    extern float g_lastFreq;       // crie variável global em tremor.cpp
    extern bool g_lastTremor;      // mesma coisa

    lastFreq = g_lastFreq;
    lastTremor = g_lastTremor;

    bool bleConnected = (NimBLEDevice::getServer() && NimBLEDevice::getServer()->getConnectedCount() > 0);
    updateDisplay(
        watch,
        lastTremor,
        lastFreq,
        watch->power->getBattPercentage(),
        bleConnected,
        useFakeData
    );

    static uint32_t lastP = 0;
    if (millis() - lastP > 2000) {
        lastP = millis();
        printAXP(watch);
    }
}
