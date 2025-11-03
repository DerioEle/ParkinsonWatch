#include "filters/filters.h"
#include "power/power.h"
#include "utils/utils.h"
#include "calibration/calibration.h"
#include "tremor/tremor.h"
#include "system/system_init.h"
#include "bluetooth/bluetooth_service.h"


bool useFakeData = false;  // false = sensor real, true = dados simulados
// Filtros globais
DCRemover dcx(0.01f), dcy(0.01f), dcz(0.01f);
OnePoleLPF lpx(0.20f), lpy(0.20f), lpz(0.20f);

void setup() {
    initSerial();
    initWatch();
    showStartupScreen();
    initSensorsAndCalibration();
    startBLE();
}

void loop() {
    // --- Atualiza IRQs e recalibração ---
    watch->power->readIRQ();
    handleRecalibration(watch, calib);

    // --- Alterna entre modo real/simulado (pressione e segure 2 s) ---
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

    // --- Controle de tempo de amostragem ---
    static uint32_t t0 = millis();
    static uint32_t next = 0;
    const uint32_t SAMPLE_MS = 10; // 100 Hz
    uint32_t now = millis();
    if ((int32_t)(now - next) < 0) return;
    next = now + SAMPLE_MS;

    float ax_hp = 0.0f, ay_hp = 0.0f, az_hp = 0.0f;
    float mag = 0.0f;

    // =================================================================
    // MODO SIMULADO
    // =================================================================
    if (useFakeData) {
        static float t = 0.0f;
        const float f = 5.0f;   // tremor 5 Hz
        const float dt = 0.01f; // 100 Hz

        // Gera aceleração simulada mais forte
        float ax = 0.7f * sinf(2 * PI * f * t);
        float ay = 0.7f * cosf(2 * PI * f * t);
        float az = -1.0f + 0.2f * sinf(2 * PI * f * t);

        t += dt;
        if (t > 1.0f / f) t = 0.0f;

        // Calcula magnitude diretamente (sem filtros)
        float mag = 1.0f + 0.8f * sinf(2 * PI * 5.0f * t);  // 5 Hz tremor

        // Saída serial
        uint32_t tms = millis() - t0;
        Serial.printf("%lu,%.6f,%.6f,%.6f,%.6f\n",
                    (unsigned long)tms, ax, ay, az, mag);

        // Processa FFT e BLE
        tremorAddSample(mag);
        tremorProcess(watch, 100.0f);

        return;
    }

    // =================================================================
    // MODO REAL
    // =================================================================
    uint8_t buf[6];
    if (!i2cReadBytes(0x19, 0x12, buf, 6)) {
        Serial.println("[IMU] Falha na leitura I2C");
        return;
    }

    // Conversão correta (12 bits)
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

    ax_hp = lpx.apply(dcx.apply(ax));
    ay_hp = lpy.apply(dcy.apply(ay));
    az_hp = lpz.apply(dcz.apply(az));
    mag = sqrtf(ax_hp * ax_hp + ay_hp * ay_hp + az_hp * az_hp);

    // Processamento e envio
    tremorAddSample(mag);
    tremorProcess(watch, 100.0f);

    // Saída serial CSV
    uint32_t tms = millis() - t0;
    Serial.printf("%lu,%.6f,%.6f,%.6f,%.6f\n",
                  (unsigned long)tms, ax_hp, ay_hp, az_hp, mag);

    // Telemetria de energia a cada 2s
    static uint32_t lastP = 0;
    if (now - lastP > 2000) {
        lastP = now;
        printAXP(watch);
    }
}
