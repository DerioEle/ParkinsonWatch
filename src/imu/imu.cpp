#include "imu.h"
#include "filters/filters.h"
#include "power/power.h"
#include "utils/utils.h"
#include <Wire.h>
#include <math.h>

static const uint8_t BMA_ADDR = 0x19;
static const uint8_t BMA_DATA_START = 0x12;
static const float LSB_TO_G = 2.0f / 2048.0f;
static const uint32_t SAMPLE_MS = 10;

bool bma423_init() {
    // Soft reset
    Wire.beginTransmission(0x19);
    Wire.write(0x7E);
    Wire.write(0xB6);
    Wire.endTransmission();
    delay(10);

    // Ativa modo normal
    Wire.beginTransmission(0x19);
    Wire.write(0x7D);
    Wire.write(0x04);  // modo normal (enable accelerometer)
    Wire.endTransmission();
    delay(10);

    // Faixa ±2g
    Wire.beginTransmission(0x19);
    Wire.write(0x41);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    // Banda / ODR = 100Hz
    Wire.beginTransmission(0x19);
    Wire.write(0x40);
    Wire.write(0x0A);
    Wire.endTransmission();
    delay(10);

    // Confirma leitura do chip ID
    Wire.beginTransmission(0x19);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(0x19, 1);
    uint8_t chipId = Wire.read();
    Serial.printf("[IMU] Chip ID: 0x%02X (esperado 0x13)\n", chipId);

    if (chipId != 0x13) {
        Serial.println("[IMU] Erro: BMA423 não respondeu corretamente.");
        return false;
    }

    Serial.println("[IMU] BMA423 inicializado e ativo (±2g, 100Hz).");
    return true;
}


void imu_readAndPrint(uint32_t t0, TTGOClass *watch) {
    static DCRemover  dcx(0.1f), dcy(0.1f), dcz(0.1f);
    static OnePoleLPF lpx(0.20f), lpy(0.20f), lpz(0.20f);
    static uint32_t next = 0;

    uint32_t now = millis();
    if ((int32_t)(now - next) < 0) return;
    next = now + SAMPLE_MS;

    uint8_t buf[6];
    if (!i2cReadBytes(BMA_ADDR, BMA_DATA_START, buf, 6)) return;

    int16_t rx = bma12(buf[1], buf[0]);
    int16_t ry = bma12(buf[3], buf[2]);
    int16_t rz = bma12(buf[5], buf[4]);

    float ax = rx * LSB_TO_G;
    float ay = ry * LSB_TO_G;
    float az = rz * LSB_TO_G;

    float ax_hp = lpx.apply(dcx.apply(ax));
    float ay_hp = lpy.apply(dcy.apply(ay));
    float az_hp = lpz.apply(dcz.apply(az));
    float mag = sqrtf(ax_hp*ax_hp + ay_hp*ay_hp + az_hp*az_hp);

    uint32_t tms = millis() - t0;
    Serial.printf("%lu,%.6f,%.6f,%.6f,%.6f\n", tms, ax_hp, ay_hp, az_hp, mag);
}
