#include <Arduino.h>
#include <LilyGoWatch.h>
#include "imu/imu.h"
#include "filters/filters.h"
#include "power/power.h"
#include "utils/utils.h"
#include "calibration/calibration.h"
#include "tremor/tremor.h"

bool useFakeData = false;  // false = sensor real, true = dados simulados

TTGOClass *watch = nullptr;
AccelCalibration calib;   // estrutura para armazenar os offsets de calibração

void setup() {
    Serial.begin(115200);
    delay(200);

    // Inicializa o T-Watch
    watch = TTGOClass::getWatch();
    watch->begin();
    auto power = watch->power;
    power->setPowerOutPut(AXP202_LDO2, AXP202_ON);  // Garante AXP ativo
    power->enableIRQ(AXP202_PEK_SHORTPRESS_IRQ | AXP202_PEK_LONGPRESS_IRQ, true);
    power->clearIRQ();
    Serial.println("[AXP] IRQ do botão habilitada");
    watch->openBL();

    // Tela inicial
    if (watch->tft) {
        watch->tft->fillScreen(TFT_BLACK);
        watch->tft->setTextColor(TFT_GREEN, TFT_BLACK);
        watch->tft->setTextSize(2);
        watch->tft->setCursor(10, 10);
        watch->tft->println("T-Watch 2020 V2");
        watch->tft->setTextSize(1);
        watch->tft->setTextColor(TFT_WHITE, TFT_BLACK);
        watch->tft->println("Inicializando...");
    }

    // Inicializa o barramento I2C e sensores
    // Inicializa o sensor
    Wire.begin();
    bma423_init();
    printAXP(watch);

    // Tenta carregar calibração da NVS
    if (loadCalibrationFromNVS(calib)) {
        Serial.println("[SYS] Calibração carregada da NVS.");
    } else {
        // Se não houver calibração salva, realiza uma nova
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

    // Cabeçalho CSV
    Serial.println("t_ms,ax_g,ay_g,az_g,mag_g");
}

void loop() {
  // Atualiza flags de interrupção do AXP202
  watch->power->readIRQ();
  handleRecalibration(watch, calib);
  if (watch->power->isPEKLongPressIRQ()) {   // Pressione e segure 2 s
      useFakeData = !useFakeData;            // alterna o modo
      watch->power->clearIRQ();

      if (watch->tft) {
          watch->tft->fillScreen(useFakeData ? TFT_BLUE : TFT_GREEN);
          watch->tft->setTextColor(TFT_WHITE, useFakeData ? TFT_BLUE : TFT_GREEN);
          watch->tft->setTextSize(2);
          watch->tft->setCursor(10, 60);
          if (useFakeData)
              watch->tft->println("MODO TESTE ATIVO");
          else
              watch->tft->println("MODO REAL ATIVO");
          delay(1200);
          watch->tft->fillScreen(TFT_BLACK);
      }

      Serial.printf("[MODE] Modo alterado: %s\n", useFakeData ? "SIMULADO" : "REAL");
  }
  watch->power->clearIRQ();
  static uint32_t t0 = millis();
  static uint32_t next = 0;

  const uint32_t SAMPLE_MS = 10;  // ~100 Hz
  uint32_t now = millis();
  if ((int32_t)(now - next) < 0) return;
  next = now + SAMPLE_MS;

  float mag;
  float ax_hp = 0.0f, ay_hp = 0.0f, az_hp = 0.0f;
if (useFakeData) {
    // --- MODO SIMULADO: gera tremor de 5 Hz ---
    static float t = 0.0f;
    mag = sin(2 * PI * 5.0f * t);  // simula tremor Parkinson 5 Hz
    t += 0.01f;                    // 100 Hz amostragem
} else {
    // --- MODO REAL: lê dados do BMA423 ---
    uint8_t buf[6];
    if (!i2cReadBytes(0x19, 0x12, buf, 6)) return;

    int16_t rx = (int16_t)((buf[1] << 8) | buf[0]) >> 4;
    if (rx & 0x0800) rx |= 0xF000;
    int16_t ry = (int16_t)((buf[3] << 8) | buf[2]) >> 4;
    if (ry & 0x0800) ry |= 0xF000;
    int16_t rz = (int16_t)((buf[5] << 8) | buf[4]) >> 4;
    if (rz & 0x0800) rz |= 0xF000;

    const float LSB_TO_G = 2.0f / 2048.0f;
    float ax = rx * LSB_TO_G;
    float ay = ry * LSB_TO_G;
    float az = rz * LSB_TO_G;

    applyCalibration(ax, ay, az, calib);

    static DCRemover  dcx(0.01f), dcy(0.01f), dcz(0.01f);
    static OnePoleLPF lpx(0.20f), lpy(0.20f), lpz(0.20f);

    float ax_hp = lpx.apply(dcx.apply(ax));
    float ay_hp = lpy.apply(dcy.apply(ay));
    float az_hp = lpz.apply(dcz.apply(az));

    mag = sqrtf(ax_hp * ax_hp + ay_hp * ay_hp + az_hp * az_hp);
}
  // Envia o valor simulado para o analisador de tremor
  tremorAddSample(mag);

  tremorProcess(watch, 100.0f);  // taxa de amostragem de ~100 Hz

  // Saída serial CSV
  uint32_t tms = millis() - t0;
  Serial.printf("%lu,%.6f,%.6f,%.6f,%.6f\n", (unsigned long)tms, ax_hp, ay_hp, az_hp, mag);

  // (Opcional) Telemetria de energia a cada 2 s
  static uint32_t lastP = 0;
  if (now - lastP > 2000) {
      lastP = now;
      printAXP(watch);
  }
}