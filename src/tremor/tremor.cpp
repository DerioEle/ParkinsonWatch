#include "tremor.h"
#include <arduinoFFT.h>
#include <bluetooth/bluetooth_service.h>

// Variáveis globais acessadas pela interface
float g_lastFreq = 0.0f;
bool g_lastTremor = false;

static const size_t N_SAMPLES = 128;       // ~1.28 s a 100 Hz
static double vReal[N_SAMPLES];
static double vImag[N_SAMPLES];
static size_t sampleIndex = 0;
static bool bufferFull = false;

void tremorAddSample(float mag) {
    vReal[sampleIndex] = mag;
    vImag[sampleIndex] = 0.0;
    sampleIndex++;
    if (sampleIndex >= N_SAMPLES) {
        sampleIndex = 0;
        bufferFull = true;
    }
}

void tremorProcess(TTGOClass *watch, float sampleRate) {
    if (!bufferFull) return;

    arduinoFFT FFT(vReal, vImag, N_SAMPLES, sampleRate);
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(FFT_FORWARD);
    FFT.ComplexToMagnitude();

    double totalEnergy = 0, bandEnergy = 0;
    double peakFreq = 0.0, peakMag = 0.0;

    for (size_t i = 1; i < N_SAMPLES / 2; i++) {
        double freq = (i * sampleRate) / N_SAMPLES;
        double mag2 = vReal[i] * vReal[i];
        totalEnergy += mag2;

        if (freq >= 4.0 && freq <= 6.0) {
            bandEnergy += mag2;
            if (vReal[i] > peakMag) {
                peakMag = vReal[i];
                peakFreq = freq;
            }
        }
    }

    double ratio = bandEnergy / (totalEnergy + 1e-9);
    bool tremorDetected = (ratio > 0.30);

    // Média da magnitude da janela
    double avgMag = 0.0;
    for (size_t i = 0; i < N_SAMPLES; i++) avgMag += vReal[i];
    avgMag /= N_SAMPLES;

    g_lastFreq = (float)peakFreq;
    g_lastTremor = tremorDetected;

    // Envio BLE com freq
    bleSendData(watch, (float)avgMag, (float)peakFreq, tremorDetected);
    extern void httpSendData(uint32_t timestamp, float mag, float freq, int battPercent, bool tremorDetected);

    uint32_t timestamp = millis();
    int battPercent = watch->power->getBattPercentage();
    httpSendData(timestamp, (float)avgMag, (float)peakFreq, battPercent, tremorDetected);

    if (tremorDetected) {
        Serial.printf("[TREMOR] Detectado! Energia 4–6Hz = %.2f%% | Freq = %.2f Hz\n",
                      ratio * 100, peakFreq);
        if (watch && watch->tft) {
            watch->tft->fillScreen(TFT_RED);
            watch->tft->setTextColor(TFT_WHITE, TFT_RED);
            watch->tft->setTextSize(2);
            watch->tft->setCursor(10, 60);
            watch->tft->println("Tremor Detectado");
            watch->tft->setTextSize(1);
            watch->tft->setCursor(10, 90);
            watch->tft->printf("Freq: %.2f Hz", peakFreq);
            delay(1000);
            watch->tft->fillScreen(TFT_BLACK);
        }
    } else {
        Serial.printf("[TREMOR] Normal. Energia 4–6Hz = %.2f%% | Freq = %.2f Hz\n",
                      ratio * 100, peakFreq);
    }

    bufferFull = false;
}
