#include "http_service.h"
#include <WiFi.h>
#include <HTTPClient.h>

static bool wifiReady = false;

// --- Conecta à rede Wi-Fi ---
void wifiSetup(const char *ssid, const char *password) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.printf("[WiFi] Conectando em %s...\n", ssid);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        wifiReady = true;
        Serial.printf("\n[WiFi] Conectado! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        wifiReady = false;
        Serial.println("\n[WiFi] Falha ao conectar.");
    }
}

bool isWiFiConnected() {
    return wifiReady && WiFi.status() == WL_CONNECTED;
}

// --- Envia dados via HTTP POST ---
void httpSendData(uint32_t timestamp, float mag, float freq, int battPercent, bool tremorDetected) {
    if (!isWiFiConnected()) return;

    HTTPClient http;
    String url = "http://192.168.0.15:5000/data";  // <-- altere para o endereço do seu servidor
    http.begin(url);

    http.addHeader("Content-Type", "application/json");

    String payload = String("{\"t\":") + timestamp +
                     ",\"batt\":" + battPercent +
                     ",\"mag\":" + String(mag, 3) +
                     ",\"freq\":" + String(freq, 2) +
                     ",\"tremor\":" + (tremorDetected ? "1" : "0") + "}";

    int httpCode = http.POST(payload);

    if (httpCode > 0) {
        Serial.printf("[HTTP] Enviado! Código %d | Resposta: %s\n", httpCode, http.getString().c_str());
    } else {
        Serial.printf("[HTTP] Erro no envio: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
}
