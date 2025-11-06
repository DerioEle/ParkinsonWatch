#include "bluetooth_service.h"
#include <ArduinoJson.h>

NimBLEServer *g_bleServer = nullptr;
bool g_bleConnected = false;
static NimBLECharacteristic *tremorChar = nullptr;

// Classe de callbacks BLE
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override {
        g_bleConnected = true;
        Serial.println("[BLE] Cliente conectado!");
    }

    void onDisconnect(NimBLEServer* pServer) override {
        g_bleConnected = false;
        Serial.println("[BLE] Cliente desconectado!");
        pServer->startAdvertising();
    }
};

void bleSetup() {
    NimBLEDevice::init("T-Watch Tremor Monitor");
    Serial.printf("[BLE DEBUG] Device init OK @ %lu\n", millis());
    g_bleServer = NimBLEDevice::createServer();

    static ServerCallbacks *serverCallbacks = new ServerCallbacks();
    g_bleServer->setCallbacks(serverCallbacks);

    Serial.printf("[BLE DEBUG] Server criado em %p, callback %p\n", g_bleServer, serverCallbacks);

    NimBLEService *pService = g_bleServer->createService("12345678-1234-1234-1234-1234567890AB");

    tremorChar = pService->createCharacteristic(
        "87654321-4321-4321-4321-BA0987654321",
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    pService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(pService->getUUID());
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("[BLE] Servidor NimBLE iniciado e advertising ativo!");
}

void bleSendData(TTGOClass *watch, float mag, float freq, bool tremorDetected) {
    if (!tremorChar) return;

    uint32_t timestamp = millis();
    int battPercent = 0;
    if (watch && watch->power)
        battPercent = watch->power->getBattPercentage();

    char jsonBuffer[128];
    snprintf(jsonBuffer, sizeof(jsonBuffer),
             "{\"t\":%lu,\"batt\":%d,\"mag\":%.3f,\"freq\":%.2f,\"tremor\":%d}",
             (unsigned long)timestamp,
             battPercent,
             mag,
             freq,
             tremorDetected ? 1 : 0);

    tremorChar->setValue(jsonBuffer);
    tremorChar->notify();

    Serial.printf("[BLE] Enviado: %s\n", jsonBuffer);
}
