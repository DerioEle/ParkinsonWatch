#include "bluetooth_service.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

static BLECharacteristic *tremorChar = nullptr;

void bleSetup() {
    BLEDevice::init("T-Watch Tremor Monitor");

    BLEServer *pServer = BLEDevice::createServer();

    // UUIDs personalizados (únicos, 128 bits)
    BLEService *pService = pServer->createService("12345678-1234-1234-1234-1234567890AB");

    tremorChar = pService->createCharacteristic(
        "87654321-4321-4321-4321-BA0987654321",
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );

    tremorChar->addDescriptor(new BLE2902());

    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(pService->getUUID());
    BLEDevice::startAdvertising();

    Serial.println("[BLE] Servidor BLE iniciado com UUIDs personalizados.");
}

void bleSendData(float mag, bool tremorDetected) {
    if (!tremorChar) return;

    char jsonBuffer[64];
    snprintf(jsonBuffer, sizeof(jsonBuffer),
             "{\"mag\":%.3f,\"tremor\":%d}", mag, tremorDetected ? 1 : 0);

    tremorChar->setValue((uint8_t*)jsonBuffer, strlen(jsonBuffer));
    tremorChar->notify();

    Serial.printf("[BLE] Enviado: %s\n", jsonBuffer);
}
