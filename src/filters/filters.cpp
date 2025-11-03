#include "filters.h"
#include <Arduino.h>

// Definidos em main.cpp
extern DCRemover dcx, dcy, dcz;
extern OnePoleLPF lpx, lpy, lpz;

void resetFilters() {
    dcx = DCRemover(0.01f);
    dcy = DCRemover(0.01f);
    dcz = DCRemover(0.01f);
    lpx = OnePoleLPF(0.20f);
    lpy = OnePoleLPF(0.20f);
    lpz = OnePoleLPF(0.20f);
    Serial.println("[FILTER] Filtros reinicializados após recalibração.");
}
