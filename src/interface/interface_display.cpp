#include "interface_display.h"

static const int GRAPH_WIDTH = 160;
static const int GRAPH_HEIGHT = 50;
static const int GRAPH_X = 40;
static const int GRAPH_Y = 160;

static float graphBuffer[GRAPH_WIDTH];
static int graphIndex = 0;

void initDisplay(TTGOClass *watch) {
    auto tft = watch->tft;
    if (!tft) return;
    tft->fillScreen(TFT_BLACK);
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->setTextSize(2);
    tft->setCursor(30, 10);
    tft->println("TREMOR MONITOR");
    tft->drawRect(GRAPH_X - 2, GRAPH_Y - 2, GRAPH_WIDTH + 4, GRAPH_HEIGHT + 4, TFT_DARKGREY);
    memset(graphBuffer, 0, sizeof(graphBuffer));
    graphIndex = 0;
}

void addToGraph(float mag) {
    mag = constrain(mag, 0.0f, 1.0f);
    if (graphIndex < GRAPH_WIDTH)
        graphBuffer[graphIndex++] = mag;
    else {
        memmove(graphBuffer, graphBuffer + 1, (GRAPH_WIDTH - 1) * sizeof(float));
        graphBuffer[GRAPH_WIDTH - 1] = mag;
    }
}

void updateDisplay(TTGOClass *watch, bool tremorDetected, float freq, int battPercent, bool bleConnected, bool useFakeData) {
    auto tft = watch->tft;
    if (!tft) return;

    // Barra de status
    uint16_t color = tremorDetected ? TFT_RED : TFT_GREEN;
    tft->fillRect(0, 40, 240, 40, color);
    tft->setTextColor(TFT_WHITE, color);
    tft->setTextSize(2);
    tft->setCursor(10, 50);
    tft->print(tremorDetected ? "TREMOR DETECTADO" : "NORMAL");

    // Informações secundárias
    tft->setTextSize(1);
    tft->setTextColor(TFT_YELLOW, TFT_BLACK);
    tft->setCursor(10, 90);
    tft->printf("Modo: %s", useFakeData ? "SIMULADO" : "REAL");

    tft->setCursor(120, 90);
    tft->printf("Freq: %.2f Hz", freq);

    tft->setCursor(10, 110);
    tft->printf("Bateria: %d%%", battPercent);

    tft->setCursor(150, 110);
    tft->setTextColor(bleConnected ? TFT_CYAN : TFT_DARKGREY, TFT_BLACK);
    tft->print(bleConnected ? "BLE ON" : "BLE OFF");

    // Hora (RTC)
    RTC_Date now = watch->rtc->getDateTime();
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->setTextSize(2);
    tft->setCursor(70, 130);
    tft->printf("%02d:%02d:%02d", now.hour, now.minute, now.second);

    // Gráfico
    tft->drawRect(GRAPH_X - 2, GRAPH_Y - 2, GRAPH_WIDTH + 4, GRAPH_HEIGHT + 4, TFT_DARKGREY);
    for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
        int x1 = GRAPH_X + i;
        int y1 = GRAPH_Y + GRAPH_HEIGHT - (int)(graphBuffer[i] * GRAPH_HEIGHT);
        int x2 = GRAPH_X + i + 1;
        int y2 = GRAPH_Y + GRAPH_HEIGHT - (int)(graphBuffer[i + 1] * GRAPH_HEIGHT);
        tft->drawLine(x1, y1, x2, y2, TFT_YELLOW);
    }
}
