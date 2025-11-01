#include "power.h"

void printAXP(TTGOClass *watch) {
    if (!watch || !watch->power) return;
    auto p = watch->power;
    p->adc1Enable(AXP202_VBUS_VOL_ADC1  | AXP202_VBUS_CUR_ADC1 |
                  AXP202_BATT_VOL_ADC1  | AXP202_BATT_CUR_ADC1, true);
    Serial.printf("[AXP] VBUS=%.0fmV,%.0fmA | BATT=%.0fmV,+%.0fmA\n",
                  p->getVbusVoltage(), p->getVbusCurrent(),
                  p->getBattVoltage(), p->getBattChargeCurrent());
}
