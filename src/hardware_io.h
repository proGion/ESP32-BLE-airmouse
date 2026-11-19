#ifndef HARDWARE_IO_H
#define HARDWARE_IO_H

#include <Arduino.h>

void initHardware();
void setLedState(bool state);
bool checkSleepMode();
void handleButtons();
void updateBatteryLevel();
#endif