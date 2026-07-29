#pragma once

#include <cstdint>

namespace PaperMonoBoard {

bool begin();
bool ready();
bool setFrontlightBrightness(uint8_t percent);
bool fadeFrontlightTo(uint8_t percent, uint16_t durationMs = 320);
uint8_t getFrontlightBrightness();
bool pollPowerButtonClick();
bool wokeByPowerButton();
uint8_t powerButtonConfig();
uint8_t wakeSource();
bool requestPowerOff();
void setEpdPower(bool enabled);
void setEpdReset(bool high);
void enableTouch();
void disableTouch();
void enableSd();
void disableSd();
void powerDownForSleep();

}  // namespace PaperMonoBoard
