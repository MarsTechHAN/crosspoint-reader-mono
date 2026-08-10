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
// True when the PMIC reports this boot was caused by a GPIO edge (EXT_WAKE) —
// on Paper Mono that is the BMI270's INT1 on PM_G4 (raise-to-wake).
bool wokeByMotion();
// Arm/disarm system wake on a rising edge of PM_G4 (IMU INT1). The PM1 keeps
// this config across shutdown, so disabling the feature must call this with
// false — a stale enable would keep waking the sleeping device on every bump.
bool setMotionWake(bool enable);
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
void powerDownEpdForDeepSleepFallback();

}  // namespace PaperMonoBoard
