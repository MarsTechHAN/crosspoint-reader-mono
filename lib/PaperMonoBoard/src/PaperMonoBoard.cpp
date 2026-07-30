#include "PaperMonoBoard.h"

#include <Arduino.h>
#include <M5Pm1.h>
#include <Wire.h>

#include <algorithm>

namespace PaperMonoBoard {
namespace {
constexpr uint8_t IOE_ADDR = 0x6F;
constexpr uint8_t IOE_REG_MODE = 0x03;
constexpr uint8_t IOE_REG_OUT = 0x05;
constexpr uint8_t IOE_REG_PULLUP = 0x09;
constexpr uint8_t IOE_REG_PULLDOWN = 0x0B;
constexpr uint8_t IOE_REG_DRIVE = 0x13;

constexpr uint8_t IOE_EPD_POWER = 2;  // IO3
constexpr uint8_t IOE_EPD_RESET = 4;  // IO5
constexpr uint8_t IOE_TOUCH_RESET = 5;  // IO6
constexpr uint8_t IOE_LED_BLUE = 1;   // IO2
constexpr uint8_t IOE_LED_GREEN = 7;  // IO8
constexpr uint8_t IOE_TOUCH_POWER = 12;  // IO13
constexpr uint8_t IOE_SD_POWER = 13;  // IO14

constexpr uint8_t PMIC_GPIO3 = 1u << 3;
constexpr uint8_t PMIC_GPIO3_FUNC_MASK = 0xC0;
constexpr uint8_t PMIC_GPIO3_PWM0 = 0xC0;
constexpr uint8_t PMIC_PWM_ENABLE = 1u << 4;
constexpr uint16_t FRONTLIGHT_PWM_HZ = 5000;
constexpr uint32_t CIE_CUBIC_DENOMINATOR = 116u * 116u * 116u;

constexpr uint16_t OUTPUT_MASK = (1u << IOE_EPD_POWER) | (1u << IOE_EPD_RESET) |
                                 (1u << IOE_TOUCH_RESET) | (1u << IOE_TOUCH_POWER) |
                                 (1u << IOE_LED_BLUE) | (1u << IOE_LED_GREEN) |
                                 (1u << IOE_SD_POWER);

bool s_ready = false;
uint16_t s_output = 0;
uint8_t s_frontlightBrightness = 0;
bool s_wokeByPowerButton = false;
uint8_t s_wakeSource = 0;
uint8_t s_powerButtonConfig = 0;
bool s_ignorePowerButtonUntilRelease = false;
bool s_powerButtonWasDown = false;
unsigned long s_powerClickPulseUntil = 0;

// The UI percentage is CIE L* perceptual lightness, not raw LED current. The
// inverse CIE curve maps equal slider steps to roughly equal perceived changes:
// Y=L*/903.3 below the toe, otherwise Y=((L*+16)/116)^3.
constexpr uint16_t perceptualDuty12(uint8_t percent) {
  if (percent == 0) return 0;
  if (percent <= 8) {
    return static_cast<uint16_t>((static_cast<uint32_t>(percent) * 4095u + 451u) / 903u);
  }
  const uint32_t v = static_cast<uint32_t>(percent) + 16u;
  const uint64_t numerator = static_cast<uint64_t>(4095u) * v * v * v;
  return static_cast<uint16_t>((numerator + CIE_CUBIC_DENOMINATOR / 2u) / CIE_CUBIC_DENOMINATOR);
}

static_assert(perceptualDuty12(0) == 0);
static_assert(perceptualDuty12(20) >= 120 && perceptualDuty12(20) <= 123);
static_assert(perceptualDuty12(50) >= 753 && perceptualDuty12(50) <= 755);
static_assert(perceptualDuty12(100) == 4095);

bool read16(uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(IOE_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  delayMicroseconds(500);
  if (Wire.requestFrom(IOE_ADDR, static_cast<uint8_t>(2), static_cast<uint8_t>(true)) != 2) return false;
  const uint8_t lo = Wire.read();
  const uint8_t hi = Wire.read();
  value = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
  return true;
}

bool write16(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(IOE_ADDR);
  Wire.write(reg);
  Wire.write(static_cast<uint8_t>(value & 0xFF));
  Wire.write(static_cast<uint8_t>(value >> 8));
  if (Wire.endTransmission() != 0) return false;
  delayMicroseconds(500);
  return true;
}

bool clearBits(uint8_t reg, uint16_t mask) {
  uint16_t value = 0;
  return read16(reg, value) && write16(reg, static_cast<uint16_t>(value & ~mask));
}

void writePin(uint8_t pin, bool high) {
  if (!s_ready) return;
  if (high) {
    s_output |= static_cast<uint16_t>(1u << pin);
  } else {
    s_output &= static_cast<uint16_t>(~(1u << pin));
  }
  write16(IOE_REG_OUT, s_output);
}

bool writeFrontlightDuty(uint8_t percent) {
  percent = std::min<uint8_t>(percent, 100);
  const uint16_t duty12 = perceptualDuty12(percent);
  const uint8_t dutyAndControl[2] = {
      static_cast<uint8_t>(duty12 & 0xFF),
      static_cast<uint8_t>(((duty12 >> 8) & 0x0F) | (percent > 0 ? PMIC_PWM_ENABLE : 0)),
  };
  if (!freeink::m5pm1::writeBytes(freeink::m5pm1::REG_PWM0_DUTY_L, dutyAndControl,
                                  sizeof(dutyAndControl)) ||
      !freeink::m5pm1::writeReg16(freeink::m5pm1::REG_PWM_FREQ_L, FRONTLIGHT_PWM_HZ)) {
    return false;
  }
  s_frontlightBrightness = percent;
  return true;
}
}  // namespace

bool begin() {
  if (s_ready) return true;

  freeink::m5pm1::beginBus();
  uint8_t wakeSource = 0;
  if (freeink::m5pm1::readWakeSource(&wakeSource)) {
    s_wakeSource = wakeSource;
    s_wokeByPowerButton = (wakeSource & freeink::m5pm1::WAKE_PWR_BUTTON) != 0;
    freeink::m5pm1::clearWakeSource();
  }
  uint8_t buttonConfig = 0;
  freeink::m5pm1::configureAppPowerButton(&buttonConfig);
  s_powerButtonConfig = buttonConfig;
  uint8_t buttonStatus = 0;
  if (freeink::m5pm1::readButtonStatus(&buttonStatus)) {
    s_powerButtonWasDown = (buttonStatus & freeink::m5pm1::BTN_PRESSED) != 0;
  }
  // The button that powered the board on must not immediately send it back to
  // sleep when it is released after boot.
  s_ignorePowerButtonUntilRelease = s_wokeByPowerButton || s_powerButtonWasDown;
  // Paper Mono's red status LED is controlled by PWR_CFG bit 4. Keep charging
  // and the board's other PMIC policy untouched.
  freeink::m5pm1::updateReg(freeink::m5pm1::REG_PWR_CFG, 1u << 4, 0);
  // Paper Mono's frontlight is driven by M5PM1 GPIO3 alternate function PWM0.
  // Clear a duty value retained by the PMIC before routing PWM0 to the pin, so
  // the light cannot flash during boot before the first display refresh.
  if (!writeFrontlightDuty(0) ||
      !freeink::m5pm1::updateReg(freeink::m5pm1::REG_GPIO_FUNC0, PMIC_GPIO3_FUNC_MASK, PMIC_GPIO3_PWM0) ||
      !freeink::m5pm1::updateReg(freeink::m5pm1::REG_GPIO_DRV, PMIC_GPIO3, 0)) {
    return false;
  }

  uint16_t uid = 0;
  if (!read16(0x00, uid)) return false;

  uint16_t mode = 0;
  if (!read16(IOE_REG_MODE, mode) || !read16(IOE_REG_OUT, s_output)) return false;

  // Establish the inactive levels before changing the pins to outputs.
  s_output &= static_cast<uint16_t>(~OUTPUT_MASK);
  if (!write16(IOE_REG_OUT, s_output)) return false;
  if (!clearBits(IOE_REG_PULLUP, OUTPUT_MASK) || !clearBits(IOE_REG_PULLDOWN, OUTPUT_MASK) ||
      !clearBits(IOE_REG_DRIVE, OUTPUT_MASK)) {
    return false;
  }
  if (!write16(IOE_REG_MODE, static_cast<uint16_t>(mode | OUTPUT_MASK))) return false;

  s_ready = true;
  // Keep the frontlight dark through controller init and the first non-flashing
  // baseline update. main.cpp fades in only after that frame completes.
  if (!writeFrontlightDuty(0)) return false;
  enableTouch();
  return true;
}

bool ready() { return s_ready; }

bool setFrontlightBrightness(uint8_t percent) {
  return fadeFrontlightTo(percent);
}

bool fadeFrontlightTo(uint8_t percent, uint16_t durationMs) {
  if (!s_ready) return false;
  percent = std::min<uint8_t>(percent, 100);
  const int start = s_frontlightBrightness;
  const int delta = static_cast<int>(percent) - start;
  if (delta == 0) return true;

  const uint8_t steps = static_cast<uint8_t>(std::min(32, std::max(8, std::abs(delta))));
  const uint16_t stepDelay = durationMs / steps;
  for (uint8_t step = 1; step <= steps; ++step) {
    const int value = start + (delta * step + (delta >= 0 ? steps / 2 : -steps / 2)) / steps;
    if (!writeFrontlightDuty(static_cast<uint8_t>(std::max(0, std::min(100, value))))) return false;
    if (stepDelay > 0 && step != steps) delay(stepDelay);
  }
  return true;
}

uint8_t getFrontlightBrightness() { return s_frontlightBrightness; }

bool pollPowerButtonClick() {
  if (!s_ready) return false;
  const unsigned long now = millis();
  uint8_t status = 0;
  if (!freeink::m5pm1::readButtonStatus(&status)) {
    return now < s_powerClickPulseUntil;
  }

  const bool down = (status & freeink::m5pm1::BTN_PRESSED) != 0;
  const bool event = (status & freeink::m5pm1::BTN_EVENT) != 0;
  if (s_ignorePowerButtonUntilRelease) {
    s_powerButtonWasDown = down;
    if (!down) s_ignorePowerButtonUntilRelease = false;
    return false;
  }

  if (down) {
    s_powerButtonWasDown = true;
  } else if (s_powerButtonWasDown || event) {
    // Emit only after release. A long hold is therefore left exclusively to
    // the PMIC, which enters download mode before an app sleep can fire.
    s_powerButtonWasDown = false;
    s_powerClickPulseUntil = now + 90;
  }
  return now < s_powerClickPulseUntil;
}

bool wokeByPowerButton() { return s_wokeByPowerButton; }

uint8_t powerButtonConfig() { return s_powerButtonConfig; }

uint8_t wakeSource() { return s_wakeSource; }

bool requestPowerOff() { return s_ready && freeink::m5pm1::requestShutdown(); }

void setEpdPower(bool enabled) {
  writePin(IOE_EPD_POWER, enabled);
  delay(enabled ? 100 : 10);
}

void setEpdReset(bool high) {
  writePin(IOE_EPD_RESET, high);
  delay(1);
}

void enableTouch() {
  writePin(IOE_TOUCH_POWER, true);
  writePin(IOE_TOUCH_RESET, false);
  delay(8);
  writePin(IOE_TOUCH_RESET, true);
  delay(10);
}

void disableTouch() {
  writePin(IOE_TOUCH_RESET, false);
  writePin(IOE_TOUCH_POWER, false);
}

void enableSd() {
  writePin(IOE_SD_POWER, true);
  delay(20);
}

void disableSd() { writePin(IOE_SD_POWER, false); }

void powerDownForSleep() {
  fadeFrontlightTo(0, 260);
  disableTouch();
  disableSd();
}

void powerDownEpdForDeepSleepFallback() {
  // The normal path is an M5PM1 hard shutdown. Keep EPD power present after
  // the controller deep-sleep command and let the PMIC collapse the system
  // rails together. This explicit reset-then-power-off sequence is only for
  // the fallback, where M5IOE1 remains powered during ESP deep sleep.
  setEpdReset(false);
  setEpdPower(false);
}

}  // namespace PaperMonoBoard

extern "C" void freeink_board_epd_power(bool enabled) { PaperMonoBoard::setEpdPower(enabled); }
extern "C" void freeink_board_epd_reset(bool high) { PaperMonoBoard::setEpdReset(high); }
