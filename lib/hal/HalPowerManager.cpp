#include "HalPowerManager.h"

#include <BoardConfig.h>
#include <Logging.h>
#include <PowerManager.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include <soc/soc_caps.h>

#include <cassert>

#include "HalGPIO.h"

#if FREEINK_DEVICE_PAPERMONO
#include <PaperMonoBoard.h>
#endif

HalPowerManager powerManager;  // Singleton instance

void HalPowerManager::begin() {
  if (BoardConfig::ACTIVE.batteryAdc >= 0) {
    pinMode(BoardConfig::ACTIVE.batteryAdc, INPUT);
  }
  normalFreq = getCpuFrequencyMhz();
  modeMutex = xSemaphoreCreateMutex();
  assert(modeMutex != nullptr);
}

void HalPowerManager::setPowerSaving(bool enabled) {
  if (normalFreq <= 0) {
    return;  // invalid state
  }

  auto wifiMode = WiFi.getMode();
  if (wifiMode != WIFI_MODE_NULL) {
    // Wifi is active, force disabling power saving
    enabled = false;
  }

  // Note: We don't use mutex here to avoid too much overhead,
  // it's not very important if we read a slightly stale value for currentLockMode
  const LockMode mode = currentLockMode;

  if (mode == None && enabled && !isLowPower) {
    LOG_DBG("PWR", "Going to low-power mode");
    if (!setCpuFrequencyMhz(LOW_POWER_FREQ)) {
      LOG_DBG("PWR", "Failed to set CPU frequency = %d MHz", LOW_POWER_FREQ);
      return;
    }
    isLowPower = true;

  } else if ((!enabled || mode != None) && isLowPower) {
    LOG_DBG("PWR", "Restoring normal CPU frequency");
    if (!setCpuFrequencyMhz(normalFreq)) {
      LOG_DBG("PWR", "Failed to set CPU frequency = %d MHz", normalFreq);
      return;
    }
    isLowPower = false;
  }

  // Otherwise, no change needed
}

void HalPowerManager::startDeepSleep(HalGPIO& gpio) const {
#ifdef ENABLE_SERIAL_LOG
  // Tear down HWCDC so the host sees a clean disconnect and the peripheral
  // doesn't hold power domains that interfere with USB-powered GPIO wake.
  // logSerial is the raw HWCDC reference; Serial is the MySerialImpl proxy
  // (which doesn't expose end()).
  logSerial.end();
#endif

#if !SOC_PM_SUPPORT_EXT1_WAKEUP
  if (gpio.isXteinkDevice() && !gpio.deviceIsX3()) {
    // X4 GPIO13 is connected to the battery latch MOSFET. Keeping it low powers
    // the MCU off on battery, while the SDK wake source still handles USB power.
    constexpr gpio_num_t GPIO_SPIWP = GPIO_NUM_13;
    gpio_set_direction(GPIO_SPIWP, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_SPIWP, 0);
    gpio_hold_en(GPIO_SPIWP);
  }
#endif

#if FREEINK_DEVICE_PAPERMONO
  // The display controller is already asleep and switched peripherals were
  // quiesced immediately before it. Let M5PM1 collapse the system rails
  // together; do not separately cut EPD power first.
  if (PaperMonoBoard::requestPowerOff()) {
    delay(1000);  // normally power disappears during this delay
  }

  // Reaching here means hard shutdown did not take effect. M5IOE1 remains
  // alive in the fallback, so hold EPD reset low before cutting its rail.
  PaperMonoBoard::powerDownEpdForDeepSleepFallback();
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  while (digitalRead(2) == LOW || digitalRead(3) == LOW) delay(20);
  freeink::PowerManager::armWakeOnPins((1ULL << 2) | (1ULL << 3), true);
  freeink::PowerManager::deepSleep();
#else
  // Cut the gated peripheral rails (touch/SD/EPD on boards like the Sticky) and
  // hold the enables off through deep sleep — otherwise the GT911 and SD card
  // stay powered all through "off" and drain the battery. No-op on boards with
  // no switched rails (X4/X3). Trade-off: no touch-to-wake; wake is the power
  // button. Must run after display.deepSleep() so the panel controller gets its
  // deep-sleep command while its rail is still up (enterDeepSleep() in main.cpp
  // guarantees that ordering).
  freeink::PowerManager::powerDownRailsForSleep();

  // Waits for the power button to be physically released (so holding it doesn't
  // immediately wake the device again), then arms the wake source and sleeps.
  freeink::PowerManager::deepSleepUntilPowerButton();
#endif
}

uint16_t HalPowerManager::getBatteryPercentage() const {
  static const BatteryMonitor battery;
  const bool checkedTelemetry = BoardConfig::ACTIVE.batteryGauge.gaugeAddr != 0 ||
                                BoardConfig::isM5StackPaperColor() || BoardConfig::isPaperMono();
  if (checkedTelemetry) {
    const unsigned long now = millis();
    if (_batteryLastPollMs != 0 && (now - _batteryLastPollMs) < BATTERY_POLL_MS) {
      return _batteryCachedPercent;
    }

    _batteryLastPollMs = now;
    if (BoardConfig::isM5StackPaperColor() || BoardConfig::isPaperMono()) {
      const BatteryMonitor::Status status = battery.readStatus();
      // Preserve the last valid state across a transient I2C failure. A real
      // unplug is a successful PMIC sample with chargingKnown=true/false, while
      // treating an unreadable sample as "not charging" makes the icon flicker.
      if (status.chargingKnown) {
        _batteryCachedChargingKnown = true;
        _batteryCachedCharging = status.charging;
      }
      if (!status.percentageKnown) {
        LOG_ERR("PWR", "M5PM1 battery telemetry unavailable; retaining %d%%", _batteryCachedPercent);
        return _batteryCachedPercent;
      }
      _batteryCachedPercent = status.percentage;
      LOG_DBG("PWR", "M5PM1 battery: %u%% %umV ext=%d vin=%ldmV usb=%ldmV src=%d",
              status.percentage, status.millivolts, status.externalPowerKnown ? status.externalPower : -1,
              static_cast<long>(status.pm1VinMv), static_cast<long>(status.pm1VinOutMv), status.pm1PowerSource);
    } else {
      uint16_t percent = 0;
      if (!battery.readPercentageChecked(percent)) {
        return _batteryCachedPercent;
      }
      _batteryCachedPercent = percent;
    }
    return _batteryCachedPercent;
  }

  // smooth the battery %.
  if (_batteryCachedPercent == 0) {
    _batteryCachedPercent = 10 * battery.readPercentage();
  } else {
    _batteryCachedPercent = (_batteryCachedPercent * 9 + battery.readPercentage() * 10) / 10;
  }
  return _batteryCachedPercent / 10;
}

bool HalPowerManager::isCharging() const {
  // Refreshes percentage and charging from one cached PMIC transaction.
  getBatteryPercentage();
  return _batteryCachedChargingKnown && _batteryCachedCharging;
}

HalPowerManager::Lock::Lock() {
  xSemaphoreTake(powerManager.modeMutex, portMAX_DELAY);
  // Current limitation: only one lock at a time
  if (powerManager.currentLockMode != None) {
    LOG_ERR("PWR", "Lock already held, ignore");
    valid = false;
  } else {
    powerManager.currentLockMode = NormalSpeed;
    valid = true;
  }
  xSemaphoreGive(powerManager.modeMutex);
  if (valid) {
    // Immediately restore normal CPU frequency if currently in low-power mode
    powerManager.setPowerSaving(false);
  }
}

HalPowerManager::Lock::~Lock() {
  xSemaphoreTake(powerManager.modeMutex, portMAX_DELAY);
  if (valid) {
    powerManager.currentLockMode = None;
  }
  xSemaphoreGive(powerManager.modeMutex);
}
