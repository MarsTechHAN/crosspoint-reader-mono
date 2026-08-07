#pragma once

#include <Arduino.h>
#include <BatteryMonitor.h>
#include <InputManager.h>
#include <Logging.h>
#include <freertos/semphr.h>

#include <cassert>

#include "HalGPIO.h"

class HalPowerManager;
extern HalPowerManager powerManager;  // Singleton

class HalPowerManager {
  int normalFreq = 0;  // MHz
  bool isLowPower = false;

  mutable int _batteryCachedPercent = 0;          // Last read battery percentage (0-100)
  mutable bool _batteryCachedPercentValid = false; // False until the first successful read, so the
                                                   // notch hysteresis doesn't anchor itself to the 0 seed
  mutable uint16_t _batteryCachedMillivolts = 0;  // Last read cell voltage; 0 = never read
  mutable bool _batteryCachedCharging = false;
  mutable bool _batteryCachedChargingKnown = false;
  mutable unsigned long _batteryLastPollMs = 0;  // Timestamp of last battery read in milliseconds

  // Samples every battery field the board can report, at most once per
  // BATTERY_POLL_MS, and updates the cache above.
  void pollBattery() const;

  enum LockMode { None, NormalSpeed };
  LockMode currentLockMode = None;
  SemaphoreHandle_t modeMutex = nullptr;  // Protect access to currentLockMode

 public:
#if BOARD_HAS_PSRAM
  static constexpr int LOW_POWER_FREQ = 80;  // MHz
#else
  static constexpr int LOW_POWER_FREQ = 10;  // MHz
#endif
  static constexpr unsigned long IDLE_POWER_SAVING_MS = 3000;  // ms
  static constexpr unsigned long BATTERY_POLL_MS = 1500;       // ms

  void begin();

  // Control CPU frequency for power saving
  void setPowerSaving(bool enabled);

  // Setup wake up GPIO and enter deep sleep
  // Should be called inside main loop() to handle the currentLockMode
  void startDeepSleep(HalGPIO& gpio) const;

  // Get battery percentage (range 0-100). On boards without a fuel gauge this is
  // derived from the cell voltage and is therefore always a multiple of 10.
  uint16_t getBatteryPercentage() const;

  // Get the cell voltage in millivolts, 0 when the board can't report one. This
  // is the raw figure behind the percentage, for the voltage readout setting.
  uint16_t getBatteryMillivolts() const;

  bool isCharging() const;

  // RAII helper class to manage power saving locks
  // Usage: create an instance of Lock in a scope to disable power saving, for example when running a task that needs
  // full performance. When the Lock instance is destroyed (goes out of scope), power saving will be re-enabled.
  class Lock {
    friend class HalPowerManager;
    bool valid = false;

   public:
    explicit Lock();
    ~Lock();

    // Non-copyable and non-movable
    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;
    Lock(Lock&&) = delete;
    Lock& operator=(Lock&&) = delete;
  };
};
