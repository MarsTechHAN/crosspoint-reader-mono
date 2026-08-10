#pragma once

#include <Arduino.h>
#include <Imu.h>

// TODO: Move enums into new header and share with CrossPointSettings.h
namespace CrossPointOrientation {
enum Value : uint8_t { PORTRAIT = 0, LANDSCAPE_CW = 1, INVERTED = 2, LANDSCAPE_CCW = 3 };
}

namespace CrossPointTiltPageTurn {
enum Value : uint8_t { TILT_OFF = 0, TILT_NORMAL = 1, TILT_INVERTED = 2 };
}

class HalTiltSensor;
extern HalTiltSensor halTiltSensor;  // Singleton

class HalTiltSensor {
  bool _available = false;
  // Full IMU init is deferred until a feature needs it: the BMI270's mandatory
  // config upload costs ~0.8 s on a cold boot, which every PMIC-shutdown wake
  // would otherwise pay even with all IMU features disabled. begin() only
  // probes presence (one WHO_AM_I read).
  bool _started = false;
  mutable Imu _sdkImu;

  // Tilt gesture state machine
  bool _tiltForwardEvent = false;  // Consumed by wasTiltedForward()
  bool _tiltBackEvent = false;     // Consumed by wasTiltedBack()
  bool _hadActivity = false;       // Non-consuming flag for sleep timer
  bool _inTilt = false;            // Currently tilted past threshold
  bool _isAwake = false;           // Tracks power state
  unsigned long _initMs = 0;       // Timestamp of sensor init
  unsigned long _lastTiltMs = 0;   // Debounce / cooldown
  unsigned long _wakeMs = 0;       // Timestamp of last wake() for stabilization

  // Tuning constants
  static constexpr float RATE_THRESHOLD_DPS = 270.0f;      // Deg/sec speed to trigger flick
  static constexpr float NEUTRAL_RATE_DPS = 50.0f;         // Must stop moving below this rate before next trigger
  static constexpr unsigned long COOLDOWN_MS = 600;        // Minimum ms between triggers
  static constexpr unsigned long POLL_INTERVAL_MS = 50;    // 20 Hz polling
  static constexpr unsigned long WAKE_STABILIZE_MS = 300;  // Ignore readings after wake

  mutable unsigned long _lastPollMs = 0;

  // Face-down watch (screen-down sleep): sampled at 2 Hz whenever enabled.
  // Axis/sign pending hardware verification — see FACE_DOWN_SIGN below.
  static constexpr unsigned long FACE_DOWN_POLL_MS = 500;
  static constexpr float FACE_DOWN_MIN_G = 0.75f;        // gravity mostly on the screen normal
  static constexpr float FACE_DOWN_MAX_ORTHO_G = 0.35f;  // and NOT on the in-plane axes
  static constexpr float FACE_DOWN_SIGN = 1.0f;          // +az = screen down (flip after testing)
  unsigned long _lastFaceDownPollMs = 0;
  unsigned long _faceDownSinceMs = 0;

  bool readGyro(float& gx, float& gy, float& gz) const;
  bool ensureStarted();

 public:
  // Call after BoardConfig has selected the active device. Presence probe
  // only; the full sensor init runs lazily on first use.
  void begin();

  // Enables tilt polling state
  bool wake();

  // Puts tilt polling state to sleep
  bool deepSleep();

  // True if an IMU is present on this device
  bool isAvailable() const { return _available; }

  // Poll the IMU: tilt gestures (reader only) and, when faceDownWatch is set,
  // the screen-down orientation timer that drives face-down auto-sleep.
  void update(const uint8_t mode, const uint8_t orientation, const bool inReader, const bool faceDownWatch = false);

  // How long the device has been lying screen-down (0 when it isn't, or when
  // the watch is off). Drives the face-down auto-sleep timeout.
  unsigned long faceDownForMs() const { return _faceDownSinceMs == 0 ? 0 : millis() - _faceDownSinceMs; }

  // Arm the IMU's motion interrupt for raise-to-wake and leave it running as
  // the host goes to sleep (instead of deepSleep()). The INT line is wired to
  // the PMIC's GPIO4 on Paper Mono.
  bool armMotionWake();

  // Returns true once per tilt-forward gesture (next page direction).
  // Consumed on read — subsequent calls return false until next gesture.
  bool wasTiltedForward();

  // Returns true once per tilt-back gesture (previous page direction).
  // Consumed on read.
  bool wasTiltedBack();

  // Non-consuming: true if any tilt activity occurred since last call.
  // Used to reset the auto-sleep inactivity timer.
  bool hadActivity();

  // Discard any pending tilt events (call when leaving reader or disabling tilt).
  void clearPendingEvents();
};
