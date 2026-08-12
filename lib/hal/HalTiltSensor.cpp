#include "HalTiltSensor.h"

#include <BoardConfig.h>
#include <Logging.h>

HalTiltSensor halTiltSensor;  // Singleton instance

bool HalTiltSensor::readGyro(float& gx, float& gy, float& gz) const {
  Imu::Sample sample;
  if (!_sdkImu.read(sample)) return false;
  gx = sample.gx;
  gy = sample.gy;
  gz = sample.gz;
  return true;
}

void HalTiltSensor::begin() {
  // Presence only: the full init (BMI270's ~0.8 s config upload on a cold
  // boot) is deferred to ensureStarted(), so boots with every IMU feature
  // disabled pay one WHO_AM_I read and nothing else.
  _available = _sdkImu.probe();
  if (_available) {
    _initMs = millis();
    _lastPollMs = millis();
    LOG_INF("GYR", "IMU present (init deferred)");
    return;
  }
  LOG_ERR("GYR", "SDK IMU not found");
}

bool HalTiltSensor::ensureStarted() {
  if (_started) return true;
  if (!_available) return false;
  if (!_sdkImu.begin()) {
    LOG_ERR("GYR", "IMU init failed");
    _available = false;
    return false;
  }
  _started = true;
  // warm=1 means the sensor was still configured when we got here, i.e. it kept
  // its rail across the reset. After a PMIC hard shutdown that answers whether
  // raise-to-wake is even physically possible on this board.
  LOG_INF("GYR", "SDK IMU initialized (warm=%d)", _sdkImu.warmStarted() ? 1 : 0);
  return true;
}

bool HalTiltSensor::wake() {
  if (!ensureStarted()) {
    return false;
  }

  if (!_sdkImu.wake()) {
    LOG_ERR("GYR", "IMU wake failed");
    return false;
  }

  _lastPollMs = millis();
  _lastTiltMs = millis();
  _wakeMs = millis();
  _isAwake = true;
  return true;
}

bool HalTiltSensor::armMotionWake() {
  if (!ensureStarted()) {
    return false;
  }
  if (!_sdkImu.armMotionWake()) {
    LOG_ERR("GYR", "IMU motion-wake arm failed");
    return false;
  }
  clearPendingEvents();
  _inTilt = false;
  _isAwake = false;
  return true;
}

bool HalTiltSensor::isRaisedPose() {
  if (!ensureStarted()) {
    return false;
  }

  // Wearable frame per the axis remap the wrist feature runs in:
  // (X_w, Y_w, Z_w) = (-Y, -X, -Z). Accumulate in that frame directly so the
  // thresholds compare against Bosch's own tilt fields without a second
  // translation step.
  float forward = 0.0f, roll = 0.0f, normal = 0.0f;
  uint8_t got = 0;
  for (uint8_t i = 0; i < RAISED_POSE_SAMPLES; ++i) {
    Imu::Sample sample;
    if (_sdkImu.read(sample)) {
      forward += -sample.ax;
      roll += -sample.ay;
      normal += -sample.az;
      ++got;
    }
    delay(RAISED_POSE_SAMPLE_MS);
  }
  if (got == 0) {
    LOG_ERR("GYR", "Raise pose: no IMU samples");
    return false;
  }

  forward /= got;
  roll /= got;
  normal /= got;
  const bool raised = normal > 0.0f && forward >= RAISED_POSE_MIN_FORWARD_G &&
                      forward <= RAISED_POSE_MAX_FORWARD_G && fabsf(roll) <= RAISED_POSE_MAX_ROLL_G;
  LOG_INF("GYR", "Raise pose: fwd=%.2fg roll=%.2fg normal=%.2fg -> %s", forward, roll, normal,
          raised ? "raised" : "not raised");
  return raised;
}

bool HalTiltSensor::deepSleep() {
  if (!_available || !_started) {
    return false;
  }

  if (!_sdkImu.sleep()) {
    LOG_ERR("GYR", "IMU sleep failed");
    return false;
  }

  clearPendingEvents();
  _inTilt = false;
  _isAwake = false;
  _faceDownSinceMs = 0;
  return true;
}

void HalTiltSensor::update(const uint8_t mode, const uint8_t orientation, const bool inReader,
                           const bool faceDownWatch) {
  if (!_available) {
    return;
  }

  // State machine: the sensor runs while any consumer needs it — tilt page
  // turns, or the face-down auto-sleep watch (which works in every activity).
  const bool wantAwake = (mode != CrossPointTiltPageTurn::TILT_OFF) || faceDownWatch;
  if (wantAwake && !_isAwake) {
    _isAwake = wake();
    return;
  } else if (!wantAwake && _isAwake) {
    _isAwake = !deepSleep();
    return;
  }
  if (!wantAwake) {
    return;
  }

  const unsigned long now = millis();
  // Stabilization: discard readings during gyro startup transient
  if ((now - _wakeMs) < WAKE_STABILIZE_MS) {
    return;
  }

  // Face-down watch: 2 Hz orientation sample, independent of the reader.
  if (faceDownWatch && (now - _lastFaceDownPollMs) >= FACE_DOWN_POLL_MS) {
    _lastFaceDownPollMs = now;
    Imu::Sample sample;
    if (_sdkImu.read(sample)) {
      const float normal = sample.az * FACE_DOWN_SIGN;
      const bool faceDown = normal > FACE_DOWN_MIN_G && fabsf(sample.ax) < FACE_DOWN_MAX_ORTHO_G &&
                            fabsf(sample.ay) < FACE_DOWN_MAX_ORTHO_G;
      if (faceDown) {
        if (_faceDownSinceMs == 0) {
          _faceDownSinceMs = now;
          LOG_DBG("GYR", "Face-down start: a=(%.2f, %.2f, %.2f)", sample.ax, sample.ay, sample.az);
        }
      } else {
        _faceDownSinceMs = 0;
      }
    }
  }

  // Everything below is the tilt page-turn gesture path (reader only).
  if ((mode == CrossPointTiltPageTurn::TILT_OFF) || !inReader) {
    return;
  }

  if ((now - _lastPollMs) < POLL_INTERVAL_MS) {
    return;
  }
  _lastPollMs = now;

  float gx, gy, gz;
  if (!readGyro(gx, gy, gz)) {
    return;
  }

  // Map the gyro axis to left/right tilt based on reader orientation.
  // On the X3 PCB: X axis = left/right in portrait, Y axis = left/right in landscape.
  // Paper Mono's BMI270 is mounted with the axis opposite the X3's QMI8658 —
  // hardware testing confirmed its correct direction is the X3's "inverted"
  // mapping. The inversion is baked in here so the Paper Mono setting is a
  // plain on/off (SettingsList shows two options there); on the X3 the
  // NORMAL/INVERTED choice keeps working as before.
  bool invert = (mode == CrossPointTiltPageTurn::TILT_INVERTED);
  if (BoardConfig::ACTIVE.sensors.imuType == BoardConfig::ImuType::Bmi270) {
    invert = true;
  }
  float tiltAxis;
  switch (orientation) {
    case CrossPointOrientation::PORTRAIT:
      tiltAxis = gx;
      break;
    case CrossPointOrientation::INVERTED:
      tiltAxis = -gx;
      break;
    case CrossPointOrientation::LANDSCAPE_CW:
      tiltAxis = -gy;
      break;
    case CrossPointOrientation::LANDSCAPE_CCW:
      tiltAxis = gy;
      break;
    default:
      tiltAxis = gx;
      break;
  }
  if (invert) {
    tiltAxis = -tiltAxis;
  }

  if (_inTilt) {
    // Wait for device to return to neutral before allowing next trigger
    if (fabsf(tiltAxis) < NEUTRAL_RATE_DPS) {
      _inTilt = false;
    }
  } else {
    // Check for new tilt gesture (with cooldown)
    if ((now - _lastTiltMs) >= COOLDOWN_MS) {
      if (tiltAxis > RATE_THRESHOLD_DPS) {
        _tiltForwardEvent = true;
        _hadActivity = true;
        _inTilt = true;
        _lastTiltMs = now;
        LOG_INF("GYR", "Forward Trigger=(%.1f) dps", tiltAxis);
      } else if (tiltAxis < -RATE_THRESHOLD_DPS) {
        _tiltBackEvent = true;
        _hadActivity = true;
        _inTilt = true;
        _lastTiltMs = now;
        LOG_INF("GYR", "Backward Trigger=(%.1f) dps", tiltAxis);
      }
    }
  }
}

bool HalTiltSensor::wasTiltedForward() {
  const bool val = _tiltForwardEvent;
  _tiltForwardEvent = false;
  return val;
}

bool HalTiltSensor::wasTiltedBack() {
  const bool val = _tiltBackEvent;
  _tiltBackEvent = false;
  return val;
}

bool HalTiltSensor::hadActivity() {
  const bool val = _hadActivity;
  _hadActivity = false;
  return val;
}

void HalTiltSensor::clearPendingEvents() {
  _tiltForwardEvent = false;
  _tiltBackEvent = false;
  _hadActivity = false;
  // Intentionally preserve _inTilt so a held tilt doesn't retrigger on next poll
}
