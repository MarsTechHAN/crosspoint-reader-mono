#pragma once

#include "activities/Activity.h"

class GrayCalibrationActivity final : public Activity {
 public:
  explicit GrayCalibrationActivity(GfxRenderer& renderer, MappedInputManager& mappedInput)
      : Activity("GrayCalibration", renderer, mappedInput) {}

  void onEnter() override;
  void onExit() override;
  void loop() override;
  void render(RenderLock&&) override;
  bool preventAutoSleep() override { return true; }

 private:
  enum class Field : uint8_t { Dark, Light };

  Field selectedField = Field::Dark;
  uint8_t darkFrames = 3;
  uint8_t lightFrames = 3;
  bool dirty = false;

  void applyParams() const;
  void adjustSelected(int delta);
  void save();
  void drawBwScreen() const;
  void drawGrayPlane(bool lsbPlane) const;
};
