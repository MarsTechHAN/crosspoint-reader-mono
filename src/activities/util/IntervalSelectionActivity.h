#pragma once

#include <I18n.h>

#include <functional>
#include <utility>

#include "MappedInputManager.h"
#include "activities/Activity.h"
#include "util/ButtonNavigator.h"

class GfxRenderer;
struct Rect;

class IntervalSelectionActivity final : public Activity {
 public:
  explicit IntervalSelectionActivity(GfxRenderer& renderer, MappedInputManager& mappedInput, const char* activityName,
                                     StrId titleId, int initialValue, int minValue, int maxValue, int smallStep,
                                     int largeStep, StrId valueFormatId = StrId::STR_NONE_OPT,
                                     bool readerActivity = false, bool ignoreInitialConfirmRelease = false,
                                     StrId maxBoundaryLabelId = StrId::STR_NONE_OPT,
                                     std::function<void(int)> valueChangedCallback = {}, bool commitOnBack = false)
      : Activity(activityName, renderer, mappedInput),
        titleId(titleId),
        valueFormatId(valueFormatId),
        maxBoundaryLabelId(maxBoundaryLabelId),
        value(initialValue),
        minValue(minValue),
        maxValue(maxValue),
        smallStep(smallStep),
        largeStep(largeStep),
        readerActivity(readerActivity),
        ignoreConfirmRelease(ignoreInitialConfirmRelease),
        valueChangedCallback(std::move(valueChangedCallback)),
        commitOnBack(commitOnBack) {}

  void onEnter() override;
  void loop() override;
  void render(RenderLock&&) override;
  bool isReaderActivity() const override { return readerActivity; }

 private:
  StrId titleId;
  StrId valueFormatId;
  StrId maxBoundaryLabelId;
  int value;
  int minValue;
  int maxValue;
  int smallStep;
  int largeStep;
  bool readerActivity;
  bool ignoreConfirmRelease;
  bool draggingBar = false;
  std::function<void(int)> valueChangedCallback;
  bool commitOnBack;
  ButtonNavigator buttonNavigator;

  void adjustValue(int delta);
  void setValue(int candidate);
  void finishFromBack();
  int clampedValue(int candidate) const;
  void drawStepHintLine(int y, StrId labelId, int step);
  // Bottom action bar, touch devices only. BaseTheme::drawButtonHints() returns
  // early when gpio.hasTouch(), so on a touch-only board this dialog otherwise
  // renders with no visible way out and the value the user just dragged is only
  // committed by an undiscoverable tap on a blank strip. loop() and render()
  // share these rects so the hit test and the drawn buttons cannot drift apart.
  void getTouchControlRects(Rect& backRect, Rect& confirmRect) const;
};
