#include "IntervalSelectionActivity.h"

#include <GfxRenderer.h>
#include <HalGPIO.h>
#include <I18n.h>

#include <algorithm>
#include <cstdio>
#include <utility>

#include "components/UITheme.h"
#include "fontIds.h"

namespace {

constexpr int TOUCH_BUTTON_HEIGHT = 56;
constexpr int TOUCH_BUTTON_MARGIN = 24;
constexpr int TOUCH_BUTTON_GAP = 24;

bool contains(const Rect& rect, const int x, const int y) {
  return x >= rect.x && x < rect.x + rect.width && y >= rect.y && y < rect.y + rect.height;
}

}  // namespace

int IntervalSelectionActivity::clampedValue(const int candidate) const {
  return std::clamp(candidate, minValue, maxValue);
}

void IntervalSelectionActivity::getTouchControlRects(Rect& backRect, Rect& confirmRect) const {
  const int screenWidth = renderer.getScreenWidth();
  // Kept inside the strip the tap handler already owned (screen bottom minus 80)
  // so nothing above the bar changes its meaning on non-touch boards.
  const int buttonY = renderer.getScreenHeight() - TOUCH_BUTTON_HEIGHT - 12;
  const int buttonWidth = std::max(1, (screenWidth - TOUCH_BUTTON_MARGIN * 2 - TOUCH_BUTTON_GAP) / 2);
  backRect = Rect{TOUCH_BUTTON_MARGIN, buttonY, buttonWidth, TOUCH_BUTTON_HEIGHT};
  confirmRect = Rect{screenWidth - TOUCH_BUTTON_MARGIN - buttonWidth, buttonY, buttonWidth, TOUCH_BUTTON_HEIGHT};
}

void IntervalSelectionActivity::onEnter() {
  Activity::onEnter();
  value = clampedValue(value);
  requestUpdate();
}

void IntervalSelectionActivity::adjustValue(const int delta) {
  setValue(value + delta);
}

void IntervalSelectionActivity::setValue(const int candidate) {
  const int next = clampedValue(candidate);
  if (next == value) return;
  value = next;
  if (valueChangedCallback) valueChangedCallback(value);
  requestUpdate();
}

void IntervalSelectionActivity::finishFromBack() {
  if (commitOnBack) {
    setResult(IntervalResult{static_cast<uint32_t>(value)});
  } else {
    ActivityResult result;
    result.isCancelled = true;
    setResult(std::move(result));
  }
  finish();
}

void IntervalSelectionActivity::drawStepHintLine(const int y, const StrId labelId, const int step) {
  char stepText[24];
  if (valueFormatId != StrId::STR_NONE_OPT) {
    snprintf(stepText, sizeof(stepText), I18N.get(valueFormatId), static_cast<unsigned int>(step));
  } else {
    snprintf(stepText, sizeof(stepText), "%d", step);
  }
  char line[64];
  snprintf(line, sizeof(line), "%s %s", I18N.get(labelId), stepText);
  renderer.drawCenteredText(SMALL_FONT_ID, y, line, true);
}

void IntervalSelectionActivity::loop() {
  if (ignoreConfirmRelease) {
    if (mappedInput.wasReleased(MappedInputManager::Button::Confirm)) {
      ignoreConfirmRelease = false;
      return;
    }
    if (!mappedInput.isPressed(MappedInputManager::Button::Confirm)) {
      ignoreConfirmRelease = false;
    }
  }

  int tx = 0;
  int ty = 0;
  const int screenWidth = renderer.getScreenWidth();
  const int barWidth = std::min(360, std::max(0, screenWidth - 40));
  constexpr int barHeight = 16;
  const int barX = std::max(0, (screenWidth - barWidth) / 2);
  const int barY = 140;

  // Live drag on the slider: once a touch lands on the bar, the value follows the
  // finger until release. Runs before the Back/Confirm handlers because the release
  // of a drag can also register as a swipe (e.g. the left-edge rightward back
  // gesture) — the drag must consume it so it can't cancel or confirm the dialog.
  if (mappedInput.isScreenTouchHeld(tx, ty)) {
    if (draggingBar || (ty >= barY - 20 && ty < barY + barHeight + 20 && tx >= barX && tx < barX + barWidth)) {
      draggingBar = true;
      const int range = std::max(1, maxValue - minValue);
      const int dragged =
          clampedValue(minValue + std::clamp(tx - barX, 0, barWidth - 1) * range / std::max(1, barWidth - 1));
      setValue(dragged);
      return;
    }
  } else if (draggingBar) {
    // Release frame of a drag: swallow the tap/swipe events it produced.
    draggingBar = false;
    return;
  }

  if (mappedInput.wasReleased(MappedInputManager::Button::Back)) {
    finishFromBack();
    return;
  }

  if (mappedInput.wasReleased(MappedInputManager::Button::Confirm)) {
    setResult(IntervalResult{static_cast<uint32_t>(value)});
    finish();
    return;
  }

  if (mappedInput.wasScreenTapped(tx, ty)) {
    if (ty >= barY - 20 && ty < barY + barHeight + 20 && tx >= barX && tx < barX + barWidth) {
      const int range = std::max(1, maxValue - minValue);
      setValue(minValue + (tx - barX) * range / std::max(1, barWidth - 1));
      return;
    }
    Rect backRect;
    Rect confirmRect;
    getTouchControlRects(backRect, confirmRect);
    if (contains(backRect, tx, ty)) {
      finishFromBack();
      return;
    }
    if (contains(confirmRect, tx, ty)) {
      setResult(IntervalResult{static_cast<uint32_t>(value)});
      finish();
      return;
    }
    if (ty >= renderer.getScreenHeight() - 80) return;
  }

  buttonNavigator.onPressAndContinuous({MappedInputManager::Button::Left}, [this] { adjustValue(-smallStep); });
  buttonNavigator.onPressAndContinuous({MappedInputManager::Button::Right}, [this] { adjustValue(smallStep); });

  // On X3 the side buttons sit on the left/right edges of the screen rather than as a vertical up/down
  // rocker (X4), so BTN_UP is physically the left button and BTN_DOWN the right one. Flip the large-step
  // direction there so the left button decreases and the right button increases, matching the layout.
  const int upDelta = gpio.deviceIsX3() ? -largeStep : largeStep;
  const int downDelta = gpio.deviceIsX3() ? largeStep : -largeStep;
  buttonNavigator.onPressAndContinuous({MappedInputManager::Button::Up}, [this, upDelta] { adjustValue(upDelta); });
  buttonNavigator.onPressAndContinuous({MappedInputManager::Button::Down},
                                       [this, downDelta] { adjustValue(downDelta); });
}

void IntervalSelectionActivity::render(RenderLock&&) {
  renderer.clearScreen();

  renderer.drawCenteredText(UI_12_FONT_ID, 15, I18N.get(titleId), true, EpdFontFamily::BOLD);

  char formattedValue[32];
  if (maxBoundaryLabelId != StrId::STR_NONE_OPT && value == maxValue) {
    snprintf(formattedValue, sizeof(formattedValue), "%s", I18N.get(maxBoundaryLabelId));
  } else if (valueFormatId != StrId::STR_NONE_OPT) {
    snprintf(formattedValue, sizeof(formattedValue), I18N.get(valueFormatId), static_cast<unsigned int>(value));
  } else {
    snprintf(formattedValue, sizeof(formattedValue), "%d", value);
  }
  renderer.drawCenteredText(UI_12_FONT_ID, 90, formattedValue, true, EpdFontFamily::BOLD);

  const int screenWidth = renderer.getScreenWidth();
  const int barWidth = std::min(360, std::max(0, screenWidth - 40));
  constexpr int barHeight = 16;
  const int barX = std::max(0, (screenWidth - barWidth) / 2);
  const int barY = 140;

  renderer.drawRect(barX, barY, barWidth, barHeight);

  const int range = std::max(1, maxValue - minValue);
  const int fillWidth = (barWidth - 4) * (value - minValue) / range;
  if (fillWidth > 0) {
    renderer.fillRect(barX + 2, barY + 2, fillWidth, barHeight - 4);
  }

  const int knobX = std::max(barX + 2, barX + 2 + fillWidth - 2);
  renderer.fillRect(knobX, barY - 4, 4, barHeight + 8, true);

  // Two-line step hint: front buttons do the small step, side buttons the large step. Built from
  // separate label + value strings (rather than splitting one localized sentence) so the layout
  // doesn't depend on translators preserving a hidden separator.
  drawStepHintLine(barY + 30, StrId::STR_STEP_HINT_FRONT, smallStep);
  drawStepHintLine(barY + 52, StrId::STR_STEP_HINT_SIDE, largeStep);

  // Touch boards get an explicit action bar: GUI.drawButtonHints() draws nothing
  // when the device has a touchscreen, so without this the only way to commit is
  // a tap on an unmarked strip and users leave without saving the value they set.
  if (mappedInput.hasTouch()) {
    Rect backRect;
    Rect confirmRect;
    getTouchControlRects(backRect, confirmRect);
    const int lineHeight = renderer.getLineHeight(UI_12_FONT_ID);
    auto drawTouchButton = [&](const Rect& rect, const char* label) {
      renderer.fillRectDither(rect.x, rect.y, rect.width, rect.height, Color::White);
      renderer.drawRect(rect.x, rect.y, rect.width, rect.height, true);
      const int textWidth = renderer.getTextWidth(UI_12_FONT_ID, label, EpdFontFamily::BOLD);
      const int textX = rect.x + (rect.width - textWidth) / 2;
      const int textY = rect.y + (rect.height - lineHeight) / 2;
      renderer.drawText(UI_12_FONT_ID, textX, textY, label, true, EpdFontFamily::BOLD);
    };
    drawTouchButton(backRect, tr(STR_BACK));
    drawTouchButton(confirmRect, tr(STR_SELECT));
  }

  const auto labels = mappedInput.mapLabels(tr(STR_BACK), tr(STR_SELECT), "-", "+");
  GUI.drawButtonHints(renderer, labels.btn1, labels.btn2, labels.btn3, labels.btn4);

  renderer.displayBuffer();
}
