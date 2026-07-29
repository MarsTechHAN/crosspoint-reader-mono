#include "GrayCalibrationActivity.h"

#include <BoardConfig.h>
#include <GfxRenderer.h>
#include <I18n.h>

#include <algorithm>
#include <cstdio>

#if FREEINK_DEVICE_PAPERMONO
#include <driver/Ssd1683Driver.h>
#endif

#include "CrossPointSettings.h"
#include "MappedInputManager.h"
#include "components/UITheme.h"
#include "fontIds.h"

namespace {
constexpr unsigned long LONG_PRESS_MS = 700;
constexpr uint8_t MIN_FRAMES = 1;
constexpr uint8_t MAX_FRAMES = 12;
constexpr int COLUMN_LEFT_X = 55;
constexpr int COLUMN_WIDTH = 175;
constexpr int COLUMN_RIGHT_X = COLUMN_LEFT_X + COLUMN_WIDTH;
// In portrait mode logical Y maps to the controller's byte-aligned X axis.
// Keeping both values divisible by 8 prevents a selected row's calibration
// window from expanding into either neighboring gray row.
constexpr int SWATCH_TOP = 144;
constexpr int SWATCH_HEIGHT = 80;
constexpr int SWATCH_STEP = SWATCH_HEIGHT;
constexpr int PARAM_TOP = 555;
constexpr int PARAM_HEIGHT = 76;
constexpr int PARAM_MARGIN = 18;
constexpr int PARAM_GAP = 14;
}  // namespace

void GrayCalibrationActivity::onEnter() {
  Activity::onEnter();
  selectedField = Field::Dark;
  darkFrames = std::clamp<uint8_t>(SETTINGS.grayDarkFrames, MIN_FRAMES, MAX_FRAMES);
  lightFrames = std::clamp<uint8_t>(SETTINGS.grayLightFrames, MIN_FRAMES, MAX_FRAMES);
  dirty = false;
  applyParams();
  requestUpdate();
}

void GrayCalibrationActivity::onExit() {
  save();
  renderer.setRenderMode(GfxRenderer::BW);
  Activity::onExit();
}

void GrayCalibrationActivity::applyParams() const {
#if FREEINK_DEVICE_PAPERMONO
  freeink::Ssd1683GrayParams params;
  params.darkFrames = darkFrames;
  params.lightFrames = lightFrames;
  freeink::ssd1683SetGrayParams(params);
#endif
}

void GrayCalibrationActivity::adjustSelected(const int delta) {
  uint8_t& value = selectedField == Field::Dark ? darkFrames : lightFrames;
  const int next = std::clamp(static_cast<int>(value) + delta, static_cast<int>(MIN_FRAMES),
                              static_cast<int>(MAX_FRAMES));
  if (next == value) return;
  value = static_cast<uint8_t>(next);
  dirty = true;
  applyParams();
  requestUpdate();
}

void GrayCalibrationActivity::save() {
  if (!dirty && SETTINGS.grayDarkFrames == darkFrames && SETTINGS.grayLightFrames == lightFrames) return;
  SETTINGS.grayDarkFrames = darkFrames;
  SETTINGS.grayLightFrames = lightFrames;
  SETTINGS.saveToFile();
  dirty = false;
}

void GrayCalibrationActivity::loop() {
  if (mappedInput.wasPressed(MappedInputManager::Button::Back)) {
    save();
    finish();
    return;
  }

  if (mappedInput.wasPressed(MappedInputManager::Button::Confirm)) {
    selectedField = selectedField == Field::Dark ? Field::Light : Field::Dark;
    requestUpdate();
    return;
  }

  if (mappedInput.wasReleased(MappedInputManager::Button::Up)) {
    if (mappedInput.getHeldTime() >= LONG_PRESS_MS) {
      save();
      finish();
    } else {
      adjustSelected(-1);
    }
    return;
  }

  if (mappedInput.wasReleased(MappedInputManager::Button::Down)) {
    if (mappedInput.getHeldTime() >= LONG_PRESS_MS) {
      selectedField = selectedField == Field::Dark ? Field::Light : Field::Dark;
      requestUpdate();
    } else {
      adjustSelected(+1);
    }
    return;
  }

  int tx = 0;
  int ty = 0;
  if (mappedInput.wasScreenTapped(tx, ty) && ty >= PARAM_TOP && ty < PARAM_TOP + PARAM_HEIGHT) {
    const int cardWidth = (renderer.getScreenWidth() - PARAM_MARGIN * 2 - PARAM_GAP) / 2;
    const Field touched = tx < PARAM_MARGIN + cardWidth + PARAM_GAP / 2 ? Field::Dark : Field::Light;
    if (selectedField != touched) {
      selectedField = touched;
      requestUpdate();
    }
  }
}

void GrayCalibrationActivity::drawBwScreen() const {
  renderer.setRenderMode(GfxRenderer::BW);
  renderer.clearScreen();

  const auto& metrics = UITheme::getInstance().getMetrics();
  const int pageWidth = renderer.getScreenWidth();
  GUI.drawHeader(renderer, Rect{0, metrics.topPadding, pageWidth, metrics.headerHeight}, tr(STR_GRAY_CALIBRATION));

  auto drawCenteredInColumn = [this](const int x, const int width, const int y, const char* text,
                                     const EpdFontFamily::Style style = EpdFontFamily::REGULAR) {
    const int textX = x + (width - renderer.getTextWidth(UI_10_FONT_ID, text, style)) / 2;
    renderer.drawText(UI_10_FONT_ID, textX, y, text, true, style);
  };
  drawCenteredInColumn(COLUMN_LEFT_X, COLUMN_WIDTH, 92, tr(STR_GRAY_OTP_REFERENCE), EpdFontFamily::BOLD);
  drawCenteredInColumn(COLUMN_RIGHT_X, COLUMN_WIDTH, 92, tr(STR_GRAY_PARTIAL_CURRENT), EpdFontFamily::BOLD);

  const char* rowLabels[] = {tr(STR_GRAY_BLACK), tr(STR_GRAY_DARK), tr(STR_GRAY_LIGHT), tr(STR_GRAY_WHITE)};
  for (int row = 0; row < 4; ++row) {
    const int y = SWATCH_TOP + row * SWATCH_STEP;
    // Scheme B lifts dark gray from black and presses light gray from white.
    const bool blackBase = row < 2;
    renderer.fillRect(COLUMN_LEFT_X, y, COLUMN_WIDTH, SWATCH_HEIGHT, blackBase);
    renderer.fillRect(COLUMN_RIGHT_X, y, COLUMN_WIDTH, SWATCH_HEIGHT, blackBase);
    const int labelY = y + (SWATCH_HEIGHT - renderer.getLineHeight(UI_10_FONT_ID)) / 2;
    renderer.drawText(UI_10_FONT_ID, 5, labelY, rowLabels[row]);
  }

  const int cardWidth = (pageWidth - PARAM_MARGIN * 2 - PARAM_GAP) / 2;
  auto drawParam = [this, cardWidth](const int x, const bool selected, const char* label, const uint8_t value,
                                    const char* direction) {
    renderer.fillRect(x, PARAM_TOP, cardWidth, PARAM_HEIGHT, selected);
    renderer.drawRect(x, PARAM_TOP, cardWidth, PARAM_HEIGHT, selected ? 3 : 1, true);
    const bool textBlack = !selected;
    char valueText[20];
    snprintf(valueText, sizeof(valueText), "%u %s", value, tr(STR_GRAY_FRAMES));
    const int labelX = x + (cardWidth - renderer.getTextWidth(UI_10_FONT_ID, label)) / 2;
    const int valueX = x + (cardWidth - renderer.getTextWidth(UI_12_FONT_ID, valueText, EpdFontFamily::BOLD)) / 2;
    const int dirX = x + (cardWidth - renderer.getTextWidth(SMALL_FONT_ID, direction)) / 2;
    renderer.drawText(UI_10_FONT_ID, labelX, PARAM_TOP + 8, label, textBlack);
    renderer.drawText(UI_12_FONT_ID, valueX, PARAM_TOP + 29, valueText, textBlack, EpdFontFamily::BOLD);
    renderer.drawText(SMALL_FONT_ID, dirX, PARAM_TOP + 55, direction, textBlack);
  };
  drawParam(PARAM_MARGIN, selectedField == Field::Dark, tr(STR_GRAY_DARK_FRAMES), darkFrames,
            tr(STR_GRAY_MORE_LIGHTER));
  drawParam(PARAM_MARGIN + cardWidth + PARAM_GAP, selectedField == Field::Light, tr(STR_GRAY_LIGHT_FRAMES),
            lightFrames, tr(STR_GRAY_MORE_DARKER));

  renderer.drawCenteredText(SMALL_FONT_ID, 654, tr(STR_GRAY_HOLD_UP_SAVE));
  renderer.drawCenteredText(SMALL_FONT_ID, 674, tr(STR_GRAY_HOLD_DOWN_SELECT));
  const auto labels = mappedInput.mapLabels(tr(STR_BACK), tr(STR_SELECT), "-", "+");
  GUI.drawButtonHints(renderer, labels.btn1, labels.btn2, labels.btn3, labels.btn4);
}

void GrayCalibrationActivity::drawGrayPlane(const bool lsbPlane) const {
  renderer.setRenderMode(lsbPlane ? GfxRenderer::GRAYSCALE_LSB : GfxRenderer::GRAYSCALE_MSB);
  renderer.clearScreen(0x00);
  // bin 1 (dark) sets both planes; bin 2 (light) sets only MSB.
  for (const int x : {COLUMN_LEFT_X, COLUMN_RIGHT_X}) {
    renderer.fillRect(x, SWATCH_TOP + SWATCH_STEP, COLUMN_WIDTH, SWATCH_HEIGHT, false);
    if (!lsbPlane) {
      renderer.fillRect(x, SWATCH_TOP + 2 * SWATCH_STEP, COLUMN_WIDTH, SWATCH_HEIGHT, false);
    }
  }
}

void GrayCalibrationActivity::render(RenderLock&&) {
  drawBwScreen();
  drawGrayPlane(true);
  renderer.copyGrayscaleLsbBuffers();
  drawGrayPlane(false);
  renderer.copyGrayscaleMsbBuffers();
  drawBwScreen();
#if FREEINK_DEVICE_PAPERMONO
  // Refresh the complete Current column through the same two-entry scheme-B
  // path used by reading pages. Keeping an unselected row at the OTP reference
  // makes a parameter edit look coupled: every render resets that row before
  // applying only the selected tone.
  renderer.displayGrayCalibration(COLUMN_RIGHT_X, SWATCH_TOP, COLUMN_WIDTH, 4 * SWATCH_HEIGHT);
#else
  renderer.displayGrayBuffer();
#endif
}
