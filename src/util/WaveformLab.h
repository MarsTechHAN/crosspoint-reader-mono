#pragma once

// Serial waveform lab for the Paper Mono SSD1677 panel: lets the web editor
// (tools/waveform-editor.html) install, persist, and A/B-test host-authored
// 111-byte LUTs over the existing CMD: serial channel.
//
// The whole module is compiled only with -DFREEINK_WAVEFORM_LAB=1 and is
// runtime-inert until a CMD:WAVE SET/LOAD installs an override: with no
// override active the driver's stock code path runs unchanged (see
// Ssd1683Driver::labSelectPageLut). CMD:WAVE OFF returns to stock behavior at
// any time.

#if FREEINK_DEVICE_PAPERMONO && FREEINK_WAVEFORM_LAB

#include <WString.h>

namespace WaveformLab {

struct CommandResult {
  bool handled = false;
  // The command changed panel-facing state (TEST): the caller should ask the
  // current activity to re-render so the result is visible immediately.
  bool requestRedraw = false;
};

// `args` is the trimmed text after "CMD:WAVE" (subcommand plus parameters).
CommandResult handleCommand(const String& args);

// Boot-time hook: when /waveforms/boot.txt names a saved set, install it as
// the runtime override. Silent no-op when the marker or SD card is absent, so
// an untouched device boots byte-identical to stock.
void loadBootWaveform();

}  // namespace WaveformLab

#endif  // FREEINK_DEVICE_PAPERMONO && FREEINK_WAVEFORM_LAB
