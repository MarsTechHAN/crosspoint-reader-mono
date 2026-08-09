#include "WaveformLab.h"

#if FREEINK_DEVICE_PAPERMONO && FREEINK_WAVEFORM_LAB

#include <HalStorage.h>
#include <Logging.h>
#include <driver/Ssd1683Driver.h>

#include <cstring>

#include "CrossPointSettings.h"

namespace WaveformLab {
namespace {

constexpr const char* WAVE_DIR = "/waveforms";
constexpr const char* BOOT_MARKER = "/waveforms/boot.txt";
constexpr uint8_t FILE_MAGIC[4] = {'C', 'P', 'W', 'F'};
constexpr uint8_t FILE_VERSION = 1;
constexpr size_t LUT_BYTES = freeink::WAVELAB_LUT_BYTES;
constexpr size_t MAX_NAME = 24;

// Replies use a stable "WAVE:" prefix so the editor can separate them from
// ordinary log traffic on the shared serial port.
#define WAVE_REPLY(...) logSerial.printf(__VA_ARGS__)

int slotForLetter(char letter) {
  switch (letter) {
    case 'A':
    case 'a':
      return freeink::WAVELAB_SLOT_PAGE;
    case 'B':
    case 'b':
      return freeink::WAVELAB_SLOT_STAGE2;
    case 'C':
    case 'c':
      return freeink::WAVELAB_SLOT_CORRECTIVE;
    default:
      return -1;
  }
}

char letterForSlot(uint8_t slot) { return static_cast<char>('A' + slot); }

int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool parseHexLut(const char* text, uint8_t out[LUT_BYTES]) {
  for (size_t i = 0; i < LUT_BYTES; ++i) {
    const int hi = hexNibble(text[2 * i]);
    if (hi < 0) return false;
    const int lo = hexNibble(text[2 * i + 1]);
    if (lo < 0) return false;
    out[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  return text[2 * LUT_BYTES] == '\0';
}

void printHexLut(const uint8_t lut[LUT_BYTES]) {
  static constexpr char DIGITS[] = "0123456789ABCDEF";
  char text[2 * LUT_BYTES + 1];
  for (size_t i = 0; i < LUT_BYTES; ++i) {
    text[2 * i] = DIGITS[lut[i] >> 4];
    text[2 * i + 1] = DIGITS[lut[i] & 0x0F];
  }
  text[2 * LUT_BYTES] = '\0';
  logSerial.print(text);
}

bool validName(const char* name) {
  const size_t len = strlen(name);
  if (len == 0 || len > MAX_NAME) return false;
  for (size_t i = 0; i < len; ++i) {
    const char c = name[i];
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
    if (!ok) return false;
  }
  return true;
}

void wavePath(const char* name, char out[64]) { snprintf(out, 64, "%s/%s.wfm", WAVE_DIR, name); }

bool saveSet(const char* name) {
  const uint8_t mask = freeink::ssd1683WaveLabActiveMask();
  if (mask == 0) {
    WAVE_REPLY("WAVE:ERR SAVE no runtime override active\n");
    return false;
  }
  if (!Storage.ensureDirectoryExists(WAVE_DIR)) {
    WAVE_REPLY("WAVE:ERR SAVE cannot create %s\n", WAVE_DIR);
    return false;
  }
  char path[64];
  wavePath(name, path);
  HalFile file;
  if (!Storage.openFileForWrite("WAVELAB", path, file)) {
    WAVE_REPLY("WAVE:ERR SAVE cannot open %s\n", path);
    return false;
  }
  uint8_t header[6] = {FILE_MAGIC[0], FILE_MAGIC[1], FILE_MAGIC[2], FILE_MAGIC[3], FILE_VERSION, mask};
  bool ok = file.write(header, sizeof(header)) == sizeof(header);
  for (uint8_t slot = 0; ok && slot <= freeink::WAVELAB_SLOT_CORRECTIVE; ++slot) {
    if (!(mask & (1u << slot))) continue;
    uint8_t lut[LUT_BYTES];
    freeink::ssd1683WaveLabGet(slot, lut);
    ok = file.write(lut, LUT_BYTES) == LUT_BYTES;
  }
  if (!ok) {
    WAVE_REPLY("WAVE:ERR SAVE short write %s\n", path);
    return false;
  }
  WAVE_REPLY("WAVE:OK SAVE %s mask=%u\n", name, mask);
  return true;
}

bool loadSet(const char* name, bool quietBoot) {
  char path[64];
  wavePath(name, path);
  HalFile file;
  if (!Storage.openFileForRead("WAVELAB", path, file)) {
    if (!quietBoot) WAVE_REPLY("WAVE:ERR LOAD missing %s\n", path);
    return false;
  }
  uint8_t header[6];
  if (file.read(header, sizeof(header)) != static_cast<int>(sizeof(header)) ||
      memcmp(header, FILE_MAGIC, sizeof(FILE_MAGIC)) != 0 || header[4] != FILE_VERSION) {
    if (!quietBoot) WAVE_REPLY("WAVE:ERR LOAD bad header %s\n", path);
    LOG_ERR("WAVELAB", "Bad waveform file header: %s", path);
    return false;
  }
  const uint8_t mask = header[5];
  // Parse fully before installing anything so a truncated file cannot leave a
  // half-applied set.
  uint8_t luts[3][LUT_BYTES];
  for (uint8_t slot = 0; slot <= freeink::WAVELAB_SLOT_CORRECTIVE; ++slot) {
    if (!(mask & (1u << slot))) continue;
    if (file.read(luts[slot], LUT_BYTES) != static_cast<int>(LUT_BYTES)) {
      if (!quietBoot) WAVE_REPLY("WAVE:ERR LOAD truncated %s\n", path);
      LOG_ERR("WAVELAB", "Truncated waveform file: %s", path);
      return false;
    }
  }
  freeink::ssd1683WaveLabOff();
  for (uint8_t slot = 0; slot <= freeink::WAVELAB_SLOT_CORRECTIVE; ++slot) {
    if (mask & (1u << slot)) freeink::ssd1683WaveLabSet(slot, luts[slot]);
  }
  if (!quietBoot) WAVE_REPLY("WAVE:OK LOAD %s mask=%u\n", name, mask);
  LOG_INF("WAVELAB", "Waveform set '%s' installed (mask=%u)", name, mask);
  return true;
}

void listSets() {
  logSerial.print("WAVE:FILES");
  HalFile dir = Storage.open(WAVE_DIR);
  if (dir && dir.isDirectory()) {
    while (true) {
      HalFile entry = dir.openNextFile();
      if (!entry) break;
      if (entry.isDirectory()) continue;
      char name[48];
      const size_t len = entry.getName(name, sizeof(name));
      if (len > 4 && strcmp(name + len - 4, ".wfm") == 0) {
        name[len - 4] = '\0';
        logSerial.printf(" %s", name);
      }
    }
  }
  logSerial.print("\n");
}

void printStatus() {
  const bool balanced = SETTINGS.readerRefreshMode == CrossPointSettings::READER_REFRESH_BALANCED;
  WAVE_REPLY("WAVE:STATUS lab=1 active=%u hold=%u baked=%u mode=%s cadence=%d light=%u frame_us=5000\n",
             freeink::ssd1683WaveLabActiveMask(), freeink::ssd1683WaveLabHold() ? 1 : 0,
             freeink::ssd1683WaveLabBakedPresent() ? 1 : 0, balanced ? "balanced" : "fast",
             SETTINGS.getRefreshFrequency(), static_cast<unsigned>(SETTINGS.grayLightFrames));
}

bool setCadence(int pages) {
  uint8_t encoded;
  switch (pages) {
    case 1:
      encoded = CrossPointSettings::REFRESH_1;
      break;
    case 5:
      encoded = CrossPointSettings::REFRESH_5;
      break;
    case 10:
      encoded = CrossPointSettings::REFRESH_10;
      break;
    case 15:
      encoded = CrossPointSettings::REFRESH_15;
      break;
    case 30:
      encoded = CrossPointSettings::REFRESH_30;
      break;
    default:
      WAVE_REPLY("WAVE:ERR CADENCE use 1|5|10|15|30\n");
      return false;
  }
  // RAM-only on purpose: a debug session must not spend SPIFFS erase cycles,
  // and a reboot restores the user's stored preference.
  SETTINGS.refreshFrequency = encoded;
  WAVE_REPLY("WAVE:OK CADENCE %d (ram-only)\n", pages);
  return true;
}

}  // namespace

CommandResult handleCommand(const String& args) {
  CommandResult result;
  result.handled = true;

  if (args.length() == 0 || args == "STATUS") {
    printStatus();
    return result;
  }

  if (args == "OFF") {
    freeink::ssd1683WaveLabOff();
    WAVE_REPLY("WAVE:OK OFF stock behavior restored\n");
    return result;
  }

  if (args == "TEST") {
    // Corrective sweep of the current screen: mark the glass unknown so the
    // next render drives every pixel, then have the activity re-render.
    freeink::ssd1683Driver().requestResync(0);
    result.requestRedraw = true;
    WAVE_REPLY("WAVE:OK TEST corrective redraw queued\n");
    return result;
  }

  if (args == "LIST") {
    listSets();
    return result;
  }

  if (args.startsWith("GET ")) {
    const int slot = slotForLetter(args.charAt(4));
    if (slot < 0) {
      WAVE_REPLY("WAVE:ERR GET use A|B|C\n");
      return result;
    }
    uint8_t lut[LUT_BYTES];
    const bool overridden = freeink::ssd1683WaveLabGet(static_cast<uint8_t>(slot), lut);
    const char* src = overridden                                                                            ? "override"
                      : (freeink::ssd1683WaveLabActiveMask() == 0 && freeink::ssd1683WaveLabBakedPresent()) ? "baked"
                                                                                                            : "builtin";
    logSerial.printf("WAVE:LUT:%c:%s:", letterForSlot(static_cast<uint8_t>(slot)), src);
    printHexLut(lut);
    logSerial.print("\n");
    return result;
  }

  if (args.startsWith("SET ")) {
    const int slot = slotForLetter(args.charAt(4));
    if (slot < 0 || args.charAt(5) != ' ') {
      WAVE_REPLY("WAVE:ERR SET usage: SET A|B|C <222 hex chars>\n");
      return result;
    }
    uint8_t lut[LUT_BYTES];
    if (!parseHexLut(args.c_str() + 6, lut)) {
      WAVE_REPLY("WAVE:ERR SET need exactly %u hex chars\n", static_cast<unsigned>(2 * LUT_BYTES));
      return result;
    }
    freeink::ssd1683WaveLabSet(static_cast<uint8_t>(slot), lut);
    WAVE_REPLY("WAVE:OK SET %c frames=%u active=%u\n", letterForSlot(static_cast<uint8_t>(slot)),
               freeink::ssd1683WaveLabLutFrames(lut), freeink::ssd1683WaveLabActiveMask());
    return result;
  }

  if (args.startsWith("CLEAR ")) {
    const int slot = slotForLetter(args.charAt(6));
    if (slot < 0) {
      WAVE_REPLY("WAVE:ERR CLEAR use A|B|C\n");
      return result;
    }
    freeink::ssd1683WaveLabClear(static_cast<uint8_t>(slot));
    WAVE_REPLY("WAVE:OK CLEAR %c active=%u\n", letterForSlot(static_cast<uint8_t>(slot)),
               freeink::ssd1683WaveLabActiveMask());
    return result;
  }

  if (args.startsWith("CADENCE ")) {
    setCadence(atoi(args.c_str() + 8));
    return result;
  }

  if (args.startsWith("HOLD ")) {
    const char c = args.charAt(5);
    if (c != '0' && c != '1') {
      WAVE_REPLY("WAVE:ERR HOLD use 0|1\n");
      return result;
    }
    // Unchanged black holds on entry 0. This is the production default, so the
    // command exists to turn it *off* for an override whose entry 0 is not idle
    // — such a LUT would bleach held black. OFF restores the default.
    freeink::ssd1683WaveLabSetHold(c == '1');
    WAVE_REPLY("WAVE:OK HOLD %c%s\n", c, c == '1' ? " (default)" : " (entry 0 drives unchanged black again)");
    return result;
  }

  if (args.startsWith("SAVE ")) {
    const char* name = args.c_str() + 5;
    if (!validName(name)) {
      WAVE_REPLY("WAVE:ERR SAVE name: [A-Za-z0-9_-]{1,%u}\n", static_cast<unsigned>(MAX_NAME));
      return result;
    }
    saveSet(name);
    return result;
  }

  if (args.startsWith("LOAD ")) {
    const char* name = args.c_str() + 5;
    if (!validName(name)) {
      WAVE_REPLY("WAVE:ERR LOAD bad name\n");
      return result;
    }
    loadSet(name, false);
    return result;
  }

  if (args.startsWith("BOOT ")) {
    const char* name = args.c_str() + 5;
    if (strcmp(name, "OFF") == 0) {
      Storage.remove(BOOT_MARKER);
      WAVE_REPLY("WAVE:OK BOOT off\n");
      return result;
    }
    if (!validName(name)) {
      WAVE_REPLY("WAVE:ERR BOOT bad name\n");
      return result;
    }
    char path[64];
    wavePath(name, path);
    if (!Storage.exists(path)) {
      WAVE_REPLY("WAVE:ERR BOOT %s not saved yet\n", name);
      return result;
    }
    if (!Storage.ensureDirectoryExists(WAVE_DIR) || !Storage.writeFile(BOOT_MARKER, String(name))) {
      WAVE_REPLY("WAVE:ERR BOOT cannot write marker\n");
      return result;
    }
    WAVE_REPLY("WAVE:OK BOOT %s\n", name);
    return result;
  }

  WAVE_REPLY("WAVE:ERR unknown; use STATUS|GET|SET|CLEAR|OFF|TEST|CADENCE|SAVE|LOAD|LIST|BOOT\n");
  return result;
}

void loadBootWaveform() {
  if (!Storage.ready() || !Storage.exists(BOOT_MARKER)) return;
  const String name = Storage.readFile(BOOT_MARKER);
  char trimmed[MAX_NAME + 1];
  snprintf(trimmed, sizeof(trimmed), "%s", name.c_str());
  for (char* c = trimmed; *c; ++c) {
    if (*c == '\r' || *c == '\n') {
      *c = '\0';
      break;
    }
  }
  if (!validName(trimmed)) {
    LOG_ERR("WAVELAB", "Ignoring invalid boot waveform marker");
    return;
  }
  if (loadSet(trimmed, true)) {
    LOG_INF("WAVELAB", "Boot waveform '%s' active (CMD:WAVE OFF to revert)", trimmed);
  }
}

}  // namespace WaveformLab

#endif  // FREEINK_DEVICE_PAPERMONO && FREEINK_WAVEFORM_LAB
