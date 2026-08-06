// Paper Mono panel lab — bare-metal SSD1677 characterisation harness.
//
// Built only by the `paper_mono_lab` PlatformIO env, which restricts
// build_src_filter to this one file. Nothing from CrossPoint or the FreeInk SDK
// is linked: this talks to the controller directly so every register write and
// every microsecond is accounted for. Its job is to answer, on real hardware:
//
//   1. What OTP waveform banks does this panel actually contain, and how long
//      does each take? (cmd 0x1A temperature-bank sweep)
//   2. What is the real frame period for each LUT frame-rate code? (timed
//      single-phase custom LUT probe -> lets us author waveforms in ms, not
//      guesses)
//   3. Where does the wall-clock of a refresh actually go — SPI, controller
//      housekeeping, or the waveform itself?
//
// Everything here is measurement scaffolding. Findings get ported into
// Ssd1683Driver; this file is not part of the shipping firmware.

#ifdef PAPER_MONO_PANEL_LAB

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_heap_caps.h>

namespace {

// ── Board wiring (Paper Mono) ────────────────────────────────────────────────
constexpr int PIN_SCK = 15;
constexpr int PIN_MOSI = 14;
constexpr int PIN_CS = 16;
constexpr int PIN_DC = 17;
constexpr int PIN_BUSY = 18;
constexpr int PIN_SDA = 47;
constexpr int PIN_SCL = 48;
// The two nav keys, active-LOW with the internal pull-up. Per the PAPER_MONO
// profile in BoardConfig.h:805, which declares InputStyle::DigitalTwoButton with
// up=GPIO2 and down=GPIO3.
constexpr int PIN_BTN_UP = 2;
constexpr int PIN_BTN_DOWN = 3;

// M5IOE1 GPIO expander: EPD power on IO3 (bit 2), EPD reset on IO5 (bit 4).
constexpr uint8_t IOE_ADDR = 0x6F;
constexpr uint8_t IOE_REG_MODE = 0x03;
constexpr uint8_t IOE_REG_OUT = 0x05;
constexpr uint8_t IOE_REG_PULLUP = 0x09;
constexpr uint8_t IOE_REG_PULLDOWN = 0x0B;
constexpr uint8_t IOE_REG_DRIVE = 0x13;
constexpr uint8_t IOE_BIT_EPD_POWER = 2;
constexpr uint8_t IOE_BIT_EPD_RESET = 4;
constexpr uint16_t IOE_OUTPUT_MASK = (1u << IOE_BIT_EPD_POWER) | (1u << IOE_BIT_EPD_RESET);

// ── Panel geometry ───────────────────────────────────────────────────────────
constexpr uint16_t PANEL_W = 800;  // source lines
constexpr uint16_t PANEL_H = 480;  // gate lines
constexpr uint16_t ROW_BYTES = PANEL_W / 8;
constexpr uint32_t PLANE_BYTES = static_cast<uint32_t>(ROW_BYTES) * PANEL_H;  // 48000

// ── Controller commands ──────────────────────────────────────────────────────
constexpr uint8_t CMD_DRIVER_OUTPUT = 0x01;
constexpr uint8_t CMD_GATE_VOLTAGE = 0x03;
constexpr uint8_t CMD_SOURCE_VOLTAGE = 0x04;
constexpr uint8_t CMD_BOOSTER = 0x0C;
constexpr uint8_t CMD_DATA_ENTRY = 0x11;
constexpr uint8_t CMD_SOFT_RESET = 0x12;
constexpr uint8_t CMD_TEMP_SENSOR_SEL = 0x18;
constexpr uint8_t CMD_TEMP_WRITE = 0x1A;
constexpr uint8_t CMD_UPDATE_CTRL1 = 0x21;
constexpr uint8_t CMD_UPDATE_CTRL2 = 0x22;
constexpr uint8_t CMD_MASTER_ACTIVATION = 0x20;
constexpr uint8_t CMD_WRITE_NEW = 0x24;  // "BW" plane  -> LUT index bit 0
constexpr uint8_t CMD_WRITE_OLD = 0x26;  // "RED" plane -> LUT index bit 1
constexpr uint8_t CMD_VCOM_WRITE = 0x2C;
constexpr uint8_t CMD_WRITE_LUT = 0x32;
constexpr uint8_t CMD_BORDER = 0x3C;
constexpr uint8_t CMD_RAM_X_WINDOW = 0x44;
constexpr uint8_t CMD_RAM_Y_WINDOW = 0x45;
constexpr uint8_t CMD_RAM_X_COUNTER = 0x4E;
constexpr uint8_t CMD_RAM_Y_COUNTER = 0x4F;

// ── State ────────────────────────────────────────────────────────────────────
uint32_t g_spiHz = 20000000;
SPISettings g_spi(20000000, MSBFIRST, SPI_MODE0);
uint16_t g_ioeOut = 0;
bool g_controllerPowered = false;
uint8_t* g_p24 = nullptr;  // LUT-index bit 0 plane
uint8_t* g_p26 = nullptr;  // LUT-index bit 1 plane
uint8_t* g_scratch = nullptr;
uint8_t* g_inv24 = nullptr;  // tonal inverse planes, used by burst()
uint8_t* g_inv26 = nullptr;
uint8_t* g_prev24 = nullptr;  // what is physically on the glass right now
uint8_t* g_prev26 = nullptr;
uint8_t* g_d24 = nullptr;  // per-pixel delta magnitude, built per pass
uint8_t* g_d26 = nullptr;

// Two host-uploaded page images, for the A<->B page-turn / ghosting test.
uint8_t* g_pgA24 = nullptr;
uint8_t* g_pgA26 = nullptr;
uint8_t* g_pgB24 = nullptr;
uint8_t* g_pgB26 = nullptr;
bool g_pgLoaded[2] = {false, false};
uint8_t g_abPage = 0;   // which slot is currently on the glass
bool g_abMode = false;  // buttons flip pages while this is set

// ── I2C / power ──────────────────────────────────────────────────────────────
bool ioeRead16(uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(IOE_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  delayMicroseconds(500);
  if (Wire.requestFrom(IOE_ADDR, static_cast<uint8_t>(2), static_cast<uint8_t>(true)) != 2) return false;
  const uint8_t lo = Wire.read();
  const uint8_t hi = Wire.read();
  value = static_cast<uint16_t>(lo) | static_cast<uint16_t>(hi << 8);
  return true;
}

bool ioeWrite16(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(IOE_ADDR);
  Wire.write(reg);
  Wire.write(static_cast<uint8_t>(value & 0xFF));
  Wire.write(static_cast<uint8_t>(value >> 8));
  if (Wire.endTransmission() != 0) return false;
  delayMicroseconds(500);
  return true;
}

bool ioeClearBits(uint8_t reg, uint16_t mask) {
  uint16_t value = 0;
  return ioeRead16(reg, value) && ioeWrite16(reg, static_cast<uint16_t>(value & ~mask));
}

void ioePin(uint8_t bit, bool high) {
  if (high) {
    g_ioeOut |= static_cast<uint16_t>(1u << bit);
  } else {
    g_ioeOut &= static_cast<uint16_t>(~(1u << bit));
  }
  ioeWrite16(IOE_REG_OUT, g_ioeOut);
}

bool boardBegin() {
  Wire.begin(PIN_SDA, PIN_SCL, 100000);
  uint16_t uid = 0;
  if (!ioeRead16(0x00, uid)) {
    Serial.println("ERR: M5IOE1 not responding at 0x6F");
    return false;
  }
  Serial.printf("M5IOE1 id=0x%04X\n", uid);
  uint16_t mode = 0;
  if (!ioeRead16(IOE_REG_MODE, mode) || !ioeRead16(IOE_REG_OUT, g_ioeOut)) return false;
  g_ioeOut &= static_cast<uint16_t>(~IOE_OUTPUT_MASK);
  if (!ioeWrite16(IOE_REG_OUT, g_ioeOut)) return false;
  ioeClearBits(IOE_REG_PULLUP, IOE_OUTPUT_MASK);
  ioeClearBits(IOE_REG_PULLDOWN, IOE_OUTPUT_MASK);
  ioeClearBits(IOE_REG_DRIVE, IOE_OUTPUT_MASK);
  if (!ioeWrite16(IOE_REG_MODE, static_cast<uint16_t>(mode | IOE_OUTPUT_MASK))) return false;
  return true;
}

void epdPower(bool on) { ioePin(IOE_BIT_EPD_POWER, on); }
void epdResetPin(bool high) { ioePin(IOE_BIT_EPD_RESET, high); }

// ── SPI primitives ───────────────────────────────────────────────────────────
void setSpiHz(uint32_t hz) {
  g_spiHz = hz;
  g_spi = SPISettings(hz, MSBFIRST, SPI_MODE0);
}

void cmd(uint8_t c) {
  SPI.beginTransaction(g_spi);
  digitalWrite(PIN_DC, LOW);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(c);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

void data(uint8_t d) {
  SPI.beginTransaction(g_spi);
  digitalWrite(PIN_DC, HIGH);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(d);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

void dataBulk(const uint8_t* d, uint32_t len) {
  SPI.beginTransaction(g_spi);
  digitalWrite(PIN_DC, HIGH);
  digitalWrite(PIN_CS, LOW);
  SPI.writeBytes(d, len);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

// ── BUSY timing ──────────────────────────────────────────────────────────────
struct Timing {
  uint32_t assertUs = 0;  // MASTER_ACTIVATION -> BUSY high (0 = never observed)
  uint32_t busyUs = 0;    // BUSY high -> BUSY low
  bool timedOut = false;
};

// Poll BUSY down. Tight for the first 3 ms (catches sub-ms operations), then
// delay(1) which yields so the idle task watchdog stays fed through a 2 s wave.
uint32_t waitBusyLow(uint32_t timeoutMs) {
  const uint32_t start = micros();
  while (digitalRead(PIN_BUSY) == HIGH) {
    const uint32_t elapsed = micros() - start;
    if (elapsed > timeoutMs * 1000UL) return 0;
    if (elapsed > 3000) delay(1);
  }
  return micros() - start;
}

Timing activateTimed(uint8_t control) {
  Timing t;
  cmd(CMD_UPDATE_CTRL2);
  data(control);
  cmd(CMD_MASTER_ACTIVATION);
  const uint32_t fired = micros();
  while (digitalRead(PIN_BUSY) != HIGH) {
    if (micros() - fired > 20000) break;  // 20 ms: no waveform started
  }
  if (digitalRead(PIN_BUSY) != HIGH) {
    t.assertUs = 0;
    t.busyUs = 0;
    return t;
  }
  t.assertUs = micros() - fired;
  // Generous: the dither soaks run tens of seconds, far longer than any real
  // page turn, and a premature give-up reports a bogus 0 us for a wave that is
  // still running.
  t.busyUs = waitBusyLow(120000);
  t.timedOut = (t.busyUs == 0);
  // 0xCC/C0 brings the analog rails up without the trailing power-down bits;
  // controls ending in 0x03 explicitly disable them. Warm 0x0C preserves the
  // prior state. Track this so gtg3 exercises the same cold/warm choice as the
  // production driver.
  if ((control & 0xC0u) == 0xC0u && (control & 0x03u) != 0x03u) {
    g_controllerPowered = true;
  } else if ((control & 0x03u) == 0x03u) {
    g_controllerPowered = false;
  }
  return t;
}

// ── Controller bring-up ──────────────────────────────────────────────────────
void hardReset() {
  epdResetPin(true);
  delay(10);
  epdResetPin(false);
  delay(10);
  epdResetPin(true);
  delay(10);
}

void setRamCounters() {
  cmd(CMD_RAM_X_COUNTER);
  data(static_cast<uint8_t>((PANEL_W - 1) & 0xFF));
  data(static_cast<uint8_t>((PANEL_W - 1) >> 8));
  cmd(CMD_RAM_Y_COUNTER);
  data(0x00);
  data(0x00);
}

// Vendor EPD_HW_Init sequence, minus the cmd 0x1A temperature bank select.
void initController() {
  hardReset();
  g_controllerPowered = false;
  waitBusyLow(1000);
  cmd(CMD_SOFT_RESET);
  waitBusyLow(1000);

  const uint8_t booster[] = {0xAE, 0xC7, 0xC3, 0xC0, 0x80};
  cmd(CMD_BOOSTER);
  for (uint8_t b : booster) data(b);

  cmd(CMD_DRIVER_OUTPUT);
  data(static_cast<uint8_t>((PANEL_H - 1) & 0xFF));
  data(static_cast<uint8_t>((PANEL_H - 1) >> 8));
  data(0x02);

  cmd(CMD_DATA_ENTRY);
  data(0x02);  // X decrement, Y increment

  cmd(CMD_RAM_X_WINDOW);
  data(static_cast<uint8_t>((PANEL_W - 1) & 0xFF));
  data(static_cast<uint8_t>((PANEL_W - 1) >> 8));
  data(0x00);
  data(0x00);

  cmd(CMD_RAM_Y_WINDOW);
  data(0x00);
  data(0x00);
  data(static_cast<uint8_t>((PANEL_H - 1) & 0xFF));
  data(static_cast<uint8_t>((PANEL_H - 1) >> 8));

  setRamCounters();
  waitBusyLow(1000);

  cmd(CMD_BORDER);
  data(0x01);
  cmd(CMD_TEMP_SENSOR_SEL);
  data(0x80);  // internal sensor
}

uint32_t writePlane(uint8_t ramCmd, const uint8_t* plane) {
  setRamCounters();
  cmd(ramCmd);
  const uint32_t started = micros();
  dataBulk(plane, PLANE_BYTES);
  return micros() - started;
}

// ── 111-byte LUT authoring ───────────────────────────────────────────────────
// Layout, decoded from the shipping tables and confirmed against the OEM
// 4-gray waveform:
//   [ 0.. 49] 5 VS entries x 10 group-bytes. Entry e = LUT index e
//             (0=white .. 3=black under the vendor plane packing, 4=VCOM).
//             Each byte holds 4 sub-phases A,B,C,D at 2 bits, MSB = phase A.
//             VS codes: 00=VSS(no drive) 01=VSH1(->black) 10=VSL(->white) 11=VSH2
//   [50.. 99] 10 groups x {TP_A, TP_B, TP_C, TP_D, RP} — frame counts + repeat
//   [100..104] frame-rate nibbles, 2 groups per byte
//   [105] VGH  [106] VSH1 [107] VSH2 [108] VSL  [109] VCOM  [110] reserved
struct Lut {
  uint8_t b[111];

  void clear() { memset(b, 0, sizeof(b)); }

  void setVs(uint8_t entry, uint8_t group, uint8_t phase, uint8_t vs) {
    uint8_t& target = b[entry * 10 + group];
    const uint8_t shift = static_cast<uint8_t>((3 - phase) * 2);
    target = static_cast<uint8_t>((target & ~(0x03u << shift)) | ((vs & 0x03u) << shift));
  }

  // Give one entry a single continuous drive of `frames` frames in group/phase A.
  void setTp(uint8_t group, uint8_t a, uint8_t bb, uint8_t c, uint8_t d, uint8_t rp) {
    b[50 + group * 5 + 0] = a;
    b[50 + group * 5 + 1] = bb;
    b[50 + group * 5 + 2] = c;
    b[50 + group * 5 + 3] = d;
    b[50 + group * 5 + 4] = rp;
  }

  void setFrameRate(uint8_t code) {
    const uint8_t packed = static_cast<uint8_t>((code & 0x0F) | ((code & 0x0F) << 4));
    for (uint8_t i = 0; i < 5; ++i) b[100 + i] = packed;
  }

  void setVoltages(uint8_t vgh, uint8_t vsh1, uint8_t vsh2, uint8_t vsl, uint8_t vcom) {
    b[105] = vgh;
    b[106] = vsh1;
    b[107] = vsh2;
    b[108] = vsl;
    b[109] = vcom;
  }
};

// Print the LUT exactly as the controller will read it, plus the frame budget
// each group actually costs. The point is to answer "did that phase run?"
// without guessing: the group table is arithmetic, and the total it predicts is
// compared against the measured BUSY window. A phase that the controller
// silently ignored shows up as a shortfall in the measured time, not as a
// missing line in a table we printed ourselves.
bool g_lutDump = false;

void dumpLut(const Lut& lut, uint16_t frames, uint32_t frameUs, uint32_t busyUs) {
  if (!g_lutDump) return;
  static const char* const kVs[4] = {"--", "+15", "-15", " +5"};
  Serial.println("  LUT  grp | e1  A   B   C   D | e2                | e3                | TPA TPB TPC TPD  RP | frames");
  uint32_t total = 0;
  for (uint8_t g = 0; g < 10; ++g) {
    const uint8_t* tp = &lut.b[50 + g * 5];
    const uint16_t per = static_cast<uint16_t>(tp[0] + tp[1] + tp[2] + tp[3]);
    const uint32_t runs = static_cast<uint32_t>(tp[4]) + 1u;  // RP counts EXTRA repeats
    const uint32_t cost = per * runs;
    bool used = cost != 0;
    for (uint8_t e = 1; e <= 3 && !used; ++e) used = lut.b[e * 10 + g] != 0;
    if (!used) continue;
    Serial.printf("       %u  |", g);
    for (uint8_t e = 1; e <= 3; ++e) {
      const uint8_t row = lut.b[e * 10 + g];
      for (uint8_t p = 0; p < 4; ++p) Serial.printf(" %s", kVs[(row >> ((3 - p) * 2)) & 0x03]);
      Serial.print(" |");
    }
    Serial.printf(" %3u %3u %3u %3u %3u | %lu\n", tp[0], tp[1], tp[2], tp[3], tp[4],
                  static_cast<unsigned long>(cost));
    total += cost;
  }
  const uint32_t predicted = total * frameUs;
  const long err = static_cast<long>(busyUs) - static_cast<long>(predicted);
  // The nominal frame period is a rounded table value, so the error grows with
  // the run -- 1600 frames drift ~20 ms without anything being wrong. Allow one
  // frame of fixed slop plus 1% of the run before calling a phase missing.
  const long slop = static_cast<long>(frameUs) + static_cast<long>(predicted / 100u);
  Serial.printf("  LUT  %lu frames (accounted %u) -> predicted %lu us, measured %lu us, delta %+ld us%s\n",
                static_cast<unsigned long>(total), frames, static_cast<unsigned long>(predicted),
                static_cast<unsigned long>(busyUs), err,
                err < -slop ? "   << WAVEFORM SHORTER THAN LUT" : "");
}

void loadLut(const Lut& lut) {
  cmd(CMD_WRITE_LUT);
  dataBulk(lut.b, 105);
  cmd(CMD_GATE_VOLTAGE);
  data(lut.b[105]);
  cmd(CMD_SOURCE_VOLTAGE);
  data(lut.b[106]);
  data(lut.b[107]);
  data(lut.b[108]);
  cmd(CMD_VCOM_WRITE);
  data(lut.b[109]);
}

// ── Test imagery ─────────────────────────────────────────────────────────────
// Levels are LUT indices: 0 = white, 1 = light, 2 = dark, 3 = black.
void setPixel(uint32_t x, uint32_t y, uint8_t level) {
  const uint32_t byteIndex = y * ROW_BYTES + (x >> 3);
  const uint8_t mask = static_cast<uint8_t>(0x80u >> (x & 7));
  if (level & 0x01) {
    g_p24[byteIndex] |= mask;
  } else {
    g_p24[byteIndex] &= static_cast<uint8_t>(~mask);
  }
  if (level & 0x02) {
    g_p26[byteIndex] |= mask;
  } else {
    g_p26[byteIndex] &= static_cast<uint8_t>(~mask);
  }
}

void fillLevel(uint8_t level) {
  memset(g_p24, (level & 0x01) ? 0xFF : 0x00, PLANE_BYTES);
  memset(g_p26, (level & 0x02) ? 0xFF : 0x00, PLANE_BYTES);
}

// Test card:
//   band 1 (y   0..239) four 200 px columns: white | light | dark | black
//   band 2 (y 240..359) eight 100 px columns W L D B B D L W (adjacency check)
//   band 3 (y 360..479) 1 px vertical lines: left half B/W, right half light/dark
void drawTestCard() {
  fillLevel(0);
  for (uint32_t y = 0; y < 240; ++y) {
    for (uint32_t x = 0; x < PANEL_W; ++x) setPixel(x, y, static_cast<uint8_t>(x / 200));
  }
  static constexpr uint8_t kRamp[8] = {0, 1, 2, 3, 3, 2, 1, 0};
  for (uint32_t y = 240; y < 360; ++y) {
    for (uint32_t x = 0; x < PANEL_W; ++x) setPixel(x, y, kRamp[x / 100]);
  }
  for (uint32_t y = 360; y < PANEL_H; ++y) {
    for (uint32_t x = 0; x < PANEL_W; ++x) {
      const bool odd = (x & 1) != 0;
      if (x < 400) {
        setPixel(x, y, odd ? 3 : 0);
      } else {
        setPixel(x, y, odd ? 2 : 1);
      }
    }
  }
}

// A 16-step horizontal staircase, for judging how far a waveform can actually
// separate levels once we start authoring our own.
// Three vertical bands (white / gray / black) over the top two thirds, and a
// fine 1-px vertical grating in each band below -- the grating is where uneven
// or ghosted grays show up first.
void drawThreeLevelSteps() {
  static constexpr uint8_t TRI[3] = {0, 1, 3};
  fillLevel(0);
  for (uint32_t y = 0; y < PANEL_H; ++y) {
    for (uint32_t x = 0; x < PANEL_W; ++x) {
      const uint8_t band = TRI[(x * 3) / PANEL_W];
      if (y < (PANEL_H * 2) / 3) {
        setPixel(x, y, band);
      } else {
        setPixel(x, y, (x & 1) ? 0 : band);
      }
    }
  }
}

// Horizontally banded W/G/B source used by the cleanup A/B. Every X position
// sees the same history, so comparing the center third with the two side thirds
// isolates the cleanup waveform instead of panel-content differences.
void drawCleanCompareSource() {
  static constexpr uint8_t LEVELS[] = {3, 1, 0, 1, 3, 0};
  for (uint32_t y = 0; y < PANEL_H; ++y) {
    const uint8_t level = LEVELS[(y / 20) % (sizeof(LEVELS) / sizeof(LEVELS[0]))];
    memset(g_p24 + y * ROW_BYTES, (level & 0x01) ? 0xFF : 0x00, ROW_BYTES);
    memset(g_p26 + y * ROW_BYTES, (level & 0x02) ? 0xFF : 0x00, ROW_BYTES);
  }
}

void drawFourLevelSteps() {
  fillLevel(0);
  for (uint32_t y = 0; y < PANEL_H; ++y) {
    for (uint32_t x = 0; x < PANEL_W; ++x) setPixel(x, y, static_cast<uint8_t>((x * 4) / PANEL_W));
  }
}

// ── Refresh recipes ──────────────────────────────────────────────────────────
void reportPlanes(uint32_t us24, uint32_t us26) {
  const uint32_t total = us24 + us26;
  const uint32_t kbps = total ? static_cast<uint32_t>((2ULL * PLANE_BYTES * 1000ULL) / total) : 0;
  Serial.printf("  planes: 0x24=%lu us  0x26=%lu us  total=%lu us  (%lu KB/s @ %lu MHz)\n",
                static_cast<unsigned long>(us24), static_cast<unsigned long>(us26),
                static_cast<unsigned long>(total), static_cast<unsigned long>(kbps),
                static_cast<unsigned long>(g_spiHz / 1000000));
}

void reportTiming(const char* label, const Timing& t) {
  Serial.printf("  %s: assert=%lu us  waveform=%lu us (%lu ms)%s\n", label,
                static_cast<unsigned long>(t.assertUs), static_cast<unsigned long>(t.busyUs),
                static_cast<unsigned long>(t.busyUs / 1000), t.timedOut ? "  TIMEOUT" : "");
}

// OTP bank refresh: pick a waveform bank with the fake-temperature register and
// fire 0xD7, which reloads the OTP LUT but deliberately omits the "load
// temperature" bit (0x20) so our written value survives and selects the bank.
void otpBankRefresh(uint8_t tempByte, bool reinit) {
  if (reinit) initController();
  cmd(CMD_TEMP_WRITE);
  data(tempByte);
  const uint32_t us24 = writePlane(CMD_WRITE_NEW, g_p24);
  const uint32_t us26 = writePlane(CMD_WRITE_OLD, g_p26);
  const Timing t = activateTimed(0xD7);
  Serial.printf("OTP bank 0x%02X (reinit=%d)\n", tempByte, reinit ? 1 : 0);
  reportPlanes(us24, us26);
  reportTiming("0xD7", t);
}

void otpControlRefresh(uint8_t control, bool reinit) {
  if (reinit) initController();
  const uint32_t us24 = writePlane(CMD_WRITE_NEW, g_p24);
  const uint32_t us26 = writePlane(CMD_WRITE_OLD, g_p26);
  const Timing t = activateTimed(control);
  Serial.printf("OTP control 0x%02X (reinit=%d)\n", control, reinit ? 1 : 0);
  reportPlanes(us24, us26);
  reportTiming("activation", t);
}

void customRefresh(const Lut& lut, uint8_t control, bool reinit, const char* label) {
  if (reinit) initController();
  cmd(CMD_BORDER);
  data(0x80);  // hold border at VCOM so it does not track the injected LUT
  const uint32_t us24 = writePlane(CMD_WRITE_NEW, g_p24);
  const uint32_t us26 = writePlane(CMD_WRITE_OLD, g_p26);
  loadLut(lut);
  cmd(CMD_UPDATE_CTRL1);
  data(0x00);
  const Timing t = activateTimed(control);
  Serial.printf("custom %s ctrl=0x%02X (reinit=%d)\n", label, control, reinit ? 1 : 0);
  reportPlanes(us24, us26);
  reportTiming("activation", t);
}

// ── Custom four-gray waveform ────────────────────────────────────────────────
// Frame periods in microseconds per frame-rate code, measured on this panel by
// timing a known frame count at each code (probeFrameRates, 60 frames vs 20
// frames so the fixed activation overhead cancels). The series is exact:
// code 1..8 = 25..200 Hz in 25 Hz steps, code 9..15 = the 12.5 Hz half-steps,
// code 0 = 15 Hz.
constexpr uint32_t FRAME_US[16] = {66400, 39800, 19900, 13300, 9950, 7975,  6650,  5700,
                                   5000,  26575, 15925, 11400, 8850, 7250,  6150,  5300};

// Source voltages, from cmd 0x04: VSH1 = +15 V, VSH2 = +5 V, VSL = -15 V.
// VSH2 being a third of VSH1 is what makes fine gray modulation practical —
// particles move ~3x slower under it, so the same timing granularity buys much
// finer control of the intermediate levels.
constexpr uint8_t VS_NONE = 0x00;   // VSS, no drive
constexpr uint8_t VS_BLACK = 0x01;  // VSH1 +15 V
constexpr uint8_t VS_WHITE = 0x02;  // VSL  -15 V
constexpr uint8_t VS_WEAK = 0x03;   // VSH2 +5 V

struct GrayParams {
  uint8_t fr = 0x04;       // frame-rate code (0x04 = 100 Hz, 9.95 ms/frame)
  uint8_t shake = 0;       // frames per half-cycle of the pre-shake
  uint8_t shakeRep = 0;    // extra repeats of the shake pair
  uint8_t toBlack = 20;    // frames driving every pixel to the black rail
  uint8_t toWhite = 20;    // frames driving every pixel to the white rail
  uint8_t a = 7;           // modulation sub-phase A (light gray and darker)
  uint8_t b = 5;           // sub-phase B (dark gray and darker)
  uint8_t c = 8;           // sub-phase C (black only)
  uint8_t modCode = VS_BLACK;
  uint8_t scheme = 1;  // 0 = reset-to-white then drive black; 1 = merged (see below)
  uint8_t border = 0x80;  // SSD1677 VCOM border (HiZ would be 0xC0)
  uint8_t ctrlCold = 0xCC;  // power up + display, hold power
  uint8_t ctrlWarm = 0x0C;  // display only, already powered
  uint8_t vgh = 0x17, vsh1 = 0x41, vsh2 = 0xA8, vsl = 0x32, vcom = 0x30;
  uint8_t dcLimit = 30;  // corrective refresh due past this many kV*frames of drift
  // Saturate-and-return tuning. The black->white transition is strongly
  // non-linear -- most of the optical swing happens in the first few frames --
  // so the two pull-backs are tuned independently instead of k*step.
  uint8_t satSwing = 30;  // frames driving changed pixels to their rail
  uint8_t pullDark = 3;   // *pulses* of white for level 2 (dark gray)
  uint8_t pullLight = 5;  // *extra* pulses for level 1, so light = pullDark + pullLight
  uint8_t frRet = 0x08;   // frame-rate code for the return pass (0x08 = 5000 us)
  // The black->white transition is only ~10 frames wide even at the fastest
  // frame rate, so one frame is ~10% of full swing -- far too coarse to place
  // two intermediate levels, and it makes the gate-line RC droop across the
  // 800 columns show up as a left/right shade difference. So the pull-back is
  // a pulse train instead of one continuous pull: pulseOn frames of white
  // followed by pulseOff frames of a weak opposing drive. The opposing frames
  // slow the net rate (more steps of control) and, because the error alternates
  // sign, the row-position error averages instead of integrating.
  uint8_t pulseOn = 1;          // driven frames per pulse
  uint8_t pulseOff = 1;         // opposing frames per pulse (0 = continuous drive)
  uint8_t pulseCode = VS_WEAK;  // opposing drive (VSH2 +5 V by default)
  uint8_t rampCode = VS_WEAK;  // drive used by 'ramp 1' (white -> black direction)
  // Single-activation 3-gray ('tri'). All times in frames at frRet.
  //
  // DC balance is NOT deferred here: each of the three driven classes is net
  // zero on its own. That is the only arrangement that survives arbitrary
  // content, because the loop W->B->W sums netB + netW and the loop W->G->W sums
  // netG + netW; requiring every loop to vanish forces every class to vanish
  // individually. A periodic "corrective" pass cannot fix it either, since the
  // debt is per pixel and depends on that pixel's own history, while any pass we
  // can issue applies the same charge to every pixel in a class.
  //
  // Group 0 is the activation, split into two equal sub-phases so each class can
  // pick its own pair of rails:
  //
  //   white   +15 +15   full excursion to black, then G1 brings it back
  //   gray    +15 -15   excursion out and back; net zero, so gray stays cheap
  //   black   -15 -15   a doubled white reset, which is also its deghost
  //
  // Everything then lands on the white rail in G1 and develops from there.
  // Balance falls out as preUp == satWhite == S, tBlack == 2S, tGray == 3S
  // (gray develops at +5 V, one third the rail, so it needs three times the
  // frames). Total is 5S frames -- 400 ms at S = 16 and 5 ms/frame.
  //
  // preUp is also the deghost. A pixel already sitting near white gets no
  // excursion from a white-only reset, so whatever residue it holds is never
  // released and each redraw develops on top of it -- which is exactly how text
  // edges creep darker over a few page turns. Set preUp = 0 to reproduce that.
  uint8_t preUp = 16;     // G0: activation, halved between the two sub-phases
  uint8_t satWhite = 16;  // G1: every driven class to the white rail
  uint8_t grayFast = 0;   // +15 V frames at the head of the gray drive (breaks balance)
  uint8_t tGray = 48;     // +5 V frames forming the middle level
  uint8_t tBlack = 32;    // +15 V frames forming black
  // Source-aware three-level GTG. One step is the current fast LUT's
  // 1-away/3-toward pulse shape at half its old cycle count. Endpoint changes
  // run two steps; white targets finish with a symmetric away/target cleanup.
  uint8_t stepAway = 1;
  uint8_t stepToward = 3;
  uint8_t stepCycles = 3;
  uint8_t postClean = 12;
  // Every dcEvery-th tri drives EVERY pixel instead of only the changed ones.
  // This is not a DC correction (the waveform is already balanced) -- it is an
  // optical scrub: 'changed' is judged against what we intended to put on the
  // glass, so any pixel whose real state fell short is otherwise never revisited.
  // Same duration as a normal refresh, and it flashes the whole screen uniformly
  // rather than in patches, which is far less noticeable than it sounds.
  uint8_t dcEvery = 8;
};

GrayParams g_gray;

// Deferred DC balance. A single refresh does not have to be net-zero -- we let
// the imbalance accumulate and settle it later with one corrective refresh,
// which is far cheaper than paying compensation frames on every update.
//
// The exact per-pixel charge depends on content, so this tracks the two
// extremes: the most positive net any pixel could have taken (a pixel that goes
// to black every time) and the most negative (one that goes to white every
// time). Real pixels lie between, so a corrective refresh triggered on the
// envelope is always early rather than late.
int32_t g_dcPos = 0;  // V*frames
int32_t g_dcNeg = 0;
uint32_t g_dcCount = 0;
uint32_t g_triCount = 0;  // page turns since boot, for the periodic scrub pass

void dcAccumulate(int32_t netMax, int32_t netMin) {
  g_dcPos += netMax;
  g_dcNeg += netMin;
  ++g_dcCount;
}

// True when the envelope has drifted far enough that a corrective (rail-to-rail,
// self-balancing) refresh is worth spending.
bool dcDue() {
  const int32_t limit = static_cast<int32_t>(g_gray.dcLimit) * 1000;
  return g_dcPos > limit || g_dcNeg < -limit;
}

void dcReport() {
  Serial.printf("DC envelope after %lu refreshes: pos=%+ld  neg=%+ld  V*frames (limit +-%ld)%s\n",
                static_cast<unsigned long>(g_dcCount), static_cast<long>(g_dcPos), static_cast<long>(g_dcNeg),
                static_cast<long>(g_gray.dcLimit) * 1000, dcDue() ? "  << CORRECTIVE DUE" : "");
}

void dcReset() {
  g_dcPos = 0;
  g_dcNeg = 0;
  g_dcCount = 0;
}

// Group plan:
//   G0 shake    A = black, B = white, repeated (loosens particles, kills history)
//   G1 to-black every entry driven to the black rail
//   G2 to-white every entry driven to the white rail — common starting point
//   G3 modulate cumulative sub-phases; each level stops at a different one
uint32_t buildGrayLut(Lut& lut, const GrayParams& p) {
  lut.clear();
  uint8_t group = 0;
  uint32_t frames = 0;

  if (p.shake > 0) {
    for (uint8_t e = 0; e < 4; ++e) {
      lut.setVs(e, group, 0, VS_BLACK);
      lut.setVs(e, group, 1, VS_WHITE);
    }
    lut.setTp(group, p.shake, p.shake, 0, 0, p.shakeRep);
    frames += static_cast<uint32_t>(2 * p.shake) * (p.shakeRep + 1);
    ++group;
  }
  if (p.scheme == 5) {
    // Scheme 5 — right-aligned modulation. The defect in every earlier scheme is
    // that a level's black drive starts from wherever the common white phase
    // left it, and the common white phase is only `toWhite` long. Entry 0 was
    // fixed by giving it the modulation window as extra white (scheme 3), which
    // is why the background went clean while the grays did not: entry 1 still
    // modulates after only `toWhite` frames of white, so its starting point
    // still carries history.
    //
    // Fix without spending a single extra frame: keep the window M = a+b+c, but
    // right-align each level's black drive inside it and fill the front with
    // white. Ordering the phases [c, b, a] puts the switch points exactly where
    // they need to be, and the LUT becomes a triangle:
    //
    //            phase C(c)  phase B(b)  phase A(a)      white   black
    //   entry0     white       white       white          M        0
    //   entry1     white       white       black          b+c      a
    //   entry2     white       black       black          c        a+b
    //   entry3     black       black       black          0        M
    //
    // Every level is now driven white right up until its own black drive starts,
    // so the lighter the level the deeper its white saturation — which is the
    // opposite of the old behaviour and exactly what the grays needed. Total
    // stays R + W + M.
    //
    // This is deliberately NOT DC balanced per refresh (net = 2*m_e - M, spread
    // 2M across levels); balance is amortised and settled by a periodic
    // compensating flush instead of being paid for on every page.
    if (p.toBlack > 0) {
      for (uint8_t e = 0; e < 4; ++e) lut.setVs(e, group, 0, VS_BLACK);
      lut.setTp(group, p.toBlack, 0, 0, 0, 0);
      frames += p.toBlack;
      ++group;
    }
    for (uint8_t e = 0; e < 4; ++e) lut.setVs(e, group, 0, VS_WHITE);
    lut.setTp(group, p.toWhite, 0, 0, 0, 0);
    frames += p.toWhite;
    ++group;

    // Phase A = c, B = b, C = a. Entry e goes black once the remaining window
    // equals its own modulation depth.
    lut.setVs(0, group, 0, VS_WHITE);
    lut.setVs(0, group, 1, VS_WHITE);
    lut.setVs(0, group, 2, VS_WHITE);
    lut.setVs(1, group, 0, VS_WHITE);
    lut.setVs(1, group, 1, VS_WHITE);
    lut.setVs(1, group, 2, p.modCode);
    lut.setVs(2, group, 0, VS_WHITE);
    lut.setVs(2, group, 1, p.modCode);
    lut.setVs(2, group, 2, p.modCode);
    lut.setVs(3, group, 0, p.modCode);
    lut.setVs(3, group, 1, p.modCode);
    lut.setVs(3, group, 2, p.modCode);
    lut.setTp(group, p.c, p.b, p.a, 0, 0);
    frames += static_cast<uint32_t>(p.a) + p.b + p.c;

    lut.setFrameRate(p.fr);
    lut.setVoltages(p.vgh, p.vsh1, p.vsh2, p.vsl, p.vcom);
    return frames;
  }
  if (p.scheme == 3) {
    // Scheme 3 — balanced, and it spends the waveform where the eye actually
    // looks. In every other scheme entry 0 (white, i.e. the page background)
    // sits at VSS for the whole modulation group: M frames of nothing. Ghosting
    // is worst exactly there, because a pixel that was black last page only got
    // `toWhite` frames of white to recover. So drive entry 0 white through the
    // modulation window too — that costs zero extra time, it is dead air
    // otherwise — and pay for the charge with an entry-0-only black phase up
    // front. Solving net = 0 per entry gives toWhite == toBlack and an extra
    // black phase of exactly M for entry 0:
    //   entry0  +P +M  -(P + M)      = 0   and gets +-(P+M) of reset
    //   entry1  +P     -(P + a)  +a  = 0
    //   entry2  +P     -(P + a+b)+a+b= 0
    //   entry3  +P     -(P + M)  +M  = 0
    const uint8_t reset = p.toBlack;
    const uint8_t m = static_cast<uint8_t>(p.a + p.b + p.c);

    // G: black. Phase A everyone, phase B entry 0 alone.
    for (uint8_t e = 0; e < 4; ++e) lut.setVs(e, group, 0, VS_BLACK);
    lut.setVs(0, group, 1, VS_BLACK);
    lut.setTp(group, reset, m, 0, 0, 0);
    frames += static_cast<uint32_t>(reset) + m;
    ++group;

    // G: white. Phase A everyone, then the per-level charge compensation.
    for (uint8_t e = 0; e < 4; ++e) lut.setVs(e, group, 0, VS_WHITE);
    for (uint8_t e = 1; e < 4; ++e) lut.setVs(e, group, 1, VS_WHITE);
    for (uint8_t e = 2; e < 4; ++e) lut.setVs(e, group, 2, VS_WHITE);
    lut.setVs(3, group, 3, VS_WHITE);
    lut.setTp(group, reset, p.a, p.b, p.c, 0);
    frames += static_cast<uint32_t>(reset) + m;
    ++group;

    // G: modulate. Entries 1-3 take cumulative black; entry 0 takes white for
    // the whole window instead of idling.
    for (uint8_t phase = 0; phase < 3; ++phase) lut.setVs(0, group, phase, VS_WHITE);
    for (uint8_t e = 1; e < 4; ++e) lut.setVs(e, group, 0, p.modCode);
    for (uint8_t e = 2; e < 4; ++e) lut.setVs(e, group, 1, p.modCode);
    lut.setVs(3, group, 2, p.modCode);
    lut.setTp(group, p.a, p.b, p.c, 0, 0);
    frames += m;

    lut.setFrameRate(p.fr);
    lut.setVoltages(p.vgh, p.vsh1, p.vsh2, p.vsl, p.vcom);
    return frames;
  }
  if (p.toBlack > 0) {
    for (uint8_t e = 0; e < 4; ++e) lut.setVs(e, group, 0, VS_BLACK);
    lut.setTp(group, p.toBlack, 0, 0, 0, 0);
    frames += p.toBlack;
    ++group;
  }
  if (p.scheme == 2) {
    // DC-balanced. Every entry gets exactly as much extra white drive as the
    // black modulation it will receive, so net impulse per entry is
    //   +K  -(K + M)  +M  =  0
    // for all four levels. Unbalanced waveforms leave a net charge that biases
    // the next refresh — that is what shows up as residual ghosting and as
    // gray levels drifting over a sequence of pages.
    // White phase: A all, B levels 1-3, C levels 2-3, D level 3.
    for (uint8_t e = 0; e < 4; ++e) lut.setVs(e, group, 0, VS_WHITE);
    for (uint8_t e = 1; e < 4; ++e) lut.setVs(e, group, 1, VS_WHITE);
    for (uint8_t e = 2; e < 4; ++e) lut.setVs(e, group, 2, VS_WHITE);
    lut.setVs(3, group, 3, VS_WHITE);
    lut.setTp(group, p.toWhite, p.a, p.b, p.c, 0);
    frames += static_cast<uint32_t>(p.toWhite) + p.a + p.b + p.c;
    ++group;

    // Black modulation, mirroring the compensation above.
    for (uint8_t e = 1; e < 4; ++e) lut.setVs(e, group, 0, p.modCode);
    for (uint8_t e = 2; e < 4; ++e) lut.setVs(e, group, 1, p.modCode);
    lut.setVs(3, group, 2, p.modCode);
    lut.setTp(group, p.a, p.b, p.c, 0, 0);
    frames += static_cast<uint32_t>(p.a) + p.b + p.c;

    lut.setFrameRate(p.fr);
    lut.setVoltages(p.vgh, p.vsh1, p.vsh2, p.vsl, p.vcom);
    return frames;
  }

  if (p.scheme == 1) {
    // Merged scheme: the to-black group above already equalised every pixel at
    // the black rail, which is history-independent. So the return to white *is*
    // the modulation — each level just stops at a different sub-phase. That
    // removes a whole full-swing phase versus scheme 0, which drives everything
    // to white and then drags the dark levels back toward black.
    //   entry3 black: no white drive at all
    //   entry2 dark : a          entry1 light: a+b          entry0 white: a+b+c
    lut.setVs(2, group, 0, VS_WHITE);
    lut.setVs(1, group, 0, VS_WHITE);
    lut.setVs(1, group, 1, VS_WHITE);
    lut.setVs(0, group, 0, VS_WHITE);
    lut.setVs(0, group, 1, VS_WHITE);
    lut.setVs(0, group, 2, VS_WHITE);
    lut.setTp(group, p.a, p.b, p.c, 0, 0);
    frames += static_cast<uint32_t>(p.a) + p.b + p.c;
    lut.setFrameRate(p.fr);
    lut.setVoltages(p.vgh, p.vsh1, p.vsh2, p.vsl, p.vcom);
    return frames;
  }

  if (p.toWhite > 0) {
    for (uint8_t e = 0; e < 4; ++e) lut.setVs(e, group, 0, VS_WHITE);
    lut.setTp(group, p.toWhite, 0, 0, 0, 0);
    frames += p.toWhite;
    ++group;
  }
  // Entry 0 (white) takes no drive at all; 1, 2, 3 accumulate A, A+B, A+B+C.
  lut.setVs(1, group, 0, p.modCode);
  lut.setVs(2, group, 0, p.modCode);
  lut.setVs(2, group, 1, p.modCode);
  lut.setVs(3, group, 0, p.modCode);
  lut.setVs(3, group, 1, p.modCode);
  lut.setVs(3, group, 2, p.modCode);
  lut.setTp(group, p.a, p.b, p.c, 0, 0);
  frames += static_cast<uint32_t>(p.a) + p.b + p.c;

  lut.setFrameRate(p.fr);
  lut.setVoltages(p.vgh, p.vsh1, p.vsh2, p.vsl, p.vcom);
  return frames;
}

void commitDisplayed();

// Deep clear: drive the whole panel through several full black/white swings at
// the slow rail, ending on white. This is the reference "erased" state — every
// ghosting measurement has to start here, otherwise you are measuring the
// accumulated history of the whole session rather than the residue left by the
// waveform under test.
void deepClear(uint8_t cycles) {
  if (cycles == 0) cycles = 4;
  fillLevel(0);  // both planes -> entry 0, so one entry's VS drives everything
  Lut lut;
  lut.clear();
  // Group 0: black then white, repeated. 30 frames each at 15 Hz = 2 s per rail,
  // far past saturation, which is the point: nothing survives it.
  for (uint8_t e = 0; e < 4; ++e) {
    lut.setVs(e, 0, 0, VS_BLACK);
    lut.setVs(e, 0, 1, VS_WHITE);
  }
  lut.setTp(0, 30, 30, 0, 0, static_cast<uint8_t>(cycles - 1));
  lut.setFrameRate(0x02);  // 50 Hz, 19900 us
  lut.setVoltages(g_gray.vgh, g_gray.vsh1, g_gray.vsh2, g_gray.vsl, g_gray.vcom);

  const uint32_t frames = 60UL * cycles;
  cmd(CMD_BORDER);
  data(g_gray.border);
  writePlane(CMD_WRITE_NEW, g_p24);
  writePlane(CMD_WRITE_OLD, g_p26);
  loadLut(lut);
  // Cold control (0xCC): powers the analog rails up and leaves them up, so the
  // refreshes that follow this clear can all run warm.
  const Timing t = activateTimed(g_gray.ctrlCold);
  commitDisplayed();
  // Equal black and white frames per cycle, so this is the corrective refresh:
  // it is net-zero by construction and resets the accumulated envelope.
  dcReset();
  Serial.printf("deep clear: %u cycles, %lu frames, %lu us (%lu ms)\n", cycles,
                static_cast<unsigned long>(frames), static_cast<unsigned long>(t.busyUs),
                static_cast<unsigned long>(t.busyUs / 1000));
}

// Drive every pixel hard to whatever is staged in g_p24/g_p26, quantised to
// black or white. Used to plant a known, fully saturated ghost source: a short
// push would leave the black under-developed and understate the residue that
// the erase then has to remove.
void hardPaint(uint8_t frames, uint8_t frCode) {
  for (uint32_t i = 0; i < PLANE_BYTES; ++i) {
    const uint8_t black = static_cast<uint8_t>(g_p24[i] & g_p26[i]);
    g_d24[i] = 0xFF;   // every pixel driven: entry 1 (white) or entry 3 (black)
    g_d26[i] = black;  // entry = 1 + 2*black
  }
  Lut lut;
  lut.clear();
  lut.setVs(1, 0, 0, VS_WHITE);
  lut.setVs(3, 0, 0, VS_BLACK);
  lut.setTp(0, frames, 0, 0, 0, 0);
  lut.setFrameRate(frCode);
  lut.setVoltages(g_gray.vgh, g_gray.vsh1, g_gray.vsh2, g_gray.vsl, g_gray.vcom);
  cmd(CMD_BORDER);
  data(g_gray.border);
  writePlane(CMD_WRITE_NEW, g_d24);
  writePlane(CMD_WRITE_OLD, g_d26);
  loadLut(lut);
  activateTimed(g_gray.ctrlWarm);
  memcpy(g_prev24, g_p24, PLANE_BYTES);
  memcpy(g_prev26, g_p26, PLANE_BYTES);
}

// How long does the white rail actually need to erase a saturated black?
//
// Every ghosting argument so far has been arithmetic on a number nobody
// measured. The deep clear spends 30 frames at 19.9 ms -- 597 ms -- per rail
// and is deliberately far past saturation; 'tri' spends 80 ms. Somewhere
// between those is the real requirement, and the whole time budget depends on
// where.
//
// One activation can only distinguish three driven classes, so the screen is
// split into three horizontal bands and each band's erase gets its own length.
// The ghost source is identical in all three (50 px vertical stripes running
// the full height), so whichever bands still show the stripes are the ones
// whose erase was too short. upF prepends a shared +15 V excursion, which
// separates "needs more volt-seconds" from "needs to be driven the other way
// first" -- those are different physics and they want different fixes.
// Plant an identical, fully saturated ghost source everywhere: 50 px vertical
// stripes running the full height, so all three measurement bands start from
// the same worst case.
void plantStripes() {
  deepClear(2);
  for (uint32_t y = 0; y < PANEL_H; ++y) {
    for (uint32_t x = 0; x < PANEL_W; ++x) setPixel(x, y, ((x / 50) & 1) ? 3 : 0);
  }
  hardPaint(30, 0x02);  // 597 ms per rail, same as the deep clear
  delay(500);           // let the pigment settle before measuring the erase
}

// Split the screen into three horizontal bands, one LUT entry each, regardless
// of what is currently on the glass.
void bandPlanes() {
  for (uint32_t y = 0; y < PANEL_H; ++y) {
    const uint8_t entry = static_cast<uint8_t>(1 + (y * 3) / PANEL_H);
    memset(g_d24 + y * ROW_BYTES, (entry & 1) ? 0xFF : 0x00, ROW_BYTES);
    memset(g_d26 + y * ROW_BYTES, (entry >> 1) ? 0xFF : 0x00, ROW_BYTES);
  }
}

// Push the banded planes with `lut` and leave the panel believing it is white.
void runBanded(Lut& lut, uint8_t frCode, uint16_t frames, uint8_t group, const char* what) {
  lut.setFrameRate(frCode);
  lut.setVoltages(g_gray.vgh, g_gray.vsh1, g_gray.vsh2, g_gray.vsl, g_gray.vcom);
  cmd(CMD_BORDER);
  data(g_gray.border);
  writePlane(CMD_WRITE_NEW, g_d24);
  writePlane(CMD_WRITE_OLD, g_d26);
  loadLut(lut);
  const Timing t = activateTimed(g_gray.ctrlWarm);
  fillLevel(0);
  memcpy(g_prev24, g_p24, PLANE_BYTES);
  memcpy(g_prev26, g_p26, PLANE_BYTES);
  dcReset();
  Serial.printf("%s ran %u frames in %u groups, %lu us (%lu ms)\n", what, frames, group,
                static_cast<unsigned long>(t.busyUs), static_cast<unsigned long>(t.busyUs / 1000));
  dumpLut(lut, frames, FRAME_US[frCode & 0x0F], t.busyUs);
}

// Does a CHARGE-BALANCED cycle clean a ghost?
//
// This is the question the whole design hangs on. Erasing to white costs
// negative charge, and per-class DC balance then demands a +15 V excursion just
// as long as the erase -- so a 500 ms erase implies a 500 ms black flash, which
// is unaffordable. The only way out is if agitation cleans without net
// displacement: real waveforms carry an "activation" phase on the theory that
// slamming the particles both ways unsticks them. Whether that actually holds
// on this pigment is not derivable, so all three bands get the SAME total time
// and the same rails, differing only in how the charge is arranged:
//
//   top     pure erase, all -15 V           negative, known to work at 500 ms
//   middle  one big cycle, +15 then -15     balanced, coarse
//   bottom  many short cycles, +15/-15      balanced, fine -- the background
//                                           polish model, and interruptible
//
// If either balanced band comes out clean, the polish can run between page
// turns for free. If both stay ghosted, whitening genuinely costs charge and
// the fast path has to carry unbalanced erase plus a periodic full refresh.
void polishTest(uint8_t half, uint8_t groups) {
  if (groups == 0 || groups > 10) groups = 10;
  if (half == 0) half = 5;
  const uint32_t frameUs = FRAME_US[g_gray.frRet & 0x0F];
  const uint16_t frames = static_cast<uint16_t>(groups) * 2u * half;
  Serial.printf("=== polish: %u groups x (%u up + %u down) = %u frames, %lu ms total per band ===\n", groups, half,
                half, frames, static_cast<unsigned long>(frames * frameUs / 1000));
  plantStripes();
  bandPlanes();

  Lut lut;
  lut.clear();
  const uint8_t flip = static_cast<uint8_t>(groups / 2);
  for (uint8_t g = 0; g < groups; ++g) {
    // top: erase the whole way
    lut.setVs(1, g, 0, VS_WHITE);
    lut.setVs(1, g, 1, VS_WHITE);
    // middle: one cycle -- black for the first half of the time, then white
    lut.setVs(2, g, 0, g < flip ? VS_BLACK : VS_WHITE);
    lut.setVs(2, g, 1, g < flip ? VS_BLACK : VS_WHITE);
    // bottom: one short cycle per group
    lut.setVs(3, g, 0, VS_BLACK);
    lut.setVs(3, g, 1, VS_WHITE);
    lut.setTp(g, half, half, 0, 0, 0);
  }
  runBanded(lut, g_gray.frRet, frames, groups, "polish");
  Serial.printf("top = pure erase (%u frames of -15 V, unbalanced)\n", frames);
  Serial.printf("middle = one balanced cycle (%u up then %u down)\n", flip * 2 * half, (groups - flip) * 2 * half);
  Serial.printf("bottom = %u balanced cycles of %u up / %u down\n", groups, half, half);
  Serial.println("a clean middle or bottom means balance is free; both ghosted means it is not");
}

// Is a small balanced cycle WEAK, or merely SLOW?
//
// polish answered the first-order question -- one coarse balanced cycle cleans
// as well as an unbalanced erase of the same length -- but it also showed that
// chopping the same time into 5-frame halves left residue behind. Those are two
// very different worlds for a background deghost:
//
//   weak   a short excursion never drags pigment across the gap, so no number
//          of repeats converges. The atomic chunk then has a hard floor, and
//          that floor IS the interruption latency.
//   slow   each small cycle does move pigment, just less of it. Repeats then
//          accumulate and the chunk can be as small as we like, which is
//          exactly what a preemptible background polish wants.
//
// So: every band runs the SAME cycle, and only the repeat count differs. If the
// residue fades monotonically down the screen, it is slow, not weak.
//
// Repeats live in RP (group runs = RP+1) rather than in separate groups, which
// is what makes hundreds of cycles fit in a 10-group LUT at all.
void cycleTest(uint8_t half, uint16_t r1, uint16_t r2, uint16_t r3) {
  if (half == 0) half = 5;
  const uint32_t frameUs = FRAME_US[g_gray.frRet & 0x0F];
  const uint16_t want[3] = {r1, r2, r3};
  uint16_t bounds[3] = {r1, r2, r3};
  for (uint8_t i = 0; i < 3; ++i) {
    for (uint8_t j = static_cast<uint8_t>(i + 1); j < 3; ++j) {
      if (bounds[j] < bounds[i]) {
        const uint16_t t = bounds[i];
        bounds[i] = bounds[j];
        bounds[j] = t;
      }
    }
  }
  Serial.printf("=== cycles: %u up / %u down per cycle (%lu ms), repeats %u / %u / %u top->bottom ===\n", half, half,
                static_cast<unsigned long>(2u * half * frameUs / 1000), r1, r2, r3);
  plantStripes();
  bandPlanes();

  Lut lut;
  lut.clear();
  uint8_t group = 0;
  uint16_t frames = 0;
  uint16_t prev = 0;
  for (uint8_t i = 0; i < 3 && group < 10; ++i) {
    if (bounds[i] <= prev) continue;
    // Bands that have already finished their quota sit at VS_NONE and are left
    // untouched while the deeper ones keep cycling.
    uint16_t chunk = static_cast<uint16_t>(bounds[i] - prev);
    while (chunk > 0 && group < 10) {
      const uint16_t runs = chunk > 256 ? 256 : chunk;  // RP is 8 bit, so 256 runs per group
      for (uint8_t e = 1; e <= 3; ++e) {
        if (want[e - 1] <= prev) continue;
        lut.setVs(e, group, 0, VS_BLACK);
        lut.setVs(e, group, 1, VS_WHITE);
      }
      lut.setTp(group, half, half, 0, 0, static_cast<uint8_t>(runs - 1));
      frames = static_cast<uint16_t>(frames + runs * 2u * half);
      chunk = static_cast<uint16_t>(chunk - runs);
      ++group;
    }
    prev = bounds[i];
  }
  runBanded(lut, g_gray.frRet, frames, group, "cycles");
  for (uint8_t e = 0; e < 3; ++e) {
    static const char* const kWhere[3] = {"top   ", "middle", "bottom"};
    Serial.printf("%s = %u cycles = %u frames, %lu ms\n", kWhere[e], want[e], want[e] * 2u * half,
                  static_cast<unsigned long>(want[e] * 2u * half * frameUs / 1000));
  }
  Serial.println("residue fading downward means small cycles are slow but converge (chunk can be tiny)");
  Serial.println("all three equally ghosted means they are weak, and the chunk has a hard floor");
}

// Append `cycles` balanced up/down cycles for one entry, using consecutive
// groups of its own so the other entries stay at VS_NONE while it runs. RP
// carries the repetition (group runs = RP+1, so 256 cycles per group).
void appendCycles(Lut& lut, uint8_t& group, uint8_t entry, uint8_t up, uint8_t down, uint16_t cycles,
                  uint16_t& frames) {
  while (cycles > 0 && group < 10) {
    const uint16_t runs = cycles > 256 ? 256 : cycles;
    lut.setVs(entry, group, 0, VS_BLACK);
    lut.setVs(entry, group, 1, VS_WHITE);
    lut.setTp(group, up, down, 0, 0, static_cast<uint8_t>(runs - 1));
    frames = static_cast<uint16_t>(frames + runs * (up + down));
    cycles = static_cast<uint16_t>(cycles - runs);
    ++group;
  }
}

// Can a ghost be cleaned WITHOUT a visible flash?
//
// cyc settled that small balanced cycles are slow rather than weak, so chunk
// size is free -- but it also made the real constraint obvious. A 5-frame half
// swings roughly half the optical range, and repeating that is a strobe, not a
// polish. What actually has to shrink is the EXCURSION, not the chunk.
//
// The one lever physics offers is inertia. Pigment in suspension has a
// mechanical time constant of tens of milliseconds; reverse the field faster
// than that and the particles cannot follow it optically, yet the field is
// still present to break loose whatever is stuck. That is what the "activation"
// phase in vendor waveforms is really for. One frame is the shortest sub-phase
// the LUT can express and 0x08 (5 ms) is the fastest frame code the table has,
// so a 1-frame half is a 100 Hz square wave -- the smallest excursion this
// hardware can produce at all.
//
// The symmetric 1+1 version is silent and does clean, so the remaining axis is
// the up:down RATIO. Lengthening only the down phase should cost nothing in
// visibility: the pixels worth scrubbing are already at or near white, and
// pushing white against an endpoint it already occupies produces no optical
// motion. All the visible excursion comes from the up frames, so `up` stays at
// one frame and only `down` grows.
//
// That breaks charge balance, deliberately. It is affordable only because the
// scrub set is bounded: a ghost exists where something was dark and has just
// turned white, so the target is "pixels that just changed to white", not every
// white pixel. Each pixel then takes a bounded number of scrubs per transition,
// and that fixed debt can be pre-paid inside tri by lengthening the white
// class's up phase to match. Debt stays bounded instead of accumulating.
//
// Bands run at different TIMES, not together, so each one's flicker can be
// judged on its own:
//
//   top     symmetric 1+1, the known-silent baseline
//   middle  never driven -- the untouched ghost, for reference
//   bottom  up+down, same total frames, so speed is compared fairly
void ditherTest(uint8_t up, uint8_t down, uint16_t frameBudget) {
  if (up == 0) up = 1;
  if (down == 0) down = 5;
  if (frameBudget == 0) frameBudget = 1024;
  const uint32_t frameUs = FRAME_US[g_gray.frRet & 0x0F];
  const uint16_t baseCycles = static_cast<uint16_t>(frameBudget / 2u);
  const uint16_t testCycles = static_cast<uint16_t>(frameBudget / (up + down));
  Serial.printf("=== dither: top = 1+1 x %u, bottom = %u+%u x %u, both ~%u frames; middle untouched ===\n", baseCycles,
                up, down, testCycles, frameBudget);
  plantStripes();
  bandPlanes();

  Lut lut;
  lut.clear();
  uint8_t group = 0;
  uint16_t frames = 0;
  appendCycles(lut, group, 1, 1, 1, baseCycles, frames);
  const uint16_t baseFrames = frames;
  appendCycles(lut, group, 3, up, down, testCycles, frames);
  runBanded(lut, g_gray.frRet, frames, group, "dither");
  Serial.printf("top    = 1+1 x %u = %u frames, %lu ms, DC neutral\n", baseCycles, baseFrames,
                static_cast<unsigned long>(baseFrames * frameUs / 1000));
  Serial.println("middle = untouched, this is the ghost as planted");
  Serial.printf("bottom = %u+%u x %u = %u frames, %lu ms, net %+ld V*frames toward white\n", up, down, testCycles,
                static_cast<uint16_t>(frames - baseFrames),
                static_cast<unsigned long>((frames - baseFrames) * frameUs / 1000),
                -15L * static_cast<long>(testCycles) * (static_cast<long>(down) - static_cast<long>(up)));
  Serial.println("bottom cleaner at equal time, and still silent, means the imbalance is worth buying");
}

void ghostLadder(uint8_t upF, uint8_t n1, uint8_t n2, uint8_t n3, uint8_t frCode) {
  const uint32_t frameUs = FRAME_US[frCode & 0x0F];
  Serial.printf("=== ghost ladder: up=%u frames, erase = %u / %u / %u frames @ %lu us ===\n", upF, n1, n2, n3,
                static_cast<unsigned long>(frameUs));
  plantStripes();
  bandPlanes();

  Lut lut;
  lut.clear();
  uint8_t group = 0;
  uint16_t frames = 0;
  if (upF > 0) {
    for (uint8_t e = 1; e <= 3; ++e) lut.setVs(e, group, 0, VS_BLACK);
    lut.setTp(group, upF, 0, 0, 0, 0);
    frames = upF;
    ++group;
  }
  uint16_t bounds[3] = {n1, n2, n3};
  for (uint8_t i = 0; i < 3; ++i) {
    for (uint8_t j = static_cast<uint8_t>(i + 1); j < 3; ++j) {
      if (bounds[j] < bounds[i]) {
        const uint16_t t = bounds[i];
        bounds[i] = bounds[j];
        bounds[j] = t;
      }
    }
  }
  const uint16_t want[3] = {n1, n2, n3};
  uint16_t prev = 0;
  for (uint8_t i = 0; i < 3 && group < 10; ++i) {
    if (bounds[i] <= prev) continue;
    for (uint8_t e = 1; e <= 3; ++e) {
      if (want[e - 1] > prev) lut.setVs(e, group, 0, VS_WHITE);
    }
    const uint8_t chunk = static_cast<uint8_t>(bounds[i] - prev);
    lut.setTp(group, chunk, 0, 0, 0, 0);
    frames = static_cast<uint16_t>(frames + chunk);
    prev = bounds[i];
    ++group;
  }
  runBanded(lut, frCode, frames, group, "erase");
  Serial.printf("top band = %u frames (%lu ms), middle = %u (%lu ms), bottom = %u (%lu ms)\n", n1,
                static_cast<unsigned long>(n1 * frameUs / 1000), n2, static_cast<unsigned long>(n2 * frameUs / 1000),
                n3, static_cast<unsigned long>(n3 * frameUs / 1000));
  Serial.println("whichever bands still show the stripes were erased too briefly");
}

// VCOM probe. Runs a LUT that drives nothing at all — every entry sits at VSS
// for the whole run — and holds it for seconds. With the source at VSS the only
// field across the film is the VCOM offset itself, so if VCOM is mistuned the
// image visibly drifts, and the direction of the drift gives the sign of the
// error: drifts dark = VCOM too negative, drifts light = too positive. This is
// the one panel parameter we cannot derive from the datasheet, because it is a
// property of this physical film, and a mistuned VCOM produces exactly the
// symptom we are chasing: faint residue that survives a saturating reset.
void holdTest(uint8_t reps) {
  if (reps == 0) reps = 8;
  Lut lut;
  lut.clear();  // all VS = VSS
  lut.setTp(0, 255, 0, 0, 0, static_cast<uint8_t>(reps - 1));
  lut.setFrameRate(0x00);  // 15 Hz, 66400 us/frame -> 255 frames = 16.9 s per rep
  lut.setVoltages(g_gray.vgh, g_gray.vsh1, g_gray.vsh2, g_gray.vsl, g_gray.vcom);
  loadLut(lut);
  const Timing t = activateTimed(g_gray.ctrlWarm);
  Serial.printf("hold: vcom=0x%02X, %u reps x 255 frames, measured=%lu us (%lu ms)\n", g_gray.vcom, reps,
                static_cast<unsigned long>(t.busyUs), static_cast<unsigned long>(t.busyUs / 1000));
}

// Net impulse per LUT entry, in frame-units weighted by drive voltage
// (VSH1 = +15 V, VSH2 = +5 V, VSL = -15 V). Zero for every entry means the
// waveform is DC balanced and leaves no residual charge behind.
void reportBalance(const Lut& lut, const GrayParams& p) {
  Serial.print("  net impulse (V*frames): ");
  for (uint8_t entry = 0; entry < 4; ++entry) {
    long net = 0;
    for (uint8_t group = 0; group < 10; ++group) {
      const uint8_t vs = lut.b[entry * 10 + group];
      const uint8_t rp = lut.b[50 + group * 5 + 4];
      for (uint8_t phase = 0; phase < 4; ++phase) {
        const uint8_t code = (vs >> ((3 - phase) * 2)) & 0x03;
        const long frames = lut.b[50 + group * 5 + phase] * (rp + 1L);
        if (code == VS_BLACK) net += 15 * frames;
        else if (code == VS_WHITE) net -= 15 * frames;
        else if (code == VS_WEAK) net += 5 * frames;
      }
    }
    static constexpr const char* kNames[4] = {"white", "light", "dark", "black"};
    Serial.printf("%s=%+ld  ", kNames[entry], net);
  }
  Serial.printf(" [scheme %u]\n", p.scheme);
}

// The staged image is now on the glass; remember it so the next update can be
// expressed as a delta against it.
void commitDisplayed() {
  memcpy(g_prev24, g_p24, PLANE_BYTES);
  memcpy(g_prev26, g_p26, PLANE_BYTES);
}

// Build one direction of the delta. `darken` selects new-old (pixels that must
// get darker); otherwise old-new. The magnitude is 0..3 in level steps and gets
// packed straight back into the two RAM planes, so the LUT index the controller
// computes is the step count for that pixel rather than its target level.
// Returns the largest magnitude present, which lets the caller shorten the LUT
// to only the phases this particular update actually needs.
uint8_t buildDeltaPlanes(bool darken) {
  uint8_t worst = 0;
  for (uint32_t i = 0; i < PLANE_BYTES; ++i) {
    const uint8_t oldLo = g_prev24[i], oldHi = g_prev26[i];
    const uint8_t newLo = g_p24[i], newHi = g_p26[i];
    uint8_t outLo = 0, outHi = 0;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const uint8_t m = static_cast<uint8_t>(0x80u >> bit);
      const int o = ((oldHi & m) ? 2 : 0) | ((oldLo & m) ? 1 : 0);
      const int n = ((newHi & m) ? 2 : 0) | ((newLo & m) ? 1 : 0);
      int mag = darken ? (n - o) : (o - n);
      if (mag <= 0) continue;
      if (mag & 1) outLo |= m;
      if (mag & 2) outHi |= m;
      if (mag > worst) worst = static_cast<uint8_t>(mag);
    }
    g_d24[i] = outLo;
    g_d26[i] = outHi;
  }
  return worst;
}

// LUT for one delta pass: three phases of `step` frames each, entry k driving
// for the first k of them. Entry 0 is dead silent, which is the whole point --
// a pixel whose level is unchanged gets no field at all, so nothing flashes.
uint32_t buildDeltaLut(Lut& lut, uint8_t code, uint8_t step, uint8_t worst) {
  lut.clear();
  for (uint8_t e = 1; e < 4; ++e) {
    for (uint8_t phase = 0; phase < e; ++phase) lut.setVs(e, 0, phase, code);
  }
  const uint8_t p1 = worst >= 1 ? step : 0;
  const uint8_t p2 = worst >= 2 ? step : 0;
  const uint8_t p3 = worst >= 3 ? step : 0;
  lut.setTp(0, p1, p2, p3, 0, 0);
  lut.setFrameRate(g_gray.fr);
  lut.setVoltages(g_gray.vgh, g_gray.vsh1, g_gray.vsh2, g_gray.vsl, g_gray.vcom);
  return static_cast<uint32_t>(p1) + p2 + p3;
}

uint32_t runDeltaPass(bool darken, uint8_t step) {
  const uint32_t buildStart = micros();
  const uint8_t worst = buildDeltaPlanes(darken);
  const uint32_t buildUs = micros() - buildStart;
  if (worst == 0) {
    Serial.printf("  %s: nothing to do (build %lu us)\n", darken ? "darken" : "lighten",
                  static_cast<unsigned long>(buildUs));
    return buildUs;
  }
  Lut lut;
  const uint32_t frames = buildDeltaLut(lut, darken ? g_gray.modCode : VS_WHITE, step, worst);
  const uint32_t us24 = writePlane(CMD_WRITE_NEW, g_d24);
  const uint32_t us26 = writePlane(CMD_WRITE_OLD, g_d26);
  loadLut(lut);
  const Timing t = activateTimed(g_gray.ctrlWarm);
  Serial.printf("  %s: maxstep=%u frames=%lu  build=%lu us  planes=%lu us  wave=%lu us\n",
                darken ? "darken" : "lighten", worst, static_cast<unsigned long>(frames),
                static_cast<unsigned long>(buildUs), static_cast<unsigned long>(us24 + us26),
                static_cast<unsigned long>(t.busyUs));
  return buildUs + us24 + us26 + t.busyUs;
}

// Flash-free 4-gray. Two activations, because 2 RAM bits give exactly 4 pixel
// classes per activation and a 4->4 transition needs 16. Splitting by direction
// gets it down to 4 per pass: a step magnitude of 0..3. Nothing is reset to a
// rail, so nothing flashes -- and nothing corrects accumulated error either,
// which is what the periodic full refresh is for.
//
// Requires equal impulse spacing (a == b == c): the delta is expressed in level
// steps, so step k must cost the same wherever it lands, or 3->0 will not undo
// 0->3.
void deltaGray() {
  const uint8_t step = g_gray.a;
  if (g_gray.b != step || g_gray.c != step) {
    Serial.printf("WARN: delta mode needs a==b==c, have %u/%u/%u -- levels will not round-trip\n", g_gray.a, g_gray.b,
                  g_gray.c);
  }
  Serial.printf("delta gray: step=%u frames (%lu us)\n", step,
                static_cast<unsigned long>(step * FRAME_US[g_gray.fr & 0x0F]));
  cmd(CMD_BORDER);
  data(g_gray.border);
  const uint32_t total = runDeltaPass(false, step) + runDeltaPass(true, step);
  commitDisplayed();
  Serial.printf("  TOTAL update = %lu us (%lu ms)\n", static_cast<unsigned long>(total),
                static_cast<unsigned long>(total / 1000));
}

// Saturate-and-return. Two activations. Every pixel that moves is driven to a
// rail first, so its final level does not depend on where it started -- no
// accumulated error, no progressive build-up, and no dependence on how many
// refreshes have gone before.
//
//   pass 1  a *bitmap*: every gray counts as black, white stays white. Changed
//           pixels are slammed to the matching rail with one long swing;
//           unchanged pixels get no field at all, so there is no flash.
//   pass 2  pull the black ones back toward white by (3 - target) steps.
//
// Pass 1 needs only three classes, which fits in 2 bits with one to spare:
//
//   code 0  unchanged                -> no drive
//   code 1  changed, target non-white -> black rail
//   code 2  changed, target white     -> white rail
//
// Pass 2's pull-back is (3 - target) masked to the changed pixels. Target 3
// (black) needs 0 steps and target 0 (white) is masked out, so the pull-back
// never exceeds *two* steps -- pass 2 is short, which is what buys the long
// saturating pass 1 without blowing the time budget.
//
// Every plane is a whole-byte expression, no per-pixel loop:
//   changed  = (prev24 ^ p24) | (prev26 ^ p26)
//   nonwhite = p24 | p26
//   pass1:  new = changed & nonwhite      old = changed & ~nonwhite
//   pass2:  new = changed & p26 & ~p24    old = changed & p24 & ~p26
// (pass2 is just 3 - target = ~target, with the two "no pull" cases folding
// onto code 0 for free.)
void saturateReturn() {
  const uint8_t swing = g_gray.satSwing;
  const uint8_t pd = g_gray.pullDark;
  const uint8_t pl = g_gray.pullLight;
  Serial.printf("saturate+return: swing=%u fr=0x%X, pull dark=%u light=%u frRet=0x%X\n", swing, g_gray.fr, pd,
                static_cast<unsigned>(pd + pl), g_gray.frRet);
  cmd(CMD_BORDER);
  data(g_gray.border);

  const uint32_t buildStart = micros();
  bool any = false;
  for (uint32_t i = 0; i < PLANE_BYTES; ++i) {
    const uint8_t chg = static_cast<uint8_t>((g_prev24[i] ^ g_p24[i]) | (g_prev26[i] ^ g_p26[i]));
    const uint8_t nonWhite = static_cast<uint8_t>(g_p24[i] | g_p26[i]);
    g_inv24[i] = chg;                                          // keep the mask for pass 2
    g_d24[i] = static_cast<uint8_t>(chg & nonWhite);           // -> black rail
    g_d26[i] = static_cast<uint8_t>(chg & ~nonWhite);          // -> white rail
    if (chg) any = true;
  }
  const uint32_t buildUs = micros() - buildStart;
  if (!any) {
    Serial.printf("  nothing changed (build %lu us)\n", static_cast<unsigned long>(buildUs));
    return;
  }

  Lut sat;
  sat.clear();
  sat.setVs(1, 0, 0, g_gray.modCode);  // changed -> black
  sat.setVs(2, 0, 0, VS_WHITE);        // changed -> white
  sat.setTp(0, swing, 0, 0, 0, 0);
  sat.setFrameRate(g_gray.fr);
  sat.setVoltages(g_gray.vgh, g_gray.vsh1, g_gray.vsh2, g_gray.vsl, g_gray.vcom);
  const uint32_t satPlanes = writePlane(CMD_WRITE_NEW, g_d24) + writePlane(CMD_WRITE_OLD, g_d26);
  loadLut(sat);
  const Timing t1 = activateTimed(g_gray.ctrlWarm);

  // Pass 2: pull back by (3 - target) steps, masked to the pixels we just moved.
  const uint32_t build2 = micros();
  for (uint32_t i = 0; i < PLANE_BYTES; ++i) {
    const uint8_t chg = g_inv24[i];
    const uint8_t lo = g_p24[i];
    const uint8_t hi = g_p26[i];
    g_d24[i] = static_cast<uint8_t>(chg & hi & ~lo);  // level 2 -> 1 step
    g_d26[i] = static_cast<uint8_t>(chg & lo & ~hi);  // level 1 -> 2 steps
  }
  const uint32_t build2Us = micros() - build2;
  // Return pass as two repeated pulse-train groups. Group 0 runs for both grays
  // (pd pulses), group 1 only for the light one (pl more), so the two levels
  // land at pd and pd+pl pulses -- independently tunable.
  Lut ret;
  ret.clear();
  uint8_t group = 0;
  if (pd > 0) {
    for (uint8_t e = 1; e <= 2; ++e) {
      ret.setVs(e, group, 0, VS_WHITE);
      ret.setVs(e, group, 1, g_gray.pulseCode);
    }
    ret.setTp(group, g_gray.pulseOn, g_gray.pulseOff, 0, 0, static_cast<uint8_t>(pd - 1));
    ++group;
  }
  if (pl > 0) {
    ret.setVs(2, group, 0, VS_WHITE);
    ret.setVs(2, group, 1, g_gray.pulseCode);
    ret.setTp(group, g_gray.pulseOn, g_gray.pulseOff, 0, 0, static_cast<uint8_t>(pl - 1));
  }
  ret.setFrameRate(g_gray.frRet);
  ret.setVoltages(g_gray.vgh, g_gray.vsh1, g_gray.vsh2, g_gray.vsl, g_gray.vcom);
  const uint32_t retPlanes = writePlane(CMD_WRITE_NEW, g_d24) + writePlane(CMD_WRITE_OLD, g_d26);
  loadLut(ret);
  const Timing t2 = activateTimed(g_gray.ctrlWarm);
  commitDisplayed();

  // Envelope: a pixel that goes black takes +Vb*swing and no pull-back; one that
  // goes white takes -15*swing. Everything else falls between the two.
  const int32_t vBlack = (g_gray.modCode == VS_WEAK) ? 5 : 15;
  dcAccumulate(vBlack * static_cast<int32_t>(swing), -15 * static_cast<int32_t>(swing));

  const uint32_t total = buildUs + satPlanes + t1.busyUs + build2Us + retPlanes + t2.busyUs;
  Serial.printf("  saturate: planes=%lu us  wave=%lu us   return: planes=%lu us  wave=%lu us\n",
                static_cast<unsigned long>(satPlanes), static_cast<unsigned long>(t1.busyUs),
                static_cast<unsigned long>(retPlanes), static_cast<unsigned long>(t2.busyUs));
  Serial.printf("  build=%lu us + %lu us  TOTAL update = %lu us (%lu ms)\n", static_cast<unsigned long>(buildUs),
                static_cast<unsigned long>(build2Us), static_cast<unsigned long>(total),
                static_cast<unsigned long>(total / 1000));
  dcReport();
}

// Three grays in ONE activation, flash-free, start-state independent.
//
// Two RAM bits give exactly four pixel classes per activation, and three grays
// need exactly four:
//
//   code 0  unchanged   -> no field at all (this is what makes it flash-free)
//   code 1  -> white
//   code 2  -> gray
//   code 3  -> black
//
// A four-gray update would need five classes (unchanged plus four targets) and
// therefore two activations. Three fits, with nothing to spare.
//
// The trick that makes it fit is that the reset-to-white is not a class, it is
// the first *phase*: every driven code is slammed to the white rail together,
// and only afterwards do the codes diverge onto their own drive schedules. So
// the final level never depends on where the pixel started, yet unchanged
// pixels are still never touched.
//
//   G0  activation, two sub-phases, one pair of rails per class:
//         white  +15 +15    out to black; G1 brings it back
//         gray   +15 -15    out and back -- an excursion that costs no charge
//         black  -15 -15    a doubled white reset
//   G1  codes 1,2,3   VSL  -15 V     everyone lands on the white rail
//   G2+ code 2        VSH2  +5 V     develop the middle level
//       code 3        VSH1 +15 V     develop black
//
// G0 is what makes this deghost. A white-only reset gives a pixel that is
// already near white no excursion at all, so whatever residue it holds is never
// released and the next development stacks on top of it -- which is precisely
// how text edges creep darker over a handful of page turns. G0 also balances
// the charge: see the GrayParams comment for why per-class balance is the only
// arrangement that holds for arbitrary content.
//
// Gray is built with VSH2 (+5 V) on purpose. Measured on this panel, VSL takes
// the full white swing in ~10 frames -- one frame is ~10% of the range, far too
// coarse to place a level, and the gate-line RC droop across the 800 columns
// then shows up as a visible left/right shade difference. VSH2 needs ~60 frames
// for a comparable swing, giving 10+ resolvable steps and no visible droop.
//
// Planes are whole-byte expressions. Quantise the staged image to three levels
// first (q24 = non-white, q26 = black), so that a gray staged as level 1 and one
// staged as level 2 compare equal against what is on the glass:
//   chg  = (prev24 ^ q24) | (prev26 ^ q26)
//   new  = chg & ~(q24 ^ q26)      (set for white and black, clear for gray)
//   old  = chg & q24               (set for gray and black)
void triLevel(bool corrective) {
  const uint8_t satF = g_gray.satWhite;
  const uint8_t fast = g_gray.grayFast;
  const uint8_t tg = g_gray.tGray;
  const uint8_t tb = g_gray.tBlack;
  const uint32_t frameUs = FRAME_US[g_gray.frRet & 0x0F];

  cmd(CMD_BORDER);
  data(g_gray.border);

  const uint32_t buildStart = micros();
  bool any = false;
  for (uint32_t i = 0; i < PLANE_BYTES; ++i) {
    const uint8_t lo = g_p24[i];
    const uint8_t hi = g_p26[i];
    const uint8_t q24 = static_cast<uint8_t>(lo | hi);  // non-white
    const uint8_t q26 = static_cast<uint8_t>(lo & hi);  // black
    // A corrective pass drives everything: the accumulated bias sits on every
    // pixel, not only the ones this page turn happens to change.
    const uint8_t chg =
        corrective ? 0xFFu : static_cast<uint8_t>((g_prev24[i] ^ q24) | (g_prev26[i] ^ q26));
    g_d24[i] = static_cast<uint8_t>(chg & ~(q24 ^ q26));
    g_d26[i] = static_cast<uint8_t>(chg & q24);
    g_inv24[i] = q24;  // stash the quantised image; it becomes prev on commit
    g_inv26[i] = q26;
    if (chg) any = true;
  }
  const uint32_t buildUs = micros() - buildStart;
  if (!any) {
    Serial.printf("tri: nothing changed (build %lu us)\n", static_cast<unsigned long>(buildUs));
    return;
  }

  // Walk the timeline, emitting one group per slice in which every code's drive
  // voltage is constant.
  Lut lut;
  lut.clear();
  uint8_t group = 0;
  uint16_t frames = 0;
  // G0, the activation. One group, two sub-phases; the class picks the rails.
  const uint8_t upHalf = static_cast<uint8_t>(g_gray.preUp / 2);
  if (upHalf > 0) {
    lut.setVs(1, group, 0, VS_BLACK);  // white: out to black...
    lut.setVs(1, group, 1, VS_BLACK);  // ...and G1 brings it home
    lut.setVs(2, group, 0, VS_BLACK);  // gray: out and back, net zero
    lut.setVs(2, group, 1, VS_WHITE);
    lut.setVs(3, group, 0, VS_WHITE);  // black: a doubled white reset
    lut.setVs(3, group, 1, VS_WHITE);
    lut.setTp(group, upHalf, upHalf, 0, 0, 0);
    frames = static_cast<uint16_t>(frames + 2u * upHalf);
    ++group;
  }
  if (satF > 0) {
    for (uint8_t c = 1; c <= 3; ++c) lut.setVs(c, group, 0, VS_WHITE);
    lut.setTp(group, satF, 0, 0, 0, 0);
    frames += satF;
    ++group;
  }
  uint16_t bounds[3] = {fast, static_cast<uint16_t>(fast + tg), tb};
  for (uint8_t i = 0; i < 3; ++i) {
    for (uint8_t j = static_cast<uint8_t>(i + 1); j < 3; ++j) {
      if (bounds[j] < bounds[i]) {
        const uint16_t t = bounds[i];
        bounds[i] = bounds[j];
        bounds[j] = t;
      }
    }
  }
  uint16_t prev = 0;
  for (uint8_t i = 0; i < 3 && group < 10; ++i) {
    const uint16_t cur = bounds[i];
    if (cur <= prev) continue;
    const uint8_t grayVs = (prev < fast) ? VS_BLACK : ((prev < fast + tg) ? VS_WEAK : VS_NONE);
    const uint8_t blackVs = (prev < tb) ? VS_BLACK : VS_NONE;
    uint16_t len = static_cast<uint16_t>(cur - prev);
    while (len > 0 && group < 10) {
      const uint8_t chunk = static_cast<uint8_t>(len > 255 ? 255 : len);
      if (grayVs != VS_NONE) lut.setVs(2, group, 0, grayVs);
      if (blackVs != VS_NONE) lut.setVs(3, group, 0, blackVs);
      lut.setTp(group, chunk, 0, 0, 0, 0);
      frames = static_cast<uint16_t>(frames + chunk);
      len = static_cast<uint16_t>(len - chunk);
      ++group;
    }
    prev = cur;
  }
  lut.setFrameRate(g_gray.frRet);
  lut.setVoltages(g_gray.vgh, g_gray.vsh1, g_gray.vsh2, g_gray.vsl, g_gray.vcom);

  const uint32_t planes = writePlane(CMD_WRITE_NEW, g_d24) + writePlane(CMD_WRITE_OLD, g_d26);
  loadLut(lut);
  const Timing t = activateTimed(g_gray.ctrlWarm);
  memcpy(g_prev24, g_inv24, PLANE_BYTES);
  memcpy(g_prev26, g_inv26, PLANE_BYTES);

  // Per-class net over this activation. Zero for all three means no pixel can
  // drift, whatever the content does; anything else has to be paid back later.
  // G0 contributes +2h to white, 0 to gray (out and back), -2h to black.
  const int32_t h = upHalf;
  const int32_t net[3] = {
      15 * (2 * h - satF),                       // white
      15 * (fast - satF) + 5 * static_cast<int32_t>(tg),  // gray
      15 * (static_cast<int32_t>(tb) - satF - 2 * h),     // black
  };
  int32_t hi = net[0], lo = net[0];
  for (uint8_t i = 1; i < 3; ++i) {
    if (net[i] > hi) hi = net[i];
    if (net[i] < lo) lo = net[i];
  }
  dcAccumulate(hi, lo);

  const uint32_t total = buildUs + planes + t.busyUs;
  Serial.printf("tri%s: preUp=%u sat=%u gray=%u+%u black=%u @ fr 0x%X (%lu us) -> %u frames, %u groups\n",
                corrective ? "*" : "", g_gray.preUp, satF, fast, tg, tb, g_gray.frRet,
                static_cast<unsigned long>(frameUs), frames, group);
  Serial.printf("  DC net per class: white=%+ld gray=%+ld black=%+ld V*frames%s\n", static_cast<long>(net[0]),
                static_cast<long>(net[1]), static_cast<long>(net[2]),
                (net[0] || net[1] || net[2]) ? "   << NOT BALANCED" : "   (balanced)");
  Serial.printf("  build=%lu us  planes=%lu us  wave=%lu us  TOTAL = %lu us (%lu ms)\n",
                static_cast<unsigned long>(buildUs), static_cast<unsigned long>(planes),
                static_cast<unsigned long>(t.busyUs), static_cast<unsigned long>(total),
                static_cast<unsigned long>(total / 1000));
  dcReport();
  dumpLut(lut, frames, frameUs, t.busyUs);
}

uint16_t buildGtg3Lut(Lut& lut, bool includeStep, bool includeWhiteClean) {
  constexpr uint8_t LIGHTEN = 1;
  constexpr uint8_t DARKEN = 2;
  constexpr uint8_t CLEAN_WHITE = 3;
  const uint8_t cycles = g_gray.stepCycles == 0 ? 1 : g_gray.stepCycles;
  uint8_t group = 0;
  uint16_t frames = 0;
  lut.clear();

  if (includeStep) {
    lut.setVs(LIGHTEN, group, 0, VS_BLACK);
    lut.setVs(LIGHTEN, group, 1, VS_WHITE);
    lut.setVs(DARKEN, group, 0, VS_WHITE);
    lut.setVs(DARKEN, group, 1, VS_BLACK);
    lut.setTp(group, g_gray.stepAway, g_gray.stepToward, 0, 0, static_cast<uint8_t>(cycles - 1));
    frames = static_cast<uint16_t>((g_gray.stepAway + g_gray.stepToward) * cycles);
    ++group;
  }

  if (includeWhiteClean && g_gray.postClean > 0) {
    // The old maintenance waveform ended away from white. This one starts by
    // releasing the endpoint and always makes the final active frame whiteward.
    lut.setVs(LIGHTEN, group, 0, VS_BLACK);
    lut.setVs(LIGHTEN, group, 1, VS_WHITE);
    lut.setVs(CLEAN_WHITE, group, 0, VS_BLACK);
    lut.setVs(CLEAN_WHITE, group, 1, VS_WHITE);
    lut.setTp(group, 1, 1, 0, 0, static_cast<uint8_t>(g_gray.postClean - 1));
    frames = static_cast<uint16_t>(frames + 2u * g_gray.postClean);
  }

  lut.setFrameRate(g_gray.frRet);
  lut.setVoltages(g_gray.vgh, g_gray.vsh1, g_gray.vsh2, g_gray.vsl, g_gray.vcom);
  return frames;
}

// Source-aware W/G/B transition used by production. A single SSD1677
// activation has only four schedules, but an exact three-state matrix needs
// hold, +/- one step and +/- two steps. Stage 1 therefore moves every changed
// pixel one level. Stage 2 moves only W<->B pixels once more and deghosts pixels
// that reached white. The endpoint transition passes through gray briefly; it
// never resets the page to a white or black reference image.
void directThreeLevel() {
  uint32_t changed = 0;
  uint32_t distanceTwo = 0;
  uint32_t cleanPixels = 0;
  const uint32_t build1Started = micros();
  for (uint32_t i = 0; i < PLANE_BYTES; ++i) {
    const uint8_t q24 = static_cast<uint8_t>(g_p24[i] | g_p26[i]);
    const uint8_t q26 = static_cast<uint8_t>(g_p24[i] & g_p26[i]);
    const uint8_t old24 = static_cast<uint8_t>(g_prev24[i] | g_prev26[i]);
    const uint8_t old26 = static_cast<uint8_t>(g_prev24[i] & g_prev26[i]);
    const uint8_t oldWhite = static_cast<uint8_t>(~old24);
    const uint8_t oldGray = static_cast<uint8_t>(old24 & ~old26);
    const uint8_t targetWhite = static_cast<uint8_t>(~q24);
    const uint8_t darken = static_cast<uint8_t>((oldWhite & q24) | (oldGray & q26));
    const uint8_t lighten = static_cast<uint8_t>((old26 & ~q26) | (oldGray & targetWhite));

    g_d24[i] = lighten;
    g_d26[i] = darken;
    g_inv24[i] = q24;
    g_inv26[i] = q26;
    changed += static_cast<uint32_t>(__builtin_popcount(static_cast<unsigned>(lighten | darken)));
    distanceTwo += static_cast<uint32_t>(
        __builtin_popcount(static_cast<unsigned>((oldWhite & q26) | (old26 & targetWhite))));
    cleanPixels +=
        static_cast<uint32_t>(__builtin_popcount(static_cast<unsigned>(old24 & targetWhite)));
  }
  const uint32_t build1Us = micros() - build1Started;
  if (changed == 0) {
    Serial.printf("gtg3: nothing changed (build %lu us)\n", static_cast<unsigned long>(build1Us));
    return;
  }

  cmd(CMD_BORDER);
  data(g_gray.border);
  Lut first;
  const uint16_t firstFrames = buildGtg3Lut(first, true, false);
  uint32_t planesUs = writePlane(CMD_WRITE_NEW, g_d24) + writePlane(CMD_WRITE_OLD, g_d26);
  loadLut(first);
  const Timing firstTiming = activateTimed(g_controllerPowered ? g_gray.ctrlWarm : g_gray.ctrlCold);
  dumpLut(first, firstFrames, FRAME_US[g_gray.frRet & 0x0F], firstTiming.busyUs);
  if (firstTiming.assertUs == 0 || firstTiming.busyUs == 0 || firstTiming.timedOut) {
    Serial.println("ERR: gtg3 stage 1 did not complete; glass state not committed");
    return;
  }

  uint16_t secondFrames = 0;
  uint32_t build2Us = 0;
  uint32_t secondBusyUs = 0;
  const bool includeStep = distanceTwo > 0;
  const bool includeWhiteClean = cleanPixels > 0 && g_gray.postClean > 0;
  if (includeStep || includeWhiteClean) {
    const uint32_t build2Started = micros();
    for (uint32_t i = 0; i < PLANE_BYTES; ++i) {
      const uint8_t q24 = g_inv24[i];
      const uint8_t q26 = g_inv26[i];
      const uint8_t old24 = static_cast<uint8_t>(g_prev24[i] | g_prev26[i]);
      const uint8_t old26 = static_cast<uint8_t>(g_prev24[i] & g_prev26[i]);
      const uint8_t targetWhite = static_cast<uint8_t>(~q24);
      const uint8_t whiteToBlack = static_cast<uint8_t>(~old24 & q26);
      const uint8_t blackToWhite = static_cast<uint8_t>(old26 & targetWhite);
      const uint8_t grayToWhite = includeWhiteClean
                                      ? static_cast<uint8_t>(old24 & ~old26 & targetWhite)
                                      : 0u;
      g_d24[i] = static_cast<uint8_t>(blackToWhite | grayToWhite);
      g_d26[i] = static_cast<uint8_t>(whiteToBlack | grayToWhite);
    }
    build2Us = micros() - build2Started;

    Lut second;
    secondFrames = buildGtg3Lut(second, includeStep, includeWhiteClean);
    planesUs += writePlane(CMD_WRITE_NEW, g_d24) + writePlane(CMD_WRITE_OLD, g_d26);
    loadLut(second);
    const Timing secondTiming = activateTimed(g_gray.ctrlWarm);
    secondBusyUs = secondTiming.busyUs;
    dumpLut(second, secondFrames, FRAME_US[g_gray.frRet & 0x0F], secondTiming.busyUs);
    if (secondTiming.assertUs == 0 || secondTiming.busyUs == 0 || secondTiming.timedOut) {
      Serial.println("ERR: gtg3 stage 2 did not complete; glass state not committed");
      return;
    }
  }

  memcpy(g_prev24, g_inv24, PLANE_BYTES);
  memcpy(g_prev26, g_inv26, PLANE_BYTES);
  const long q = 15L * (static_cast<long>(g_gray.stepToward) - g_gray.stepAway) *
                 (g_gray.stepCycles == 0 ? 1 : g_gray.stepCycles);
  Serial.printf("gtg3: changed=%lu long=%lu white-clean=%lu Q=%+ld V*frames/level; frames=%u+%u\n",
                static_cast<unsigned long>(changed), static_cast<unsigned long>(distanceTwo),
                static_cast<unsigned long>(cleanPixels), q, firstFrames, secondFrames);
  Serial.printf("  build=%lu+%lu us planes=%lu us waves=%lu+%lu us total=%lu ms\n",
                static_cast<unsigned long>(build1Us), static_cast<unsigned long>(build2Us),
                static_cast<unsigned long>(planesUs), static_cast<unsigned long>(firstTiming.busyUs),
                static_cast<unsigned long>(secondBusyUs),
                static_cast<unsigned long>((build1Us + build2Us + planesUs + firstTiming.busyUs + secondBusyUs) /
                                           1000));
}

// Same source history and completed W/G/B->white transition everywhere, then a
// third activation applies post-clean only to the center third. The final glass
// is therefore A | B | A: side thirds have no cleanup, center has the candidate
// cleanup. This is deliberately an optical test; timings alone cannot choose B.
void cleanupAbCompare(uint8_t loops) {
  if (loops == 0) loops = 3;
  const uint8_t savedClean = g_gray.postClean;
  const bool savedDump = g_lutDump;
  g_lutDump = false;
  g_gray.postClean = 0;

  deepClear(2);
  for (uint8_t i = 0; i < loops; ++i) {
    drawCleanCompareSource();
    directThreeLevel();
    fillLevel(0);
    directThreeLevel();
  }

  g_gray.postClean = savedClean;
  g_lutDump = savedDump;
  if (savedClean == 0) {
    Serial.println("abclean: postClean is 0; final screen is all A (no cleanup)");
    return;
  }

  memset(g_d24, 0x00, PLANE_BYTES);
  memset(g_d26, 0x00, PLANE_BYTES);
  const uint16_t firstByte = ROW_BYTES / 3;
  const uint16_t afterLastByte = static_cast<uint16_t>((ROW_BYTES * 2) / 3);
  for (uint32_t y = 0; y < PANEL_H; ++y) {
    memset(g_d24 + y * ROW_BYTES + firstByte, 0xFF, afterLastByte - firstByte);
    memset(g_d26 + y * ROW_BYTES + firstByte, 0xFF, afterLastByte - firstByte);
  }

  Lut clean;
  const uint16_t frames = buildGtg3Lut(clean, false, true);
  cmd(CMD_BORDER);
  data(g_gray.border);
  const uint32_t planesUs = writePlane(CMD_WRITE_NEW, g_d24) + writePlane(CMD_WRITE_OLD, g_d26);
  loadLut(clean);
  const Timing timing = activateTimed(g_controllerPowered ? g_gray.ctrlWarm : g_gray.ctrlCold);
  dumpLut(clean, frames, FRAME_US[g_gray.frRet & 0x0F], timing.busyUs);
  Serial.printf("abclean: %u identical loops; A=side thirds/no clean, B=center third/%u cycles\n", loops,
                savedClean);
  Serial.printf("  cleanup=%u frames planes=%lu us wave=%lu us%s\n", frames,
                static_cast<unsigned long>(planesUs), static_cast<unsigned long>(timing.busyUs),
                (timing.assertUs == 0 || timing.busyUs == 0 || timing.timedOut) ? "  ERROR" : "");
  Serial.println("  Please judge the physical screen: center(B) vs both sides(A).");
}

// ── Page slots: real text pages, uploaded from the host ─────────────────────
//
// Solid bands say nothing about how the waveform handles antialiased glyph
// edges, which is the whole point of having grays on a reader. Rasterising text
// on the controller would mean pulling the font decompressor into the lab, so
// the host renders the page instead and ships the two bit planes over CDC. That
// also means any content -- Latin, CJK, a real book page -- can be tested
// without reflashing.
//
// Wire format: PLANE_BYTES of the 0x24 plane, then PLANE_BYTES of the 0x26
// plane, raw, no framing. Level = (p26 bit << 1) | (p24 bit), same packing the
// rest of the lab uses.
//
// The transfer is chunked and pulled, not streamed. USB CDC has no application
// flow control and the Arduino RX ring silently DISCARDS whatever does not fit,
// so a free-running host outruns this loop and the tail of the image is lost
// (measured: ~36 KB of 96 KB arrived). The device therefore acknowledges each
// chunk with a single 'K' and the host waits for it, which caps the data in
// flight at one chunk and makes the ring size irrelevant.
constexpr uint32_t PAGE_CHUNK = 2048;

void receivePage(uint8_t slot) {
  uint8_t* dst24 = slot ? g_pgB24 : g_pgA24;
  uint8_t* dst26 = slot ? g_pgB26 : g_pgA26;
  Serial.printf("send %lu raw bytes in %lu-byte chunks: plane24 then plane26\n",
                static_cast<unsigned long>(2 * PLANE_BYTES), static_cast<unsigned long>(PAGE_CHUNK));
  Serial.flush();

  const uint32_t started = millis();
  uint32_t got = 0;
  while (got < 2 * PLANE_BYTES) {
    uint8_t* dst = (got < PLANE_BYTES) ? (dst24 + got) : (dst26 + (got - PLANE_BYTES));
    const uint32_t room = (got < PLANE_BYTES) ? (PLANE_BYTES - got) : (2 * PLANE_BYTES - got);
    const uint32_t want = room < PAGE_CHUNK ? room : PAGE_CHUNK;
    const size_t read = Serial.readBytes(dst, want);
    got += read;
    if (read < want) break;  // readBytes only returns short on timeout
    Serial.write('K');
    Serial.flush();
  }

  if (got < 2 * PLANE_BYTES) {
    g_pgLoaded[slot] = false;
    Serial.printf("page %c: TRUNCATED, got %lu of %lu bytes\n", slot ? 'B' : 'A', static_cast<unsigned long>(got),
                  static_cast<unsigned long>(2 * PLANE_BYTES));
    return;
  }
  g_pgLoaded[slot] = true;
  Serial.printf("page %c loaded: %lu bytes in %lu ms\n", slot ? 'B' : 'A', static_cast<unsigned long>(got),
                static_cast<unsigned long>(millis() - started));
}

// Stage a slot and push it with the production source-aware 3-gray waveform.
bool showPage(uint8_t slot) {
  if (!g_pgLoaded[slot]) {
    Serial.printf("page %c not loaded; upload it first\n", slot ? 'B' : 'A');
    return false;
  }
  memcpy(g_p24, slot ? g_pgB24 : g_pgA24, PLANE_BYTES);
  memcpy(g_p26, slot ? g_pgB26 : g_pgA26, PLANE_BYTES);
  g_abPage = slot;
  ++g_triCount;
  Serial.printf("-- page %c (gtg3) --\n", slot ? 'B' : 'A');
  directThreeLevel();
  return true;
}

// Deghost, then hand the panel over to the buttons. Starting from a deep clear
// matters: without it the first page turn is measured against whatever the
// previous experiment left in the pigment, and every later frame inherits that.
void abStart(uint8_t clearCycles) {
  if (!g_pgLoaded[0] || !g_pgLoaded[1]) {
    Serial.println("need both pages; upload A and B first");
    return;
  }
  Serial.printf("deghosting with %u clear cycles...\n", clearCycles);
  deepClear(clearCycles);
  showPage(0);
  g_abMode = true;
  Serial.println("A/B mode: DOWN(GPIO3) = turn page, UP(GPIO2) = deep clear + redraw");
  Serial.println("any serial command exits A/B mode");
}

// Edge-detect the two nav keys. Called from loop() only while g_abMode is set.
void pollAbButtons() {
  static bool upWas = false;
  static bool downWas = false;
  static uint32_t lastEvent = 0;

  const bool up = digitalRead(PIN_BTN_UP) == LOW;
  const bool down = digitalRead(PIN_BTN_DOWN) == LOW;
  const uint32_t now = millis();
  const bool armed = now - lastEvent > 250;

  if (down && !downWas && armed) {
    lastEvent = now;
    showPage(g_abPage ^ 1u);
    Serial.println("ready");
  } else if (up && !upWas && armed) {
    lastEvent = now;
    // The reference render: same page, but preceded by a full deep clear, so the
    // user can flip between "after a page turn" and "from clean" and see exactly
    // what the single activation left behind.
    Serial.println("-- deep clear + redraw (reference) --");
    const uint8_t page = g_abPage;
    deepClear(2);
    showPage(page);
    Serial.println("ready");
  }
  upWas = up;
  downWas = down;
}

// Measure the panel's optical response to white drive time, in one shot.
//
// Saturate the whole screen to black, then apply `bands - 1` successive white
// pulses of `stepFrames` each, masking every pulse to the rows below the last
// one. Band k therefore ends up with exactly k * stepFrames frames of white
// drive, so the screen becomes a calibrated black-to-white ramp: the panel's
// transfer curve, read directly off the glass.
//
// This is what tells us where to put the two gray levels. Guessing pullDark /
// pullLight one flash cycle at a time is hopeless when the curve is this
// non-linear; here we get every point at once, and each band spans the full
// width so the left/right gate-line droop is visible in the same picture.
void rampTest(uint8_t stepFrames, uint8_t bands, uint8_t dir) {
  if (stepFrames == 0) stepFrames = 2;
  if (bands < 2) bands = 16;
  if (bands > 32) bands = 32;
  const uint16_t rowsPerBand = static_cast<uint16_t>(PANEL_H / bands);
  // dir 0: start black, walk toward white. dir 1: start white, walk toward
  // black with rampCode -- the direction where VSH2 (+5 V) is available, so the
  // walk can be slow and finely divided without needing an opposing pulse.
  const uint8_t satCode = dir ? VS_WHITE : g_gray.modCode;
  const uint8_t driveCode = dir ? g_gray.rampCode : VS_WHITE;

  cmd(CMD_BORDER);
  data(g_gray.border);

  // Saturate everything: both planes 0xFF -> index 3 for every pixel.
  memset(g_d24, 0xFF, PLANE_BYTES);
  Lut sat;
  sat.clear();
  sat.setVs(3, 0, 0, satCode);
  sat.setTp(0, g_gray.satSwing, 0, 0, 0, 0);
  sat.setFrameRate(g_gray.fr);
  sat.setVoltages(g_gray.vgh, g_gray.vsh1, g_gray.vsh2, g_gray.vsl, g_gray.vcom);
  writePlane(CMD_WRITE_NEW, g_d24);
  writePlane(CMD_WRITE_OLD, g_d24);
  loadLut(sat);
  const Timing t0 = activateTimed(g_gray.ctrlWarm);

  // One pulse train per step, each masked to the rows at or below band i.
  Lut pulse;
  pulse.clear();
  pulse.setVs(3, 0, 0, driveCode);
  if (g_gray.pulseOff > 0) pulse.setVs(3, 0, 1, g_gray.pulseCode);
  pulse.setTp(0, g_gray.pulseOn, g_gray.pulseOff, 0, 0, static_cast<uint8_t>(stepFrames - 1));
  pulse.setFrameRate(g_gray.frRet);
  pulse.setVoltages(g_gray.vgh, g_gray.vsh1, g_gray.vsh2, g_gray.vsl, g_gray.vcom);

  uint32_t pulseUs = 0;
  uint32_t planeUs = 0;
  for (uint8_t i = 1; i < bands; ++i) {
    const uint32_t from = static_cast<uint32_t>(i) * rowsPerBand * ROW_BYTES;
    memset(g_d24, 0x00, from);
    memset(g_d24 + from, 0xFF, PLANE_BYTES - from);
    planeUs += writePlane(CMD_WRITE_NEW, g_d24) + writePlane(CMD_WRITE_OLD, g_d24);
    loadLut(pulse);
    pulseUs += activateTimed(g_gray.ctrlWarm).busyUs;
  }

  const uint32_t frameUs = FRAME_US[g_gray.frRet & 0x0F];
  const uint8_t perPulse = static_cast<uint8_t>(g_gray.pulseOn + g_gray.pulseOff);
  Serial.printf("ramp dir=%u (%s): %u bands x %u rows, %u pulse(s)/step, pulse=%u(code %u)+%u(code %u) @ fr 0x%X "
                "(%lu us/frame)\n",
                dir, dir ? "white->black" : "black->white", bands, rowsPerBand, stepFrames, g_gray.pulseOn, driveCode,
                g_gray.pulseOff, g_gray.pulseCode, g_gray.frRet, static_cast<unsigned long>(frameUs));
  Serial.println("band(top=0)  pulses  drive_frames  elapsed_us");
  for (uint8_t i = 0; i < bands; ++i) {
    const uint32_t p = static_cast<uint32_t>(i) * stepFrames;
    Serial.printf("  %2u         %4lu     %4lu          %6lu\n", i, static_cast<unsigned long>(p),
                  static_cast<unsigned long>(p * g_gray.pulseOn), static_cast<unsigned long>(p * perPulse * frameUs));
  }
  Serial.printf("saturate=%lu us  pulses=%lu us  planes=%lu us  TOTAL=%lu ms\n",
                static_cast<unsigned long>(t0.busyUs), static_cast<unsigned long>(pulseUs),
                static_cast<unsigned long>(planeUs),
                static_cast<unsigned long>((t0.busyUs + pulseUs + planeUs) / 1000));
  Serial.println("(screen is a calibration ramp, not a valid image -- run 'clear' before 'sat')");
}

// Flash-free B/W. Old image in 0x26, new image in 0x24, so the LUT index the
// controller computes is literally (old << 1) | new -- the controller's own
// OLD/NEW semantics, which is what those two RAM planes were designed for. No
// per-pixel work at all, and unchanged pixels (indices 0 and 3) get no drive.
void bwTransition() {
  Lut lut;
  lut.clear();
  const uint8_t swing = static_cast<uint8_t>(g_gray.a + g_gray.b + g_gray.c);
  lut.setVs(1, 0, 0, g_gray.modCode);  // old white, new black
  lut.setVs(2, 0, 0, VS_WHITE);        // old black, new white
  lut.setTp(0, swing, 0, 0, 0, 0);
  lut.setFrameRate(g_gray.fr);
  lut.setVoltages(g_gray.vgh, g_gray.vsh1, g_gray.vsh2, g_gray.vsl, g_gray.vcom);

  cmd(CMD_BORDER);
  data(g_gray.border);
  const uint32_t us24 = writePlane(CMD_WRITE_NEW, g_p26);    // new
  const uint32_t us26 = writePlane(CMD_WRITE_OLD, g_prev26);  // old
  loadLut(lut);
  const Timing t = activateTimed(g_gray.ctrlWarm);
  commitDisplayed();
  Serial.printf("bw transition: swing=%u frames  planes=%lu us  wave=%lu us  TOTAL=%lu us (%lu ms)\n", swing,
                static_cast<unsigned long>(us24 + us26), static_cast<unsigned long>(t.busyUs),
                static_cast<unsigned long>(us24 + us26 + t.busyUs),
                static_cast<unsigned long>((us24 + us26 + t.busyUs) / 1000));
}

void runGray(bool warm) {
  Lut lut;
  const uint32_t frames = buildGrayLut(lut, g_gray);
  const uint32_t predictedUs = frames * FRAME_US[g_gray.fr & 0x0F];

  if (!warm) initController();
  cmd(CMD_BORDER);
  data(g_gray.border);
  const uint32_t us24 = writePlane(CMD_WRITE_NEW, g_p24);
  const uint32_t us26 = writePlane(CMD_WRITE_OLD, g_p26);
  loadLut(lut);
  cmd(CMD_UPDATE_CTRL1);
  data(0x00);
  const uint8_t control = warm ? g_gray.ctrlWarm : g_gray.ctrlCold;
  const Timing t = activateTimed(control);

  Serial.printf("gray %s ctrl=0x%02X fr=0x%X (%lu us/frame)\n", warm ? "warm" : "cold", control, g_gray.fr,
                static_cast<unsigned long>(FRAME_US[g_gray.fr & 0x0F]));
  Serial.printf("  plan: shake=%ux2x%u black=%u white=%u mod=%u/%u/%u code=%u  frames=%lu\n", g_gray.shake,
                g_gray.shakeRep + 1, g_gray.toBlack, g_gray.toWhite, g_gray.a, g_gray.b, g_gray.c, g_gray.modCode,
                static_cast<unsigned long>(frames));
  reportBalance(lut, g_gray);
  reportPlanes(us24, us26);
  Serial.printf("  predicted waveform=%lu us  measured=%lu us  delta=%ld us\n",
                static_cast<unsigned long>(predictedUs), static_cast<unsigned long>(t.busyUs),
                static_cast<long>(t.busyUs) - static_cast<long>(predictedUs));
  Serial.printf("  TOTAL refresh = %lu us (%lu ms)\n", static_cast<unsigned long>(us24 + us26 + t.busyUs),
                static_cast<unsigned long>((us24 + us26 + t.busyUs) / 1000));
  commitDisplayed();
}

// Back-to-back refreshes with no host round trip, alternating an image with its
// tonal inverse. Reports the real achievable cadence and, crucially, how much
// of each cycle is *not* the waveform — that gap is what "time between
// refreshes" actually costs.
void burst(uint8_t count) {
  Lut lut;
  const uint32_t frames = buildGrayLut(lut, g_gray);
  Serial.printf("=== burst x%u (waveform %lu frames) ===\n", count,
                static_cast<unsigned long>(frames));
  for (uint32_t i = 0; i < PLANE_BYTES; ++i) {
    g_inv24[i] = static_cast<uint8_t>(~g_p24[i]);
    g_inv26[i] = static_cast<uint8_t>(~g_p26[i]);
  }
  cmd(CMD_BORDER);
  data(g_gray.border);
  loadLut(lut);

  uint32_t sumCycle = 0, sumGap = 0;
  for (uint8_t i = 0; i < count; ++i) {
    const uint32_t cycleStart = micros();
    const bool inverted = (i & 1) != 0;
    writePlane(CMD_WRITE_NEW, inverted ? g_inv24 : g_p24);
    writePlane(CMD_WRITE_OLD, inverted ? g_inv26 : g_p26);
    const uint32_t waveStart = micros();
    const Timing t = activateTimed(g_gray.ctrlWarm);
    const uint32_t cycle = micros() - cycleStart;
    const uint32_t gap = (waveStart - cycleStart) + (cycle - (waveStart - cycleStart) - t.busyUs);
    sumCycle += cycle;
    sumGap += gap;
    Serial.printf("  %u: cycle=%lu us  waveform=%lu us  overhead=%lu us\n", i,
                  static_cast<unsigned long>(cycle), static_cast<unsigned long>(t.busyUs),
                  static_cast<unsigned long>(gap));
  }
  Serial.printf("  avg cycle=%lu us  avg overhead=%lu us  -> %lu refresh/s\n",
                static_cast<unsigned long>(sumCycle / count), static_cast<unsigned long>(sumGap / count),
                static_cast<unsigned long>(1000000UL / (sumCycle / count)));
  Serial.println("=== burst done ===");
}

void printGrayParams() {
  Serial.printf("scheme=%u fr=0x%X shake=%u shakeRep=%u toBlack=%u toWhite=%u a=%u b=%u c=%u mod=%u\n", g_gray.scheme,
                g_gray.fr, g_gray.shake, g_gray.shakeRep, g_gray.toBlack, g_gray.toWhite, g_gray.a, g_gray.b, g_gray.c,
                g_gray.modCode);
  Serial.printf("border=0x%02X ctrlCold=0x%02X ctrlWarm=0x%02X vgh=0x%02X vsh1=0x%02X vsh2=0x%02X vsl=0x%02X vcom=0x%02X\n",
                g_gray.border, g_gray.ctrlCold, g_gray.ctrlWarm, g_gray.vgh, g_gray.vsh1, g_gray.vsh2, g_gray.vsl,
                g_gray.vcom);
  Serial.printf("tri: preUp=%u satWhite=%u grayFast=%u tGray=%u tBlack=%u frRet=0x%X dcEvery=%u lutdump=%u\n",
                g_gray.preUp, g_gray.satWhite, g_gray.grayFast, g_gray.tGray, g_gray.tBlack, g_gray.frRet,
                g_gray.dcEvery, g_lutDump ? 1 : 0);
  Serial.printf("     balanced when preUp==satWhite, tBlack==2*satWhite, tGray==3*satWhite\n");
  Serial.printf("gtg3: away=%u toward=%u cycles=%u postClean=%u (Q=%ld nominal V*frames per level)\n",
                g_gray.stepAway, g_gray.stepToward, g_gray.stepCycles, g_gray.postClean,
                15L * (static_cast<long>(g_gray.stepToward) - g_gray.stepAway) * g_gray.stepCycles);
}

bool setGrayParam(const String& key, long value) {
  const uint8_t v = static_cast<uint8_t>(value);
  if (key == "fr") g_gray.fr = static_cast<uint8_t>(value & 0x0F);
  else if (key == "shake") g_gray.shake = v;
  else if (key == "shakerep") g_gray.shakeRep = v;
  else if (key == "toblack") g_gray.toBlack = v;
  else if (key == "towhite") g_gray.toWhite = v;
  else if (key == "a") g_gray.a = v;
  else if (key == "b") g_gray.b = v;
  else if (key == "c") g_gray.c = v;
  else if (key == "mod") g_gray.modCode = static_cast<uint8_t>(value & 0x03);
  else if (key == "scheme") g_gray.scheme = v;
  else if (key == "border") g_gray.border = v;
  else if (key == "ctrlcold") g_gray.ctrlCold = v;
  else if (key == "ctrlwarm") g_gray.ctrlWarm = v;
  else if (key == "vgh") g_gray.vgh = v;
  else if (key == "vsh1") g_gray.vsh1 = v;
  else if (key == "vsh2") g_gray.vsh2 = v;
  else if (key == "vsl") g_gray.vsl = v;
  else if (key == "vcom") g_gray.vcom = v;
  else if (key == "dclimit") g_gray.dcLimit = v;
  else if (key == "swing") g_gray.satSwing = v;
  else if (key == "pulldark") g_gray.pullDark = v;
  else if (key == "pulllight") g_gray.pullLight = v;
  else if (key == "frret") g_gray.frRet = static_cast<uint8_t>(value & 0x0F);
  else if (key == "pulseon") g_gray.pulseOn = v;
  else if (key == "pulseoff") g_gray.pulseOff = v;
  else if (key == "pulsecode") g_gray.pulseCode = static_cast<uint8_t>(value & 0x03);
  else if (key == "rampcode") g_gray.rampCode = static_cast<uint8_t>(value & 0x03);
  else if (key == "preup") g_gray.preUp = v;
  else if (key == "satwhite") g_gray.satWhite = v;
  else if (key == "grayfast") g_gray.grayFast = v;
  else if (key == "tgray") g_gray.tGray = v;
  else if (key == "tblack") g_gray.tBlack = v;
  else if (key == "dcevery") g_gray.dcEvery = v;
  else if (key == "stepaway") g_gray.stepAway = v;
  else if (key == "steptoward") g_gray.stepToward = v;
  else if (key == "stepcycles") g_gray.stepCycles = v;
  else if (key == "postclean") g_gray.postClean = v;
  else if (key == "lutdump") g_lutDump = v != 0;
  // One knob for the whole balanced family: everything else follows from S.
  else if (key == "tri") {
    g_gray.preUp = v;
    g_gray.satWhite = v;
    g_gray.tBlack = static_cast<uint8_t>(2 * v);
    g_gray.tGray = static_cast<uint8_t>(3 * v);
    g_gray.grayFast = 0;
  } else return false;
  return true;
}

// ── Experiments ──────────────────────────────────────────────────────────────

// Sweep the fake-temperature register to enumerate the panel's OTP waveform
// banks. Distinct durations = distinct banks. Runs against a solid mid pattern
// so every bank has work to do; the screen content is meaningless here, the
// timing table is the output.
void sweepOtpBanks(uint8_t from, uint8_t to, uint8_t step) {
  Serial.println("=== OTP bank sweep (cmd 0x1A + 0x22=0xD7) ===");
  Serial.println("temp,assert_us,waveform_us");
  drawTestCard();
  initController();
  for (uint16_t t = from; t <= to; t = static_cast<uint16_t>(t + step)) {
    cmd(CMD_TEMP_WRITE);
    data(static_cast<uint8_t>(t));
    writePlane(CMD_WRITE_NEW, g_p24);
    writePlane(CMD_WRITE_OLD, g_p26);
    const Timing timing = activateTimed(0xD7);
    Serial.printf("0x%02X,%lu,%lu\n", static_cast<unsigned>(t), static_cast<unsigned long>(timing.assertUs),
                  static_cast<unsigned long>(timing.busyUs));
    if (step == 0) break;
  }
  Serial.println("=== sweep done ===");
}

// Time a custom LUT that drives exactly `frames` frames, for every frame-rate
// code 0..15. waveform_us / frames = the real frame period for that code, which
// is what lets us author waveforms in milliseconds instead of by trial.
void probeFrameRates(uint8_t frames) {
  Serial.printf("=== frame-rate probe (%u frames, single phase) ===\n", frames);
  Serial.println("fr_code,waveform_us,us_per_frame,hz");
  fillLevel(0);
  initController();
  for (uint8_t code = 0; code < 16; ++code) {
    Lut lut;
    lut.clear();
    // Every LUT index drives VSL (toward white) for `frames` frames in group 0
    // phase A. Content-independent, so the measured time is pure waveform.
    for (uint8_t entry = 0; entry < 4; ++entry) lut.setVs(entry, 0, 0, 0x02);
    lut.setTp(0, frames, 0, 0, 0, 0);
    lut.setFrameRate(code);
    lut.setVoltages(0x17, 0x41, 0xA8, 0x32, 0x30);

    cmd(CMD_BORDER);
    data(0x80);
    writePlane(CMD_WRITE_NEW, g_p24);
    writePlane(CMD_WRITE_OLD, g_p26);
    loadLut(lut);
    const Timing t = activateTimed(0xC7);
    const uint32_t perFrame = frames ? t.busyUs / frames : 0;
    Serial.printf("0x%X,%lu,%lu,%lu\n", code, static_cast<unsigned long>(t.busyUs),
                  static_cast<unsigned long>(perFrame), static_cast<unsigned long>(perFrame ? 1000000UL / perFrame : 0));
  }
  Serial.println("=== probe done ===");
}

// Confirm the fixed overhead per activation: same LUT, zero drive frames.
void probeActivationOverhead() {
  Serial.println("=== activation overhead (0 drive frames) ===");
  Lut lut;
  lut.clear();
  lut.setTp(0, 1, 0, 0, 0, 0);
  lut.setFrameRate(0x02);
  lut.setVoltages(0x17, 0x41, 0xA8, 0x32, 0x30);
  initController();
  loadLut(lut);
  for (uint8_t i = 0; i < 5; ++i) {
    const Timing t = activateTimed(i == 0 ? 0xCC : 0x0C);
    Serial.printf("  pass %u ctrl=0x%02X assert=%lu us busy=%lu us\n", i, i == 0 ? 0xCC : 0x0C,
                  static_cast<unsigned long>(t.assertUs), static_cast<unsigned long>(t.busyUs));
  }
  Serial.println("=== overhead done ===");
}

// SPI throughput at several clocks, so the plane-write cost is a measured
// number rather than an assumption.
void benchSpi() {
  Serial.println("=== SPI plane write bench ===");
  static constexpr uint32_t kClocks[] = {10000000, 20000000, 26666666, 40000000, 53333333, 80000000};
  const uint32_t saved = g_spiHz;
  for (uint32_t hz : kClocks) {
    setSpiHz(hz);
    SPI.end();
    SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);
    pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_DC, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    uint32_t best = 0xFFFFFFFF;
    for (uint8_t i = 0; i < 3; ++i) {
      const uint32_t us = writePlane(CMD_WRITE_NEW, g_p24);
      if (us < best) best = us;
    }
    Serial.printf("  %lu MHz: %lu us/plane (%lu KB/s)\n", static_cast<unsigned long>(hz / 1000000),
                  static_cast<unsigned long>(best), static_cast<unsigned long>((PLANE_BYTES * 1000ULL) / best));
  }
  setSpiHz(saved);
  SPI.end();
  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_DC, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  Serial.println("=== bench done ===");
}

// ── Command shell ────────────────────────────────────────────────────────────
void printHelp() {
  Serial.println();
  Serial.println("Paper Mono panel lab");
  Serial.println("  h            help");
  Serial.println("  card         draw test card into host planes");
  Serial.println("  steps        draw 4-level staircase into host planes");
  Serial.println("  solid <0-3>  fill host planes with one level");
  Serial.println("  init         hard reset + controller init");
  Serial.println("  g4 [temp]    OTP 4-gray refresh (default temp 0x5A, trigger 0xD7)");
  Serial.println("  g4w [temp]   same, no re-init (warm path)");
  Serial.println("  full         OTP full refresh (0xF7)");
  Serial.println("  fast         OTP fast refresh (0x1A=0x6A, 0xD7)");
  Serial.println("  part         OTP partial refresh (0xFF)");
  Serial.println("  ctrl <hex>   arbitrary 0x22 trigger with current planes");
  Serial.println("  sweep [a b s] enumerate OTP banks by fake temperature");
  Serial.println("  fr [frames]  frame-rate code probe (default 20 frames)");
  Serial.println("  ovh          per-activation fixed overhead");
  Serial.println("  spi          SPI plane-write bench across clocks");
  Serial.println("  hz <mhz>     set SPI clock");
  Serial.println("  gw           run custom 4-gray waveform (cold, powers up)");
  Serial.println("  gww          run custom 4-gray waveform (warm, keeps power)");
  Serial.println("  set <k> <v>  tune a waveform parameter");
  Serial.println("  params       print waveform parameters");
  Serial.println("  off          power the controller down");
  Serial.println("  burst <n>    n back-to-back refreshes, report real cadence");
  Serial.println("  clear [n]    deep erase to white (n full swings) before a ghost test");
  Serial.println("  hold [n]     drive nothing for n x 17 s -- shows VCOM drift");
  Serial.println("  dg           flash-free 4->4 gray delta update (2 activations)");
  Serial.println("  bw           flash-free B/W transition update (1 activation)");
  Serial.println("  sat          saturate changed pixels then pull back (2 activations)");
  Serial.println("  tri          3 grays in ONE activation, DC-balanced per class");
  Serial.println("  tri*         same, but drives every pixel (the periodic scrub)");
  Serial.println("  gtg3         source-aware 3->3 update; fast two-step endpoints + white deghost");
  Serial.println("  abclean [n]  optical A|B|A: sides no-clean, center post-clean after n loops");
  Serial.println("  set stepaway|steptoward|stepcycles|postclean <n>  tune gtg3");
  Serial.println("  set tri <S>  balanced family: preUp=sat=S, tBlack=2S, tGray=3S");
  Serial.println("  set lutdump 1  print the built LUT and predicted-vs-measured time");
  Serial.println("  steps3       draw 3-level staircase into host planes");
  Serial.println("  ramp [s b d] optical response ramp: b bands, s frames apart, dir d");
  Serial.println("  ghost <up> <a> <b> <c> [fr]  erase-length ladder, 3 bands top->bottom");
  Serial.println("  polish [half] [groups]       does a CHARGE-BALANCED cycle clean a ghost?");
  Serial.println("  cyc <half> <r1> <r2> <r3>    same small cycle, 3 repeat counts: weak or just slow?");
  Serial.println("  dith <up> <down> <frames>    dither ratio vs the silent 1+1 baseline, equal time");
  Serial.println("  dc           print the accumulated DC envelope");
  Serial.println("  img <a|b>    receive a 96000-byte page image into slot A or B");
  Serial.println("  page <a|b>   show slot A or B with 'tri'");
  Serial.println("  flip         show the other slot");
  Serial.println("  ab [n]       deep clear (n cycles), show A, then buttons flip pages");
  Serial.println();
}

String tokenAt(const String& line, int index) {
  int start = 0;
  for (int i = 0; i <= index; ++i) {
    start = line.indexOf(' ', start);
    if (start < 0) return String();
    ++start;
  }
  const int end = line.indexOf(' ', start);
  return end < 0 ? line.substring(start) : line.substring(start, end);
}

long argAt(const String& line, int index, long fallback) {
  int start = 0;
  for (int i = 0; i <= index; ++i) {
    start = line.indexOf(' ', start);
    if (start < 0) return fallback;
    ++start;
  }
  const String token = line.substring(start);
  if (token.length() == 0) return fallback;
  if (token.startsWith("0x") || token.startsWith("0X")) return strtol(token.c_str() + 2, nullptr, 16);
  return strtol(token.c_str(), nullptr, 10);
}

void handle(const String& line) {
  if (line.length() == 0) return;
  Serial.printf("> %s\n", line.c_str());
  if (g_abMode) {
    g_abMode = false;
    Serial.println("(left A/B mode)");
  }

  if (line == "h" || line == "help") {
    printHelp();
  } else if (line == "card") {
    drawTestCard();
    Serial.println("test card staged");
  } else if (line == "steps") {
    drawFourLevelSteps();
    Serial.println("staircase staged");
  } else if (line.startsWith("solid")) {
    fillLevel(static_cast<uint8_t>(argAt(line, 0, 0) & 0x03));
    Serial.println("solid staged");
  } else if (line == "init") {
    const uint32_t started = micros();
    initController();
    Serial.printf("init done in %lu us\n", static_cast<unsigned long>(micros() - started));
  } else if (line.startsWith("g4w")) {
    otpBankRefresh(static_cast<uint8_t>(argAt(line, 0, 0x5A)), false);
  } else if (line.startsWith("g4")) {
    otpBankRefresh(static_cast<uint8_t>(argAt(line, 0, 0x5A)), true);
  } else if (line == "full") {
    otpControlRefresh(0xF7, true);
  } else if (line == "fast") {
    otpBankRefresh(0x6A, true);
  } else if (line == "part") {
    otpControlRefresh(0xFF, false);
  } else if (line.startsWith("ctrl")) {
    otpControlRefresh(static_cast<uint8_t>(argAt(line, 0, 0xC7)), false);
  } else if (line.startsWith("sweep")) {
    const uint8_t from = static_cast<uint8_t>(argAt(line, 0, 0x00));
    const uint8_t to = static_cast<uint8_t>(argAt(line, 1, 0xF0));
    const uint8_t step = static_cast<uint8_t>(argAt(line, 2, 0x10));
    sweepOtpBanks(from, to, step);
  } else if (line.startsWith("fr")) {
    probeFrameRates(static_cast<uint8_t>(argAt(line, 0, 20)));
  } else if (line == "ovh") {
    probeActivationOverhead();
  } else if (line == "spi") {
    benchSpi();
  } else if (line == "gww") {
    runGray(true);
  } else if (line == "gw") {
    runGray(false);
  } else if (line == "dg") {
    deltaGray();
  } else if (line == "sat") {
    saturateReturn();
  } else if (line == "bw") {
    bwTransition();
  } else if (line == "tri") {
    triLevel(false);
  } else if (line == "tri*") {
    triLevel(true);
  } else if (line == "gtg3") {
    directThreeLevel();
  } else if (line.startsWith("abclean")) {
    cleanupAbCompare(static_cast<uint8_t>(argAt(line, 0, 3)));
  } else if (line.startsWith("img")) {
    const String slot = tokenAt(line, 0);
    receivePage((slot == "b" || slot == "B" || slot == "1") ? 1 : 0);
  } else if (line.startsWith("page")) {
    const String slot = tokenAt(line, 0);
    showPage((slot == "b" || slot == "B" || slot == "1") ? 1 : 0);
  } else if (line == "flip") {
    showPage(g_abPage ^ 1u);
  } else if (line.startsWith("ab")) {
    abStart(static_cast<uint8_t>(argAt(line, 0, 4)));
  } else if (line == "steps3") {
    drawThreeLevelSteps();
    Serial.println("3-level staircase staged");
  } else if (line == "dc") {
    dcReport();
  } else if (line.startsWith("ramp")) {
    rampTest(static_cast<uint8_t>(argAt(line, 0, 2)), static_cast<uint8_t>(argAt(line, 1, 16)),
             static_cast<uint8_t>(argAt(line, 2, 0)));
  } else if (line.startsWith("polish")) {
    polishTest(static_cast<uint8_t>(argAt(line, 0, 5)), static_cast<uint8_t>(argAt(line, 1, 10)));
  } else if (line.startsWith("dith")) {
    ditherTest(static_cast<uint8_t>(argAt(line, 0, 1)), static_cast<uint8_t>(argAt(line, 1, 5)),
               static_cast<uint16_t>(argAt(line, 2, 1024)));
  } else if (line.startsWith("cyc")) {
    cycleTest(static_cast<uint8_t>(argAt(line, 0, 5)), static_cast<uint16_t>(argAt(line, 1, 10)),
              static_cast<uint16_t>(argAt(line, 2, 20)), static_cast<uint16_t>(argAt(line, 3, 40)));
  } else if (line.startsWith("ghost")) {
    ghostLadder(static_cast<uint8_t>(argAt(line, 0, 0)), static_cast<uint8_t>(argAt(line, 1, 16)),
                static_cast<uint8_t>(argAt(line, 2, 40)), static_cast<uint8_t>(argAt(line, 3, 100)),
                static_cast<uint8_t>(argAt(line, 4, 0x08)));
  } else if (line.startsWith("hold")) {
    holdTest(static_cast<uint8_t>(argAt(line, 0, 8)));
  } else if (line.startsWith("clear")) {
    deepClear(static_cast<uint8_t>(argAt(line, 0, 4)));
  } else if (line.startsWith("burst")) {
    burst(static_cast<uint8_t>(argAt(line, 0, 4)));
  } else if (line == "params") {
    printGrayParams();
  } else if (line == "off") {
    activateTimed(0x03);
    Serial.println("controller powered down");
  } else if (line.startsWith("set ")) {
    String key = tokenAt(line, 0);
    key.toLowerCase();
    const long value = argAt(line, 1, -1);
    if (value < 0 || !setGrayParam(key, value)) {
      Serial.println("bad set; try: set a 7");
    } else {
      printGrayParams();
    }
  } else if (line.startsWith("hz")) {
    setSpiHz(static_cast<uint32_t>(argAt(line, 0, 20)) * 1000000UL);
    SPI.end();
    SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);
    pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_DC, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    Serial.printf("SPI clock = %lu Hz\n", static_cast<unsigned long>(g_spiHz));
  } else {
    Serial.println("unknown command; 'h' for help");
  }
  Serial.println("ready");
}

}  // namespace

void setup() {
  Serial.setRxBufferSize(8192);  // page upload arrives far faster than the shell polls
  Serial.begin(115200);
  Serial.setTimeout(3000);
  const uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(300);
  Serial.println();
  Serial.println("=== Paper Mono panel lab boot ===");

  g_p24 = static_cast<uint8_t*>(heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  g_p26 = static_cast<uint8_t*>(heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  g_scratch = static_cast<uint8_t*>(heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  g_inv24 = static_cast<uint8_t*>(heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  g_inv26 = static_cast<uint8_t*>(heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  g_prev24 = static_cast<uint8_t*>(heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  g_prev26 = static_cast<uint8_t*>(heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  g_d24 = static_cast<uint8_t*>(heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  g_d26 = static_cast<uint8_t*>(heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  g_pgA24 = static_cast<uint8_t*>(heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  g_pgA26 = static_cast<uint8_t*>(heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  g_pgB24 = static_cast<uint8_t*>(heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  g_pgB26 = static_cast<uint8_t*>(heap_caps_malloc(PLANE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!g_p24 || !g_p26 || !g_scratch || !g_inv24 || !g_inv26 || !g_prev24 || !g_prev26 || !g_d24 || !g_d26 ||
      !g_pgA24 || !g_pgA26 || !g_pgB24 || !g_pgB26) {
    Serial.println("ERR: plane allocation failed");
    return;
  }
  // Deterministic software baseline for direct-transition experiments. Run
  // `clear` before judging optics so the physical glass has the same baseline.
  memset(g_prev24, 0x00, PLANE_BYTES);
  memset(g_prev26, 0x00, PLANE_BYTES);

  if (!boardBegin()) {
    Serial.println("ERR: board bring-up failed");
    return;
  }
  epdPower(true);
  delay(100);

  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_DC, OUTPUT);
  pinMode(PIN_BUSY, INPUT);
  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_DC, HIGH);
  setSpiHz(20000000);
  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);

  drawTestCard();
  initController();
  Serial.printf("BUSY idle level = %d\n", digitalRead(PIN_BUSY));
  printHelp();
  Serial.println("ready");
}

void loop() {
  static String line;
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      String trimmed = line;
      trimmed.trim();
      line = "";
      handle(trimmed);
    } else if (line.length() < 96) {
      line += c;
    }
  }
  if (g_abMode) pollAbButtons();
  delay(5);
}

#endif  // PAPER_MONO_PANEL_LAB
