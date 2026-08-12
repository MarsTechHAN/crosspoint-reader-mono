#include <Arduino.h>
#include <BoardConfig.h>
#include <Epub.h>
#include <FontCacheManager.h>
#include <FontDecompressor.h>
#include <GfxRenderer.h>
#include <HalClock.h>
#include <HalDisplay.h>
#include <HalGPIO.h>
#include <HalPowerManager.h>
#include <HalStorage.h>
#include <HalSystem.h>
#include <HalTiltSensor.h>
#include <I18n.h>
#include <Logging.h>
#include <SPI.h>
#include <WiFi.h>
#include <builtinFonts/all.h>

#if FREEINK_DEVICE_PAPERMONO
#include <M5Pm1.h>
#include <PaperMonoBoard.h>
#include <SDCardManager.h>
#include <Wire.h>
#include <driver/Ssd1683Driver.h>
#endif

#include <cstring>

#include "CrossPointSettings.h"
#include "CrossPointState.h"
#include "FlashTtfFont.h"
#include "KOReaderCredentialStore.h"
#include "MappedInputManager.h"
#include "OpdsServerStore.h"
#include "RecentBooksStore.h"
#include "SdCardFontSystem.h"
#include "activities/Activity.h"
#include "activities/ActivityManager.h"
#include "activities/settings/SdFirmwareUpdateActivity.h"
#include "components/UITheme.h"
#include "fontIds.h"
#include "images/LoadingIcon.h"
#include "util/ButtonNavigator.h"
#include "util/ScreenshotUtil.h"
#include "util/WaveformLab.h"

GfxRenderer renderer(display);
MappedInputManager mappedInputManager(gpio, renderer);

#if FREEINK_DEVICE_PAPERMONO
static uint8_t paperMonoPowerButtonHook() {
  return PaperMonoBoard::pollPowerButtonClick() ? static_cast<uint8_t>(1u << InputManager::BTN_POWER) : 0;
}

// Input-edge wakeup for the main loop's tail wait. The loop otherwise sleeps
// delay(10) (or delay(50) once power saving engages), which every button press
// and touch contact pays as sampling latency. Both buttons and the FT6336 INT
// line are direct, interrupt-capable GPIOs; the ISR gives this semaphore so
// the tail wait collapses the instant an edge lands. A dedicated semaphore —
// NOT a task notification — because requestUpdateAndWait() already blocks the
// loop task on ulTaskNotifyTake and a spurious ISR give would wake it early.
static StaticSemaphore_t inputWakeSemaphoreStorage;
static SemaphoreHandle_t inputWakeSemaphore = nullptr;

static void IRAM_ATTR onInputEdgeIsr() {
  BaseType_t higherPriorityWoken = pdFALSE;
  if (inputWakeSemaphore) xSemaphoreGiveFromISR(inputWakeSemaphore, &higherPriorityWoken);
  portYIELD_FROM_ISR(higherPriorityWoken);
}

static void attachInputWakeInterrupts() {
  inputWakeSemaphore = xSemaphoreCreateBinaryStatic(&inputWakeSemaphoreStorage);
  const auto attachPin = [](const int8_t pin, const int mode) {
    if (pin >= 0) attachInterrupt(digitalPinToInterrupt(pin), onInputEdgeIsr, mode);
  };
  // Both edges for the buttons: DigitalTwoButton emits its short-press event
  // on release, so the release edge is the one that must not wait out a tick.
  attachPin(BoardConfig::ACTIVE.input.up, CHANGE);
  attachPin(BoardConfig::ACTIVE.input.down, CHANGE);
  // FT6336 INT is held low while a contact is present; the falling edge is
  // the touch-down the reader's instant turn path acts on.
  attachPin(BoardConfig::ACTIVE.touch.irq, FALLING);
}
#endif
ActivityManager activityManager(renderer, mappedInputManager);
FontDecompressor fontDecompressor;
SdCardFontSystem sdFontSystem;
FontCacheManager fontCacheManager(renderer.getFontMap(), renderer.getSdCardFonts());
#if FREEINK_DEVICE_PAPERMONO
FlashTtfFont builtinCjkFont;
#endif
static unsigned long allowSleepAt = 0;
#if FREEINK_DEVICE_PAPERMONO
// RAM-only test override. It is set through CMD:STAYAWAKE and intentionally
// disappears on reset so normal saved power settings are never changed.
static unsigned long debugSleepTimeoutMs = 0;
#endif

// Fonts
EpdFont notoserif14RegularFont(&notoserif_14_regular);
EpdFont notoserif14BoldFont(&notoserif_14_bold);
EpdFont notoserif14ItalicFont(&notoserif_14_italic);
EpdFont notoserif14BoldItalicFont(&notoserif_14_bolditalic);
EpdFontFamily notoserif14FontFamily(&notoserif14RegularFont, &notoserif14BoldFont, &notoserif14ItalicFont,
                                    &notoserif14BoldItalicFont);
#ifndef OMIT_FONTS
EpdFont notoserif12RegularFont(&notoserif_12_regular);
EpdFont notoserif12BoldFont(&notoserif_12_bold);
EpdFont notoserif12ItalicFont(&notoserif_12_italic);
EpdFont notoserif12BoldItalicFont(&notoserif_12_bolditalic);
EpdFontFamily notoserif12FontFamily(&notoserif12RegularFont, &notoserif12BoldFont, &notoserif12ItalicFont,
                                    &notoserif12BoldItalicFont);
EpdFont notoserif16RegularFont(&notoserif_16_regular);
EpdFont notoserif16BoldFont(&notoserif_16_bold);
EpdFont notoserif16ItalicFont(&notoserif_16_italic);
EpdFont notoserif16BoldItalicFont(&notoserif_16_bolditalic);
EpdFontFamily notoserif16FontFamily(&notoserif16RegularFont, &notoserif16BoldFont, &notoserif16ItalicFont,
                                    &notoserif16BoldItalicFont);
EpdFont notoserif18RegularFont(&notoserif_18_regular);
EpdFont notoserif18BoldFont(&notoserif_18_bold);
EpdFont notoserif18ItalicFont(&notoserif_18_italic);
EpdFont notoserif18BoldItalicFont(&notoserif_18_bolditalic);
EpdFontFamily notoserif18FontFamily(&notoserif18RegularFont, &notoserif18BoldFont, &notoserif18ItalicFont,
                                    &notoserif18BoldItalicFont);

EpdFont notosans12RegularFont(&notosans_12_regular);
EpdFont notosans12BoldFont(&notosans_12_bold);
EpdFont notosans12ItalicFont(&notosans_12_italic);
EpdFont notosans12BoldItalicFont(&notosans_12_bolditalic);
EpdFontFamily notosans12FontFamily(&notosans12RegularFont, &notosans12BoldFont, &notosans12ItalicFont,
                                   &notosans12BoldItalicFont);
EpdFont notosans14RegularFont(&notosans_14_regular);
EpdFont notosans14BoldFont(&notosans_14_bold);
EpdFont notosans14ItalicFont(&notosans_14_italic);
EpdFont notosans14BoldItalicFont(&notosans_14_bolditalic);
EpdFontFamily notosans14FontFamily(&notosans14RegularFont, &notosans14BoldFont, &notosans14ItalicFont,
                                   &notosans14BoldItalicFont);
EpdFont notosans16RegularFont(&notosans_16_regular);
EpdFont notosans16BoldFont(&notosans_16_bold);
EpdFont notosans16ItalicFont(&notosans_16_italic);
EpdFont notosans16BoldItalicFont(&notosans_16_bolditalic);
EpdFontFamily notosans16FontFamily(&notosans16RegularFont, &notosans16BoldFont, &notosans16ItalicFont,
                                   &notosans16BoldItalicFont);
EpdFont notosans18RegularFont(&notosans_18_regular);
EpdFont notosans18BoldFont(&notosans_18_bold);
EpdFont notosans18ItalicFont(&notosans_18_italic);
EpdFont notosans18BoldItalicFont(&notosans_18_bolditalic);
EpdFontFamily notosans18FontFamily(&notosans18RegularFont, &notosans18BoldFont, &notosans18ItalicFont,
                                   &notosans18BoldItalicFont);

#endif  // OMIT_FONTS

EpdFont smallFont(&notosans_8_regular);
EpdFontFamily smallFontFamily(&smallFont);

// UI chrome renders in the bold cut at both weights: the bold face fills the
// REGULAR slot as well as the BOLD one.
//
// E-ink's soft particle edges eat stroke weight, and Ubuntu Regular at 10/12 px
// comes out noticeably lighter on the panel than it looks on a monitor -- next
// to the book face it reads washed out. Doing it at registration rather than at
// the call sites keeps measurement and drawing consistent for free: getTextWidth
// and truncatedText resolve through this same family, so every truncation and
// centring stays correct without touching the ~130 places that name a style.
// Call sites that already ask for BOLD (Lyra's list titles) are unaffected.
//
// SMALL_FONT_ID keeps its regular weight -- notosans_8 ships no bold cut -- which
// leaves the size contrast between a UI_12 title and a SMALL subtitle intact.
EpdFont ui10RegularFont(&ubuntu_10_regular);
EpdFont ui10BoldFont(&ubuntu_10_bold);
EpdFontFamily ui10FontFamily(&ui10BoldFont, &ui10BoldFont);

EpdFont ui12RegularFont(&ubuntu_12_regular);
EpdFont ui12BoldFont(&ubuntu_12_bold);
EpdFontFamily ui12FontFamily(&ui12BoldFont, &ui12BoldFont);

// measurement of power button press duration calibration value
unsigned long t1 = 0;
unsigned long t2 = 0;

// Definitions for SilentRestart.h. RTC_NOINIT survives ESP.restart() but not power loss.
RTC_NOINIT_ATTR uint32_t silentRebootMagic;
RTC_NOINIT_ATTR uint32_t silentRebootTarget;
constexpr uint32_t SILENT_REBOOT_MAGIC = 0xC1EAB007;
constexpr uint32_t SILENT_REBOOT_TARGET_HOME = 0;
constexpr uint32_t SILENT_REBOOT_TARGET_READER = 1;

// How the device is coming back to life, resolved once at boot. Both resume
// flows suppress the splash and leave the panel holding its pre-boot frame; a
// plain boot shows the splash. See setup() for the resolution.
enum class BootResume : uint8_t {
  Splash,       // cold boot, flash, panic, or plain reboot
  Silent,       // heap-defrag ESP.restart() (RTC flag; lost on power loss)
  QuickResume,  // wake from a quick-resume deep sleep (SD flag; survives power loss)
};

// Latched true once enterDeepSleep() commits to sleeping, before it tears down
// the current activity. WiFi activities call silentRestart() in onExit() to
// clear heap fragmentation on the way out, but deep sleep is a full chip reset
// on wake and already clears the heap, so rebooting here would just power the
// device back up against the user's sleep gesture. Never cleared:
// startDeepSleep() does not return, so a set latch only ends at the wakeup reset.
static bool deepSleepInProgress = false;

void silentRestart() {
  if (deepSleepInProgress) return;  // sleeping supersedes the heap-defrag reboot
  silentRebootTarget = SILENT_REBOOT_TARGET_HOME;
  silentRebootMagic = SILENT_REBOOT_MAGIC;
  LOG_DBG("MAIN", "Silent restart (target=home)");
  // E-ink retains the previous frame until Home's first paint lands (~2-3s).
  // Without an overlay, users don't see the reboot and fire input through to
  // Home. Select on the default selectorIndex=0 then opens the most-recent
  // book, looking like a trampoline back to the reader they just exited.
  GUI.drawPopup(renderer, tr(STR_LOADING_POPUP));
  delay(50);
  ESP.restart();
}

void silentRestartToReader() {
  if (deepSleepInProgress) return;  // sleeping supersedes the heap-defrag reboot
  silentRebootTarget = SILENT_REBOOT_TARGET_READER;
  silentRebootMagic = SILENT_REBOOT_MAGIC;
  LOG_DBG("MAIN", "Silent restart (target=reader)");
  GUI.drawPopup(renderer, tr(STR_LOADING_POPUP));
  delay(50);
  ESP.restart();
}

void waitForPowerRelease() {
  gpio.update();
  while (gpio.isPressed(HalGPIO::BTN_POWER)) {
    delay(50);
    gpio.update();
  }
}

constexpr char SLEEP_FRAME_FILE[] = "/.crosspoint/sleep_frame.bin";

static void saveSleepFrameBuffer() {
  HalFile file;
  if (!Storage.openFileForWrite("SLP", SLEEP_FRAME_FILE, file)) return;
  file.write(renderer.getFrameBuffer(), renderer.getBufferSize());
  file.close();
}

static bool loadSleepFrameBuffer() {
  HalFile file;
  if (!Storage.openFileForRead("SLP", SLEEP_FRAME_FILE, file)) return false;
  const size_t bufferSize = display.getBufferSize();
  const size_t bytesRead = file.read(display.getFrameBuffer(), bufferSize);
  file.close();
  if (bytesRead != bufferSize) {
    Storage.remove(SLEEP_FRAME_FILE);
    return false;
  }
  Storage.remove(SLEEP_FRAME_FILE);
  return true;
}

#if FREEINK_DEVICE_PAPERMONO
// Raise-to-wake bench. Every command it adds pokes BMI270 and M5PM1 registers
// directly, behind the HAL, and two of them block the main loop for minutes --
// tools for characterising the wake chain, not for a device in someone's bag.
// Off unless the build asks for it; with the flag clear, sleepTestMode is a
// compile-time -1 and every variant branch below folds away.
#ifndef FREEINK_SLEEP_LAB
#define FREEINK_SLEEP_LAB 0
#endif
#if FREEINK_SLEEP_LAB
// Diagnostic override for the raise-to-wake investigation, driven by
// CMD:SLEEPX. -1 means "behave exactly as shipped".
int sleepTestMode = -1;
// How long a CMD:SLEEPX shutdown stays off before the PMIC powers the board
// back on by itself. Long enough that a genuine sleep is unmistakable (the bug
// re-powers in ~0.6 s) and that the IMU rail, if it is not being held, has long
// since decayed through the BMI270's power-on reset.
constexpr uint32_t kSleepTestReturnSeconds = 30;
#else
constexpr int sleepTestMode = -1;
#endif
#endif

// Enter deep sleep mode
void enterDeepSleep(bool fromTimeout = false) {
  HalPowerManager::Lock powerLock;  // Ensure we are at normal CPU frequency for sleep preparation
  APP_STATE.lastSleepFromReader = activityManager.isReaderActivity();

  const bool isQuickResumeSleep =
      SETTINGS.sleepScreen == CrossPointSettings::SLEEP_SCREEN_MODE::QUICK_RESUME ||
      (fromTimeout &&
       SETTINGS.quickResumeSleepScreen == CrossPointSettings::QUICK_RESUME_SLEEP_SCREEN::QUICK_RESUME_AFTER_TIMEOUT);
  APP_STATE.showBootScreen = !isQuickResumeSleep;

  APP_STATE.saveToFile();

  // Commit to sleeping before goToSleep() runs the outgoing activity's onExit():
  // a WiFi activity would otherwise silentRestart() here and reboot instead.
  deepSleepInProgress = true;
  activityManager.goToSleep(fromTimeout);

#if FREEINK_DEVICE_PAPERMONO
  // No-op today, and deliberately kept as the hook for "flush any deferred
  // panel work before deepSleep() cuts the EPD rail". No driver in the SDK
  // overrides PanelDriver::runMaintenance()/hasPendingMaintenance(), so this
  // currently costs one virtual call. It is safe only because SleepActivity
  // commits its own gray sequence synchronously in onEnter(); do not add a
  // deferred sleep-screen stage without implementing the hook. Locked because
  // the render task is a second controller consumer.
  {
    RenderLock sleepFlushLock;
    renderer.runDisplayMaintenance();
  }
#endif

  if (isQuickResumeSleep) {
    saveSleepFrameBuffer();
  }

  // Tear down WiFi so the modem power domain isn't held alive across deep sleep.
  // Wake from deep sleep is effectively a chip reset, so no state needs to survive.
  if (WiFi.getMode() != WIFI_MODE_NULL) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

#if FREEINK_DEVICE_PAPERMONO
  // Match the board's hard-off order: extinguish the frontlight and quiesce
  // switched peripherals before putting the panel controller to sleep. No
  // further board-bus traffic is needed before the PMIC shutdown command.
  //
  // This must precede arming raise-to-wake below. The frontlight is 260 ms of
  // PWM on PM1 GPIO3, which shares its wake interrupt line with GPIO4 — arming
  // first meant the fade itself supplied the wake edge, and the device came
  // straight back up without being touched.
  PaperMonoBoard::powerDownForSleep();
#endif

  // Raise-to-wake: instead of powering the IMU down, leave the accelerometer
  // running with the any-motion interrupt mapped to INT1 (wired to the PMIC's
  // GPIO4). Configure the sensor here, but do NOT arm the PMIC yet -- that
  // happens after the display powers down, below.
#if FREEINK_DEVICE_PAPERMONO
  // sleepTestMode: -1 follow settings, 0 force off, 1 arm the PMIC with the IMU
  // powered down, 2 force normal. Mode 1 is the discriminator -- if the board
  // still wakes with the IMU dark, the edge cannot be coming from INT1.
  const bool wantMotionWake = sleepTestMode < 0 ? SETTINGS.raiseToWake : (sleepTestMode > 0);
  const bool keepImuAlive = sleepTestMode != 1;
  if (!keepImuAlive) halTiltSensor.deepSleep();
  const bool motionSensorArmed = wantMotionWake && (!keepImuAlive || halTiltSensor.armMotionWake());
  if (!motionSensorArmed) {
    // The PM1 keeps its wake config across shutdown, so "off" must be written
    // as deliberately as "on" -- a stale enable wakes the device on every bump.
    PaperMonoBoard::setMotionWake(false);
    halTiltSensor.deepSleep();
  }
#if FREEINK_SLEEP_LAB
  if ((sleepTestMode >= 3 && sleepTestMode <= 5) && motionSensorArmed) {
    // Mode 3 unmaps any-motion from INT1 (INT1_MAP_FEAT = 0) so the feature can
    // never assert the pin. Mode 4 goes further and disables the INT1 output
    // driver itself (INT1_IO_CTRL = 0). Together they separate "the motion
    // engine fired" from "the configured pin emitted an edge when its rail
    // died" from "the accelerometer merely being powered matters".
    // Mode 5 completes the 2x2: INT1's driver left enabled but the
    // accelerometer powered down (PWR_CTRL = 0). Driver-off/accel-off (mode 1)
    // and driver-off/accel-on (mode 4) both sleep; driver-on/accel-on wakes. If
    // mode 5 also wakes, the enabled driver alone is sufficient and the edge is
    // a shutdown transient on the pin. If it sleeps, the sensor is pulsing INT1
    // despite nothing being mapped to it.
    const uint8_t reg = sleepTestMode == 3 ? 0x56 : (sleepTestMode == 4 ? 0x53 : 0x7D);
    Wire.beginTransmission(BoardConfig::ACTIVE.sensors.imuAddr);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(0x00));
    Wire.endTransmission();
    LOG_INF("SLP", "Test mode %d: cleared BMI270 reg 0x%02X", sleepTestMode, reg);
  }
#endif  // FREEINK_SLEEP_LAB
#else
  halTiltSensor.deepSleep();
#endif
  display.deepSleep();
  LOG_DBG("MAIN", "Entering deep sleep");

#if FREEINK_DEVICE_PAPERMONO
  // Arm the PMIC last, after the panel is asleep.
  //
  // GPIO3 and GPIO4 share one wake interrupt line, so anything that disturbs
  // either pin between arming and power-off is indistinguishable from motion.
  // Arming before display.deepSleep() put the EPD rail transition inside that
  // window and the board woke itself within half a second every time, with the
  // PMIC reporting WAKE_EXT_GPIO (0x20) even though both pins read low at the
  // moment of arming. Doing it here shrinks the window to the shutdown command
  // itself.
  if (motionSensorArmed) {
    if (PaperMonoBoard::setMotionWake(true)) {
#if FREEINK_SLEEP_LAB
      // Read the arming back out of the PMIC rather than trusting the writes.
      // This is the last state the chip holds before the rails collapse, so it
      // is the only honest record of what the shutdown was actually armed on.
      // Bench-only: these are four more I2C transactions inside the arm-to-
      // shutdown window that the comment above is about keeping short.
      uint8_t wakeEn = 0, wakeCfg = 0, pull1 = 0, gpioIn = 0;
      freeink::m5pm1::readReg(freeink::m5pm1::REG_GPIO_WAKE_EN, &wakeEn);
      freeink::m5pm1::readReg(freeink::m5pm1::REG_GPIO_WAKE_CFG, &wakeCfg);
      freeink::m5pm1::readReg(freeink::m5pm1::REG_GPIO_PULL1, &pull1);
      freeink::m5pm1::readReg(freeink::m5pm1::REG_GPIO_IN, &gpioIn);
      LOG_INF("SLP", "Raise-to-wake armed: WAKE_EN=0x%02X CFG=0x%02X PULL1=0x%02X GPIO_IN=0x%02X", wakeEn, wakeCfg,
              pull1, gpioIn);
#endif
    } else {
      // Reached when INT1 never went quiet: arming a rising-edge wake on a line
      // that is still moving would shut down and immediately reboot.
      LOG_INF("SLP", "Raise-to-wake unavailable, sleeping without it");
      PaperMonoBoard::setMotionWake(false);
      halTiltSensor.deepSleep();
    }
  }

  // Last thing before the shutdown command: drop the PMIC rails nothing needs
  // while the system is off. Deliberately after display.deepSleep() and after
  // arming, so neither the panel nor the wake window can be affected by it.
  PaperMonoBoard::powerDownRailsForShutdown();
#endif

  powerManager.startDeepSleep(gpio);
}

void setupDisplayAndFonts(bool seamless = false) {
  display.begin(seamless);
  renderer.begin();
  activityManager.begin();
  LOG_DBG("MAIN", "Display initialized");

  // Initialize font decompressor for compressed reader fonts
  if (!fontDecompressor.init()) {
    LOG_ERR("MAIN", "Font decompressor init failed");
  }
  fontCacheManager.setFontDecompressor(&fontDecompressor);
  renderer.setFontCacheManager(&fontCacheManager);
  renderer.insertFont(NOTOSERIF_14_FONT_ID, notoserif14FontFamily);
#ifndef OMIT_FONTS
  renderer.insertFont(NOTOSERIF_12_FONT_ID, notoserif12FontFamily);
  renderer.insertFont(NOTOSERIF_16_FONT_ID, notoserif16FontFamily);
  renderer.insertFont(NOTOSERIF_18_FONT_ID, notoserif18FontFamily);

  renderer.insertFont(NOTOSANS_12_FONT_ID, notosans12FontFamily);
  renderer.insertFont(NOTOSANS_14_FONT_ID, notosans14FontFamily);
  renderer.insertFont(NOTOSANS_16_FONT_ID, notosans16FontFamily);
  renderer.insertFont(NOTOSANS_18_FONT_ID, notosans18FontFamily);
#endif  // OMIT_FONTS
  renderer.insertFont(UI_10_FONT_ID, ui10FontFamily);
  renderer.insertFont(UI_12_FONT_ID, ui12FontFamily);
  renderer.insertFont(SMALL_FONT_ID, smallFontFamily);

#if FREEINK_DEVICE_PAPERMONO
  if (builtinCjkFont.begin()) {
    for (const uint8_t pointSize : FlashTtfFont::POINT_SIZES) {
      renderer.insertFont(builtinCjkFontId(pointSize), EpdFontFamily(builtinCjkFont.font(pointSize)));
    }

    // LXGW WenKai's Han em box appears optically smaller than the bundled
    // Latin faces at the same nominal point size. Route every built-in CJK
    // fallback above the Latin nominal size; measurement and drawing resolve
    // through this same ID.
    //
    // The UI faces run at N+6 rather than the reader's N+4: a Han glyph packs
    // far more strokes into the same box than a Latin letter, and the UI fonts
    // are the smallest in the build (Ubuntu 10/12 pt, Noto Sans 8 pt), so the
    // stroke pitch there lands below what this panel resolves cleanly. Reader
    // body text keeps N+4 — it is already large and user-adjustable.
    renderer.setBuiltinFallbackFont(SMALL_FONT_ID, BUILTIN_CJK_14_FONT_ID);
    renderer.setBuiltinFallbackFont(UI_10_FONT_ID, BUILTIN_CJK_16_FONT_ID);
    renderer.setBuiltinFallbackFont(UI_12_FONT_ID, BUILTIN_CJK_18_FONT_ID);
#ifndef OMIT_FONTS
    renderer.setBuiltinFallbackFont(NOTOSERIF_12_FONT_ID, BUILTIN_CJK_16_FONT_ID);
    renderer.setBuiltinFallbackFont(NOTOSERIF_16_FONT_ID, BUILTIN_CJK_20_FONT_ID);
    renderer.setBuiltinFallbackFont(NOTOSERIF_18_FONT_ID, BUILTIN_CJK_22_FONT_ID);
    renderer.setBuiltinFallbackFont(NOTOSANS_12_FONT_ID, BUILTIN_CJK_16_FONT_ID);
    renderer.setBuiltinFallbackFont(NOTOSANS_14_FONT_ID, BUILTIN_CJK_18_FONT_ID);
    renderer.setBuiltinFallbackFont(NOTOSANS_16_FONT_ID, BUILTIN_CJK_20_FONT_ID);
    renderer.setBuiltinFallbackFont(NOTOSANS_18_FONT_ID, BUILTIN_CJK_22_FONT_ID);
#endif
    renderer.setBuiltinFallbackFont(NOTOSERIF_14_FONT_ID, BUILTIN_CJK_18_FONT_ID);
  }
#endif

  // Discover and load SD card fonts
  sdFontSystem.begin(renderer);

  LOG_DBG("MAIN", "Fonts setup");
}

void setup() {
#if FREEINK_DEVICE_PAPERMONO
  const bool paperMonoBoardReady = PaperMonoBoard::begin();
  InputManager::setButtonHook(paperMonoPowerButtonHook);
  SDCardManager::getInstance().setPowerHook(PaperMonoBoard::enableSd);
#endif
  BoardConfig::holdPowerRails();

  t1 = millis();

#ifdef ENABLE_SERIAL_LOG
  // Earliest possible Serial setup. The 250 ms stall before begin() lets the
  // USB Serial/JTAG peripheral finish power-on and lets the host complete USB
  // enumeration before we touch the CDC state — otherwise cold boot races
  // and the host has to be physically replugged for logs to flow. Warm reboot
  // worked without the delay because USB was already enumerated.
  delay(250);
  Serial.begin(115200);
#if LOG_SERIAL_HAS_TX_TIMEOUT
  logSerial.setTxTimeoutMs(1);  // This is a load-bearing 1. Do not modify.
#endif
#endif

  HalSystem::begin();

#if FREEINK_DEVICE_PAPERMONO
  if (!paperMonoBoardReady) {
    LOG_ERR("MAIN", "Paper Mono M5IOE1 initialization failed");
  } else {
    LOG_INF("MAIN", "Paper Mono PMIC: wake=0x%02X btn_cfg=0x%02X", PaperMonoBoard::wakeSource(),
            PaperMonoBoard::powerButtonConfig());
  }
#endif

  // Read-and-clear so a panic later in setup() doesn't loop into silent reboot.
  // Bound the target range too — RTC_NOINIT memory is uninitialized on cold boot.
  const bool isSilentReboot = (silentRebootMagic == SILENT_REBOOT_MAGIC);
  const uint32_t snapshotTarget =
      (isSilentReboot && silentRebootTarget <= SILENT_REBOOT_TARGET_READER) ? silentRebootTarget : 0;
  silentRebootMagic = 0;
  silentRebootTarget = 0;

  gpio.begin();
#if FREEINK_DEVICE_PAPERMONO
  // After gpio.begin(): the pins carry their INPUT_PULLUP modes by now.
  attachInputWakeInterrupts();
#endif
  powerManager.begin();
  halTiltSensor.begin();
  halClock.begin();

  LOG_INF("MAIN", "Hardware: %s", BoardConfig::ACTIVE.name);

  // SD Card Initialization
  // We need 6 open files concurrently when parsing a new chapter
  if (!Storage.begin()) {
    LOG_ERR("MAIN", "SD card initialization failed");
    setupDisplayAndFonts(isSilentReboot);
    activityManager.goToFullScreenMessage("SD card error", EpdFontFamily::BOLD);
    return;
  }

  HalSystem::checkPanic();

  // Not fatal — the struct initializers are a valid configuration — but a boot
  // that silently reverts every setting is the exact symptom users report as
  // "settings won't save", so make the load failure visible in the log.
  if (!SETTINGS.loadFromFile()) {
    LOG_INF("MAIN", "No stored settings loaded; using defaults");
  }
#if FREEINK_DEVICE_PAPERMONO
  freeink::Ssd1683GrayParams grayParams;
  grayParams.darkFrames = SETTINGS.grayDarkFrames;
  grayParams.lightFrames = SETTINGS.grayLightFrames;
  freeink::ssd1683SetGrayParams(grayParams);
  LOG_INF("MAIN", "Paper Mono gray calibration: dark=%u light=%u", grayParams.darkFrames, grayParams.lightFrames);
  // Paper Mono's tilt setting is a plain on/off (the BMI270 mounting inversion
  // is baked into HalTiltSensor); fold a stale 3-state INVERTED value into ON
  // so the two-option settings UI never indexes past its label list.
  if (SETTINGS.tiltPageTurn >= CrossPointSettings::TILT_NVERTED) {
    SETTINGS.tiltPageTurn = CrossPointSettings::TILT_NORMAL;
  }

  // Raise-to-wake pose gate.
  //
  // The BMI270 fires the lift gesture on "large attitude change, ending in an
  // accepted attitude", and its accepted attitudes are bounded only from above
  // (max_tilt_pd/pu/lr/ll, with no minimum anywhere in Bosch's field set). Flat
  // is therefore the *most* accepted pose there is, so shaking the device and
  // setting it down woke it exactly as reliably as picking it up. Nothing in
  // the sensor's configuration can express "but not flat", so the bound lives
  // here instead: re-read the pose on the way up and, if the device is not
  // actually being held, go straight back to sleep.
  //
  // Deliberately placed before the display, SD-backed state loads and the
  // splash: a rejected wake should cost a couple of seconds of dark boot and
  // leave no visible trace. Gated on the PMIC's latched wake source alone
  // rather than SETTINGS.raiseToWake, so it also covers a stale PMIC arming;
  // the power button wakes with 0x04 and always boots through.
  if (PaperMonoBoard::wokeByMotion() && !halTiltSensor.isRaisedPose()) {
    LOG_INF("MAIN", "Motion wake without a raised pose; returning to sleep");
    // Re-arm before shutting down. HOLD_CFG's LDO hold auto-clears on every
    // shutdown, so a spurious wake that just powered off again would leave the
    // accelerometer unpowered and raise-to-wake dead until the next
    // power-button boot.
    const bool rearmed = SETTINGS.raiseToWake && halTiltSensor.armMotionWake();
    // Same order as enterDeepSleep(): quiesce switched peripherals first,
    // because the frontlight fade is 260 ms of PWM on PM1 GPIO3, which shares
    // its wake interrupt line with GPIO4.
    PaperMonoBoard::powerDownForSleep();
    if (!rearmed) {
      // Either the user does not want raise-to-wake (so the arming that woke us
      // was stale and must be cleared, or the loop repeats forever) or the IMU
      // refused to arm.
      PaperMonoBoard::setMotionWake(false);
      halTiltSensor.deepSleep();
    } else if (!PaperMonoBoard::setMotionWake(true)) {
      LOG_INF("MAIN", "Raise-to-wake unavailable, sleeping without it");
      PaperMonoBoard::setMotionWake(false);
      halTiltSensor.deepSleep();
    }
    powerManager.startDeepSleep(gpio);
  }
#endif
#if FREEINK_DEVICE_PAPERMONO && FREEINK_WAVEFORM_LAB
  // Opt-in only: installs a runtime LUT override when the user has explicitly
  // saved a boot marker via CMD:WAVE BOOT; otherwise a silent no-op.
  WaveformLab::loadBootWaveform();
#endif
  APP_STATE.loadFromFile();
  RECENT_BOOKS.loadFromFile();
  I18N.setLanguage(static_cast<Language>(SETTINGS.language));
  KOREADER_STORE.loadFromFile();
  OPDS_STORE.loadFromFile();
  UITheme::getInstance().reload();
  ButtonNavigator::setMappedInputManager(mappedInputManager);

  const auto wakeupReason = gpio.getWakeupReason();
  switch (wakeupReason) {
    case HalGPIO::WakeupReason::PowerButton:
      LOG_DBG("MAIN", "Verifying power button press duration");
      if (!gpio.verifyPowerButtonWakeup(SETTINGS.getPowerButtonDuration(),
                                        SETTINGS.shortPwrBtn == CrossPointSettings::SHORT_PWRBTN::SLEEP)) {
        powerManager.startDeepSleep(gpio);
      }
      break;
    case HalGPIO::WakeupReason::AfterUSBPower:
      // If USB power caused a cold boot, go back to sleep
      LOG_DBG("MAIN", "Wakeup reason: After USB Power");
      powerManager.startDeepSleep(gpio);
      break;
    case HalGPIO::WakeupReason::AfterFlash:
      // After flashing, just proceed to boot
    case HalGPIO::WakeupReason::Other:
    default:
      break;
  }

  // Recovery firmware mode: hold left side button (BTN_UP) together with the power button at
  // boot to skip directly to the SD-card firmware update screen. Useful on devices where USB
  // flashing has been locked down (e.g. recent X3 firmware).
  bool recoveryFirmwareMode = false;
  if (wakeupReason == HalGPIO::WakeupReason::PowerButton) {
    // Refresh the cached button state a few times — isPressed() needs ~half a second to settle
    // after boot per the HalGPIO contract. Use a millis-based deadline so we always wait the full
    // settle window even if the loop body takes longer than expected on slow boots.
    const unsigned long settleStart = millis();
    while (millis() - settleStart < 500) {
      gpio.update();
      delay(10);
    }
    if (gpio.isPressed(HalGPIO::BTN_UP)) {
      recoveryFirmwareMode = true;
      LOG_INF("MAIN", "Recovery firmware mode (UP + POWER held at boot)");
    }
  }

  // First serial output only here to avoid timing inconsistencies for power button press duration verification
  LOG_DBG("MAIN", "Starting CrossPoint version " CROSSPOINT_VERSION);

  // Resolve the single boot-presentation decision. Skipping the splash also
  // skips the panel-clearing pass and the X3 initial-full-sync arming (see
  // HalDisplay::begin), so the first paint is FAST_REFRESH (~500ms) over the
  // retained frame and input dispatches against a visible UI.
  const BootResume resume = isSilentReboot              ? BootResume::Silent
                            : !APP_STATE.showBootScreen ? BootResume::QuickResume
                                                        : BootResume::Splash;
  bool allowFastInitialReaderRefresh = false;
  bool initialActivityPaintComplete = false;

  setupDisplayAndFonts(resume != BootResume::Splash);

  switch (resume) {
    case BootResume::Silent:
      // Splash skipped: the routing block below picks the target activity; the
      // panel keeps showing the pre-reboot popup until that first paint lands.
      break;
    case BootResume::QuickResume:
      // One-shot flag: re-arm the splash for the next non-quick-resume boot. Save
      // before any painting so a hang in the blocking paint path can't strand
      // us in a quick-resume-with-no-frame loop on the next boot.
      APP_STATE.showBootScreen = true;
      APP_STATE.saveToFile();
      if (loadSleepFrameBuffer()) {
        const bool useDifferentialRefresh = gpio.deviceIsX3();
        if (useDifferentialRefresh) {
          // begin() clears the X3 controller RAM, so restore the saved frame as
          // the baseline before replacing the moon with the loading icon.
          renderer.cleanupGrayscaleWithFrameBuffer();
        }

        const auto pageHeight = renderer.getScreenHeight();
        renderer.drawImage(LoadingIcon, 0, pageHeight - LOADINGICON_HEIGHT, LOADINGICON_WIDTH, LOADINGICON_HEIGHT);
        if (useDifferentialRefresh) {
          renderer.displayGrayscaleBase(HalDisplay::FAST_REFRESH);
          allowFastInitialReaderRefresh = true;
        } else {
          renderer.displayBuffer(HalDisplay::HALF_REFRESH);
        }
      } else {
        activityManager.goToBoot();  // frame file missing, fall back to the splash
      }
      break;
    case BootResume::Splash:
      activityManager.goToBoot();
      break;
  }

  if (recoveryFirmwareMode) {
    // Skip normal home/reader routing: jump straight into the SD firmware picker.
    activityManager.replaceActivity(
        std::make_unique<SdFirmwareUpdateActivity>(renderer, mappedInputManager, /*recoveryMode=*/true));
  } else if (HalSystem::isRebootFromPanic()) {
    // If we rebooted from a panic, go to crash report screen to show the panic info
    activityManager.goToCrashReport();
  } else if (resume == BootResume::Silent && snapshotTarget == SILENT_REBOOT_TARGET_READER &&
             !APP_STATE.openEpubPath.empty()) {
    activityManager.goToReader(APP_STATE.openEpubPath);
  } else if (resume == BootResume::Silent) {
    // target == home (or reader with no open book): land on home — don't fall
    // through to the sleep-wake "resume reader" logic, which fires on stale
    // openEpubPath + lastSleepFromReader from a prior session.
    activityManager.goHome();
  } else if (APP_STATE.openEpubPath.empty() || !APP_STATE.lastSleepFromReader ||
             mappedInputManager.isPressed(MappedInputManager::Button::Back) || APP_STATE.readerActivityLoadCount > 0) {
    // Boot to home screen if no book is open, last sleep was not from reader, back button is held, or reader activity
    // crashed (indicated by readerActivityLoadCount > 0)
    activityManager.goHome();
  } else {
    // Clear app state to avoid getting into a boot loop if the epub doesn't load
    const auto path = APP_STATE.openEpubPath;
    APP_STATE.openEpubPath = "";
    APP_STATE.readerActivityLoadCount++;
    APP_STATE.saveToFile();
    activityManager.goToReader(path, allowFastInitialReaderRefresh);
  }

  if (resume == BootResume::Silent) {
    // Block until the first paint physically completes. refreshDisplay()
    // waits on the panel BUSY pin so when this returns the user can see the
    // new activity. Without the wait, an edge captured by gpio.update()
    // during boot dispatches against an invisible Home and the default
    // selectorIndex=0 opens the most-recent book.
    activityManager.requestUpdateAndWait();
    initialActivityPaintComplete = true;
    // Absorb any button held at this point into currentState as a non-edge:
    // two gpio.update() calls separated by > InputManager's 5ms debounce
    // transition the held bit through lastDebounceTime into currentState
    // without setting pressedEvents, so the first loop()'s own gpio.update()
    // sees state == currentState and emits nothing.
    gpio.update();
    delay(10);
    gpio.update();
  }

#if FREEINK_DEVICE_PAPERMONO
  // Paper Mono boots with its frontlight held at 0%. Wait for the initial UI
  // paint and its direct non-flashing baseline update before bringing the
  // light up.
  if (!initialActivityPaintComplete) activityManager.requestUpdateAndWait();
  LOG_INF("MAIN", "Paper Mono initial refresh complete; fading frontlight to %u%%", SETTINGS.frontlightBrightness);
  if (paperMonoBoardReady && !PaperMonoBoard::fadeFrontlightTo(SETTINGS.frontlightBrightness, 420)) {
    LOG_ERR("MAIN", "Paper Mono frontlight fade-in failed");
  } else if (paperMonoBoardReady) {
    LOG_INF("MAIN", "Paper Mono frontlight ready at %u%%", PaperMonoBoard::getFrontlightBrightness());
  }
#endif

  // Ensure we're not still holding the power button before leaving setup
  waitForPowerRelease();
  allowSleepAt = millis() + 2000;
}

void loop() {
  static unsigned long maxLoopDuration = 0;
  const unsigned long loopStartTime = millis();
  static unsigned long lastMemPrint = 0;

  gpio.setSharedConfirmPowerShortPressEmitsPower(SETTINGS.shortPwrBtn == CrossPointSettings::SHORT_PWRBTN::SLEEP);
  gpio.update();
#if FREEINK_DEVICE_PAPERMONO
  // Input is sampled on the main task while display work runs on the render
  // task. Signal it at the raw edge so four-gray refinement/background cleanup
  // can yield even before the current activity turns the gesture into a render.
  const bool rawInputActivity = gpio.wasAnyPressed() || gpio.wasAnyReleased() || gpio.wasTouchActivity();
  if (rawInputActivity) {
    activityManager.noteUserInteraction();
  }
  bool rawInputActive = false;
  for (uint8_t button = HalGPIO::BTN_BACK; button <= HalGPIO::BTN_POWER; ++button) {
    rawInputActive = rawInputActive || gpio.isPressed(button);
  }
  float touchX = 0.0f;
  float touchY = 0.0f;
  rawInputActive = rawInputActive || gpio.isTouchHeldAt(touchX, touchY);
  activityManager.setUserInputActive(rawInputActive);
#endif
  halTiltSensor.update(SETTINGS.tiltPageTurn, SETTINGS.orientation, activityManager.isReaderActivity(),
                       SETTINGS.getFaceDownSleepMs() > 0);

  renderer.setFadingFix(SETTINGS.fadingFix);

  if (Serial && millis() - lastMemPrint >= 10000) {
    LOG_INF("MEM", "Free: %d bytes, Total: %d bytes, Min Free: %d bytes, MaxAlloc: %d bytes", ESP.getFreeHeap(),
            ESP.getHeapSize(), ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
    lastMemPrint = millis();
  }

  // Handle incoming serial commands,
  // nb: we use logSerial from logging to avoid deprecation warnings
  if (logSerial.available() > 0) {
    String line = logSerial.readStringUntil('\n');
    if (line.startsWith("CMD:")) {
      String cmd = line.substring(4);
      cmd.trim();
      if (cmd == "SCREENSHOT") {
        const uint32_t bufferSize = display.getBufferSize();
        logSerial.printf("SCREENSHOT_START:%d\n", bufferSize);
        uint8_t* buf = display.getFrameBuffer();
        logSerial.write(buf, bufferSize);
        logSerial.printf("SCREENSHOT_END\n");
      } else if (cmd.startsWith("TAP ")) {
        int x = -1;
        int y = -1;
        if (sscanf(cmd.c_str() + 4, "%d %d", &x, &y) == 2) {
          mappedInputManager.injectDebugTap(x, y);
          activityManager.noteUserInteraction();
          LOG_INF("INPUT", "Injected tap at logical (%d,%d)", x, y);
        } else {
          LOG_ERR("INPUT", "Usage: CMD:TAP <x> <y>");
        }
      } else if (cmd == "NEXT" || cmd == "PREV") {
        const int x = cmd == "NEXT" ? renderer.getScreenWidth() * 5 / 6 : renderer.getScreenWidth() / 6;
        const int y = renderer.getScreenHeight() / 2;
        mappedInputManager.injectDebugTap(x, y);
        activityManager.noteUserInteraction();
        LOG_INF("INPUT", "Injected %s tap at logical (%d,%d)", cmd.c_str(), x, y);
      } else if (cmd == "HOME") {
        activityManager.noteUserInteraction();
        activityManager.goHome();
        LOG_INF("INPUT", "Injected HOME navigation");
#if FREEINK_DEVICE_PAPERMONO
      } else if (cmd == "SETTINGS") {
        activityManager.noteUserInteraction();
        activityManager.goToSettings();
        LOG_INF("INPUT", "Injected SETTINGS navigation");
#endif
#if FREEINK_DEVICE_PAPERMONO
      } else if (cmd == "STAYAWAKE") {
        debugSleepTimeoutMs = 2UL * 60UL * 60UL * 1000UL;
        activityManager.noteUserInteraction();
        LOG_INF("SLP", "RAM-only auto-sleep timeout override: %lu ms", debugSleepTimeoutMs);
      } else if (cmd == "WAKESRC") {
        // The PMIC latches why it powered the board on, and begin() reads and
        // clears it. USB CDC is not up yet when that happens, so the boot log
        // line is unreliable over serial -- report the retained copy instead.
        const uint8_t src = PaperMonoBoard::wakeSource();
        LOG_INF("SLP", "Latched wake source=0x%02X (btn=%d extGpio=%d) motion=%d", src,
                (src & freeink::m5pm1::WAKE_PWR_BUTTON) ? 1 : 0, (src & freeink::m5pm1::WAKE_EXT_GPIO) ? 1 : 0,
                PaperMonoBoard::wokeByMotion() ? 1 : 0);
      } else if (cmd == "SLEEP") {
        // Sleep on demand. The raise-to-wake failure mode only shows itself
        // across a real shutdown, and waiting out the auto-sleep timeout makes
        // that a multi-minute loop per attempt. Reports the PMIC's own wake
        // reason on the way back up (see the "Paper Mono PMIC: wake=" line).
        LOG_INF("SLP", "Sleep requested over serial");
        logSerial.flush();
        enterDeepSleep();
#if FREEINK_SLEEP_LAB
      } else if (cmd.startsWith("SLEEPX")) {
        // Same as SLEEP, but forces one of the raise-to-wake arming variants so
        // the wake source can be attributed. See sleepTestMode.
        sleepTestMode = cmd.substring(6).toInt();
        // Arm the PMIC's own countdown to power the board back on, so a test
        // that correctly STAYS asleep still returns on its own instead of
        // costing a walk-over-and-press-the-button round trip. The boot log's
        // WAKE_SRC then separates the two outcomes: 0x01 = the timer fired
        // (i.e. it slept properly), 0x20 = the wake GPIO fired (the bug).
        if (!freeink::m5pm1::armPowerOnTimer(kSleepTestReturnSeconds)) {
          LOG_ERR("SLP", "Failed to arm PM1 power-on timer; device may stay off");
        }
        LOG_INF("SLP", "Sleep requested over serial, arming mode %d (auto power-on in %u s)", sleepTestMode,
                static_cast<unsigned>(kSleepTestReturnSeconds));
        logSerial.flush();
        enterDeepSleep();
        sleepTestMode = -1;
      } else if (cmd == "LIFTSLEEP") {
        // The acceptance test for raise-to-wake: shut down with the lift gesture
        // armed and NO power-on timer, so the only ways back are a real pickup
        // and the power button. Unlike SLEEPX this cannot mask a failure with a
        // timed return, and unlike SLEEP it does not depend on the user setting.
        // Read the boot log's "Paper Mono PMIC: wake=" line for the verdict:
        // 0x20 = EXT_GPIO, the lift woke it; 0x04 = the power button, i.e. the
        // gesture never fired.
        sleepTestMode = 2;  // force motion wake armed, IMU left alive
        LOG_INF("SLP", "LIFTSLEEP: shutting down with the lift gesture armed and no timer");
        logSerial.flush();
        enterDeepSleep();
        sleepTestMode = -1;
      } else if (cmd == "PMICDUMP") {
        // PWR_CFG (0x06) and HOLD_CFG (0x07) decide which rails survive the
        // shutdown. HOLD_CFG bit5 holds the 3.3V LDO through power-off and
        // auto-clears on every shutdown, so it has to be re-armed each time.
        uint8_t r[6] = {};
        for (uint8_t i = 0; i < 6; ++i) freeink::m5pm1::readReg(static_cast<uint8_t>(0x04 + i), &r[i]);
        LOG_INF("PM1",
                "SRC=0x%02X WAKE=0x%02X PWR_CFG=0x%02X (chg=%d dcdc=%d ldo=%d boost=%d) HOLD_CFG=0x%02X (ldo=%d)", r[0],
                r[1], r[2], r[2] & 1, (r[2] >> 1) & 1, (r[2] >> 2) & 1, (r[2] >> 3) & 1, r[3], (r[3] >> 5) & 1);
        // GPIO block: MODE/OUT/IN/DRV/PULL0/PULL1/FUNC0/FUNC1/WAKE_EN/WAKE_CFG.
        // G4 is the IMU INT1 wake line, so its pull (PULL1 bits [1:0]), its
        // function select (FUNC1 -- the datasheet has a dedicated WAKE mode our
        // code never selects) and its edge (WAKE_CFG bit4) all matter.
        uint8_t g[10] = {};
        for (uint8_t i = 0; i < 10; ++i) freeink::m5pm1::readReg(static_cast<uint8_t>(0x10 + i), &g[i]);
        LOG_INF("PM1",
                "GPIO MODE=0x%02X OUT=0x%02X IN=0x%02X DRV=0x%02X PULL0=0x%02X PULL1=0x%02X FUNC0=0x%02X FUNC1=0x%02X "
                "WAKE_EN=0x%02X WAKE_CFG=0x%02X",
                g[0], g[1], g[2], g[3], g[4], g[5], g[6], g[7], g[8], g[9]);
        LOG_INF("PM1", "  -> G4: level=%d pull=%d(0=none,1=up,2=down) func=%d wake_en=%d edge=%s", (g[2] >> 4) & 1,
                g[5] & 0x3, g[7] & 0x3, (g[8] >> 4) & 1, ((g[9] >> 4) & 1) ? "rising" : "falling");
      } else if (cmd.startsWith("HOLDCFG")) {
        // HOLD_CFG (0x07): [6] boost hold, [5] LDO(3.3V) hold, [4:0] GPIO4..0
        // output-state hold. Documented to auto-clear on reset/download/
        // shutdown, so whether a hold actually survives a shutdown is an
        // empirical question -- this lets the next sleep test set it either way.
        String arg = cmd.substring(7);
        arg.trim();
        uint8_t before = 0;
        freeink::m5pm1::readReg(0x07, &before);
        if (arg.length() > 0) {
          const uint8_t want = static_cast<uint8_t>(strtoul(arg.c_str(), nullptr, 16));
          freeink::m5pm1::writeReg(0x07, want);
        }
        uint8_t after = 0;
        freeink::m5pm1::readReg(0x07, &after);
        LOG_INF("PM1", "HOLD_CFG 0x%02X -> 0x%02X (ldo_hold=%d gpio_hold=0x%02X)", before, after, (after >> 5) & 1,
                after & 0x1F);
      } else if (cmd.startsWith("WRIST")) {
        // Lift-gesture bench. armMotionWake() leaves the sensor in the exact
        // sleep-time configuration -- axis remap on page 1, wrist-wear wake-up
        // on page 7 -- and this reads both pages back, so a write that never
        // landed is distinguishable from a gesture window that is simply wrong.
        // Each sample also prints the accel projected into the wearable frame
        // and whether that attitude sits inside the gesture window, so a pickup
        // that does not fire says which limit rejected it.
        const uint8_t a = BoardConfig::ACTIVE.sensors.imuAddr;
        activityManager.noteUserInteraction();
        if (!halTiltSensor.armMotionWake()) {
          LOG_ERR("IMU", "WRIST: armMotionWake failed");
        } else {
          auto rd = [&](uint8_t reg, uint8_t* dst, uint8_t len) {
            Wire.beginTransmission(a);
            Wire.write(reg);
            return Wire.endTransmission(false) == 0 && Wire.requestFrom(a, len) == len && [&] {
              for (uint8_t i = 0; i < len; ++i) dst[i] = Wire.read();
              return true;
            }();
          };
          auto wr = [&](uint8_t reg, uint8_t v) {
            Wire.beginTransmission(a);
            Wire.write(reg);
            Wire.write(v);
            return Wire.endTransmission() == 0;
          };

          wr(0x7C, 0x00);  // advanced power save off: required for feature-page access
          delayMicroseconds(500);
          uint8_t page[16] = {};
          wr(0x2F, 1);  // FEAT_PAGE = 1: axis remap at 0x04, any-motion at 0x0C
          rd(0x30, page, sizeof(page));
          LOG_INF("IMU", "page1: axis_map=%02X %02X (want %02X, z_neg bit0) anymot_en=%d", page[0x04], page[0x05], 0xA5,
                  (page[0x0F] >> 7) & 1);
          wr(0x2F, 7);  // FEAT_PAGE = 7: wrist-wear wake-up at 0x00
          rd(0x30, page, sizeof(page));
          LOG_INF("IMU", "page7: en=%d focus=%u nonfocus=%u lr=%u ll=%u pd=%u pu=%u", (page[0x00] >> 4) & 1,
                  page[0x02] | page[0x03] << 8, page[0x04] | page[0x05] << 8, page[0x06] | page[0x07] << 8,
                  page[0x08] | page[0x09] << 8, page[0x0A] | page[0x0B] << 8, page[0x0C] | page[0x0D] << 8);

          uint8_t st = 0, pwrCtrl = 0, pwrConf = 0, accConf = 0, mapFeat = 0, ioCtrl = 0;
          rd(0x21, &st, 1);
          rd(0x40, &accConf, 1);
          rd(0x53, &ioCtrl, 1);
          rd(0x56, &mapFeat, 1);
          rd(0x7C, &pwrConf, 1);
          rd(0x7D, &pwrCtrl, 1);
          LOG_INF(
              "IMU",
              "INTERNAL_STATUS=0x%02X ACC_CONF=0x%02X INT1_IO=0x%02X MAP_FEAT=0x%02X PWR_CONF=0x%02X PWR_CTRL=0x%02X",
              st, accConf, ioCtrl, mapFeat, pwrConf, pwrCtrl);
          LOG_INF(
              "IMU",
              "WRIST armed for 120 s -- pick the device up into a reading pose whenever you are ready, a few times.");

          // Long window, change-triggered logging: a fixed 20 s window kept
          // missing the pickup entirely, and a full 600-sample dump is unreadable.
          // Prints on any event, on any axis moving more than 80 mg, and on a 5 s
          // heartbeat -- plus the most-tilted frame of the whole run, which is
          // what pins the in-plane axis signs.
          constexpr int WRIST_SAMPLES = 600;
          PaperMonoBoard::setMotionWake(true);
          unsigned hits = 0;
          int lastX = 0, lastY = 0, lastZ = 0;
          int peakX = 0, peakY = 0, peakZ = 1000;  // frame with the smallest |az|
          for (int i = 0; i < WRIST_SAMPLES; ++i) {
            // The loop blocks the main loop for the whole window; without this
            // the inactivity timer sees the gap on the way out and sleeps.
            activityManager.noteUserInteraction();
            uint8_t acc[6] = {};
            uint8_t s0 = 0, gpioIn = 0, gpioIrq = 0;
            rd(0x0C, acc, sizeof(acc));
            rd(0x1C, &s0, 1);
            freeink::m5pm1::readReg(freeink::m5pm1::REG_GPIO_IN, &gpioIn);
            // INT_LATCH is off, so INT1 and INT_STATUS_0 pulse for about one
            // feature-engine cycle and a 200 ms poll misses most of them. The
            // PMIC latches the edge in hardware, which is also exactly what has
            // to happen for a wake, so read and clear that instead of trusting
            // the live pin level.
            freeink::m5pm1::readReg(freeink::m5pm1::REG_IRQ_STATUS_GPIO, &gpioIrq);
            const bool edge = (gpioIrq >> 4) & 1;
            if (edge) freeink::m5pm1::clearGpioIrq();
            const int mx = static_cast<int16_t>(acc[0] | acc[1] << 8) * 1000L / 16384;
            const int my = static_cast<int16_t>(acc[2] | acc[3] << 8) * 1000L / 16384;
            const int mz = static_cast<int16_t>(acc[4] | acc[5] << 8) * 1000L / 16384;
            const bool wrist = (s0 >> 3) & 1;  // INT_STATUS_0 bit3 = wrist wear wake-up
            const bool g4 = (gpioIn >> 4) & 1;
            if (edge) ++hits;
            if (abs(mz) < abs(peakZ)) {
              peakX = mx;
              peakY = my;
              peakZ = mz;
            }
            const bool moved = abs(mx - lastX) > 80 || abs(my - lastY) > 80 || abs(mz - lastZ) > 80;
            if (edge || wrist || !g4 || moved || (i % 25) == 0) {
              // Wearable frame per the remap: (X_w, Y_w, Z_w) = (-Y, -X, -Z).
              // The tilt limits are sines, so in mg they compare directly:
              // pd 179/2048 = 87, pu 1978/2048 = 966, lr/ll 1024/2048 = 500.
              const int wx = -my, wy = -mx, wz = -mz;
              const bool inWin = wz > 0 && wy >= -87 && wy <= 966 && abs(wx) <= 500;
              LOG_INF("IMU", "t=%3d |%5dmg %5dmg %5dmg| w=(%5d %5d %5d) win=%d EDGE=%d wrist=%d G4=%d st0=0x%02X", i,
                      mx, my, mz, wx, wy, wz, inWin ? 1 : 0, edge ? 1 : 0, wrist ? 1 : 0, g4 ? 1 : 0, s0);
            }
            lastX = mx;
            lastY = my;
            lastZ = mz;
            delay(200);
          }
          PaperMonoBoard::setMotionWake(false);
          halTiltSensor.deepSleep();
          LOG_INF("IMU", "WRIST done: %u latched PMIC edges over %d samples; most-tilted frame |%dmg %dmg %dmg|", hits,
                  WRIST_SAMPLES, peakX, peakY, peakZ);
        }
      } else if (cmd == "INTPROBE") {
        // Catch a phantom INT1 assertion without spending a power cycle.
        // Everything is unmapped from the pin (MAP_FEAT = MAP_DATA = 0) and
        // INT_LATCH is turned ON, so any assertion -- however brief -- sticks
        // and shows up as a stable G4 = 0 on the PMIC side. Runs the
        // accelerometer for 6 s, then powers it down and watches 6 s more:
        // the sleep matrix says the edge needs the accel running, so G4 should
        // latch low in the first half and stay quiet in the second.
        activityManager.noteUserInteraction();
        const uint8_t a = BoardConfig::ACTIVE.sensors.imuAddr;
        auto wr = [a](uint8_t reg, uint8_t val) {
          Wire.beginTransmission(a);
          Wire.write(reg);
          Wire.write(val);
          Wire.endTransmission();
        };
        if (!halTiltSensor.armMotionWake()) {
          LOG_ERR("IMU", "INTPROBE: armMotionWake failed");
        } else {
          wr(0x56, 0x00);  // INT1_MAP_FEAT: nothing mapped
          wr(0x58, 0x00);  // INT_MAP_DATA: no drdy / FIFO
          wr(0x55, 0x01);  // INT_LATCH: latched, so a us-wide pulse persists
          uint8_t st[2] = {};
          Wire.beginTransmission(a);
          Wire.write(0x1C);
          if (Wire.endTransmission(false) == 0 && Wire.requestFrom(a, static_cast<uint8_t>(2)) == 2) {
            st[0] = Wire.read();
            st[1] = Wire.read();
          }
          LOG_INF("IMU", "INTPROBE: latched, nothing mapped; accel ON for 6 s (leave the device still)");
          for (int i = 0; i < 24; ++i) {
            if (i == 12) {
              wr(0x7D, 0x00);  // PWR_CTRL: accelerometer off
              LOG_INF("IMU", "INTPROBE: accel OFF");
            }
            uint8_t gpioIn = 0;
            freeink::m5pm1::readReg(freeink::m5pm1::REG_GPIO_IN, &gpioIn);
            uint8_t s0 = 0;
            Wire.beginTransmission(a);
            Wire.write(0x1C);
            if (Wire.endTransmission(false) == 0 && Wire.requestFrom(a, static_cast<uint8_t>(1)) == 1) s0 = Wire.read();
            LOG_INF("IMU", "INTPROBE t=%d accel=%d G4=%d INT_STATUS0=0x%02X", i, i < 12, (gpioIn >> 4) & 1, s0);
            delay(500);
          }
          PaperMonoBoard::setMotionWake(false);
          halTiltSensor.deepSleep();
          LOG_INF("IMU", "INTPROBE done");
        }
      } else if (cmd == "IMUTEST") {
        // Raise-to-wake chain probe: arm exactly the sleep-time configuration,
        // then sample the BMI270's live INT status and the PM1's GPIO input
        // for 8 s while the user shakes / rests the device. INT1 is active-LOW,
        // so G4=1 is at rest and G4=0 is motion. Shows precisely where the
        // chain breaks: feature not firing (anymot stays 0), INT1 not reaching
        // the PMIC (anymot 1 but G4 stays 1), or PMIC-side wake.
        activityManager.noteUserInteraction();
        {
          // Snapshot INT_MAP_DATA *before* arming. armMotionWake() now clears
          // it, so this is the only place the config blob's default is visible.
          // A non-zero bit0..2 here is the spurious-wake source: the data path
          // pulsing INT1 at the sample rate.
          uint8_t pre = 0;
          bool preOk = false;
          const uint8_t a = BoardConfig::ACTIVE.sensors.imuAddr;
          // Advanced power save must be off before the BMI270 answers register
          // reads reliably (Bosch API rule -- armMotionWake does the same). Read
          // it cold first, then again with APS disabled: a difference tells us
          // the cold read was garbage rather than a real register value.
          uint8_t cold = 0;
          Wire.beginTransmission(a);
          Wire.write(0x58);
          if (Wire.endTransmission(false) == 0 && Wire.requestFrom(a, static_cast<uint8_t>(1)) == 1) cold = Wire.read();
          Wire.beginTransmission(a);
          Wire.write(0x7C);  // PWR_CONF
          Wire.write(static_cast<uint8_t>(0x00));
          Wire.endTransmission();
          delayMicroseconds(500);
          Wire.beginTransmission(a);
          Wire.write(0x58);
          if (Wire.endTransmission(false) == 0 && Wire.requestFrom(a, static_cast<uint8_t>(1)) == 1) {
            pre = Wire.read();
            preOk = true;
          }
          LOG_INF("IMU", "INT_MAP_DATA cold=0x%02X, APS-off read=0x%02X ok=%d (drdy_int1=%d fwm_int1=%d ffull_int1=%d)",
                  cold, pre, preOk ? 1 : 0, (pre >> 2) & 1, (pre >> 1) & 1, pre & 1);
        }
        if (halTiltSensor.armMotionWake()) {
          const bool pm1Ok = PaperMonoBoard::setMotionWake(true);
          uint8_t wakeEn = 0;
          uint8_t wakeCfg = 0;
          freeink::m5pm1::readReg(freeink::m5pm1::REG_GPIO_WAKE_EN, &wakeEn);
          freeink::m5pm1::readReg(freeink::m5pm1::REG_GPIO_WAKE_CFG, &wakeCfg);
          LOG_INF("IMU", "Armed: pm1Ok=%d WAKE_EN=0x%02X WAKE_CFG=0x%02X (expect G4 bit: 0x10)", pm1Ok, wakeEn,
                  wakeCfg);
          const uint8_t imuAddr = BoardConfig::ACTIVE.sensors.imuAddr;
          // Read the whole INT block back. Every conclusion drawn from the sleep
          // experiments assumes these writes landed; confirm rather than assume.
          // INT_MAP_DATA (0x58) matters most: we never write it, so a stale
          // drdy/FIFO mapping there would pulse INT1 at the sample rate.
          uint8_t intRegs[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
          Wire.beginTransmission(imuAddr);
          Wire.write(0x53);  // INT1_IO_CTRL .. INT_MAP_DATA
          if (Wire.endTransmission(false) == 0 && Wire.requestFrom(imuAddr, static_cast<uint8_t>(6)) == 6) {
            for (uint8_t& r : intRegs) r = Wire.read();
          }
          LOG_INF("IMU",
                  "INT1_IO_CTRL=0x%02X(want 0x0C) LATCH=0x%02X MAP_FEAT=0x%02X(want 0x40) MAP_DATA=0x%02X(want 0)",
                  intRegs[0], intRegs[2], intRegs[3], intRegs[5]);
          for (int i = 0; i < 16; ++i) {
            uint8_t intStatus = 0;
            Wire.beginTransmission(imuAddr);
            Wire.write(0x1C);  // BMI270 INT_STATUS_0; bit6 = any-motion
            if (Wire.endTransmission(false) == 0 && Wire.requestFrom(imuAddr, static_cast<uint8_t>(1)) == 1) {
              intStatus = Wire.read();
            }
            uint8_t gpioIn = 0;
            freeink::m5pm1::readReg(freeink::m5pm1::REG_GPIO_IN, &gpioIn);
            LOG_INF("IMU", "INT_STATUS0=0x%02X anymot=%d | PM1 GPIO_IN=0x%02X G4=%d (0=motion)", intStatus,
                    (intStatus >> 6) & 1, gpioIn, (gpioIn >> 4) & 1);
            delay(500);
          }
          PaperMonoBoard::setMotionWake(false);
          halTiltSensor.deepSleep();  // loop() re-wakes it if tilt/face-down is on
          LOG_INF("IMU", "IMUTEST done; motion wake disarmed");
        } else {
          LOG_ERR("IMU", "IMUTEST: armMotionWake failed");
        }
#endif  // FREEINK_SLEEP_LAB
#endif
#if FREEINK_DEVICE_PAPERMONO && FREEINK_WAVEFORM_LAB
      } else if (cmd == "WAVE" || cmd.startsWith("WAVE ")) {
        String waveArgs = cmd.substring(4);
        waveArgs.trim();
        const WaveformLab::CommandResult waveResult = WaveformLab::handleCommand(waveArgs);
        if (waveResult.requestRedraw) {
          activityManager.noteUserInteraction();
          activityManager.requestUpdate();
        }
#endif
      } else if (cmd == "BATTERY") {
        const BatteryMonitor::Status status = BatteryMonitor().readStatus();
        logSerial.printf(
            "BATTERY supported=%d percent=%d:%u mv=%d:%u ext=%d:%d charging=%d:%d vin=%ld usb=%ld src=%d\n",
            status.supported, status.percentageKnown, status.percentage, status.millivoltsKnown, status.millivolts,
            status.externalPowerKnown, status.externalPowerKnown ? status.externalPower : false, status.chargingKnown,
            status.chargingKnown ? status.charging : false, static_cast<long>(status.pm1VinMv),
            static_cast<long>(status.pm1VinOutMv), status.pm1PowerSource);
      }
    }
  }

  // Check for any user activity (button press or release) or active background work
  static unsigned long lastActivityTime = millis();
  if (gpio.wasAnyPressed() || gpio.wasAnyReleased() || gpio.wasTouchActivity() || halTiltSensor.hadActivity() ||
      activityManager.preventAutoSleep()) {
    lastActivityTime = millis();         // Reset inactivity timer
    powerManager.setPowerSaving(false);  // Restore normal CPU frequency on user activity
  }

  static bool screenshotButtonsReleased = true;
  static bool screenshotComboActive = false;
#if FREEINK_DEVICE_PAPERMONO
  // The board hook emits POWER only after the PMIC has classified and released
  // a short click. Act on that edge immediately; a physical long hold never
  // reaches this path and remains reserved for PMIC download mode.
  if (millis() >= allowSleepAt && gpio.wasPressed(HalGPIO::BTN_POWER)) {
    enterDeepSleep();
    return;
  }
#endif
  if (gpio.isPressed(HalGPIO::BTN_POWER) && gpio.isPressed(HalGPIO::BTN_DOWN)) {
    screenshotComboActive = true;
    if (screenshotButtonsReleased) {
      screenshotButtonsReleased = false;
      {
        RenderLock lock;
        ScreenshotUtil::takeScreenshot(renderer);
      }
    }
    return;
  }
  if (screenshotComboActive) {
    if (gpio.isPressed(HalGPIO::BTN_POWER)) return;
    if (gpio.wasReleased(HalGPIO::BTN_POWER)) {
      screenshotButtonsReleased = true;
      screenshotComboActive = false;
      return;
    }
    screenshotButtonsReleased = true;
    screenshotComboActive = false;
  }

  const unsigned long sleepTimeoutMs =
#if FREEINK_DEVICE_PAPERMONO
      debugSleepTimeoutMs != 0 ? debugSleepTimeoutMs :
#endif
                               SETTINGS.getSleepTimeoutMs();
  if (sleepTimeoutMs > 0 && millis() - lastActivityTime >= sleepTimeoutMs) {
    LOG_DBG("SLP", "Auto-sleep triggered after %lu ms of inactivity", sleepTimeoutMs);
    enterDeepSleep(true);
    // This should never be hit as `enterDeepSleep` calls esp_deep_sleep_start
    return;
  }

  // Face-down auto-sleep: the IMU watch (HalTiltSensor) times how long the
  // device has been lying screen-down; past the configured delay, sleep now.
  const uint32_t faceDownSleepMs = SETTINGS.getFaceDownSleepMs();
  if (faceDownSleepMs > 0 && millis() >= allowSleepAt && halTiltSensor.faceDownForMs() >= faceDownSleepMs) {
    LOG_INF("SLP", "Face-down for %lu ms; sleeping", static_cast<unsigned long>(faceDownSleepMs));
    enterDeepSleep(true);
    return;
  }

#if !FREEINK_DEVICE_PAPERMONO
  if (millis() >= allowSleepAt && gpio.isPressed(HalGPIO::BTN_POWER) &&
      gpio.getPowerButtonHeldTime() > SETTINGS.getPowerButtonDuration()) {
    // If the screenshot combination is potentially being pressed, don't sleep
    if (gpio.isPressed(HalGPIO::BTN_DOWN)) {
      return;
    }
    enterDeepSleep();
    // This should never be hit as `enterDeepSleep` calls esp_deep_sleep_start
    return;
  }
#endif

  // Refresh screen when power button is short-pressed with FORCE_REFRESH setting.
  if (SETTINGS.shortPwrBtn == CrossPointSettings::SHORT_PWRBTN::FORCE_REFRESH &&
      mappedInputManager.wasReleased(MappedInputManager::Button::Power)) {
    LOG_DBG("MAIN", "Manual screen refresh triggered");
    if (!activityManager.handleForcedRefresh()) {
      RenderLock lock;
      renderer.displayBuffer(HalDisplay::HALF_REFRESH);
    }
  }

  // Refresh the battery icon from the same PMIC source used to draw it. Paper
  // Mono's USB GPIO does not track PM1 VIN after unplug, so relying on the old
  // GPIO edge leaves the charging badge latched indefinitely.
#if FREEINK_DEVICE_PAPERMONO
  static bool chargingStateInitialized = false;
  static bool lastChargingState = false;
  static unsigned long lastChargingPollMs = 0;
  const unsigned long chargingNow = millis();
  if (lastChargingPollMs == 0 || chargingNow - lastChargingPollMs >= HalPowerManager::BATTERY_POLL_MS) {
    lastChargingPollMs = chargingNow;
    const bool charging = powerManager.isCharging();
    if (chargingStateInitialized && charging != lastChargingState) {
      LOG_INF("PWR", "External power changed: charging=%d", charging);
      activityManager.requestUpdate();
    }
    lastChargingState = charging;
    chargingStateInitialized = true;
  }
#else
  if (gpio.wasUsbStateChanged()) activityManager.requestUpdate();
#endif

  const unsigned long activityStartTime = millis();
  activityManager.loop();
#if FREEINK_DEVICE_PAPERMONO
  // The Activity has now consumed this iteration's press/release snapshot and
  // queued any resulting frame. Let the controller worker re-evaluate the
  // foreground queue before it considers background maintenance or power-off.
  activityManager.finishUserInteractionDispatch();
#endif
  const unsigned long activityDuration = millis() - activityStartTime;

  const unsigned long loopDuration = millis() - loopStartTime;
  if (loopDuration > maxLoopDuration) {
    maxLoopDuration = loopDuration;
    if (maxLoopDuration > 50) {
      LOG_DBG("LOOP", "New max loop duration: %lu ms (activity: %lu ms)", maxLoopDuration, activityDuration);
    }
  }

  // Add delay at the end of the loop to prevent tight spinning
  // When an activity requests skip loop delay (e.g., webserver running), use yield() for faster response
  // Otherwise, use longer delay to save power
  if (activityManager.skipLoopDelay()) {
    powerManager.setPowerSaving(false);  // Make sure we're at full performance when skipLoopDelay is requested
    yield();                             // Give FreeRTOS a chance to run tasks, but return immediately
  } else {
    unsigned long tailWaitMs;
    if (millis() - lastActivityTime >= HalPowerManager::IDLE_POWER_SAVING_MS) {
      // If we've been inactive for a while, increase the delay to save power
      powerManager.setPowerSaving(true);  // Lower CPU frequency after extended inactivity
      tailWaitMs = 50;
    } else {
      // Short delay to prevent tight loop while still being responsive
      tailWaitMs = 10;
    }
#if FREEINK_DEVICE_PAPERMONO
    // Same sleep budget, but an input-edge ISR collapses it immediately, so a
    // press is sampled on the next pass instead of up to a full tick later.
    if (inputWakeSemaphore) {
      xSemaphoreTake(inputWakeSemaphore, pdMS_TO_TICKS(tailWaitMs));
    } else {
      delay(tailWaitMs);
    }
#else
    delay(tailWaitMs);
#endif
  }
}
