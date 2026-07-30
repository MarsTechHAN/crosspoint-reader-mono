#pragma once

#include <BookArena.h>
#include <EpdFont.h>
#include <render/TtfFont.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include <esp_partition.h>

class FlashTtfFont {
 public:
  static constexpr size_t FACE_COUNT = 8;
  static constexpr std::array<uint8_t, FACE_COUNT> POINT_SIZES = {8, 10, 12, 14, 16, 18, 20, 22};

  FlashTtfFont();
  ~FlashTtfFont();

  FlashTtfFont(const FlashTtfFont&) = delete;
  FlashTtfFont& operator=(const FlashTtfFont&) = delete;

  bool begin();
  bool ready() const { return ready_; }
  const EpdFont* font(uint8_t pointSize) const;

 private:
  struct Face {
    EpdFontData data{};
    EpdFont font;
    EpdGlyph glyph{};
    FlashTtfFont* owner = nullptr;
    uint16_t pixelSize = 0;

    Face() : font(&data) {}
  };

  static const EpdGlyph* loadGlyph(void* ctx, uint32_t codepoint);
  static const uint8_t* loadBitmap(void* ctx, const EpdGlyph* glyph);
  static bool covers(void* ctx, uint32_t codepoint);
  static int8_t kern(void* ctx, uint32_t leftCodepoint, uint32_t rightCodepoint);
  static uint16_t advance(void* ctx, uint32_t codepoint);

  static constexpr size_t GLYPH_ARENA_SIZE = 512 * 1024;

  std::array<Face, FACE_COUNT> faces_{};
  freeink::book::Arena arena_{};
  freeink::book::TtfFont ttf_{};
  void* arenaMemory_ = nullptr;
  const uint8_t* mappedFont_ = nullptr;
  esp_partition_mmap_handle_t mmapHandle_ = 0;
  bool mapped_ = false;
  bool ready_ = false;
};

constexpr int BUILTIN_CJK_8_FONT_ID = -190800008;
constexpr int BUILTIN_CJK_10_FONT_ID = -190800010;
constexpr int BUILTIN_CJK_12_FONT_ID = -190800012;
constexpr int BUILTIN_CJK_14_FONT_ID = -190800014;
constexpr int BUILTIN_CJK_16_FONT_ID = -190800016;
constexpr int BUILTIN_CJK_18_FONT_ID = -190800018;
constexpr int BUILTIN_CJK_20_FONT_ID = -190800020;
constexpr int BUILTIN_CJK_22_FONT_ID = -190800022;

constexpr int builtinCjkFontId(uint8_t pointSize) {
  switch (pointSize) {
    case 8:
      return BUILTIN_CJK_8_FONT_ID;
    case 10:
      return BUILTIN_CJK_10_FONT_ID;
    case 12:
      return BUILTIN_CJK_12_FONT_ID;
    case 14:
      return BUILTIN_CJK_14_FONT_ID;
    case 16:
      return BUILTIN_CJK_16_FONT_ID;
    case 18:
      return BUILTIN_CJK_18_FONT_ID;
    case 20:
      return BUILTIN_CJK_20_FONT_ID;
    case 22:
      return BUILTIN_CJK_22_FONT_ID;
    default:
      return 0;
  }
}
