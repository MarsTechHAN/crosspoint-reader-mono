#pragma once

#include <BookArena.h>
#include <EpdFont.h>
#include <esp_partition.h>
#include <render/TtfFont.h>

#include <array>
#include <cstddef>
#include <cstdint>

class FlashTtfFont {
 public:
  static constexpr size_t FACE_COUNT = 8;
  static constexpr std::array<uint8_t, FACE_COUNT> POINT_SIZES = {8, 10, 12, 14, 16, 18, 20, 22};

  // LXGW WenKai ships one weight, and the UI needs a heavier one: at UI sizes a
  // Han glyph's strokes land near the panel's resolving limit, and e-ink's
  // particle spread rounds them off further, so UI labels read washed out next
  // to the Latin faces beside them. These faces synthesize the missing weight
  // (see emboldenInto) at the three sizes the UI falls back to. Reader body
  // text is deliberately excluded — it is larger, user-scaled, and a whole page
  // of synthetic bold is fatiguing — which is why these need their own face and
  // font id rather than a flag on the shared ones.
  static constexpr size_t BOLD_FACE_COUNT = 3;
  static constexpr std::array<uint8_t, BOLD_FACE_COUNT> BOLD_POINT_SIZES = {14, 16, 18};

  FlashTtfFont();
  ~FlashTtfFont();

  FlashTtfFont(const FlashTtfFont&) = delete;
  FlashTtfFont& operator=(const FlashTtfFont&) = delete;

  bool begin();
  bool ready() const { return ready_; }
  const EpdFont* font(uint8_t pointSize) const;
  // Synthetic-bold face for one of BOLD_POINT_SIZES; nullptr for any other size.
  const EpdFont* boldFont(uint8_t pointSize) const;

 private:
  struct Face {
    EpdFontData data{};
    EpdFont font;
    EpdGlyph glyph{};
    FlashTtfFont* owner = nullptr;
    uint16_t pixelSize = 0;
    // Synthetic bold. `scratch` holds the emboldened copy of whichever glyph
    // was rasterized last; it is per-face because the bitmap handler's contract
    // is one live glyph at a time (GfxRenderer::getGlyphBitmap), and sizing it
    // to the largest glyph seen so far keeps it off the hot path.
    bool embolden = false;
    uint8_t* scratch = nullptr;
    size_t scratchCapacity = 0;

    Face() : font(&data) {}
  };

  static const EpdGlyph* loadGlyph(void* ctx, uint32_t codepoint);
  static const uint8_t* loadBitmap(void* ctx, const EpdGlyph* glyph);
  static bool covers(void* ctx, uint32_t codepoint);
  static int8_t kern(void* ctx, uint32_t leftCodepoint, uint32_t rightCodepoint);
  static uint16_t advance(void* ctx, uint32_t codepoint);

  static constexpr size_t GLYPH_ARENA_SIZE = 512 * 1024;

  void initFaceMetrics(Face& face);
  // Writes the emboldened form of `src` into `face.scratch` and returns it, or
  // nullptr if the scratch buffer could not be grown.
  static const uint8_t* emboldenInto(Face& face, const freeink::book::GlyphBitmap& src);

  std::array<Face, FACE_COUNT> faces_{};
  std::array<Face, BOLD_FACE_COUNT> boldFaces_{};
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

// Synthetic-bold ids, one per BOLD_POINT_SIZES entry. Distinct from the regular
// ids above because the two faces coexist: the UI draws Han in the bold face at
// the same nominal size the reader draws it in the regular one.
constexpr int BUILTIN_CJK_BOLD_14_FONT_ID = -190800114;
constexpr int BUILTIN_CJK_BOLD_16_FONT_ID = -190800116;
constexpr int BUILTIN_CJK_BOLD_18_FONT_ID = -190800118;

constexpr int builtinCjkBoldFontId(uint8_t pointSize) {
  switch (pointSize) {
    case 14:
      return BUILTIN_CJK_BOLD_14_FONT_ID;
    case 16:
      return BUILTIN_CJK_BOLD_16_FONT_ID;
    case 18:
      return BUILTIN_CJK_BOLD_18_FONT_ID;
    default:
      return 0;
  }
}

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
