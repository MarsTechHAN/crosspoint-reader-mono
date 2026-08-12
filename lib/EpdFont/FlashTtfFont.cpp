#include "FlashTtfFont.h"

#include <Logging.h>
#include <esp_heap_caps.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

constexpr char FONT_MAGIC[8] = {'C', 'P', 'F', 'T', 'T', 'F', '1', '\0'};
constexpr uint32_t FONT_PARTITION_SUBTYPE = 0x40;
constexpr float DISPLAY_DPI = 150.0f;

struct FontPartitionHeader {
  char magic[8];
  uint32_t fontSize;
  uint32_t crc32;
};

static_assert(sizeof(FontPartitionHeader) == 16);

uint16_t pointToPixelSize(uint8_t pointSize) {
  return static_cast<uint16_t>(std::lround(pointSize * DISPLAY_DPI / 72.0f));
}

// Synthetic bold strength, as a fraction of 256 applied to the dilated pixels.
//
// The emboldener spreads each glyph one pixel right and one pixel up, but at
// this weight rather than at full coverage. That matters because GfxRenderer
// turns coverage into ink at 50% (AA_GRAY_HIGH): at 160/256 a neighbouring
// pixel has to be ~80% covered before the spread reaches the glass, so stroke
// interiors gain a solid pixel while antialiased edges only gain a little gray.
// The result is about one pixel of extra weight instead of the two a plain
// dilation would give, which at 29-38 px Han is the difference between "a bit
// bolder" and "filled in".
constexpr int EMBOLDEN_STRENGTH_256 = 160;

}  // namespace

FlashTtfFont::FlashTtfFont() {
  for (size_t i = 0; i < faces_.size(); ++i) {
    faces_[i].owner = this;
    faces_[i].pixelSize = pointToPixelSize(POINT_SIZES[i]);
  }
  for (size_t i = 0; i < boldFaces_.size(); ++i) {
    boldFaces_[i].owner = this;
    boldFaces_[i].pixelSize = pointToPixelSize(BOLD_POINT_SIZES[i]);
    boldFaces_[i].embolden = true;
  }
}

FlashTtfFont::~FlashTtfFont() {
  for (auto& face : boldFaces_) heap_caps_free(face.scratch);
  if (mapped_) esp_partition_munmap(mmapHandle_);
  if (arenaMemory_) heap_caps_free(arenaMemory_);
}

bool FlashTtfFont::begin() {
  if (ready_) return true;

  const auto subtype = static_cast<esp_partition_subtype_t>(FONT_PARTITION_SUBTYPE);
  const esp_partition_t* partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, subtype, "cjkfont");
  if (!partition) {
    LOG_ERR("CJK", "Built-in CJK font partition not found");
    return false;
  }

  FontPartitionHeader header{};
  if (esp_partition_read(partition, 0, &header, sizeof(header)) != ESP_OK ||
      std::memcmp(header.magic, FONT_MAGIC, sizeof(FONT_MAGIC)) != 0 || header.fontSize < 12 ||
      header.fontSize > partition->size - sizeof(header)) {
    LOG_ERR("CJK", "Built-in CJK font header invalid");
    return false;
  }

  const void* mapped = nullptr;
  const size_t mappedSize = sizeof(header) + header.fontSize;
  if (esp_partition_mmap(partition, 0, mappedSize, ESP_PARTITION_MMAP_DATA, &mapped, &mmapHandle_) != ESP_OK) {
    LOG_ERR("CJK", "Could not mmap %u-byte CJK font", static_cast<unsigned>(header.fontSize));
    return false;
  }
  mapped_ = true;
  mappedFont_ = static_cast<const uint8_t*>(mapped) + sizeof(header);

  arenaMemory_ = heap_caps_malloc(GLYPH_ARENA_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!arenaMemory_) {
    LOG_ERR("CJK", "Could not allocate %u-byte glyph cache", static_cast<unsigned>(GLYPH_ARENA_SIZE));
    return false;
  }
  arena_.init(arenaMemory_, GLYPH_ARENA_SIZE);
  if (!ttf_.init(mappedFont_, header.fontSize, arena_)) {
    LOG_ERR("CJK", "LXGW WenKai TrueType initialization failed");
    return false;
  }

  // Bold faces share their regular twin's line height, ascent and advances --
  // only the coverage map differs. Callers that size a row or centre a label
  // from the fallback's metrics therefore lay out identically either way.
  for (auto& face : faces_) initFaceMetrics(face);
  for (auto& face : boldFaces_) initFaceMetrics(face);

  ready_ = true;
  LOG_INF("CJK", "LXGW WenKai GB2312 ready: %u bytes, %u-byte PSRAM cache, crc=%08lx",
          static_cast<unsigned>(header.fontSize), static_cast<unsigned>(GLYPH_ARENA_SIZE),
          static_cast<unsigned long>(header.crc32));
  return true;
}

void FlashTtfFont::initFaceMetrics(Face& face) {
  face.data.advanceY = static_cast<uint8_t>(std::min<int>(255, ttf_.lineHeight(face.pixelSize)));
  face.data.ascender = ttf_.ascent(face.pixelSize);
  face.data.descender = face.data.ascender - face.data.advanceY;
  face.data.is2Bit = false;
  face.data.glyphMissHandler = loadGlyph;
  face.data.glyphMissCtx = &face;
  face.data.coverageHandler = covers;
  face.data.glyphBitmapHandler = loadBitmap;
  face.data.glyphBitmapBpp = 8;
  face.data.kerningHandler = kern;
  face.data.advanceHandler = advance;
}

const EpdFont* FlashTtfFont::font(uint8_t pointSize) const {
  for (size_t i = 0; i < POINT_SIZES.size(); ++i) {
    if (POINT_SIZES[i] == pointSize) return &faces_[i].font;
  }
  return nullptr;
}

const EpdFont* FlashTtfFont::boldFont(uint8_t pointSize) const {
  for (size_t i = 0; i < BOLD_POINT_SIZES.size(); ++i) {
    if (BOLD_POINT_SIZES[i] == pointSize) return &boldFaces_[i].font;
  }
  return nullptr;
}

const uint8_t* FlashTtfFont::emboldenInto(Face& face, const freeink::book::GlyphBitmap& src) {
  const int sw = src.width;
  const int sh = src.height;
  const int dw = sw + 1;
  const int dh = sh + 1;
  const size_t needed = static_cast<size_t>(dw) * static_cast<size_t>(dh);
  if (needed > face.scratchCapacity) {
    // PSRAM: this is cold-path, one buffer per UI size, and it grows to the
    // largest glyph the face has drawn and then stops.
    auto* grown = static_cast<uint8_t*>(heap_caps_realloc(face.scratch, needed, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!grown) return nullptr;
    face.scratch = grown;
    face.scratchCapacity = needed;
  }

  // Dilate one pixel right and one pixel up, at EMBOLDEN_STRENGTH_256. Up
  // rather than down so the glyph keeps its footing on the baseline and the
  // added weight goes into the x-height, where a Han glyph's stroke pitch is
  // tightest. The destination is one pixel taller, so the caller must also
  // raise `top` by one to keep the bitmap over the same baseline.
  const uint8_t* s = src.pixels;
  uint8_t* d = face.scratch;
  for (int dy = 0; dy < dh; ++dy) {
    for (int dx = 0; dx < dw; ++dx) {
      const int here = (dx < sw && dy < sh) ? s[dy * sw + dx] : 0;
      int spread = 0;
      if (dx > 0 && dy < sh) spread = std::max(spread, static_cast<int>(s[dy * sw + dx - 1]));
      if (dy > 0 && dx < sw) spread = std::max(spread, static_cast<int>(s[(dy - 1) * sw + dx]));
      if (dx > 0 && dy > 0) spread = std::max(spread, static_cast<int>(s[(dy - 1) * sw + dx - 1]));
      d[dy * dw + dx] = static_cast<uint8_t>(std::max(here, (spread * EMBOLDEN_STRENGTH_256) >> 8));
    }
  }
  return d;
}

const EpdGlyph* FlashTtfFont::loadGlyph(void* ctx, uint32_t codepoint) {
  auto& face = *static_cast<Face*>(ctx);
  if (!face.owner->ttf_.hasGlyph(codepoint)) return nullptr;
  const freeink::book::GlyphBitmap* bitmap = face.owner->ttf_.rasterize(codepoint, face.pixelSize);
  if (!bitmap) return nullptr;

  // The emboldened bitmap is a pixel wider and a pixel taller, and sits a pixel
  // higher. The advance is deliberately left alone: Han side bearings absorb
  // one pixel of ink, and matching the regular face's advance means bold and
  // regular measure identically, so nothing that was laid out against one can
  // overflow when drawn with the other.
  const int grow = face.embolden ? 1 : 0;
  const int width = bitmap->width + grow;
  const int height = bitmap->height + grow;
  if (width > UINT8_MAX || height > UINT8_MAX) return nullptr;

  face.glyph.width = static_cast<uint8_t>(width);
  face.glyph.height = static_cast<uint8_t>(height);
  face.glyph.advanceX = static_cast<uint16_t>(std::max<int>(0, bitmap->advance) << fp4::FRAC_BITS);
  face.glyph.left = bitmap->xoff;
  face.glyph.top = static_cast<int16_t>(-bitmap->yoff + grow);
  face.glyph.dataLength = static_cast<uint16_t>(width * height);
  face.glyph.dataOffset = codepoint;
  return &face.glyph;
}

const uint8_t* FlashTtfFont::loadBitmap(void* ctx, const EpdGlyph* glyph) {
  auto& face = *static_cast<Face*>(ctx);
  const freeink::book::GlyphBitmap* bitmap = face.owner->ttf_.rasterize(glyph->dataOffset, face.pixelSize);
  if (!bitmap) return nullptr;
  if (!face.embolden) return bitmap->pixels;
  return emboldenInto(face, *bitmap);
}

bool FlashTtfFont::covers(void* ctx, uint32_t codepoint) {
  auto& face = *static_cast<Face*>(ctx);
  return face.owner->ttf_.hasGlyph(codepoint);
}

int8_t FlashTtfFont::kern(void* ctx, uint32_t leftCodepoint, uint32_t rightCodepoint) {
  auto& face = *static_cast<Face*>(ctx);
  const int fixed = face.owner->ttf_.kerning(leftCodepoint, rightCodepoint, face.pixelSize, freeink::book::StyleNone)
                    << fp4::FRAC_BITS;
  return static_cast<int8_t>(std::clamp(fixed, static_cast<int>(INT8_MIN), static_cast<int>(INT8_MAX)));
}

uint16_t FlashTtfFont::advance(void* ctx, uint32_t codepoint) {
  auto& face = *static_cast<Face*>(ctx);
  const int pixels = face.owner->ttf_.advance(codepoint, face.pixelSize, freeink::book::StyleNone);
  return static_cast<uint16_t>(std::max(0, pixels) << fp4::FRAC_BITS);
}
