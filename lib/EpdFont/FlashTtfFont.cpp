#include "FlashTtfFont.h"

#include <Logging.h>

#include <algorithm>
#include <cmath>
#include <cstring>

#include <esp_heap_caps.h>

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

}  // namespace

FlashTtfFont::FlashTtfFont() {
  for (size_t i = 0; i < faces_.size(); ++i) {
    faces_[i].owner = this;
    faces_[i].pixelSize = pointToPixelSize(POINT_SIZES[i]);
  }
}

FlashTtfFont::~FlashTtfFont() {
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

  for (auto& face : faces_) {
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

  ready_ = true;
  LOG_INF("CJK", "LXGW WenKai GB2312 ready: %u bytes, %u-byte PSRAM cache, crc=%08lx", 
          static_cast<unsigned>(header.fontSize), static_cast<unsigned>(GLYPH_ARENA_SIZE),
          static_cast<unsigned long>(header.crc32));
  return true;
}

const EpdFont* FlashTtfFont::font(uint8_t pointSize) const {
  for (size_t i = 0; i < POINT_SIZES.size(); ++i) {
    if (POINT_SIZES[i] == pointSize) return &faces_[i].font;
  }
  return nullptr;
}

const EpdGlyph* FlashTtfFont::loadGlyph(void* ctx, uint32_t codepoint) {
  auto& face = *static_cast<Face*>(ctx);
  if (!face.owner->ttf_.hasGlyph(codepoint)) return nullptr;
  const freeink::book::GlyphBitmap* bitmap = face.owner->ttf_.rasterize(codepoint, face.pixelSize);
  if (!bitmap || bitmap->width > UINT8_MAX || bitmap->height > UINT8_MAX) return nullptr;

  face.glyph.width = static_cast<uint8_t>(bitmap->width);
  face.glyph.height = static_cast<uint8_t>(bitmap->height);
  face.glyph.advanceX = static_cast<uint16_t>(std::max<int>(0, bitmap->advance) << fp4::FRAC_BITS);
  face.glyph.left = bitmap->xoff;
  face.glyph.top = -bitmap->yoff;
  face.glyph.dataLength = static_cast<uint16_t>(bitmap->width * bitmap->height);
  face.glyph.dataOffset = codepoint;
  return &face.glyph;
}

const uint8_t* FlashTtfFont::loadBitmap(void* ctx, const EpdGlyph* glyph) {
  auto& face = *static_cast<Face*>(ctx);
  const freeink::book::GlyphBitmap* bitmap = face.owner->ttf_.rasterize(glyph->dataOffset, face.pixelSize);
  return bitmap ? bitmap->pixels : nullptr;
}

bool FlashTtfFont::covers(void* ctx, uint32_t codepoint) {
  auto& face = *static_cast<Face*>(ctx);
  return face.owner->ttf_.hasGlyph(codepoint);
}

int8_t FlashTtfFont::kern(void* ctx, uint32_t leftCodepoint, uint32_t rightCodepoint) {
  auto& face = *static_cast<Face*>(ctx);
  const int fixed = face.owner->ttf_.kerning(leftCodepoint, rightCodepoint, face.pixelSize,
                                              freeink::book::StyleNone)
                    << fp4::FRAC_BITS;
  return static_cast<int8_t>(std::clamp(fixed, static_cast<int>(INT8_MIN), static_cast<int>(INT8_MAX)));
}

uint16_t FlashTtfFont::advance(void* ctx, uint32_t codepoint) {
  auto& face = *static_cast<Face*>(ctx);
  const int pixels = face.owner->ttf_.advance(codepoint, face.pixelSize, freeink::book::StyleNone);
  return static_cast<uint16_t>(std::max(0, pixels) << fp4::FRAC_BITS);
}
