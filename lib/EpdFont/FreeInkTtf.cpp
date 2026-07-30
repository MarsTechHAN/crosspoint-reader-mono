// Reuse FreeInkBook's proven stb_truetype engine without linking the rest of
// FreeInkBook (which carries private Expat/miniz copies that conflict with the
// reader's existing EPUB stack).
#include "../../freeink-sdk/libs/book/FreeInkBook/src/render/TtfFont.cpp"
