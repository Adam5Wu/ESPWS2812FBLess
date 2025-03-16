// Light-show pixel
#ifndef ZWESP8266_LSPIXEL
#define ZWESP8266_LSPIXEL

#include <cstdint>
#include <string.h>

#include "sdkconfig.h"

namespace zw_esp8266::lightshow {

using RGB8B_FLAT = uint8_t[3];

#define RGB8B_LAYOUT_MAP(OP) \
  OP(RGB888, r, g, b)        \
  OP(RBG888, r, b, g)        \
  OP(GRB888, g, r, b)        \
  OP(GBR888, g, b, r)        \
  OP(BRG888, b, r, g)        \
  OP(BGR888, b, g, r)

#define RGB8B_UNION_TYPE_FORWARD(name, c1, c2, c3) union name;
RGB8B_LAYOUT_MAP(RGB8B_UNION_TYPE_FORWARD)
#undef RGB8B_UNION_TYPE_FORWARD

#define RGB8B_ENUM_VAL(name, c1, c2, c3) name,
enum class RGB8BLayout : uint8_t { RGB8B_LAYOUT_MAP(RGB8B_ENUM_VAL) };
#undef RGB8B_ENUM_VAL

#define RGB8B_LAYOUT(c1, c2, c3)   \
  struct __attribute__((packed)) { \
    uint8_t c1;                    \
    uint8_t c2;                    \
    uint8_t c3;                    \
  };                               \
  RGB8B_FLAT u;

// Internal pixel is always 8-bit [R,G,B].
struct RGB8BPixel {
  union {
    RGB8B_LAYOUT(r, g, b)
  };

  // Statically produce a pixel of specific layout in `out`.
#define LAYOUT_TRANS_FORWARD(name, c1, c2, c3) inline void Transcribe(name& out);
  RGB8B_LAYOUT_MAP(LAYOUT_TRANS_FORWARD)
#undef LAYOUT_TRANS_FORWARD

  // Dynamically produce a pixel in `out` using the given `layout`.
  inline void Transcribe(RGB8BLayout layout, RGB8B_FLAT& out);

  static constexpr RGB8BPixel BLACK() { return {0x00, 0x00, 0x00}; }
  static constexpr RGB8BPixel RED() { return {0xFF, 0x00, 0x00}; }
  static constexpr RGB8BPixel GREEN() { return {0x00, 0xFF, 0x00}; }
  static constexpr RGB8BPixel BLUE() { return {0x00, 0x00, 0xFF}; }
  static constexpr RGB8BPixel YELLOW() { return {0xFF, 0xFF, 0x00}; }
  static constexpr RGB8BPixel CYAN() { return {0x00, 0xFF, 0xFF}; }
  static constexpr RGB8BPixel MAGENTA() { return {0xFF, 0x00, 0xFF}; }
  static constexpr RGB8BPixel WHITE() { return {0xFF, 0xFF, 0xFF}; }
};

#define RGB8B_UNION_TYPEDEF(name, c1, c2, c3)                \
  union name {                                               \
    RGB8B_LAYOUT(c1, c2, c3)                                 \
    inline name& operator=(const RGB8BPixel& in) {           \
      return c1 = in.c1, c2 = in.c2, c3 = in.c3, *this;      \
    }                                                        \
  };                                                         \
  inline bool operator==(const name& lhs, const name& rhs) { \
    uint32_t lhv = 0, rhv = 0;                               \
    reinterpret_cast<name&>(lhv) = lhs;                      \
    reinterpret_cast<name&>(rhv) = rhs;                      \
    return lhv == rhv;                                       \
  }                                                          \
  inline bool operator!=(const name& lhs, const name& rhs) { return !(lhs == rhs); }

RGB8B_LAYOUT_MAP(RGB8B_UNION_TYPEDEF)
#undef RGB8B_UNION_TYPEDEF

//---------------------------------------------
// Implementation of RGB8BPixel transcriptions
#define LAYOUT_TRANS_IMPL(name, c1, c2, c3) \
  inline void RGB8BPixel::Transcribe(name& out) { out = *this; }
RGB8B_LAYOUT_MAP(LAYOUT_TRANS_IMPL)
#undef LAYOUT_TRANS_IMPL

inline void RGB8BPixel::Transcribe(RGB8BLayout layout, RGB8B_FLAT& out) {
#define LAYOUT_DYN_TRANS(name, c1, c2, c3) \
  case RGB8BLayout::name:                  \
    reinterpret_cast<name&>(out) = *this;  \
    break;
  switch (layout) { RGB8B_LAYOUT_MAP(LAYOUT_DYN_TRANS) }
#undef LAYOUT_DYN_TRANS
}

//---------------------------------------------
// RGB8BPixel comparators
inline bool operator==(const RGB8BPixel& lhs, const RGB8BPixel& rhs) {
  return reinterpret_cast<const RGB888&>(lhs) == reinterpret_cast<const RGB888&>(rhs);
}
inline bool operator!=(const RGB8BPixel& lhs, const RGB8BPixel& rhs) { return !(lhs == rhs); }

#undef RGB8B_LAYOUT
#undef RGB8B_LAYOUT_MAP

inline constexpr RGB8BLayout RGB8BDefaultLayout =
#if defined(CONFIG_ESP2812FBLESS_PIXEL_RGB888)
    RGB8BLayout::RGB888;
#elif defined(CONFIG_ESP2812FBLESS_PIXEL_RBG888)
    RGB8BLayout::RBG888;
#elif defined(CONFIG_ESP2812FBLESS_PIXEL_GRB888)
    RGB8BLayout::GRB888;
#elif defined(CONFIG_ESP2812FBLESS_PIXEL_GBR888)
    RGB8BLayout::GBR888;
#elif defined(CONFIG_ESP2812FBLESS_PIXEL_BRG888)
    RGB8BLayout::BRG888;
#elif defined(CONFIG_ESP2812FBLESS_PIXEL_BGR888)
    RGB8BLayout::BGR888;
#else
    "Unrecognized pixel format configuration!";
#endif

// An RGB 8-bit pixel data with a 8-bit alpha channel
// The alpha channel specified the transparency, which allows
// the pixel to "blend" with the background.
// See `RGBA8BBlendPixel`.
struct RGBA8BPixel : RGB8BPixel {
  union {
    struct {
      uint8_t r;
      uint8_t g;
      uint8_t b;
      uint8_t a;
    };
    uint32_t data;
  };

  RGBA8BPixel() = default;
  RGBA8BPixel(uint8_t r8, uint8_t g8, uint8_t b8, uint8_t a8) : r(r8), g(g8), b(b8), a(a8) {}
  RGBA8BPixel(const RGB8BPixel& in, uint8_t alpha) : RGBA8BPixel(in.r, in.g, in.b, alpha) {}

  // Access the RGB data as `RGB8BPixel`, ignoring the alpha channel.
  operator const RGB8BPixel&() const { return *reinterpret_cast<const RGB8BPixel*>(this); }
  operator RGB8BPixel&() { return *reinterpret_cast<RGB8BPixel*>(this); }
};

// Stores a pre-computed RGB pixel for color blending.
// A color blend operation is:
//   SrcColor*Alpha/256 + DstColor*(256-Alpha)/256
// The source is pre-determined, but the destination is not.
// Hence half of the blending can be done ahead of time and
// cached for faster operation.
struct RGBA8BBlendPixel {
  union {
    struct {
      uint8_t r;   // Pre-multiplied R: Src.r*Alpha/256
      uint8_t g;   // Pre-multiplied G: Src.g*Alpha/256
      uint8_t b;   // Pre-multiplied B: Src.b*Alpha/256
      uint8_t ca;  // Complementary Alpha: 255-Alpha
    };
    uint32_t data;
  };

  RGBA8BBlendPixel() = default;
  RGBA8BBlendPixel(const RGBA8BPixel& in) { *this = in; }
  RGBA8BBlendPixel& operator=(const RGBA8BPixel& in) {
    return precompute(in.r, in.g, in.b, in.a), *this;
  }

  void precompute(uint8_t ir, uint8_t ig, uint8_t ib, uint8_t ia) {
    r = (uint8_t)(((uint32_t)ir * ia) >> 8);
    g = (uint8_t)(((uint32_t)ig * ia) >> 8);
    b = (uint8_t)(((uint32_t)ib * ia) >> 8);
    ca = UINT8_MAX - ia;
  }
  RGB8BPixel blend(const RGB8BPixel& bg) {
    return {(uint8_t)(r + (((uint32_t)bg.r * ca + bg.r) >> 8)),
            (uint8_t)(g + (((uint32_t)bg.g * ca + bg.g) >> 8)),
            (uint8_t)(b + (((uint32_t)bg.b * ca + bg.b) >> 8))};
  }
};

}  // namespace zw_esp8266::lightshow

#endif  // ZWESP8266_LSPIXEL