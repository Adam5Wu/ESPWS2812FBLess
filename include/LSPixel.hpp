// Light-show pixel
#ifndef ZWESP8266_LSPIXEL
#define ZWESP8266_LSPIXEL

#include <cstdint>
#include <string.h>

#include "sdkconfig.h"

// Whether to avoid division for alpha-blending
#define ALPHABLEND_WITH_DIV 0

namespace zw_esp8266::lightshow {

using RGB8BFlatType = uint8_t[3];

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
  RGB8BFlatType u8;

// Internal pixel is always 8-bit [R,G,B].
struct RGB8BPixel {
  using _this = RGB8BPixel;

  union {
    RGB8B_LAYOUT(r, g, b)
  };

  operator RGB888&() { return *reinterpret_cast<RGB888*>(this); }
  operator const RGB888&() const { return *reinterpret_cast<const RGB888*>(this); }

  // Statically produce a pixel of specific layout in `out`.
#define LAYOUT_TRANS_FORWARD(name, c1, c2, c3) inline void Transcribe(name& out);
  RGB8B_LAYOUT_MAP(LAYOUT_TRANS_FORWARD)
#undef LAYOUT_TRANS_FORWARD

  // Dynamically produce a pixel in `out` using the given `layout`.
  inline void Transcribe(RGB8BLayout layout, RGB8BFlatType& out);

  static constexpr _this BLACK() { return {0x00, 0x00, 0x00}; }
  static constexpr _this RED() { return {0xFF, 0x00, 0x00}; }
  static constexpr _this GREEN() { return {0x00, 0xFF, 0x00}; }
  static constexpr _this BLUE() { return {0x00, 0x00, 0xFF}; }
  static constexpr _this YELLOW() { return {0xFF, 0xFF, 0x00}; }
  static constexpr _this CYAN() { return {0x00, 0xFF, 0xFF}; }
  static constexpr _this MAGENTA() { return {0xFF, 0x00, 0xFF}; }
  static constexpr _this WHITE() { return {0xFF, 0xFF, 0xFF}; }
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
// RGB8BPixel Implementations

#define LAYOUT_TRANS_IMPL(name, c1, c2, c3) \
  inline void RGB8BPixel::Transcribe(name& out) { out = *this; }
RGB8B_LAYOUT_MAP(LAYOUT_TRANS_IMPL)
#undef LAYOUT_TRANS_IMPL

inline void RGB8BPixel::Transcribe(RGB8BLayout layout, RGB8BFlatType& out) {
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

using RGBA8BFlatType = uint32_t;

// Little endian byte order
#define RGBA_FLATNAME(c1, c2, c3, c4) c4##c3##c2##c1

#define RGBA8B_LAYOUT(c1, c2, c3, c4) \
  struct __attribute__((packed)) {    \
    uint8_t c1;                       \
    uint8_t c2;                       \
    uint8_t c3;                       \
    uint8_t c4;                       \
  };                                  \
  RGBA8BFlatType RGBA_FLATNAME(c1, c2, c3, c4);

union RGBA8888 {
  RGBA8B_LAYOUT(r, g, b, a)

  operator RGB888&() { return *reinterpret_cast<RGB888*>(this); }
  operator const RGB888&() const { return *reinterpret_cast<const RGB888*>(this); }
};

// An RGB 8-bit pixel data with a 8-bit alpha channel
// The alpha channel specifies the transparency, allowing the pixel to "blend" with the background.
struct RGBA8BPixel {
  using _this = RGBA8BPixel;

  union {
    RGBA8B_LAYOUT(r, g, b, a)
  };

  RGBA8BPixel() = default;
  constexpr RGBA8BPixel(uint8_t r8, uint8_t g8, uint8_t b8, uint8_t a8)
      : r(r8), g(g8), b(b8), a(a8) {}
  constexpr RGBA8BPixel(const RGB888& in, uint8_t alpha) : RGBA8BPixel(in.r, in.g, in.b, alpha) {}
  constexpr RGBA8BPixel(const RGBA8888& in)
      : RGBA_FLATNAME(r, g, b, a)(in.RGBA_FLATNAME(r, g, b, a)) {}

  inline bool IsTransparent() const { return a == 0; }
  inline bool IsSolid() const { return a == UINT8_MAX; }

  operator const RGBA8888&() const { return *reinterpret_cast<const RGBA8888*>(this); }
  operator RGBA8888&() { return *reinterpret_cast<RGBA8888*>(this); }

  // Access the RGB data as `RGB8BPixel`, ignoring the alpha channel.
  operator const RGB8BPixel&() const { return *reinterpret_cast<const RGB8BPixel*>(this); }
  operator RGB8BPixel&() { return *reinterpret_cast<RGB8BPixel*>(this); }

  static constexpr _this TRANSPARENT(const RGB888& in = RGB8BPixel::BLACK()) { return {in, 0x00}; }
  static constexpr _this SOLID(const RGB888& in) { return {in, UINT8_MAX}; }

  // Why pre-multiplied alpha blending is not used:
  //
  // Pre-multiplied alpha blending works by *separately* computing the overlay pixels
  // and the blend background and combine them together. However, since each rounding
  // occurs at computation, the combined data may have off-by-1 rounding errors.
  //
  // On a regular display the pixel responds to values linearly, so 0x01 looks almost
  // indistinguishable from 0x00 or 0x02; but LDE strips responds very differently.
  // 0x00, 0x01 and 0x02 produces vastly different luminosity. So, off-by-one rounding
  // errors will show up as very visible defects.
  //
  // // Perform optimized alpha multiplication.
  // // Note that the return value only has valid RGB data (the A channel is cleared).
  // inline RGBA8888 AlphaMult() const {
  //   uint32_t br = abgr & 0xFF00FF;
  //   br = (br * a) + 0x800080;
  //   br += (br >> 8) & 0xFF00FF;
  //   uint32_t xg = (abgr >> 8) & 0xFF00FF;
  //   xg = (xg * a) + 0x800080;
  //   xg += (xg >> 8) & 0xFF00FF;
  //   return RGBA8888{.abgr = ((br >> 8) & 0xFF00FF) | (xg & 0xFF00)};
  // }

  inline RGB888 Blend(const RGB888& bg) const {
    if (IsTransparent()) return bg;
    if (IsSolid()) return *reinterpret_cast<const RGB888*>(this);

#if ALPHABLEND_WITH_DIV
    return {(uint8_t)(bg.r + (int32_t)((r - bg.r) * a) / UINT8_MAX),
            (uint8_t)(bg.g + (int32_t)((g - bg.g) * a) / UINT8_MAX),
            (uint8_t)(bg.b + (int32_t)((b - bg.b) * a) / UINT8_MAX)};
#else

// from + (to - from)*alpha / 255
// x/255 = (x + x/255) / 256 ~= (x + x/256) / 256
#define ALPHABLEND_NO_DIV(store, from, to, alpha) \
  {                                               \
    int32_t mult = (int32_t)(to - from) * alpha;  \
    store = from + ((mult + (mult >> 8)) >> 8);   \
  }

    RGB888 result;
    ALPHABLEND_NO_DIV(result.r, bg.r, r, a)
    ALPHABLEND_NO_DIV(result.g, bg.g, g, a)
    ALPHABLEND_NO_DIV(result.b, bg.b, b, a)
    return result;

#undef ALPHABLEND_NO_DIV

#endif  // ALPHABLEND_WITH_DIV
  }
};

#undef RGBA8B_LAYOUT

}  // namespace zw_esp8266::lightshow

#endif  // ZWESP8266_LSPIXEL