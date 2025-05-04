// Light-show pixel
#ifndef ZWLIGHTSHOW_PIXEL
#define ZWLIGHTSHOW_PIXEL

#include <cstdint>
#include <cmath>
#include <string.h>

#include "sdkconfig.h"

namespace zw::esp8266::lightshow {

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

#define RGB8B_UNION_TYPEDEF(name, c1, c2, c3)                       \
  union name {                                                      \
    RGB8B_LAYOUT(c1, c2, c3)                                        \
    inline name& operator=(const RGB8BPixel& in) {                  \
      return c1 = in.c1, c2 = in.c2, c3 = in.c3, *this;             \
    }                                                               \
  };                                                                \
  inline bool operator==(const name& lhs, const name& rhs) {        \
    /* Optimized for little endian */                               \
    return (*reinterpret_cast<const uint32_t*>(&lhs) & 0xFFFFFF) == \
           (*reinterpret_cast<const uint32_t*>(&rhs) & 0xFFFFFF);   \
  }                                                                 \
  inline bool operator!=(const name& lhs, const name& rhs) { return !(lhs == rhs); }

RGB8B_LAYOUT_MAP(RGB8B_UNION_TYPEDEF)
#undef RGB8B_UNION_TYPEDEF

inline std::string to_string(const RGB888& in) {
  std::string ret(10, ' ');
  snprintf(&ret.front(), 11, "RGB:%02X%02X%02X", in.r, in.g, in.b);
  return ret;
}

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
  RGBA8888& operator=(const RGB888& in) {
    // Optimized for little endian
    return (abgr = *reinterpret_cast<const uint32_t*>(&in) & 0xFFFFFF), *this;
  }
};

inline std::string to_string(const RGBA8888& in) {
  std::string ret(14, ' ');
  snprintf(&ret.front(), 15, "RGBA:%02X%02X%02X@%02X", in.r, in.g, in.b, in.a);
  return ret;
}

// Implements optimized 8-bit alpha multiplication for blending
//
// Algorithm from https://arxiv.org/pdf/2202.02864:
// When x in [0,255]:
//   x*a / 255 ~= (x * (a|a<<8) + 0x8080) / (256*256)
//
// My own extension for diff blending where x can be in [-255,-1]:
//   x*a / 255 ~= (x * (a|a<<8) + 0x7F7F) / (256*256)
// Verified for the entire range of -x*a, where {-x,a} in [0,255],
// this produced identical results to: (-x*a) / (double)255 + 0.5
//
// To help understand the algorithm, the idea is
//   x/255 = (x + x/255) / 256 ~= (x + x/256 + delta) / 256
//
// Parameters:
//   x: must have range [-255, 255]
//   a: must have range [0, 255]
//
// Results are in range [-255, 255]
inline int16_t Fast8BDiffAlphaPremult(int32_t x, uint16_t a) {
  return x ? ((x * ((a << 8) | a) + (x > 0 ? 0x8080 : 0x7F7F)) >> 16) : 0;
}

// An RGB 8-bit pixel data with an 8-bit alpha channel
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

  inline RGB888 Blend(const RGB888& bg) const {
    if (IsTransparent()) return bg;
    if (IsSolid()) return *reinterpret_cast<const RGB888*>(this);

#ifdef CONFIG_ESP2812FBLESS_ALPHA_BLEND_NO_DIV
    return {(uint8_t)(bg.r + Fast8BDiffAlphaPremult((int32_t)r - bg.r, a)),
            (uint8_t)(bg.g + Fast8BDiffAlphaPremult((int32_t)g - bg.g, a)),
            (uint8_t)(bg.b + Fast8BDiffAlphaPremult((int32_t)b - bg.b, a))};
#else
    return {(uint8_t)(bg.r + (int32_t)((r - bg.r) * a + (UINT8_MAX >> 1)) / UINT8_MAX),
            (uint8_t)(bg.g + (int32_t)((g - bg.g) * a + (UINT8_MAX >> 1)) / UINT8_MAX),
            (uint8_t)(bg.b + (int32_t)((b - bg.b) * a + (UINT8_MAX >> 1)) / UINT8_MAX)};
#endif
  }
};

// An RGB 8-bit premultiplied pixel data with an 8-bit complementary alpha channel
// Note: may generate visible defects when at low values. See `RGB10BBlendPixel` below.
struct RGB8BBlendPixel {
  using _this = RGB8BBlendPixel;

  union {
    RGBA8B_LAYOUT(pr, pg, pb, ca)
  };

  RGB8BBlendPixel() = default;
  RGB8BBlendPixel(const RGBA8BPixel& in) { *this = in; }
  _this& operator=(const RGBA8BPixel& in) { return precompute(in.r, in.g, in.b, in.a), *this; }

  inline bool IsTransparent() const { return ca == UINT8_MAX; }
  inline bool IsSolid() const { return ca == 0; }

  static const _this TRANSPARENT(const RGB888& in = RGB8BPixel::BLACK()) { return {{in, 0x00}}; }
  static const _this SOLID(const RGB888& in) { return {{in, UINT8_MAX}}; }

  void precompute(uint8_t r8, uint8_t g8, uint8_t b8, uint8_t a8) {
    ca = UINT8_MAX - a8;

#ifdef CONFIG_ESP2812FBLESS_ALPHA_BLEND_NO_DIV
    pr = Fast8BDiffAlphaPremult(r8, a8);
    pg = Fast8BDiffAlphaPremult(g8, a8);
    pb = Fast8BDiffAlphaPremult(b8, a8);
#else
    pr = ((int16_t)r8 * a8 + (UINT8_MAX >> 1)) / UINT8_MAX;
    pg = ((int16_t)g8 * a8 + (UINT8_MAX >> 1)) / UINT8_MAX;
    pb = ((int16_t)b8 * a8 + (UINT8_MAX >> 1)) / UINT8_MAX;
#endif
  }

  inline RGB888 Blend(const RGB888& bg) const {
    if (IsTransparent()) return bg;
    if (IsSolid()) return *reinterpret_cast<const RGB888*>(this);

#ifdef CONFIG_ESP2812FBLESS_ALPHA_BLEND_NO_DIV
    return {(uint8_t)(pr + Fast8BDiffAlphaPremult(bg.r, ca)),
            (uint8_t)(pg + Fast8BDiffAlphaPremult(bg.g, ca)),
            (uint8_t)(pb + Fast8BDiffAlphaPremult(bg.b, ca))};
#else
    return {(uint8_t)(pr + ((uint16_t)bg.r * ca + (UINT8_MAX >> 1)) / UINT8_MAX),
            (uint8_t)(pg + ((uint16_t)bg.g * ca + (UINT8_MAX >> 1)) / UINT8_MAX),
            (uint8_t)(pb + ((uint16_t)bg.b * ca + (UINT8_MAX >> 1)) / UINT8_MAX)};
#endif
  }
};

#undef RGBA8B_LAYOUT

// Implements optimized alpha multiplication with 10-bit return for blending
//
// Why the 2 extra bits in the return?
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
// My extended algorithm based on https://arxiv.org/pdf/2202.02864.
// When x in [0,255]:
//   (4x*a)/255 ~= (x * (a>>6|a<<2|a<<10) + 0x8080) / (256*256)
// Verified for the entire range of x*a, where {x,a} in [0,255],
// this produced identical results to: (4x*a) / (double)255 + 0.5
//
// Parameters:
//   x: must have range [0, 255]
//   a: must have range [0, 255]
//
// Results are in range [0, 1023]
inline uint16_t FastAlphaPremult10B(uint32_t x, uint32_t a) {
  return x ? ((x * ((a << 10) | (a << 2) | (a >> 6)) + 0x8080) >> 16) : 0;
}

// An RGB 10-bit premultiplied pixel data with an 8-bit complementary alpha channel
struct RGB10BBlendPixel {
  using _this = RGB10BBlendPixel;

  union {
    struct {
      uint32_t pr10 : 10;
      uint32_t pg10 : 10;
      uint32_t pb10 : 10;
      uint32_t _ : 2;
    };
    uint32_t rgb10;
  };
  uint8_t ca;

  RGB10BBlendPixel() = default;
  RGB10BBlendPixel(const RGBA8BPixel& in) { *this = in; }
  _this& operator=(const RGBA8BPixel& in) { return precompute(in.r, in.g, in.b, in.a), *this; }

  inline bool IsTransparent() const { return ca == UINT8_MAX; }
  inline bool IsSolid() const { return ca == 0; }

  static const _this TRANSPARENT(const RGB888& in = RGB8BPixel::BLACK()) { return {{in, 0x00}}; }
  static const _this SOLID(const RGB888& in) { return {{in, UINT8_MAX}}; }

  void precompute(uint8_t r8, uint8_t g8, uint8_t b8, uint8_t a8) {
    ca = UINT8_MAX - a8;

#ifdef CONFIG_ESP2812FBLESS_ALPHA_BLEND_NO_DIV
    pr10 = FastAlphaPremult10B(r8, a8);
    pg10 = FastAlphaPremult10B(g8, a8);
    pb10 = FastAlphaPremult10B(b8, a8);
#else
    pr10 = (((int32_t)r8 << 2) * a8 + (UINT8_MAX >> 1)) / UINT8_MAX;
    pg10 = (((int32_t)g8 << 2) * a8 + (UINT8_MAX >> 1)) / UINT8_MAX;
    pb10 = (((int32_t)b8 << 2) * a8 + (UINT8_MAX >> 1)) / UINT8_MAX;
#endif
  }

  inline RGB888 Blend(const RGB888& bg) const {
    if (IsTransparent()) return bg;
    if (IsSolid()) return {(uint8_t)(pr10 >> 2), (uint8_t)(pg10 >> 2), (uint8_t)(pb10 >> 2)};

#ifdef CONFIG_ESP2812FBLESS_ALPHA_BLEND_NO_DIV
    return {(uint8_t)((pr10 + FastAlphaPremult10B(bg.r, ca) + 1) >> 2),
            (uint8_t)((pg10 + FastAlphaPremult10B(bg.g, ca) + 1) >> 2),
            (uint8_t)((pb10 + FastAlphaPremult10B(bg.b, ca) + 1) >> 2)};
#else
    return {
        (uint8_t)((pr10 + (((uint32_t)bg.r << 2) * ca + (UINT8_MAX >> 1)) / UINT8_MAX + 1) >> 2),
        (uint8_t)((pg10 + (((uint32_t)bg.g << 2) * ca + (UINT8_MAX >> 1)) / UINT8_MAX + 1) >> 2),
        (uint8_t)((pb10 + (((uint32_t)bg.b << 2) * ca + (UINT8_MAX >> 1)) / UINT8_MAX + 1) >> 2)};
#endif
  }
};

#if defined(CONFIG_ESP2812FBLESS_ALPHA_BLEND_STRAIGHT)
using AlphaBlendPixel = RGBA8BPixel;
#elif defined(CONFIG_ESP2812FBLESS_ALPHA_BLEND_PREMULT_10B)
using AlphaBlendPixel = RGB10BBlendPixel;
#elif defined(CONFIG_ESP2812FBLESS_ALPHA_BLEND_PREMULT_8B)
using AlphaBlendPixel = RGB8BBlendPixel;
#else
#error Unsupported alpha blending mode!
#endif

struct HSVPixel {
  using _this = HSVPixel;

  float h, s, v;

  HSVPixel() = default;
  constexpr HSVPixel(float h, float s, float v) : h(h), s(s), v(v) {}
  HSVPixel(const RGB888& in) { *this = in; }

  _this& hue_rotate(float deg) {
    h = std::fmod(h + deg, 360);
    if (h < 0) h += 360;

    return *this;
  }

  _this& operator=(const RGB888& in) {
    uint8_t in_max = std::max({in.r, in.g, in.b});
    uint8_t in_min = std::min({in.r, in.g, in.b});
    uint8_t in_delta = in_max - in_min;

    if (in_delta != 0) {
      if (in_max == in.r) {
        h = (float)(((int16_t)in.g - in.b) * 60) / in_delta + (in.g >= in.b ? 0 : 360);
      } else if (in_max == in.g) {
        h = (float)(((int16_t)in.b - in.r) * 60) / in_delta + 120;
      } else if (in_max == in.b) {
        h = (float)(((int16_t)in.r - in.g) * 60) / in_delta + 240;
      }
      s = (float)in_delta / in_max;
      v = (float)in_max / UINT8_MAX;
    } else {
      h = s = v = 0;
    }

    return *this;
  }

  operator RGB888() {
    if (s == 0.0F)
      return {(uint8_t)(v * UINT8_MAX), (uint8_t)(v * UINT8_MAX), (uint8_t)(v * UINT8_MAX)};

    float h_domain;
    float f = std::modf(h / 60, &h_domain);

    float p = v * (1.0F - s);
    float q = v * (1.0F - s * f);
    float t = v * (1.0F - s * (1.0F - f));

    switch ((int)h_domain) {
      case 0:
        return {(uint8_t)(v * UINT8_MAX), (uint8_t)(t * UINT8_MAX), (uint8_t)(p * UINT8_MAX)};
      case 1:
        return {(uint8_t)(q * UINT8_MAX), (uint8_t)(v * UINT8_MAX), (uint8_t)(p * UINT8_MAX)};
      case 2:
        return {(uint8_t)(p * UINT8_MAX), (uint8_t)(v * UINT8_MAX), (uint8_t)(t * UINT8_MAX)};
      case 3:
        return {(uint8_t)(p * UINT8_MAX), (uint8_t)(q * UINT8_MAX), (uint8_t)(v * UINT8_MAX)};
      case 4:
        return {(uint8_t)(t * UINT8_MAX), (uint8_t)(p * UINT8_MAX), (uint8_t)(v * UINT8_MAX)};
      case 5:
        return {(uint8_t)(v * UINT8_MAX), (uint8_t)(p * UINT8_MAX), (uint8_t)(q * UINT8_MAX)};
    }

    // Should not happen
    return RGB8BPixel::BLACK();
  }
};

inline std::string to_string(const HSVPixel in) {
  std::string ret(24, ' ');
  uint16_t h_10x = (uint16_t)std::roundf(in.h * 10);
  uint16_t s_1000x = (uint16_t)std::roundf(in.s * 1000);
  uint16_t v_1000x = (uint16_t)std::roundf(in.v * 1000);
  snprintf(&ret.front(), 25, "HSV:%d.%d,%d.%d%%,%d.%d%%", h_10x / 10, h_10x % 10, s_1000x / 10,
           s_1000x % 10, v_1000x / 10, v_1000x % 10);
  return ret;
}

}  // namespace zw::esp8266::lightshow

#endif  // ZWLIGHTSHOW_PIXEL