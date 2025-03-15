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
  RGB8BPixel() = default;
  RGB8BPixel(uint8_t r8, uint8_t g8, uint8_t b8) : r(r8), g(g8), b(b8) {}

  // Statically produce a pixel of specific layout in `out`.
#define LAYOUT_TRANS_FORWARD(name, c1, c2, c3) inline void Transcribe(name& out);
  RGB8B_LAYOUT_MAP(LAYOUT_TRANS_FORWARD)
#undef LAYOUT_TRANS_FORWARD

  // Dynamically produce a pixel in `out` using the given `layout`.
  inline void Transcribe(RGB8BLayout layout, RGB8B_FLAT& out);
};

#define RGB8B_UNION_TYPEDEF(name, c1, c2, c3)           \
  union name {                                          \
    RGB8B_LAYOUT(c1, c2, c3)                            \
    name() : c1(0), c2(0), c3(0) {}                     \
    name(const RGB8BPixel& in) { *this = in; }          \
    inline name& operator=(const RGB8BPixel& in) {      \
      return c1 = in.c1, c2 = in.c2, c3 = in.c3, *this; \
    }                                                   \
  };
RGB8B_LAYOUT_MAP(RGB8B_UNION_TYPEDEF)
#undef RGB8B_UNION_TYPEDEF

// Implementation of RGB8BPixel transcriptions
#define LAYOUT_TRANS_IMPL(name, c1, c2, c3) \
  inline void RGB8BPixel::Transcribe(name& out) { out = *this; }
RGB8B_LAYOUT_MAP(LAYOUT_TRANS_IMPL)
#undef LAYOUT_TRANS_IMPL

// Dynamically produce a pixel in `out` using the given `layout`.
inline void RGB8BPixel::Transcribe(RGB8BLayout layout, RGB8B_FLAT& out) {
#define LAYOUT_DYN_TRANS(name, c1, c2, c3) \
  case RGB8BLayout::name:                  \
    reinterpret_cast<name&>(out) = *this;  \
    break;
  switch (layout) { RGB8B_LAYOUT_MAP(LAYOUT_DYN_TRANS) }
#undef LAYOUT_DYN_TRANS
}

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

inline bool operator==(const RGB8BPixel& lhs, const RGB8BPixel& rhs) {
  static_assert(sizeof(RGB8BPixel) <= sizeof(uint32_t));
  uint32_t lhv = 0, rhv = 0;
  reinterpret_cast<RGB8BPixel&>(lhv) = lhs;
  reinterpret_cast<RGB8BPixel&>(rhv) = rhs;
  return lhv == rhv;
}
inline bool operator!=(const RGB8BPixel& lhs, const RGB8BPixel& rhs) { return !(lhs == rhs); }

inline RGB8BPixel BLACK() { return {0x00, 0x00, 0x00}; }
inline RGB8BPixel RED() { return {0xFF, 0x00, 0x00}; }
inline RGB8BPixel GREEN() { return {0x00, 0xFF, 0x00}; }
inline RGB8BPixel BLUE() { return {0x00, 0x00, 0xFF}; }
inline RGB8BPixel WHITE() { return {0xFF, 0xFF, 0xFF}; }
inline RGB8BPixel YELLOW() { return {0xFF, 0xFF, 0x00}; }
inline RGB8BPixel MAGENTA() { return {0xFF, 0x00, 0xFF}; }
inline RGB8BPixel CYAN() { return {0x00, 0xFF, 0xFF}; }

#undef RGB8B_LAYOUT

}  // namespace zw_esp8266::lightshow

#endif  // ZWESP8266_LSPIXEL