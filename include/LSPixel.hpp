// Light-show pixel
#ifndef ZWESP8266_LSPIXEL
#define ZWESP8266_LSPIXEL

#include <cstdint>
#include <string.h>

#include "sdkconfig.h"

namespace zw_esp8266::lightshow {

#define DECLARE_RGB8BPIXEL(name, c1, c2, c3)                                      \
  union name {                                                                    \
    struct __attribute__((packed)) {                                              \
      uint8_t c1;                                                                 \
      uint8_t c2;                                                                 \
      uint8_t c3;                                                                 \
    };                                                                            \
    uint8_t u[3];                                                                 \
    name() = default;                                                             \
    name(uint8_t r8, uint8_t g8, uint8_t b8) : c1(c1##8), c2(c2##8), c3(c3##8) {} \
  }

DECLARE_RGB8BPIXEL(RGB888, r, g, b);
DECLARE_RGB8BPIXEL(RBG888, r, b, g);
DECLARE_RGB8BPIXEL(GRB888, g, r, b);
DECLARE_RGB8BPIXEL(GBR888, g, b, r);
DECLARE_RGB8BPIXEL(BRG888, b, r, g);
DECLARE_RGB8BPIXEL(BGR888, b, g, r);

#undef DECLARE_RGB8BPIXEL

#if defined(CONFIG_ESP2812FBLESS_PIXEL_RGB888)
using RGB8BPixel = RGB888;
#elif defined(CONFIG_ESP2812FBLESS_PIXEL_RBG888)
using RGB8BPixel = RBG888;
#elif defined(CONFIG_ESP2812FBLESS_PIXEL_GRB888)
using RGB8BPixel = GRB888;
#elif defined(CONFIG_ESP2812FBLESS_PIXEL_GBR888)
using RGB8BPixel = GBR888;
#elif defined(CONFIG_ESP2812FBLESS_PIXEL_BRG888)
using RGB8BPixel = BRG888;
#elif defined(CONFIG_ESP2812FBLESS_PIXEL_BGR888)
using RGB8BPixel = BGR888;
#else
#error "Unrecognized pixel format configuration!"
#endif

inline bool operator==(const RGB8BPixel& lhs, const RGB8BPixel& rhs) {
  return memcmp(&lhs, &rhs, sizeof(RGB8BPixel)) == 0;
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

}  // namespace zw_esp8266::lightshow

#endif  // ZWESP8266_LSPIXEL