// Light-show pixel
#ifndef ZWESP8266_LSPIXEL
#define ZWESP8266_LSPIXEL

#include <cstdint>
#include <string.h>

namespace zw_esp8266::lightshow {

union RGB888 {
  struct __attribute__((packed)) {
    uint8_t r;
    uint8_t g;
    uint8_t b;
  };
  uint8_t u[3];

  RGB888() = default;
  RGB888(uint8_t r8, uint8_t g8, uint8_t b8) : r(r8), g(g8), b(b8) {}
};

union GRB888 {
  struct __attribute__((packed)) {
    uint8_t g;
    uint8_t r;
    uint8_t b;
  };
  uint8_t u[3];

  GRB888() = default;
  GRB888(uint8_t r8, uint8_t g8, uint8_t b8) : g(g8), r(r8), b(b8) {}
};

using RGB8BPixel = GRB888;

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