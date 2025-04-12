// Light-show utilities
#ifndef ZWLIGHTSHOW_UTILS
#define ZWLIGHTSHOW_UTILS

#include <cmath>
#include <algorithm>

#include "assert.h"

namespace zw::esp8266::lightshow {

using ProgressionType = uint16_t;

inline constexpr uint8_t PGRS_PRECISION = 12;
inline constexpr uint8_t PGRS_DIVNUM_FACTOR = 6;
inline constexpr uint32_t PGRS_MAX_DIVNUM = UINT32_MAX >> PGRS_DIVNUM_FACTOR;
inline constexpr uint8_t PGRS_DENUM_FACTOR = PGRS_PRECISION - PGRS_DIVNUM_FACTOR;
inline constexpr uint32_t PGRS_MIN_DIVISOR = 1U << PGRS_DENUM_FACTOR;
inline constexpr uint32_t PGRS_DENOM = 1U << PGRS_PRECISION;
inline constexpr ProgressionType PGRS_FULL = PGRS_DENOM;
inline constexpr ProgressionType PGRS_MIDWAY = (PGRS_FULL - 1) >> 1;

// Convert a fraction [0.0, 1.0] into progression value [0, PGRS_FULL]
constexpr ProgressionType PGRS(float frac) { return frac * PGRS_FULL; }

// Computes the "progression" of `cur` from `total` with 12-bit precision.
inline ProgressionType progression(uint32_t cur, uint32_t total) {
  if (cur >= total) return PGRS_FULL;
  if (total > PGRS_MIN_DIVISOR) {
    assert(cur <= PGRS_MAX_DIVNUM);
    return (cur << PGRS_DIVNUM_FACTOR) / (total >> PGRS_DENUM_FACTOR);
  }
  return (cur << PGRS_PRECISION) / total;
}

// Convert the progression value to 8-bit alpha [0,255].
inline constexpr uint8_t ALPHA_PRECISION = 8;

inline uint8_t pgrs_to_alpha(ProgressionType pgrs) {
  assert(pgrs <= PGRS_FULL);
  return ((uint32_t)pgrs * UINT8_MAX) >> PGRS_PRECISION;
}

// Blend two values using the above approx-permillage value.
inline constexpr uint32_t BLEND_MAX_VAL = UINT32_MAX >> (PGRS_PRECISION + 1);

inline uint32_t blend_value(uint32_t from, uint32_t to, ProgressionType pgrs) {
  assert((from <= BLEND_MAX_VAL) && (to <= BLEND_MAX_VAL) && (pgrs <= PGRS_FULL));
  return from + ((((int32_t)to - from) * pgrs + PGRS_MIDWAY) >> PGRS_PRECISION);
}

// Map a progression value to a Gaussian probability
// Formula: e^(-x^2/2) curve between [-3.16, 3.16]
inline ProgressionType pgrs_map_gaussian(ProgressionType pgrs) {
  // x = 3.16 * rx ==> -x^2/2 = -10 * rx^2 / 2 = -5 * rx^2
  float rx = (float)((int16_t)pgrs - PGRS_MIDWAY) / PGRS_MIDWAY;
  return std::exp(rx * rx * -5.0F) * PGRS_FULL;
}

// Map a progression value to a symmetrical exponential rise and decay
// Formula: mirrored e^x curve between [-5.3, 0]
inline ProgressionType pgrs_map_exponential(ProgressionType pgrs) {
  float x = -5.3F * (float)std::abs((int16_t)pgrs - PGRS_MIDWAY) / PGRS_MIDWAY;
  return std::exp(x) * PGRS_FULL;
}

// Map a progression value to a symmetrical sigmoid rise and decay
// Formula: mirrored 2/(1+e^(-x)) curve between [-5.3, 0]
inline ProgressionType pgrs_map_sigmoid(ProgressionType pgrs) {
  float x = 5.3F * (float)std::abs((int16_t)pgrs - PGRS_MIDWAY) / PGRS_MIDWAY;
  return 2.0F / (1.0F + std::exp(x)) * PGRS_FULL;
}

// Map a progression value to a symmetrical saw-tooth rise and decay
inline ProgressionType pgrs_map_sawtooth(ProgressionType pgrs) {
  return ((pgrs <= PGRS_MIDWAY) ? pgrs : PGRS_FULL - pgrs) << 1;
}

}  // namespace zw::esp8266::lightshow

#endif  // ZWLIGHTSHOW_UTILS