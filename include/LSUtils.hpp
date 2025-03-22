// Light-show utilities
#ifndef ZWESP8266_LSUTILS
#define ZWESP8266_LSUTILS

#include <algorithm>
#include <variant>

#include "esp_err.h"

namespace zw_esp8266::lightshow {

template <typename T>
class DataOrError {
 public:
  DataOrError(T&& data) : data_or_error_(std::in_place_index<0>, std::move(data)) {}
  DataOrError(esp_err_t error) : data_or_error_(std::in_place_index<1>, error) {}

  DataOrError(const DataOrError&) = delete;
  DataOrError& operator=(const DataOrError&) = delete;

  T& operator*() { return std::get<0>(data_or_error_); }
  const T& operator*() const { return std::get<0>(data_or_error_); }
  T* operator->() { return &std::get<0>(data_or_error_); }
  const T* operator->() const { return &std::get<0>(data_or_error_); }

  operator bool() const { return data_or_error_.index() == 0; }
  esp_err_t error() const {
    return data_or_error_.index() == 1 ? std::get<1>(data_or_error_) : ESP_OK;
  }

 private:
  std::variant<T, esp_err_t> data_or_error_;
};

#define UNIQUE_VAR(prefix) prefix##__COUNTER__

#define ASSIGN_OR_RETURN(val, statement)                                    \
  auto UNIQUE_VAR(data_or_error) = (statement);                             \
  if (!UNIQUE_VAR(data_or_error)) return UNIQUE_VAR(data_or_error).error(); \
  val = std::move(*UNIQUE_VAR(data_or_error))

// Computes the "progression" of `cur` from `total` with 12-bit precision.
using ProgressionType = uint16_t;

inline constexpr uint8_t PGRS_PRECISION = 12;
inline constexpr uint8_t PGRS_DIVNUM_FACTOR = 6;
inline constexpr uint32_t PGRS_MAX_DIVNUM = UINT32_MAX >> (PGRS_DIVNUM_FACTOR);
inline constexpr uint8_t PGRS_DENUM_FACTOR = PGRS_PRECISION - PGRS_DIVNUM_FACTOR;
inline constexpr uint32_t PGRS_MIN_DIVISOR = 1U << (PGRS_DENUM_FACTOR);
inline constexpr uint32_t PGRS_DENOM = 1U << PGRS_PRECISION;

constexpr ProgressionType PGRS(float frac) { return PGRS_DENOM * frac; }

inline ProgressionType progression(uint32_t cur, uint32_t total) {
  if (cur >= total) return PGRS_DENOM;
  if (total > PGRS_MIN_DIVISOR) {
    assert(cur <= PGRS_MAX_DIVNUM);
    return (cur << PGRS_DIVNUM_FACTOR) / (total >> PGRS_DENUM_FACTOR);
  }
  return (cur << PGRS_PRECISION) / total;
}

// Convert the progression value to 8-bit alpha [0,255].
inline constexpr uint8_t ALPHA_PRECISION = 8;

inline uint8_t pgrs_to_alpha(ProgressionType pgrs) {
  assert(pgrs <= PGRS_DENOM);
  return ((uint32_t)pgrs * UINT8_MAX) >> PGRS_PRECISION;
}

// Blend two values using the above approx-permillage value.
inline constexpr uint32_t BLEND_MAX_VAL = UINT32_MAX >> PGRS_PRECISION;

inline uint32_t blend_value(uint32_t from, uint32_t to, ProgressionType pgrs) {
  assert((from <= BLEND_MAX_VAL) && (to <= BLEND_MAX_VAL) && (pgrs <= PGRS_DENOM));
  return from + (((int32_t)(to - from) * pgrs + ((PGRS_DENOM - 1) >> 1)) >> PGRS_PRECISION);
}

}  // namespace zw_esp8266::lightshow

#endif  // ZWESP8266_LSUTILS