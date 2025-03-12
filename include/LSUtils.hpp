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

// Computes the permillage value using integer arithmetics.
inline uint16_t progression_pmr(uint32_t cur, uint32_t total) {
  assert((cur <= 0xFFFFFFFF / 10) && (total <= 0xFFFFFFFF / 10));
  return std::min((cur * 10) / (total / 100), 1000U);
}

// Blend two values using a permillage value.
inline uint32_t blend_value(uint32_t from, uint32_t to, uint16_t pmr) {
  assert((from <= 0xFFFFFFFF / 1000) && (to <= 0xFFFFFFFF / 1000) && (pmr <= 1000));
  return from + (int32_t)(to - from) * pmr / 1000;
}

}  // namespace zw_esp8266::lightshow

#endif  // ZWESP8266_LSUTILS