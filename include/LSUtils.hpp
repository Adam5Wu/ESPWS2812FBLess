// Light-show utilities
#ifndef ZWESP8266_LSUTILS
#define ZWESP8266_LSUTILS

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

}  // namespace zw_esp8266::lightshow

#endif  // ZWESP8266_LSUTILS