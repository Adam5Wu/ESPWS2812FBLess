#include "LSFrame.hpp"

#include <cmath>

#include "esp_log.h"

#include "LSUtils.hpp"
#include "LSPixel.hpp"

namespace zw_esp8266::lightshow {
namespace {

inline constexpr char TAG[] = "LSFrame";

}  // namespace

void ColorDotFrame::Init() {
  dot_pos_ = (float)(size - 1) * dot.pos_pmr / PROGRESSION_DENOM;

  float half_dot_size = ((float)dot.glow / 10 + 1) / 2;
  start_pos_ = std::max((int32_t)std::lroundf(dot_pos_ - half_dot_size), 0);
  end_pos_ = std::min((int32_t)std::lroundf(dot_pos_ + half_dot_size), size - 1);
  // half-dot-size ~= x*sigma
  // 2*sigma^2 = 2*(half-dot-size)^2/x^2 = (half-dot-size)^2/(x^2/2)
  // When x=3, x^2/2 = 4.5; when x^2/2 = 5, x ~= 3.16
  two_sigma_sqr_ = half_dot_size * half_dot_size / 5;
}

PixelWithStatus ColorDotFrame::GetPixelData() {
  StripSizeType scan_pos = index_++;
  if (scan_pos >= start_pos_ && scan_pos <= end_pos_) {
    PixelWithStatus result = {};
    float x = dot_pos_ - scan_pos;
    uint16_t pmr = std::exp(-x * x / two_sigma_sqr_) * 1000;
    result.pixel.u[0] = blend_value(bgcolor.u[0], dot.color.u[0], pmr);
    result.pixel.u[1] = blend_value(bgcolor.u[1], dot.color.u[1], pmr);
    result.pixel.u[2] = blend_value(bgcolor.u[2], dot.color.u[2], pmr);
    return result;
  }

  return {.pixel = bgcolor, .end_of_frame = scan_pos >= size};
}

}  // namespace zw_esp8266::lightshow