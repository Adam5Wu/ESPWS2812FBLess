#include "LSFrame.hpp"

#include <cmath>

#include "esp_log.h"

#include "LSUtils.hpp"

namespace zw_esp8266::lightshow {
namespace {

inline constexpr char TAG[] = "LSFrame";

}  // namespace

void ColorDotFrame::Init() {
  float half_dot_size = ((float)dot.glow / 10 + 1) / 2;
  float dot_pos = (float)(size - 1) * dot.pos_pmr / 1000;
  start_pos = std::max((int32_t)std::lroundf(dot_pos - half_dot_size), 0);
  end_pos = std::min((int32_t)std::lroundf(dot_pos + half_dot_size), size - 1);
  pix_.resize(end_pos - start_pos + 1);

  float two_sigma_sqr = half_dot_size * half_dot_size / 4;
  for (int i = start_pos; i <= end_pos; i++) {
    float x = dot_pos - i;
    uint16_t pmr = std::exp(-x * x / two_sigma_sqr) * 1000;
    pix_[i - start_pos].u[0] = blend_value(bgcolor.u[0], dot.color.u[0], pmr);
    pix_[i - start_pos].u[1] = blend_value(bgcolor.u[1], dot.color.u[1], pmr);
    pix_[i - start_pos].u[2] = blend_value(bgcolor.u[2], dot.color.u[2], pmr);
  }
}

}  // namespace zw_esp8266::lightshow