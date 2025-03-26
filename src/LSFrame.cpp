#include "LSFrame.hpp"

#include <cmath>

#include "esp_log.h"

#include "LSUtils.hpp"
#include "LSPixel.hpp"

namespace zw_esp8266::lightshow {
namespace {

inline constexpr char TAG[] = "LSFrame";

}  // namespace

//------------------------------------
// BlenderFrame Implementations

BlenderFrame::BlenderFrame(std::unique_ptr<Frame> base_frame) : Frame(base_frame->size) {
  // Translucent frame cannot serve as a base frame.
  assert(!base_frame->IsTranslucent());

  // If the base is already a blender frame, avoid nesting in certain situations.
  if (base_frame->Type() == FrameType::kBlender) {
    auto& blender_base = static_cast<BlenderFrame&>(*base_frame);
    // No blend frame, completely transparent, or translucent blend frame: keep base frame.
    if (blender_base.blend_frame_ == nullptr || blender_base.alpha_ == 0 ||
        blender_base.blend_frame_->IsTranslucent()) {
      base_frame_ = std::move(blender_base.base_frame_);
      return;
    }
    // Complete opaque blend frame: keep blend frame.
    if (blender_base.blend_frame_ != nullptr && blender_base.alpha_ == UINT8_MAX) {
      base_frame_ = std::move(blender_base.blend_frame_);
      return;
    }
  }
  base_frame_ = std::move(base_frame);
}

void BlenderFrame::UpdateOverlay(std::unique_ptr<Frame> blend_frame, uint8_t alpha) {
  assert(base_frame_->size == blend_frame->size);
  blend_frame_ = std::move(blend_frame);
  translucent_blend_ = blend_frame_->IsTranslucent();
  alpha_ = alpha;  // Ignored if `translucent_blend_`
  // We are not certain whether the new blend_frame's internal position is aligned.
  // Just invoke a reset to all three frame entities.
  Reset();
}

PixelWithStatus BlenderFrame::GetPixelData() {
  if (blend_frame_ == nullptr || (alpha_ == 0 && !translucent_blend_)) {
    return base_frame_->GetPixelData();
  }

  StripSizeType scan_pos = index_++;

  auto overlay = translucent_blend_ ? blend_frame_->GetPixelData().blend_pixel
                                    : AlphaBlendPixel({blend_frame_->GetPixelData().rgb, alpha_});

  return PixelWithStatus{.rgb = overlay.Blend(base_frame_->GetPixelData().rgb),
                         .end_of_frame = scan_pos >= size};
}

//------------------------------------
// ColorDotFrame Implementations

ColorDotFrame::ColorDotFrame(StripSizeType strip_size, const DotState& dot, RGB888 bgcolor)
    : Frame(strip_size), dot(dot), bgcolor(bgcolor) {
  dot_pos_ = (float)(size - 1) * dot.pos_pgrs / PGRS_FULL + 0.5;
  half_dot_size_ = ((float)dot.glow / 10 + 1) / 2;
  start_pos_ = std::max((int32_t)std::floor(dot_pos_ - half_dot_size_), 0);
  end_pos_ = std::min((int32_t)std::ceil(dot_pos_ + half_dot_size_), size - 1);

  // half-dot-size ~= x*sigma
  // 2*sigma^2 = 2*(half-dot-size)^2/x^2 = (half-dot-size)^2/(x^2/2)
  // When x=3, x^2/2 = 4.5; when x^2/2 = 5, x ~= 3.16
  two_sigma_sqr_ = half_dot_size_ * half_dot_size_ / 5;
}

PixelWithStatus ColorDotFrame::GetPixelData() {
  StripSizeType scan_pos = index_++;
  if (scan_pos >= start_pos_ && scan_pos <= end_pos_) {
    PixelWithStatus result = {};
    float x = dot_pos_ - scan_pos;
    ProgressionType pgrs = std::exp(-x * x / two_sigma_sqr_) * PGRS_FULL + 0.5;
    result.pixel.r = blend_value(bgcolor.r, dot.color.r, pgrs);
    result.pixel.g = blend_value(bgcolor.g, dot.color.g, pgrs);
    result.pixel.b = blend_value(bgcolor.b, dot.color.b, pgrs);
    return result;
  }

  return PixelWithStatus{.rgb = bgcolor, .end_of_frame = scan_pos >= size};
}

//------------------------------------
// WiperFrame Implementations

WiperFrame::WiperFrame(StripSizeType strip_size, const WiperState& wiper, BladeGenerator blade_func)
    : Frame(strip_size), wiper(wiper), blade_func_(blade_func) {
  float blade_size = strip_size * wiper.width;
  half_blade_size_ = blade_size / 2;
  // ======blade======|--strip--|======blade======
  // center--^  -2 -1 0 1 2 ...          ^--center
  //         |<---- blade pos range ---->|
  // Since #0 is actually the first (valid) pixel, we want the blade center
  // position at 0.0 to fall right *before* it, not at the center of pixel #0
  //          ... | pixel -1 | pixel 0 | pixel 1 | ...
  // Blade center pos 0.0 ---^
  blade_center_ = ((size + blade_size) * wiper.pos_pgrs / PGRS_FULL) - half_blade_size_ - 0.5;
  start_pos_ = std::max((int32_t)std::ceil(blade_center_ - half_blade_size_), 0);
  end_pos_ = std::min((int32_t)std::floor(blade_center_ + half_blade_size_), size - 1);
}

PixelWithStatus WiperFrame::GetPixelData() {
  StripSizeType scan_pos = index_++;
  return PixelWithStatus{
      .blend_pixel = (scan_pos >= start_pos_ && scan_pos <= end_pos_)
                         ? GetBladePixel(scan_pos)
                         : AlphaBlendPixel{(scan_pos < start_pos_) ? wiper.l_color : wiper.r_color},
      .end_of_frame = scan_pos >= size};
}

//------------------------------------
// ComputedWiperFrame Implementations

AlphaBlendPixel ComputedWiperFrame::GetBladePixel(StripSizeType idx) const {
  // ==|======*========------
  // l    ^   c
  // If c = 6.1 and half-blade size (h) = 7.2
  // When scan_pos (p) = 3
  // - The blade-left is at c-h = -1.1
  // - The on-blade position is p-c+h = 4.1
  //
  // |----======*======------
  //      l   ^ c
  // If c = 9.6 and half-blade size (h) = 5.5
  // When scan_pos (p) = 9
  // - The blade-left is at c-h = 4.1
  // - The on-blade position is p-c+h = 4.9
  float on_blade_pos = idx - blade_center_ + half_blade_size_;
  ProgressionType on_blade_pgrs = on_blade_pos * PGRS_MIDWAY / half_blade_size_;
  ProgressionType color_pgrs = blade_func_(on_blade_pgrs);

  RGBA8888 result = wiper.color;
  const RGBA8888 color_from = (on_blade_pgrs < PGRS_MIDWAY) ? wiper.l_color : wiper.r_color;
  if ((RGB888)color_from != (RGB888)wiper.color) {
    result.r = blend_value(color_from.r, wiper.color.r, color_pgrs);
    result.g = blend_value(color_from.g, wiper.color.g, color_pgrs);
    result.b = blend_value(color_from.b, wiper.color.b, color_pgrs);
  }
  result.a = (uint8_t)blend_value(color_from.a, wiper.color.a, color_pgrs);
  ESP_LOGD(TAG, "%d: From %s, to %s, pgrs %d = %s", on_blade_pgrs, to_string(color_from).c_str(),
           to_string(wiper.color).c_str(), color_pgrs, to_string(result).c_str());
  return {result};
}

//------------------------------------
// CachedWiperFrame Implementations

CachedWiperFrame::CachedWiperFrame(StripSizeType strip_size, const WiperState& wiper,
                                   BladeGenerator blade_func)
    : ComputedWiperFrame(strip_size, wiper, blade_func) {
  // Pre-compute the pixel data
  blend_pixels_.resize(end_pos_ - start_pos_ + 1);

  for (StripSizeType i = start_pos_; i <= end_pos_; ++i) {
    blend_pixels_[(i - start_pos_)] = ComputedWiperFrame::GetBladePixel(i);
  }
}

AlphaBlendPixel CachedWiperFrame::GetBladePixel(StripSizeType idx) const {
  return blend_pixels_[idx - start_pos_];
}

}  // namespace zw_esp8266::lightshow