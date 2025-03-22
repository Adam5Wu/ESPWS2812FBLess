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
// ComputedColorDotFrame Implementations

ComputedColorDotFrame::ComputedColorDotFrame(StripSizeType strip_size, const DotState& dot,
                                             RGB888 bgcolor)
    : ColorDotFrame(strip_size, dot), bgcolor(bgcolor) {
  dot_pos_ = (float)(size - 1) * dot.pos_pgrs / PGRS_DENOM;

  float half_dot_size = ((float)dot.glow / 10 + 1) / 2;
  start_pos_ = std::max((int32_t)std::lroundf(dot_pos_ - half_dot_size), 0);
  end_pos_ = std::min((int32_t)std::lroundf(dot_pos_ + half_dot_size), size - 1);
  // half-dot-size ~= x*sigma
  // 2*sigma^2 = 2*(half-dot-size)^2/x^2 = (half-dot-size)^2/(x^2/2)
  // When x=3, x^2/2 = 4.5; when x^2/2 = 5, x ~= 3.16
  two_sigma_sqr_ = half_dot_size * half_dot_size / 5;
}

PixelWithStatus ComputedColorDotFrame::GetPixelData() {
  StripSizeType scan_pos = index_++;
  if (scan_pos >= start_pos_ && scan_pos <= end_pos_) {
    PixelWithStatus result = {};
    float x = dot_pos_ - scan_pos;
    ProgressionType pgrs = std::exp(-x * x / two_sigma_sqr_) * PGRS_DENOM + 0.5;
    result.pixel.r = blend_value(bgcolor.r, dot.color.r, pgrs);
    result.pixel.g = blend_value(bgcolor.g, dot.color.g, pgrs);
    result.pixel.b = blend_value(bgcolor.b, dot.color.b, pgrs);
    return result;
  }

  return PixelWithStatus{.rgb = bgcolor, .end_of_frame = scan_pos >= size};
}

//------------------------------------
// BlendedColorDotFrame Implementations

BlendedColorDotFrame::BlendedColorDotFrame(StripSizeType strip_size, const DotState& dot)
    : ColorDotFrame(strip_size, dot) {
  float dot_pos_ = (float)(size - 1) * dot.pos_pgrs / PGRS_DENOM;

  float half_dot_size = ((float)dot.glow / 10 + 1) / 2;
  start_pos_ = std::max((int32_t)std::lroundf(dot_pos_ - half_dot_size), 0);
  end_pos_ = std::min((int32_t)std::lroundf(dot_pos_ + half_dot_size), size - 1);
  // half-dot-size ~= x*sigma
  // 2*sigma^2 = 2*(half-dot-size)^2/x^2 = (half-dot-size)^2/(x^2/2)
  // When x=3, x^2/2 = 4.5; when x^2/2 = 5, x ~= 3.16
  float two_sigma_sqr_ = half_dot_size * half_dot_size / 5;

  // Pre-compute the pixel data
  blend_pixels_.resize(end_pos_ - start_pos_ + 1);
  for (StripSizeType i = start_pos_; i <= end_pos_; ++i) {
    float x = dot_pos_ - i;
    uint8_t alpha = std::exp(-x * x / two_sigma_sqr_) * UINT8_MAX + 0.5;
    blend_pixels_[(i - start_pos_)] = RGBA8BPixel(dot.color, alpha);
  }
}

PixelWithStatus BlendedColorDotFrame::GetPixelData() {
  StripSizeType scan_pos = index_++;
  return PixelWithStatus{.blend_pixel = (scan_pos >= start_pos_ && scan_pos <= end_pos_)
                                            ? blend_pixels_[scan_pos - start_pos_]
                                            : AlphaBlendPixel::TRANSPARENT(),
                         .end_of_frame = scan_pos >= size};
}

}  // namespace zw_esp8266::lightshow