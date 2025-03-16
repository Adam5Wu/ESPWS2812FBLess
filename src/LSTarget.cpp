#include "LSTarget.hpp"

#include <memory>

#include "esp_log.h"

#include "LSUtils.hpp"
#include "LSPixel.hpp"
#include "LSFrame.hpp"

namespace zw_esp8266::lightshow {
namespace {

inline constexpr char TAG[] = "LSTarget";

std::unique_ptr<Frame> RenderMovingDotProgression(StripSizeType strip_size, RGB8BPixel bg_color,
                                                  DotState from, DotState to,
                                                  ProgressionType pgrs) {
  DotState cur;
  cur.color.u[0] = blend_value(from.color.u[0], to.color.u[0], pgrs);
  cur.color.u[1] = blend_value(from.color.u[1], to.color.u[1], pgrs);
  cur.color.u[2] = blend_value(from.color.u[2], to.color.u[2], pgrs);
  cur.glow = blend_value(from.glow, to.glow, pgrs);
  cur.pos_pgrs = blend_value(from.pos_pgrs, to.pos_pgrs, pgrs);
  return std::make_unique<ColorDotFrame>(strip_size, cur, bg_color);
}

}  // namespace

DataOrError<uint32_t> Target::PrepareDuration(uint32_t duration_ms) {
  if (duration_ms > kMaxTargetDurationMS) {
    ESP_LOGW(TAG, "Target duration too long");
    return ESP_ERR_INVALID_ARG;
  }
  return duration_ms * 1000;
}

//-------------------------
// DotTarget

DataOrError<std::unique_ptr<Target>> DotTarget::Create(uint32_t duration_ms, DotState dot,
                                                       std::optional<DotState> def_dot) {
  ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
  if (dot.pos_pgrs > PGRS_DENOM) {
    ESP_LOGW(TAG, "Invalid dot position");
    return ESP_ERR_INVALID_ARG;
  }
  return std::unique_ptr<Target>(new DotTarget(duration_us, dot, def_dot));
}

std::unique_ptr<Frame> DotTarget::RenderInit(std::unique_ptr<Frame> base_frame) {
  frame_size_ = base_frame->size;
  switch (base_frame->Type()) {
    case Frame::FrameType::kUniformColor:
      bg_color_ = static_cast<const UniformColorFrame&>(*base_frame).color;
      base_dot_ = def_dot_.value_or(DotState{.color = bg_color_, .glow = 0, .pos_pgrs = 0});
      break;

    case Frame::FrameType::kColorDot: {
      auto& base_dot_frame = static_cast<const ColorDotFrame&>(*base_frame);
      bg_color_ = base_dot_frame.bgcolor;
      base_dot_ = base_dot_frame.dot;
      break;
    }

    default:
      ESP_LOGW(TAG, "Unsupported base frame type");
      bg_color_ = RGB8BPixel::BLACK();
      base_dot_ = def_dot_.value_or(DotState{.color = bg_color_, .glow = 0, .pos_pgrs = 0});
  }
  return nullptr;
}

std::unique_ptr<Frame> DotTarget::RenderFrame(ProgressionType pgrs) {
  return RenderMovingDotProgression(frame_size_, bg_color_, base_dot_, dot, pgrs);
}

}  // namespace zw_esp8266::lightshow