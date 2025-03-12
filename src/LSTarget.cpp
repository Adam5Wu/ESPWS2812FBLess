#include "LSTarget.hpp"

#include "esp_log.h"

#include "LSUtils.hpp"
#include "LSPixel.hpp"
#include "LSFrame.hpp"

namespace zw_esp8266::lightshow {
namespace {

inline constexpr char TAG[] = "LSTarget";

std::unique_ptr<Frame> RenderUniformColorProgression(StripSizeType frame_size,
                                                     RGB8BPixel from_color, RGB8BPixel to_color,
                                                     uint16_t progression) {
  RGB8BPixel cur_color;
  cur_color.u[0] = blend_value(from_color.u[0], to_color.u[0], progression);
  cur_color.u[1] = blend_value(from_color.u[1], to_color.u[1], progression);
  cur_color.u[2] = blend_value(from_color.u[2], to_color.u[2], progression);
  return std::make_unique<UniformColorFrame>(frame_size, cur_color);
}

std::unique_ptr<Frame> RenderMovingDotProgression(StripSizeType strip_size, RGB8BPixel bg_color,
                                                  DotState from, DotState to,
                                                  uint16_t progression) {
  DotState cur;
  cur.color.u[0] = blend_value(from.color.u[0], to.color.u[0], progression);
  cur.color.u[1] = blend_value(from.color.u[1], to.color.u[1], progression);
  cur.color.u[2] = blend_value(from.color.u[2], to.color.u[2], progression);
  cur.glow = blend_value(from.glow, to.glow, progression);
  cur.pos_pmr = blend_value(from.pos_pmr, to.pos_pmr, progression);
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

DataOrError<std::unique_ptr<Target>> UniformColorTarget::Create(uint32_t duration_ms,
                                                                RGB8BPixel color) {
  ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
  return std::unique_ptr<Target>(new UniformColorTarget(duration_us, color));
}

esp_err_t UniformColorTarget::RenderInit(const Frame& base_frame) {
  frame_size_ = base_frame.size;
  switch (base_frame.Type()) {
    case Frame::FrameType::kUniformColor:
      base_color_ = static_cast<const UniformColorFrame&>(base_frame).color;
      return ESP_OK;

    default:
      ESP_LOGW(TAG, "Unsupported base frame type");
      base_color_ = BLACK();
      return ESP_ERR_NOT_SUPPORTED;
  }
}

std::unique_ptr<Frame> UniformColorTarget::RenderFrame(uint32_t time_passed_us) {
  return RenderUniformColorProgression(frame_size_, base_color_, color_,
                                       progression_pmr(time_passed_us, duration_us));
}

DataOrError<std::unique_ptr<Target>> DotTarget::Create(uint32_t duration_ms, DotState dot,
                                                       std::optional<DotState> def_dot) {
  ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
  if (dot.pos_pmr > 1000) {
    ESP_LOGW(TAG, "Invalid dot position");
    return ESP_ERR_INVALID_ARG;
  }
  return std::unique_ptr<Target>(new DotTarget(duration_us, dot, def_dot));
}

esp_err_t DotTarget::RenderInit(const Frame& base_frame) {
  frame_size_ = base_frame.size;
  switch (base_frame.Type()) {
    case Frame::FrameType::kUniformColor:
      bg_color_ = static_cast<const UniformColorFrame&>(base_frame).color;
      base_dot_ = def_dot_.value_or(DotState{.color = bg_color_, .glow = 0, .pos_pmr = 0});
      return ESP_OK;

    case Frame::FrameType::kColorDot: {
      auto base_dot_frame = static_cast<const ColorDotFrame&>(base_frame);
      bg_color_ = base_dot_frame.bgcolor;
      base_dot_ = base_dot_frame.dot;
      return ESP_OK;
    }

    default:
      ESP_LOGW(TAG, "Unsupported base frame type");
      bg_color_ = BLACK();
      base_dot_ = def_dot_.value_or(DotState{.color = bg_color_, .glow = 0, .pos_pmr = 0});;
      return ESP_ERR_NOT_SUPPORTED;
  }
}

std::unique_ptr<Frame> DotTarget::RenderFrame(uint32_t time_passed_us) {
  return RenderMovingDotProgression(frame_size_, bg_color_, base_dot_, dot_,
                                    progression_pmr(time_passed_us, duration_us));
}

}  // namespace zw_esp8266::lightshow