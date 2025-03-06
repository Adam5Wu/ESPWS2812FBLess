#include "LSTarget.hpp"

#include "esp_log.h"

#include "LSFrame.hpp"

namespace zw_esp8266::lightshow {
namespace {

inline constexpr char TAG[] = "LSTarget";

std::unique_ptr<Frame> RenderUniformColorProgression(StripSizeType frame_size,
                                                     RGB8BPixel from_color, RGB8BPixel to_color,
                                                     uint32_t duration_us,
                                                     uint32_t time_passed_us) {
  RGB8BPixel cur_color;
  if (time_passed_us < duration_us) {
    int32_t pmr = time_passed_us / (duration_us / 1000);
    cur_color.u[0] = from_color.u[0] + (int32_t)(to_color.u[0] - from_color.u[0]) * pmr / 1000;
    cur_color.u[1] = from_color.u[1] + (int32_t)(to_color.u[1] - from_color.u[1]) * pmr / 1000;
    cur_color.u[2] = from_color.u[2] + (int32_t)(to_color.u[2] - from_color.u[2]) * pmr / 1000;
  } else {
    cur_color = to_color;
  }
  return std::make_unique<UniformColorFrame>(frame_size, cur_color);
}

std::unique_ptr<Frame> RenderMovingDotProgression(StripSizeType strip_size, RGB8BPixel bg_color,
                                                  DotState from, DotState to, uint32_t duration_us,
                                                  uint32_t time_passed_us) {
  DotState cur;
  if (time_passed_us < duration_us) {
    int32_t pmr = time_passed_us / (duration_us / 1000);
    cur.pos_pmr = from.pos_pmr + (int32_t)(to.pos_pmr - from.pos_pmr) * pmr / 1000;
    if (to.color != from.color) {
      cur.color.u[0] = from.color.u[0] + (int32_t)(to.color.u[0] - from.color.u[0]) * pmr / 1000;
      cur.color.u[1] = from.color.u[1] + (int32_t)(to.color.u[1] - from.color.u[1]) * pmr / 1000;
      cur.color.u[2] = from.color.u[2] + (int32_t)(to.color.u[2] - from.color.u[2]) * pmr / 1000;
    } else {
      cur.color = to.color;
    }
  } else {
    cur = to;
  }
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
  return RenderUniformColorProgression(frame_size_, base_color_, color_, duration_us,
                                       time_passed_us);
}

DataOrError<std::unique_ptr<Target>> DotTarget::Create(uint32_t duration_ms, DotState dot) {
  ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
  if (dot.pos_pmr > 1000) {
    ESP_LOGW(TAG, "Invalid dot position");
    return ESP_ERR_INVALID_ARG;
  }
  return std::unique_ptr<Target>(new DotTarget(duration_us, dot));
}

esp_err_t DotTarget::RenderInit(const Frame& base_frame) {
  frame_size_ = base_frame.size;
  switch (base_frame.Type()) {
    case Frame::FrameType::kUniformColor:
      bg_color_ = static_cast<const UniformColorFrame&>(base_frame).color;
      base_dot_ = {.pos_pmr = 0, .color = bg_color_};
      return ESP_OK;

    case Frame::FrameType::kColorDot: {
      auto base_dot_frame = static_cast<const ColorDotFrame&>(base_frame);
      bg_color_ = base_dot_frame.bgcolor;
      base_dot_ = base_dot_frame.dot;
      return ESP_OK;
    }

    default:
      ESP_LOGW(TAG, "Unsupported base frame type");
      base_dot_ = {.pos_pmr = 0, .color = dot_.color};
      bg_color_ = BLACK();
      return ESP_ERR_NOT_SUPPORTED;
  }
}

std::unique_ptr<Frame> DotTarget::RenderFrame(uint32_t time_passed_us) {
  return RenderMovingDotProgression(frame_size_, bg_color_, base_dot_, dot_, duration_us,
                                    time_passed_us);
}

}  // namespace zw_esp8266::lightshow