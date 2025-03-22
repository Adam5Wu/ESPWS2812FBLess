#include "LSTarget.hpp"

#include <memory>

#include "esp_log.h"

#include "LSUtils.hpp"
#include "LSPixel.hpp"
#include "LSFrame.hpp"

#include "ESPIDF_shim.h"

namespace zw_esp8266::lightshow {
namespace {

inline constexpr char TAG[] = "LSTarget";

}  // namespace

DataOrError<uint32_t> Target::PrepareDuration(uint32_t duration_ms) {
  if (duration_ms > kMaxTargetDurationMS) {
    ESP_LOGW(TAG, "Target duration too long");
    return ESP_ERR_INVALID_ARG;
  }
  return duration_ms * 1000;
}

//-------------------------
// ColorDotTarget

esp_err_t ColorDotTarget::ValidateDots(DotState dot, std::optional<DotState> def_dot) {
  if (dot.pos_pgrs > PGRS_DENOM) {
    ESP_LOGW(TAG, "Invalid dot position");
    return ESP_ERR_INVALID_ARG;
  }
  if (def_dot.has_value() && def_dot->pos_pgrs > PGRS_DENOM) {
    ESP_LOGW(TAG, "Invalid default dot position");
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

void ColorDotTarget::BaseFrameInit(const Frame* base_frame) {
  // The dot color to use if the base frame doesn't contain dot info and no default dot provided.
  RGB888 no_def_dot_color = RGB8BPixel::BLACK();

  frame_size_ = base_frame->size;
  switch (base_frame->Type()) {
    case Frame::FrameType::kComputedColorDot: {
      base_dot_ = static_cast<const ColorDotFrame&>(*base_frame).dot;
      return;
    }

    case Frame::FrameType::kBlender: {
      auto& blender_frame = static_cast<const BlenderFrame&>(*base_frame);
      if (blender_frame.blend_frame()->Type() == Frame::FrameType::kBlendedColorDot) {
        base_dot_ = static_cast<const BlendedColorDotFrame&>(*blender_frame.blend_frame()).dot;
        return;
      }

      // Try find bg color from the blend base frame
      if (blender_frame.base_frame()->Type() == Frame::FrameType::kUniformColor)
        no_def_dot_color = static_cast<const UniformColorFrame&>(*blender_frame.base_frame()).color;
    } break;

    default:
      // Try find bg color from the base frame
      if (base_frame->Type() == Frame::FrameType::kUniformColor)
        no_def_dot_color = static_cast<const UniformColorFrame&>(*base_frame).color;
  }
  base_dot_ = def_dot_.value_or(DotState{.color = no_def_dot_color, .glow = 0, .pos_pgrs = 0});
}

DotState ColorDotTarget::CurrentDotState(ProgressionType pgrs) const {
  return {.color = {(uint8_t)blend_value(base_dot_.color.r, dot.color.r, pgrs),
                    (uint8_t)blend_value(base_dot_.color.g, dot.color.g, pgrs),
                    (uint8_t)blend_value(base_dot_.color.b, dot.color.b, pgrs)},
          .glow = (uint8_t)blend_value(base_dot_.glow, dot.glow, pgrs),
          .pos_pgrs = (ProgressionType)blend_value(base_dot_.pos_pgrs, dot.pos_pgrs, pgrs)};
}

//-------------------------
// ComputedColorDotTarget

DataOrError<std::unique_ptr<Target>> ComputedColorDotTarget::Create(
    uint32_t duration_ms, DotState dot, std::optional<DotState> def_dot) {
  ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
  ESP_RETURN_ON_ERROR(ValidateDots(dot, def_dot));
  return std::unique_ptr<Target>(new ComputedColorDotTarget(duration_us, dot, def_dot));
}

std::unique_ptr<Frame> ComputedColorDotTarget::RenderInit(std::unique_ptr<Frame> base_frame) {
  switch (base_frame->Type()) {
    case Frame::FrameType::kUniformColor:
      bg_color_ = static_cast<const UniformColorFrame&>(*base_frame).color;
      break;

    case Frame::FrameType::kComputedColorDot: {
      bg_color_ = static_cast<const ComputedColorDotFrame&>(*base_frame).bgcolor;
    } break;

    case Frame::FrameType::kBlender: {
      auto& blender_frame = static_cast<const BlenderFrame&>(*base_frame);
      if (blender_frame.base_frame()->Type() == Frame::FrameType::kUniformColor) {
        bg_color_ = static_cast<const UniformColorFrame&>(*blender_frame.base_frame()).color;
        break;
      }
      [[fallthrough]];
    }

    default:
      ESP_LOGW(TAG, "Unable to find background color");
      bg_color_ = RGB8BPixel::BLACK();
  }
  BaseFrameInit(base_frame.get());
  return nullptr;
}

std::unique_ptr<Frame> ComputedColorDotTarget::RenderFrame(ProgressionType pgrs) {
  return std::make_unique<ComputedColorDotFrame>(frame_size_, CurrentDotState(pgrs), bg_color_);
}

//-------------------------
// BlendedColorDotTarget

DataOrError<std::unique_ptr<Target>> BlendedColorDotTarget::Create(
    uint32_t duration_ms, DotState dot, std::optional<DotState> def_dot) {
  ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
  ESP_RETURN_ON_ERROR(ValidateDots(dot, def_dot));
  return std::unique_ptr<Target>(new BlendedColorDotTarget(duration_us, dot, def_dot));
}

std::unique_ptr<Frame> BlendedColorDotTarget::RenderInit(std::unique_ptr<Frame> base_frame) {
  BaseFrameInit(base_frame.get());
  // We need the renderer to do blending
  return std::move(base_frame);
}

std::unique_ptr<Frame> BlendedColorDotTarget::RenderFrame(ProgressionType pgrs) {
  return std::make_unique<BlendedColorDotFrame>(frame_size_, CurrentDotState(pgrs));
}

}  // namespace zw_esp8266::lightshow