#include "LSTarget.hpp"

#include <memory>

#include "esp_log.h"

#include "ZWUtils.hpp"

#include "LSUtils.hpp"
#include "LSPixel.hpp"
#include "LSFrame.hpp"

namespace zw::esp8266::lightshow {
namespace {

inline constexpr char TAG[] = "LSTarget";

}  // namespace

utils::DataOrError<uint32_t> StaticDurationTarget::PrepareDuration(uint32_t duration_ms) {
  if (duration_ms > kMaxTargetDurationMS) {
    ESP_LOGW(TAG, "Target duration too long");
    return ESP_ERR_INVALID_ARG;
  }
  return duration_ms * 1000;
}

//-------------------------
// ColorDotTarget

esp_err_t ColorDotTarget::ValidateDots(DotState dot, std::optional<DotState> def_dot) {
  if (dot.pos_pgrs > PGRS_FULL) {
    ESP_LOGW(TAG, "Invalid dot position");
    return ESP_ERR_INVALID_ARG;
  }
  if (def_dot.has_value() && def_dot->pos_pgrs > PGRS_FULL) {
    ESP_LOGW(TAG, "Invalid default dot position");
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

DotState ColorDotTarget::CurrentDotState(ProgressionType pgrs) const {
  return {.color = {(uint8_t)blend_value(base_dot_.color.r, dot.color.r, pgrs),
                    (uint8_t)blend_value(base_dot_.color.g, dot.color.g, pgrs),
                    (uint8_t)blend_value(base_dot_.color.b, dot.color.b, pgrs)},
          .glow = (uint8_t)blend_value(base_dot_.glow, dot.glow, pgrs),
          .pos_pgrs = (ProgressionType)blend_value(base_dot_.pos_pgrs, dot.pos_pgrs, pgrs)};
}

utils::DataOrError<Target::RefPtr> ColorDotTarget::Create(uint32_t duration_ms, DotState dot,
                                                          std::optional<DotState> def_dot) {
  ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
  ESP_RETURN_ON_ERROR(ValidateDots(dot, def_dot));
  return RefPtr(new ColorDotTarget(duration_us, dot, def_dot));
}

std::unique_ptr<Frame> ColorDotTarget::RenderInit(std::unique_ptr<Frame> base_frame) {
  base_frame = DrawingTarget::RenderInit(std::move(base_frame));
  switch (base_frame->Type()) {
    case Frame::FrameType::kUniformColor:
      bg_color_ = static_cast<const UniformColorFrame&>(*base_frame).color;
      break;

    case Frame::FrameType::kColorDot: {
      bg_color_ = static_cast<const ColorDotFrame&>(*base_frame).bgcolor;
      base_dot_ = static_cast<const ColorDotFrame&>(*base_frame).dot;
      return nullptr;
    }

    case Frame::FrameType::kBlender: {
      // Try find bg color from the blend base frame
      auto& blender_frame = static_cast<const BlenderFrame&>(*base_frame);
      if (blender_frame.base_frame()->Type() == Frame::FrameType::kUniformColor)
        bg_color_ = static_cast<const UniformColorFrame&>(*blender_frame.base_frame()).color;
    } break;

    default:
      ESP_LOGW(TAG, "Unable to find background color");
      bg_color_ = RGB8BPixel::BLACK();
  }

  base_dot_ = def_dot_.value_or(DotState{.color = bg_color_, .glow = 0, .pos_pgrs = 0});
  return nullptr;
}

std::unique_ptr<Frame> ColorDotTarget::RenderFrame(ProgressionType pgrs) {
  return std::make_unique<ColorDotFrame>(frame_size_, CurrentDotState(pgrs), bg_color_);
}

//------------------------------------
// WiperTarget

utils::DataOrError<Target::RefPtr> WiperTarget::Create(uint32_t duration_ms, const Config& config) {
  ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
  return RefPtr(new WiperTarget(duration_us, config));
}

std::unique_ptr<Frame> WiperTarget::RenderFrame(ProgressionType pgrs) {
  if (pgrs == 0 || pgrs == PGRS_FULL) {
    RGBA8BPixel uniform_color =
        (pgrs == 0) ? ((config_.dir == Direction::LeftToRight) ? config_.r_color : config_.l_color)
                    : ((config_.dir == Direction::LeftToRight) ? config_.l_color : config_.r_color);
    if (uniform_color.IsSolid())
      return std::make_unique<UniformColorFrame>(frame_size_, (RGB8BPixel)uniform_color);
  }

  WiperState state;
  reinterpret_cast<WiperProp&>(state) = config_;
  state.pos_pgrs = (config_.dir == Direction::LeftToRight) ? pgrs : PGRS_FULL - pgrs;

  switch (config_.render_method) {
    case RenderMethod::Realtime:
      return std::make_unique<ComputedWiperFrame>(frame_size_, state, config_.blade_gen);
    case RenderMethod::Precomputed:
      return std::make_unique<CachedWiperFrame>(frame_size_, state, config_.blade_gen);
    default:
      ESP_LOGW(TAG, "Unrecognized wiper render method");
      return std::make_unique<UniformColorFrame>(frame_size_, RGB8BPixel::BLACK());
  }
}

WiperTarget::Config WiperTarget::DotWipeConfig(float width, RGB888 color, Direction dir) {
  return {
      {
          .color = RGBA8BPixel::SOLID(color),
          .l_color = RGBA8BPixel::TRANSPARENT(),
          .r_color = RGBA8BPixel::TRANSPARENT(),
          .width = width,
      },
      .blade_gen = pgrs_map_gaussian,
      .dir = dir,
  };
}

WiperTarget::Config WiperTarget::SpotWipeConfig(float width, RGB888 color, Direction dir) {
  return {
      {
          .color = RGBA8BPixel::SOLID(color),
          .l_color = RGBA8BPixel::TRANSPARENT(color),
          .r_color = RGBA8BPixel::TRANSPARENT(color),
          .width = width,
      },
      .blade_gen = pgrs_map_exponential,
      .dir = dir,
  };
}

WiperTarget::Config WiperTarget::ColorWipeConfig(float width, RGB888 color, Direction dir) {
  return {
      {
          .color = RGBA8BPixel(color, /*alpha=*/0x7F),
          .l_color = (dir == Direction::LeftToRight) ? RGBA8BPixel::SOLID(color)
                                                     : RGBA8BPixel::TRANSPARENT(color),
          .r_color = (dir == Direction::LeftToRight) ? RGBA8BPixel::TRANSPARENT(color)
                                                     : RGBA8BPixel::SOLID(color),
          .width = width,
      },
      .blade_gen = pgrs_map_sigmoid,
      .dir = dir,
  };
}

//------------------------------------
// RGBColorWheelTarget

utils::DataOrError<Target::RefPtr> RGBColorWheelTarget::Create(uint32_t duration_ms,
                                                               const Config& config) {
  ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
  if (config.width < 1) {
    ESP_LOGW(TAG, "Invalid wheel width");
    return ESP_ERR_INVALID_ARG;
  }
  return RefPtr(new RGBColorWheelTarget(duration_us, config));
}

std::unique_ptr<Frame> RGBColorWheelTarget::RenderInit(std::unique_ptr<Frame> base_frame) {
  base_frame = DrawingTarget::RenderInit(std::move(base_frame));
  if (base_frame->Type() == Frame::FrameType::kRGBColorWheel) {
    auto& wheel_frame = static_cast<const RGBColorWheelFrame&>(*base_frame);
    if (wheel_frame.state.width == config_.width &&
        (wheel_frame.state.pos_pgrs & (PGRS_DENOM - 1)) ==
            (config_.wheel_from & (PGRS_DENOM - 1))) {
      // This is just a continuation of a previous color wheel, no need to blend.
      intensity_from_ = wheel_frame.state.intensity;
      return nullptr;
    }
  }
  // Not a continuation from a previous color wheel.
  intensity_from_ = config_.intensity;
  return std::move(base_frame);
}

std::unique_ptr<Frame> RGBColorWheelTarget::RenderFrame(ProgressionType pgrs) {
  ColorWheelState state;
  reinterpret_cast<ColorWheelProp&>(state) = config_;
  state.pos_pgrs = blend_value(config_.wheel_from, config_.wheel_to, pgrs);
  state.intensity = blend_value(intensity_from_, config_.intensity, pgrs);

  return std::make_unique<RGBColorWheelFrame>(frame_size_, state, config_.wheel_gen);
}

//------------------------------------
// HSVColorWheelTarget

utils::DataOrError<Target::RefPtr> HSVColorWheelTarget::Create(uint32_t duration_ms,
                                                               const Config& config) {
  ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
  if (config.width < 1) {
    ESP_LOGW(TAG, "Invalid wheel width");
    return ESP_ERR_INVALID_ARG;
  }
  return RefPtr(new HSVColorWheelTarget(duration_us, config));
}

std::unique_ptr<Frame> HSVColorWheelTarget::RenderInit(std::unique_ptr<Frame> base_frame) {
  base_frame = DrawingTarget::RenderInit(std::move(base_frame));
  if (base_frame->Type() == Frame::FrameType::kHSVColorWheel) {
    auto& wheel_frame = static_cast<const HSVColorWheelFrame&>(*base_frame);
    if (wheel_frame.state.width == config_.width &&
        (wheel_frame.state.pos_pgrs & (PGRS_DENOM - 1)) ==
            (config_.wheel_from & (PGRS_DENOM - 1))) {
      // This is just a continuation of a previous color wheel, no need to blend.
      intensity_from_ = wheel_frame.state.intensity;
      return nullptr;
    }
  }
  // Not a continuation from a previous color wheel.
  intensity_from_ = config_.intensity;
  return std::move(base_frame);
}

std::unique_ptr<Frame> HSVColorWheelTarget::RenderFrame(ProgressionType pgrs) {
  ColorWheelState state;
  reinterpret_cast<ColorWheelProp&>(state) = config_;
  state.pos_pgrs = blend_value(config_.wheel_from, config_.wheel_to, pgrs);
  state.intensity = blend_value(intensity_from_, config_.intensity, pgrs);

  return std::make_unique<HSVColorWheelFrame>(frame_size_, state);
}
}  // namespace zw::esp8266::lightshow