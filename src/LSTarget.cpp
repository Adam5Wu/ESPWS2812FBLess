#include "LSTarget.hpp"

#include <memory>
#include <cmath>

#include "esp_log.h"

#include "LSUtils.hpp"
#include "LSPixel.hpp"
#include "LSFrame.hpp"

#include "ESPIDF_shim.h"

namespace zw_esp8266::lightshow {
namespace {

inline constexpr char TAG[] = "LSTarget";

// Gaussian "dot": e^(-x^2/2) curve between [-3.16, 3.16]
ProgressionType wiper_blade_gen_dot_(ProgressionType pgrs) {
  // x = 3.16 * rx ==> -x^2/2 = -10 * rx^2 / 2 = -5 * rx^2
  float rx = (float)((int16_t)pgrs - PGRS_MIDWAY) / PGRS_MIDWAY;
  return std::exp(rx * rx * -5.0F) * PGRS_FULL;
}

// Exponential "spot": mirrored e^x curve between [-5.3, 0]
ProgressionType wiper_blade_gen_spot_(ProgressionType pgrs) {
  float x = -5.3F * (float)std::abs((int16_t)pgrs - PGRS_MIDWAY) / PGRS_MIDWAY;
  return std::exp(x) * PGRS_FULL;
}

// Sigmoid "wave": mirrored 2/(1+e^(-x)) curve between [-5.3, 0]
// Note that this is not a complete wave because the blend color is swapped at x = 0
// Effectively we are "fading back" to a new color.
ProgressionType wiper_blade_gen_wave_(ProgressionType pgrs) {
  float x = 5.3F * (float)std::abs((int16_t)pgrs - PGRS_MIDWAY) / PGRS_MIDWAY;
  return 2.0F / (1.0F + std::exp(x)) * PGRS_FULL;
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

DataOrError<std::unique_ptr<Target>> ColorDotTarget::Create(uint32_t duration_ms, DotState dot,
                                                            std::optional<DotState> def_dot) {
  ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
  ESP_RETURN_ON_ERROR(ValidateDots(dot, def_dot));
  return std::unique_ptr<Target>(new ColorDotTarget(duration_us, dot, def_dot));
}

std::unique_ptr<Frame> ColorDotTarget::RenderInit(std::unique_ptr<Frame> base_frame) {
  frame_size_ = base_frame->size;
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

DataOrError<std::unique_ptr<Target>> WiperTarget::Create(uint32_t duration_ms,
                                                         const Config& config) {
  ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
  return std::unique_ptr<Target>(new WiperTarget(duration_us, config));
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
      .blade_gen = wiper_blade_gen_dot_,
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
      .blade_gen = wiper_blade_gen_spot_,
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
      .blade_gen = wiper_blade_gen_wave_,
      .dir = dir,
  };
}

}  // namespace zw_esp8266::lightshow