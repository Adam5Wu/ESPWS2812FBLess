// Light-show target
#ifndef ZWESP8266_LSTARGET
#define ZWESP8266_LSTARGET

#include <cstdint>
#include <optional>
#include <memory>
#include <utility>

#include "esp_err.h"

#include "LSUtils.hpp"
#include "LSPixel.hpp"
#include "LSFrame.hpp"

namespace zw_esp8266::lightshow {

// With 12-bit progression, this is a little more than 60 seconds.
inline constexpr uint32_t kMaxTargetDurationMS = PGRS_MAX_DIVNUM / 1000;

// Represents a pre-defined state of the light-show.
// The light-show will transition to this state over a certain duration.
class Target {
 public:
  // The duration to reach the target state in microseconds.
  const uint32_t duration_us;

  virtual ~Target() = default;

  // Prepare the target for rendering.
  // The `base_frame` is offered by the rendered as the starting state.
  // - By default the `base_frame` is returned (basically giving back the ownership
  //   to the renderer). The renderer will perform generic color blending between
  //   the `base_frame` and the frames produced by the target, spliced across the
  //   duration of the transition.
  // - If the target will provide its own custom transition from the `base_frame`,
  //   it should take the ownership of the `base_frame` and return nullptr.
  virtual std::unique_ptr<Frame> RenderInit(std::unique_ptr<Frame> base_frame) {
    frame_size_ = base_frame->size;
    return std::move(base_frame);
  }

  // Render a frame of the light-show with the given progression.
  virtual std::unique_ptr<Frame> RenderFrame(ProgressionType pgrs) = 0;

  // Prepare the duration in microseconds.
  // Due to the range and precision of progression computation, we only support
  // duration with granularity of milliseconds, and up to `kMaxTargetDurationMS`.
  static DataOrError<uint32_t> PrepareDuration(uint32_t duration_ms);

 protected:
  StripSizeType frame_size_;

  Target(uint32_t duration_us) : duration_us(duration_us) {}
};

// A target that displays a uniform color.
// Thanks to the generic blending capability of the renderer, this target does *not*
// handle transition at all! So the implementation is extremely simple.
class UniformColorTarget : public Target {
 public:
  const RGB888 color;

  static DataOrError<std::unique_ptr<Target>> Create(uint32_t duration_ms, RGB888 color) {
    ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
    return std::unique_ptr<Target>(new UniformColorTarget(duration_us, color));
  }
  std::unique_ptr<Frame> RenderFrame(ProgressionType pgrs) override {
    // We don't deal with progression, the renderer will blend for us.
    return std::make_unique<UniformColorFrame>(frame_size_, color);
  }

 protected:
  UniformColorTarget(uint32_t duration_us, RGB888 color) : Target(duration_us), color(color) {}
};

// A target that displays a (moving and glowing) colored dot.
// Render each pixel of the dot using real-time floating point arithmetics.
//
// Note that this implementation is just a demonstration of technical possibility, as well as
// a utility to generate a small amount to frame underflow to help determining proper reset times.
// For the best effect, the pixel data produced by complex computations should be pre-computed.
class ColorDotTarget : public Target {
 public:
  const DotState dot;

  static DataOrError<std::unique_ptr<Target>> Create(
      uint32_t duration_ms, DotState dot, std::optional<DotState> def_dot = std::nullopt);

  std::unique_ptr<Frame> RenderInit(std::unique_ptr<Frame> base_frame) override;
  std::unique_ptr<Frame> RenderFrame(ProgressionType pgrs) override;

 protected:
  // If provided, will be used as base_dot if the base_frame is not a `ColorDotFrame`.
  // (Otherwise, will start with a hard-coded default dot, see `RenderInit()`.)
  const std::optional<DotState> def_dot_;
  RGB888 bg_color_;
  DotState base_dot_;

  ColorDotTarget(uint32_t duration_us, DotState dot, std::optional<DotState> def_dot)
      : Target(duration_us), dot(dot), def_dot_(def_dot) {}

  static esp_err_t ValidateDots(DotState dot, std::optional<DotState> def_dot);
  DotState CurrentDotState(ProgressionType pgrs) const;
};

// A target that displays wiper effect.
class WiperTarget : public Target {
 public:
  enum class Direction {
    LeftToRight,  // From the beginning of the strip towards the end.
    RightToLeft,  // From the end of the strip towards the beginning.
  };
  enum class RenderMethod {
    Realtime,
    Precomputed,
  };
  struct Config : WiperProp {
    WiperFrame::BladeGenerator blade_gen;
    Direction dir;
    RenderMethod render_method = RenderMethod::Precomputed;
  };

  static DataOrError<std::unique_ptr<Target>> Create(uint32_t duration_ms, const Config& config);

  // Render the wiper as a "dot", rendered using Gaussian PDF between [-3.3,3.3].
  static Config DotWipeConfig(float width, RGB888 color, Direction dir);

  // Render the wiper as a "spot", rendered using mirrored exponential curve between [-3.3,0].
  // Note that due to the exponential "spike" at the center, if the blade width is small
  // the overall luminosity will decrease significantly when the blade moves in between pixels.
  static Config SpotWipeConfig(float width, RGB888 color, Direction dir);

  // Render the wiper as a "wipe", from the base frame to a uniform color.
  // The transition is rendered using logistic curve between [-3.3, 3.3].
  static Config ColorWipeConfig(float width, RGB888 color, Direction dir);

  std::unique_ptr<Frame> RenderFrame(ProgressionType pgrs) override;

 protected:
  const Config config_;

  WiperTarget(uint32_t duration_us, const Config& config) : Target(duration_us), config_(config) {}
};

// A target that displays RGB color wheel.
class RGBColorWheelTarget : public Target {
 public:
  struct Config : ColorWheelProp {
    ProgressionType wheel_from, wheel_to;
    ProgressionType intensity = PGRS_FULL;
    RGBColorWheelFrame::WheelGenerator wheel_gen = pgrs_map_exponential;
  };

  static DataOrError<std::unique_ptr<Target>> Create(uint32_t duration_ms, const Config& config);

  std::unique_ptr<Frame> RenderInit(std::unique_ptr<Frame> base_frame) override;
  std::unique_ptr<Frame> RenderFrame(ProgressionType pgrs) override;

 protected:
  const Config config_;
  ProgressionType intensity_from_;

  RGBColorWheelTarget(uint32_t duration_us, const Config& config)
      : Target(duration_us), config_(config) {}
};

// A target that displays HSV color wheel.
class HSVColorWheelTarget : public Target {
 public:
  struct Config : ColorWheelProp {
    ProgressionType wheel_from, wheel_to;
    ProgressionType intensity = PGRS_FULL;
  };

  static DataOrError<std::unique_ptr<Target>> Create(uint32_t duration_ms, const Config& config);

  std::unique_ptr<Frame> RenderInit(std::unique_ptr<Frame> base_frame) override;
  std::unique_ptr<Frame> RenderFrame(ProgressionType pgrs) override;

 protected:
  const Config config_;
  ProgressionType intensity_from_;

  HSVColorWheelTarget(uint32_t duration_us, const Config& config)
      : Target(duration_us), config_(config) {}
};

}  // namespace zw_esp8266::lightshow

#endif  // ZWESP8266_LSTARGET