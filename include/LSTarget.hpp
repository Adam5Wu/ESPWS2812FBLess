// Light-show target
#ifndef ZWESP8266_LSTARGET
#define ZWESP8266_LSTARGET

#include <cstdint>
#include <optional>
#include <memory>

#include "esp_err.h"

#include "LSPixel.hpp"
#include "LSUtils.hpp"
#include "LSFrame.hpp"

namespace zw_esp8266::lightshow {

inline constexpr uint32_t kMaxTargetDurationMS = 360 * 1000;

// Represents a pre-defined state of the light-show.
// The light-show will transition to this state over a certain duration.
class Target {
 public:
  // The duration to reach the target state in microseconds.
  const uint32_t duration_us;

  virtual ~Target() = default;

  // Prepare the target for rendering.
  virtual esp_err_t RenderInit(const Frame& base_frame) = 0;

  // Render a frame of the light-show, given the time passed since the target started.
  virtual std::unique_ptr<Frame> RenderFrame(uint32_t time_passed_us) = 0;

  // Prepare the duration in microseconds.
  // Due to the range and precision of color transition computation, we only support
  // duration with granularity of milliseconds, and up to kMaxTargetDurationMS.
  static DataOrError<uint32_t> PrepareDuration(uint32_t duration_ms);

 protected:
  Target(uint32_t duration_us) : duration_us(duration_us) {}
};

// A target that displays a uniform color.
class UniformColorTarget : public Target {
 public:
  static DataOrError<std::unique_ptr<Target>> Create(uint32_t duration_ms, RGB8BPixel color);
  esp_err_t RenderInit(const Frame& base_frame) override;
  std::unique_ptr<Frame> RenderFrame(uint32_t time_passed_us) override;

 protected:
  const RGB8BPixel color_;

  RGB8BPixel base_color_;
  StripSizeType frame_size_;

  UniformColorTarget(uint32_t duration_us, RGB8BPixel color) : Target(duration_us), color_(color) {}
};

class DotTarget : public Target {
 public:
  static DataOrError<std::unique_ptr<Target>> Create(
      uint32_t duration_ms, DotState dot, std::optional<DotState> def_dot = std::nullopt);
  esp_err_t RenderInit(const Frame& base_frame) override;
  std::unique_ptr<Frame> RenderFrame(uint32_t time_passed_us) override;

 protected:
  // The target dot state
  const DotState dot_;
  // If provided, will be used as base_dot if the base_frame is not a `ColorDotFrame`.
  // (Otherwise, will start with a hard-coded default dot, see `RenderInit()`.)
  const std::optional<DotState> def_dot_;

  DotState base_dot_;
  RGB8BPixel bg_color_;
  StripSizeType frame_size_;

  DotTarget(uint32_t duration_us, DotState dot, std::optional<DotState> def_dot)
      : Target(duration_us), dot_(dot), def_dot_(def_dot) {}
};

}  // namespace zw_esp8266::lightshow

#endif  // ZWESP8266_LSTARGET