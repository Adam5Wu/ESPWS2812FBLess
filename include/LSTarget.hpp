// Light-show target
#ifndef ZWLIGHTSHOW_TARGET
#define ZWLIGHTSHOW_TARGET

#include <cstdint>
#include <optional>
#include <memory>
#include <utility>
#include <queue>

#include "esp_err.h"

#include "ZWUtils.hpp"

#include "LSUtils.hpp"
#include "LSPixel.hpp"
#include "LSFrame.hpp"

namespace zw::esp8266::lightshow {

// With a 12-bit progression, this is a little more than 60 seconds (~67.1s)
inline constexpr uint32_t kMaxTargetDurationUS = PGRS_MAX_DIVNUM;
inline constexpr uint32_t kMaxTargetDurationMS = kMaxTargetDurationUS / 1000;

inline constexpr uint8_t kNoLoop = 0;
inline constexpr uint8_t kInfiniteLoop = UINT8_MAX;

// Represents a pre-defined state of the light-show.
// The light-show will transition to this state over a certain duration.
class Target {
 public:
  using RefPtr = std::unique_ptr<Target>;

  virtual ~Target() = default;

  // The duration to reach the target state in microseconds.
  virtual uint32_t DurationUS() = 0;

  // Whether the target requests looping
  // 0 = no loop;
  // 1~254 = finite number of loops;
  // 255 = infinite looping (until aborted).
  virtual uint8_t Loop() { return kNoLoop; }

  // Prepare the target for rendering.
  // The `base_frame` is offered by the renderer as the starting state.
  // - By default the `base_frame` is returned (basically giving back the ownership
  //   to the renderer). The renderer will perform generic color blending between
  //   the `base_frame` and the frames produced by the target, spliced across the
  //   duration of the transition.
  // - If the target will provide its own custom transition from the `base_frame`,
  //   it should take the ownership of the `base_frame` and return nullptr.
  virtual std::unique_ptr<Frame> RenderInit(std::unique_ptr<Frame> base_frame) {
    return std::move(base_frame);
  }

  // Render a frame of the light-show with the given progression.
  virtual std::unique_ptr<Frame> RenderFrame(uint32_t time_passed, ProgressionType pgrs) = 0;
};

// Target with static duration
class StaticDurationTarget : public Target {
 public:
  uint32_t DurationUS() override { return duration_us_; }

 protected:
  const uint32_t duration_us_;

  // Convert the duration in milliseconds to microseconds.
  static utils::DataOrError<uint32_t> PrepareDuration(uint32_t duration_ms);

  StaticDurationTarget(uint32_t duration_us) : duration_us_(duration_us) {}
};

class ChainedTarget : public Target {
 public:
  using TargetQueue = std::deque<RefPtr>;

  ChainedTarget() = default;
  ChainedTarget(TargetQueue&& targets) : targets_(std::move(targets)) {}

  void Prepend(RefPtr target) { targets_.push_front(std::move(target)); }
  void Append(RefPtr target) { targets_.push_back(std::move(target)); }

  // Short-hand Enqueue for easy cascading from target creation.
  esp_err_t PrependOrError(utils::DataOrError<RefPtr>&& target) {
    if (!target) return target.error();
    Prepend(std::move(*target));
    return ESP_OK;
  }
  esp_err_t AppendOrError(utils::DataOrError<RefPtr>&& target) {
    if (!target) return target.error();
    Append(std::move(*target));
    return ESP_OK;
  }

  uint32_t DurationUS() {
    // This function is only supposed to be called after rendering start.
#ifdef NDEBUG
    if (!iterator_.has_value()) return 0;
#else
    assert(iterator_.has_value());
#endif

    return (*iterator_ != targets_.end()) ? (**iterator_)->DurationUS() : 0;
  }

  std::unique_ptr<Frame> RenderInit(std::unique_ptr<Frame> base_frame) override {
    if (!iterator_.has_value()) iterator_ = targets_.begin();
    return (**iterator_)->RenderInit(std::move(base_frame));
  }

  std::unique_ptr<Frame> RenderFrame(uint32_t time_passed, ProgressionType pgrs) override {
    // This function is only supposed to be called after rendering start.
#ifdef NDEBUG
    if (!iterator_.has_value()) return nullptr;
#else
    assert(iterator_.has_value());
#endif

    return (**iterator_)->RenderFrame(time_passed, pgrs);
  }

  esp_err_t SetLoop(uint8_t loop) {
    // This function is only supposed to be called **before** rendering start.
    if (iterator_.has_value()) return ESP_ERR_INVALID_STATE;
    
    loop_ = loop;
    return ESP_OK;
  }

  uint8_t Loop() override {
    // This function is only supposed to be called after rendering start.
#ifdef NDEBUG
    if (!iterator_.has_value()) return kNoLoop;
#else
    assert(iterator_.has_value());
#endif

    uint8_t sub_loop = (**iterator_)->Loop();
    if (sub_loop != kNoLoop) return sub_loop;

    if (++*iterator_ == targets_.end()) {
      if (loop_ == 0) return kNoLoop;

      if (loop_ != kInfiniteLoop) --loop_;
      iterator_ = targets_.begin();
    }
    return 1;
  }

 protected:
  TargetQueue targets_;
  uint8_t loop_ = kNoLoop;

  std::optional<TargetQueue::iterator> iterator_;
};

// A target that does nothing (but kill time :P)
class NoopTarget : public StaticDurationTarget {
 public:
  static utils::DataOrError<RefPtr> Create(uint32_t duration_ms) {
    ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
    return RefPtr(new NoopTarget(duration_us));
  }
  std::unique_ptr<Frame> RenderFrame(uint32_t time_passed, ProgressionType pgrs) override {
    return nullptr;
  }

 protected:
  NoopTarget(uint32_t duration_us) : StaticDurationTarget(duration_us) {}
};

class DrawingTarget : public StaticDurationTarget {
 public:
  std::unique_ptr<Frame> RenderInit(std::unique_ptr<Frame> base_frame) override {
    frame_size_ = base_frame->size;
    return Target::RenderInit(std::move(base_frame));
  }

 protected:
  StripSizeType frame_size_;

  DrawingTarget(uint32_t duration_us) : StaticDurationTarget(duration_us) {}
};

// A target that displays a uniform color.
// Thanks to the generic blending capability of the renderer, this target does *not*
// handle transition at all! So the implementation is extremely simple.
class UniformColorTarget : public DrawingTarget {
 public:
  const RGB888 color;

  static utils::DataOrError<RefPtr> Create(uint32_t duration_ms, RGB888 color) {
    ASSIGN_OR_RETURN(uint32_t duration_us, PrepareDuration(duration_ms));
    return RefPtr(new UniformColorTarget(duration_us, color));
  }
  std::unique_ptr<Frame> RenderFrame(uint32_t time_passed, ProgressionType pgrs) override {
    // We don't deal with progression, the renderer will blend for us.
    return std::make_unique<UniformColorFrame>(frame_size_, color);
  }

 protected:
  UniformColorTarget(uint32_t duration_us, RGB888 color)
      : DrawingTarget(duration_us), color(color) {}
};

// A target that displays a (moving and glowing) colored dot.
// Render each pixel of the dot using real-time floating point arithmetics.
//
// Note that this implementation is just a demonstration of technical possibility, as well as
// a utility to generate a small amount to frame underflow to help determining proper reset times.
// For the best effect, the pixel data produced by complex computations should be pre-computed.
class ColorDotTarget : public DrawingTarget {
 public:
  const DotState dot;

  static utils::DataOrError<RefPtr> Create(uint32_t duration_ms, DotState dot,
                                           std::optional<DotState> def_dot = std::nullopt);

  std::unique_ptr<Frame> RenderInit(std::unique_ptr<Frame> base_frame) override;
  std::unique_ptr<Frame> RenderFrame(uint32_t time_passed, ProgressionType pgrs) override;

 protected:
  // If provided, will be used as base_dot if the base_frame is not a `ColorDotFrame`.
  // (Otherwise, will start with a hard-coded default dot, see `RenderInit()`.)
  const std::optional<DotState> def_dot_;
  RGB888 bg_color_;
  DotState base_dot_;

  ColorDotTarget(uint32_t duration_us, DotState dot, std::optional<DotState> def_dot)
      : DrawingTarget(duration_us), dot(dot), def_dot_(def_dot) {}

  static esp_err_t ValidateDots(DotState dot, std::optional<DotState> def_dot);
  DotState CurrentDotState(ProgressionType pgrs) const;
};

// A target that displays wiper effect.
class WiperTarget : public DrawingTarget {
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

  static utils::DataOrError<RefPtr> Create(uint32_t duration_ms, const Config& config);

  // Render the wiper as a "dot", rendered using Gaussian PDF between [-3.3,3.3].
  static Config DotWipeConfig(float width, RGB888 color, Direction dir);

  // Render the wiper as a "spot", rendered using mirrored exponential curve between [-3.3,0].
  // Note that due to the exponential "spike" at the center, if the blade width is small
  // the overall luminosity will decrease significantly when the blade moves in between pixels.
  static Config SpotWipeConfig(float width, RGB888 color, Direction dir);

  // Render the wiper as a "wipe", from the base frame to a uniform color.
  // The transition is rendered using logistic curve between [-3.3, 3.3].
  static Config ColorWipeConfig(float width, RGB888 color, Direction dir);

  std::unique_ptr<Frame> RenderFrame(uint32_t time_passed, ProgressionType pgrs) override;

 protected:
  const Config config_;

  WiperTarget(uint32_t duration_us, const Config& config)
      : DrawingTarget(duration_us), config_(config) {}
};

// A target that displays RGB color wheel.
class RGBColorWheelTarget : public DrawingTarget {
 public:
  struct Config : ColorWheelProp {
    ProgressionType wheel_from, wheel_to;
    ProgressionType intensity = PGRS_FULL;
    RGBColorWheelFrame::WheelGenerator wheel_gen = pgrs_map_exponential;
  };

  static utils::DataOrError<RefPtr> Create(uint32_t duration_ms, const Config& config);

  std::unique_ptr<Frame> RenderInit(std::unique_ptr<Frame> base_frame) override;
  std::unique_ptr<Frame> RenderFrame(uint32_t time_passed, ProgressionType pgrs) override;

 protected:
  const Config config_;
  ProgressionType intensity_from_;

  RGBColorWheelTarget(uint32_t duration_us, const Config& config)
      : DrawingTarget(duration_us), config_(config) {}
};

// A target that displays HSV color wheel.
class HSVColorWheelTarget : public DrawingTarget {
 public:
  struct Config : ColorWheelProp {
    ProgressionType wheel_from, wheel_to;
    ProgressionType intensity = PGRS_FULL;
  };

  static utils::DataOrError<RefPtr> Create(uint32_t duration_ms, const Config& config);

  std::unique_ptr<Frame> RenderInit(std::unique_ptr<Frame> base_frame) override;
  std::unique_ptr<Frame> RenderFrame(uint32_t time_passed, ProgressionType pgrs) override;

 protected:
  const Config config_;
  ProgressionType intensity_from_;

  HSVColorWheelTarget(uint32_t duration_us, const Config& config)
      : DrawingTarget(duration_us), config_(config) {}
};

}  // namespace zw::esp8266::lightshow

#endif  // ZWLIGHTSHOW_TARGET