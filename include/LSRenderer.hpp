// Light-show renderer
#ifndef ZWLIGHTSHOW_RENDERER
#define ZWLIGHTSHOW_RENDERER

#include <cstdint>
#include <optional>
#include <queue>
#include <memory>

#include "esp_err.h"

#include "FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "ZWUtils.hpp"

#include "LSFrame.hpp"
#include "LSTarget.hpp"

namespace zw::esp8266::lightshow {

// Realistically, to get ~30fps the maximum number of pixel is around 1024.
// Increasing to 1280, you only get ~25fps which is probably the minimum for transition to be
// considered "smooth". Adding more pixels, transition will start to feel like "turning slides".
//
// A more reasonable way to control super long strips is using 1:N driving, e.g. every group
// of 8 LEDs are driven by one controller, that allows controlling a 4096 *LED* strip as a
// 512 *pixel* strip, which can reach ~60pfs.
inline constexpr StripSizeType kMaxStripSize = 4096;
inline constexpr uint8_t kDefaultFPS = 60;
// This value may be updated higher, but to reliably reach the target fps, you will also need
// to increase RTOS tick frequency to match this value.
inline constexpr uint8_t kMaxFPS = 120;

inline constexpr EventBits_t RENDERER_INIT_FRAME = BIT0;    // Once for the lifetime of a renderer
inline constexpr EventBits_t RENDERER_START_TARGET = BIT1;  // Also the "previous target done" event
inline constexpr EventBits_t RENDERER_ABORT_TARGET = BIT2;
inline constexpr EventBits_t RENDERER_IDLE_TARGET = BIT3;

class Renderer {
 public:
  static utils::DataOrError<std::unique_ptr<Renderer>> Create(StripSizeType strip_size,
                                                              uint8_t target_fps = kDefaultFPS);

  ~Renderer() { vSemaphoreDelete(target_lock_); };

  // Enqueue a target to be rendered.
  // - If `drop_ongoing` is true, all queued and ongoing targets will be dropped,
  //   which will effectively start rendering the new target immediately.
  // - Otherwise, the new target is placed at the end of queue and will be rendered
  //   in the prescribed order.
  void Enqueue(std::unique_ptr<Target> target, bool drop_ongoing = false);

  // Short-hand Enqueue for easy cascading from target creation.
  esp_err_t EnqueueOrError(utils::DataOrError<std::unique_ptr<Target>>&& target,
                           bool drop_ongoing = false) {
    if (!target) return target.error();
    Enqueue(std::move(*target), drop_ongoing);
    return ESP_OK;
  }

  // Clear all queued targets.
  // - If `drop_ongoing` is true, the ongoing target will be abandoned, which will
  //   effectively pause the light-show immediately.
  // - Otherwise, the ongoing target will be rendered to completion, only subsequently
  //   queued targets will be dropped.
  void Clear(bool drop_ongoing = true);

  // Get the next frame to render.
  // Returns nullptr if no new frame is available.
  Frame* RenderFrame();

  // Wait for any ephemeral event to occur.
  // The events are triggered right before the processing happen
  EventBits_t WaitFor(EventBits_t events, TickType_t timeout) const {
    return xEventGroupWaitBits(events_, events, true, false, timeout);
  }

  // Call these function as a part of serialized set up
  // NOT thread-safe!
  StripSizeType StripSize() const { return base_frame_->size; }
  uint32_t FrameInterval() const { return frame_interval_us_; }

 private:
  const uint32_t frame_interval_us_;
  SemaphoreHandle_t target_lock_;
  mutable EventGroupHandle_t events_;
  std::unique_ptr<Frame> base_frame_;

  BlenderFrame* blender_frame_ = nullptr;
  uint64_t base_frame_time_ = 0;
  uint64_t target_base_time_ = 0;
  uint32_t overshoot_us_ = 0;
  std::queue<std::unique_ptr<Target>> targets_;

  Renderer(uint32_t frame_interval_us, SemaphoreHandle_t&& target_lock, EventGroupHandle_t&& events,
           std::unique_ptr<Frame> init_frame)
      : frame_interval_us_(frame_interval_us),
        target_lock_(std::move(target_lock)),
        events_(std::move(events)),
        base_frame_(std::move(init_frame)) {}
};

}  // namespace zw::esp8266::lightshow

#endif  // ZWLIGHTSHOW_RENDERER