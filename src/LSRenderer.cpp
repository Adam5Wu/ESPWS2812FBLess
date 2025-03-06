#include "LSRenderer.hpp"

#include "esp_timer.h"
#include "esp_log.h"
namespace zw_esp8266::lightshow {

inline constexpr char TAG[] = "LSRenderer";

DataOrError<std::unique_ptr<Renderer>> Renderer::Create(StripSizeType strip_size,
                                                        uint8_t target_fps) {
  if (strip_size == 0 || strip_size > kMaxStripSize) {
    ESP_LOGW(TAG, "Invalid strip size");
    return ESP_ERR_INVALID_ARG;
  }
  if (target_fps == 0 || target_fps > kMaxFPS) {
    ESP_LOGW(TAG, "Invalid target FPS");
    return ESP_ERR_INVALID_ARG;
  }
  SemaphoreHandle_t target_lock = xSemaphoreCreateMutex();
  if (target_lock == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate target lock");
    return ESP_ERR_NO_MEM;
  }
  EventGroupHandle_t events = xEventGroupCreate();
  if (events == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate events");
    return ESP_ERR_NO_MEM;
  }
  return std::unique_ptr<Renderer>(
      new Renderer(1000000U / target_fps, std::move(target_lock), std::move(events),
                   std::make_unique<UniformColorFrame>(strip_size, BLACK())));
}

void Renderer::Enqueue(std::unique_ptr<Target> target, bool drop_ongoing) {
  // Wait forever for the lock, not expecting failure
  xSemaphoreTake(target_lock_, portMAX_DELAY);
  if (drop_ongoing) {
    if (!targets_.empty()) {
      xEventGroupSetBits(events_, RENDERER_ABORT_TARGET);
      while (!targets_.empty()) targets_.pop();
      target_base_time_ = 0;
    }
  }
  targets_.push(std::move(target));
  xSemaphoreGive(target_lock_);
}

void Renderer::Clear(bool drop_ongoing) {
  // Wait forever for the lock, not expecting failure
  xSemaphoreTake(target_lock_, portMAX_DELAY);
  if (!targets_.empty()) {
    auto cur_target = std::move(targets_.front());
    while (!targets_.empty()) targets_.pop();
    if (drop_ongoing) {
      xEventGroupSetBits(events_, RENDERER_ABORT_TARGET);
      target_base_time_ = 0;
    } else {
      targets_.push(std::move(cur_target));
    }
  }
  xSemaphoreGive(target_lock_);
}

Frame* Renderer::RenderFrame() {
  Frame* result = nullptr;

  // Wait forever for the lock, not expecting failure
  xSemaphoreTake(target_lock_, portMAX_DELAY);
  uint64_t current_time = esp_timer_get_time();

  if (base_frame_time_ == 0) {
    base_frame_time_ = current_time;
    result = base_frame_.get();
    xEventGroupSetBits(events_, RENDERER_INIT_FRAME);
  } else {
    uint64_t delta_t = current_time - base_frame_time_;
    if (delta_t >= frame_interval_us_) {
      // Step by whole frame interval.
      uint32_t whole_frame_time = delta_t - delta_t % frame_interval_us_;
      base_frame_time_ += whole_frame_time;
    next_target:
      if (!targets_.empty()) {
        Target& cur_target = *targets_.front();
        if (target_base_time_ == 0) {
          if (cur_target.RenderInit(*base_frame_) != ESP_OK) {
            ESP_LOGD(TAG, "Failed to initialize target");
            xEventGroupClearBits(events_, ~RENDERER_FAILED_TARGET);
            xEventGroupSetBits(events_, RENDERER_FAILED_TARGET);
            targets_.pop();
            goto next_target;
          }
          target_base_time_ = base_frame_time_ - frame_interval_us_;
          // If we started too late, add missed frame time to overshoot,
          // which will be smoothly caught up later.
          overshoot_us_ += whole_frame_time - frame_interval_us_;
          ESP_LOGD(TAG, "Current overshoot: %d us", overshoot_us_);
          xEventGroupClearBits(events_, ~RENDERER_START_TARGET);
          xEventGroupSetBits(events_, RENDERER_START_TARGET);
        }
        if (overshoot_us_) {
          // Catch up with the overshoots from previous targets.
          // Since the maximum fps is 100, the shortest possible frame interval is 10ms.
          // Speed up by 1/10 of that, i.e. 1ms, should be unnoticeable.
          uint32_t catchup_us = std::min(overshoot_us_, 1000U);
          target_base_time_ -= catchup_us;
          overshoot_us_ -= catchup_us;
        }
        // Note that smooth catchup only applies for overshoots between targets.
        // Any tardiness during a target transition will result in frame skipping.
        uint32_t time_passed = current_time - target_base_time_;
        base_frame_ = cur_target.RenderFrame(time_passed);
        result = base_frame_.get();

        if (time_passed >= cur_target.duration_us) {
          // There maybe some overtime.
          uint32_t overtime = time_passed - cur_target.duration_us;
          // If it is less than one frame interval, it will be automatically corrected.
          // So we only need to track the whole-frame overshoots.
          overshoot_us_ += overtime - overtime % frame_interval_us_;

          xEventGroupClearBits(events_, ~RENDERER_FINISH_TARGET);
          xEventGroupSetBits(events_, RENDERER_FINISH_TARGET);
          targets_.pop();
          target_base_time_ = 0;
        }
      } else {
        // If there are no more targets, we don't need to track the overshoot.
        xEventGroupSetBits(events_, RENDERER_NO_MORE_TARGET);
        overshoot_us_ = 0;
      }
    }
  }
  xSemaphoreGive(target_lock_);
  return result;
}

}  // namespace zw_esp8266::lightshow