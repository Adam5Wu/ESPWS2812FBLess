#include "LSRenderer.hpp"

#include "esp_timer.h"
#include "esp_log.h"

#include "ZWUtils.hpp"

#include "LSPixel.hpp"
#include "LSFrame.hpp"
#include "LSTarget.hpp"

namespace zw::esp8266::lightshow {

inline constexpr char TAG[] = "LSRenderer";

utils::DataOrError<std::unique_ptr<Renderer>> Renderer::Create(StripSizeType strip_size,
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
                   std::make_unique<UniformColorFrame>(strip_size, RGB8BPixel::BLACK())));
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
  xEventGroupClearBits(events_, RENDERER_IDLE_TARGET);
  xSemaphoreGive(target_lock_);
}

void Renderer::Clear(bool drop_ongoing) {
  // Wait forever for the lock, not expecting failure
  xSemaphoreTake(target_lock_, portMAX_DELAY);
  if (!targets_.empty()) {
    if (drop_ongoing) xEventGroupSetBits(events_, RENDERER_ABORT_TARGET);

    auto cur_target = std::move(targets_.front());
    while (!targets_.empty()) targets_.pop();
    if (drop_ongoing) {
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
    xEventGroupSetBits(events_, RENDERER_INIT_FRAME);
    result = base_frame_.get();
  } else {
    uint64_t delta_t = current_time - base_frame_time_;
    if (delta_t >= frame_interval_us_) {
      // Step by whole frame interval.
      uint32_t whole_frame_time = delta_t - delta_t % frame_interval_us_;
      base_frame_time_ += whole_frame_time;

      if (!targets_.empty()) {
        Target& cur_target = *targets_.front();
        if (target_base_time_ == 0) {
          auto base_frame = cur_target.RenderInit(std::move(base_frame_));
          if (base_frame != nullptr) {
            ESP_LOGD(TAG, "Frame blending requested");
            auto blender_frame = std::make_unique<BlenderFrame>(std::move(base_frame));
            blender_frame_ = blender_frame.get();
            base_frame_ = std::move(blender_frame);
          } else {
            ESP_LOGD(TAG, "No frame blending");
            blender_frame_ = nullptr;
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
          // Speed up each frame's progress by 1ms should be unnoticeable.
          uint32_t catchup_us = std::min(overshoot_us_, 1000U);
          target_base_time_ -= catchup_us;
          overshoot_us_ -= catchup_us;
        }
        // Note that smooth catchup only applies for overshoots between targets.
        // Any tardiness during a target transition will result in frame skipping.
        uint32_t time_passed = current_time - target_base_time_;
        ProgressionType pgrs = progression(time_passed, cur_target.duration_us);
        auto new_frame_ = cur_target.RenderFrame(pgrs);

        if (pgrs == PGRS_FULL) {
          if (new_frame_ != nullptr) {
            // This is the last frame, if it is not translucent no blending needed.
            if (blender_frame_ == nullptr || !new_frame_->IsTranslucent()) {
              base_frame_ = std::move(new_frame_);
            } else {
              // Blending is still needed for translucent last frame.
              blender_frame_->UpdateOverlay(std::move(new_frame_), UINT8_MAX);
            }
          } else {
            // Re-render the same frame
            base_frame_->Reset();
          }

          targets_.pop();
          target_base_time_ = 0;

          // There maybe some overtime.
          uint32_t overtime = time_passed - cur_target.duration_us;
          // If it is less than one frame interval, it will be automatically corrected.
          // So we only need to track the whole-frame overshoots.
          overshoot_us_ += overtime - overtime % frame_interval_us_;
        } else {
          if (new_frame_ != nullptr) {
            // Blend frame if requested.
            if (blender_frame_ != nullptr) {
              blender_frame_->UpdateOverlay(std::move(new_frame_), pgrs_to_alpha(pgrs));
            } else {
              base_frame_ = std::move(new_frame_);
            }
          } else {
            // Re-render the same frame
            base_frame_->Reset();
          }
        }

        // A new frame has been rendered
        result = base_frame_.get();
      } else {
        // If there are no more targets, we don't need to track the overshoot.
        xEventGroupSetBits(events_, RENDERER_IDLE_TARGET);
        overshoot_us_ = 0;

        // No frame was rendered
      }
    }
  }
  xSemaphoreGive(target_lock_);
  return result;
}

}  // namespace zw::esp8266::lightshow