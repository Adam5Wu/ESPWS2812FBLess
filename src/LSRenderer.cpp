#include "LSRenderer.hpp"

#include "esp_timer.h"
#include "esp_log.h"

#include "ZWUtils.hpp"

#include "LSUtils.hpp"
#include "LSPixel.hpp"
#include "LSFrame.hpp"
#include "LSTarget.hpp"

namespace zw::esp8266::lightshow {

namespace {

inline constexpr char TAG[] = "LSRenderer";

inline constexpr uint8_t SMOOTH_BLEND_DIVISOR = 3;
inline constexpr uint8_t SMOOTH_BLEND_2X_FRAC_SIZE = PGRS_PRECISION - SMOOTH_BLEND_DIVISOR - 1;
inline constexpr uint8_t SMOOTH_BLEND_4X_FRAC_SIZE = SMOOTH_BLEND_2X_FRAC_SIZE - 1;
inline constexpr ProgressionType SMOOTH_BLEND_2X_FRAC = (1 << SMOOTH_BLEND_2X_FRAC_SIZE) - 1;
inline constexpr ProgressionType SMOOTH_BLEND_4X_FRAC = (1 << SMOOTH_BLEND_4X_FRAC_SIZE) - 1;
inline constexpr ProgressionType SMOOTH_BLEND_HIGH =
    PGRS_DENOM - (PGRS_DENOM >> SMOOTH_BLEND_DIVISOR);

// Pseudo-random mapping to reduce sweeping effect
inline constexpr uint8_t SMOOTH_BLEND_4X_RAND_MAP[] = {
    0, 3, 1, 2, 6, 4, 7, 5, 11, 9, 10, 8, 13, 14, 12, 15,
};

std::vector<uint8_t> AlphaMask2X(ProgressionType pgrs) {
  ProgressionType pgrs_wait = pgrs & SMOOTH_BLEND_HIGH;
  ProgressionType pgrs_active = pgrs_wait | ((pgrs & SMOOTH_BLEND_2X_FRAC) << 1);

  std::vector<uint8_t> alpha_mask(2);
  uint8_t active_idx = (pgrs >> SMOOTH_BLEND_2X_FRAC_SIZE) & 1;
  if (active_idx) {
    alpha_mask[0] = pgrs_to_alpha(pgrs_wait | (SMOOTH_BLEND_2X_FRAC << 1));
    alpha_mask[1] = pgrs_to_alpha(pgrs_active);
  } else {
    alpha_mask[0] = pgrs_to_alpha(pgrs_active);
    alpha_mask[1] = pgrs_to_alpha(pgrs_wait);
  }
  return alpha_mask;
}

std::vector<uint8_t> AlphaMask4X(ProgressionType pgrs) {
  ProgressionType pgrs_wait = pgrs & SMOOTH_BLEND_HIGH;
  ProgressionType pgrs_active = pgrs_wait | ((pgrs & SMOOTH_BLEND_4X_FRAC) << 2);

  std::vector<uint8_t> alpha_mask(4);
  uint8_t active_idx = (pgrs >> SMOOTH_BLEND_4X_FRAC_SIZE) & 3;
  for (uint8_t i = 0; i < 4; i++) {
    if (i < active_idx) {
      alpha_mask[i] = pgrs_to_alpha(pgrs_wait | (SMOOTH_BLEND_4X_FRAC << 2));
    } else if (i == active_idx) {
      alpha_mask[i] = pgrs_to_alpha(pgrs_active);
    } else {
      alpha_mask[i] = pgrs_to_alpha(pgrs_wait);
    }
  }
  return alpha_mask;
}

std::vector<uint8_t> AlphaMask4XR(ProgressionType pgrs) {
  ProgressionType pgrs_wait = pgrs & SMOOTH_BLEND_HIGH;
  ProgressionType pgrs_active = pgrs_wait | ((pgrs & SMOOTH_BLEND_4X_FRAC) << 2);

  std::vector<uint8_t> alpha_mask(4);
  uint8_t active_idx = (pgrs >> SMOOTH_BLEND_4X_FRAC_SIZE) & 3;
  for (uint8_t i = 0; i < 4; i++) {
    if (i < active_idx) {
      alpha_mask[SMOOTH_BLEND_4X_RAND_MAP[i]] =
          pgrs_to_alpha(pgrs_wait | (SMOOTH_BLEND_4X_FRAC << 2));
    } else if (i == active_idx) {
      alpha_mask[SMOOTH_BLEND_4X_RAND_MAP[i]] = pgrs_to_alpha(pgrs_active);
    } else {
      alpha_mask[SMOOTH_BLEND_4X_RAND_MAP[i]] = pgrs_to_alpha(pgrs_wait);
    }
  }
  return alpha_mask;
}

std::vector<uint8_t> AlphaMask4X4R(ProgressionType pgrs) {
  ProgressionType pgrs_wait = pgrs & SMOOTH_BLEND_HIGH;
  ProgressionType pgrs_active = pgrs_wait | ((pgrs & SMOOTH_BLEND_4X_FRAC) << 2);

  std::vector<uint8_t> alpha_mask(4 * 4);
  uint8_t active_idx = (pgrs >> SMOOTH_BLEND_4X_FRAC_SIZE) & 3;
  for (uint8_t i = 0; i < 4 * 4; i++) {
    uint8_t idx = i % 4;
    if (idx < active_idx) {
      alpha_mask[SMOOTH_BLEND_4X_RAND_MAP[i]] =
          pgrs_to_alpha(pgrs_wait | (SMOOTH_BLEND_4X_FRAC << 2));
    } else if (idx == active_idx) {
      alpha_mask[SMOOTH_BLEND_4X_RAND_MAP[i]] = pgrs_to_alpha(pgrs_active);
    } else {
      alpha_mask[SMOOTH_BLEND_4X_RAND_MAP[i]] = pgrs_to_alpha(pgrs_wait);
    }
  }
  return alpha_mask;
}

}  // namespace

utils::DataOrError<std::unique_ptr<Renderer>> Renderer::Create(StripSizeType strip_size,
                                                               uint8_t target_fps,
                                                               BlendMode blend_mode) {
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
  return std::unique_ptr<Renderer>(new Renderer(
      1000000U / target_fps, std::move(target_lock), std::move(events),
      std::make_unique<UniformColorFrame>(strip_size, RGB8BPixel::BLACK()), blend_mode));
}

void Renderer::Enqueue(Target::RefPtr target) {
  // Wait forever for the lock, not expecting failure
  xSemaphoreTake(target_lock_, portMAX_DELAY);
  targets_.push(std::move(target));
  xEventGroupClearBits(events_, RENDERER_IDLE_TARGET);
  xSemaphoreGive(target_lock_);
}

void Renderer::Skip() {
  // Wait forever for the lock, not expecting failure
  xSemaphoreTake(target_lock_, portMAX_DELAY);
  if (!targets_.empty()) {
    xEventGroupSetBits(events_, RENDERER_ABORT_TARGET);
    targets_.pop();
    target_base_time_ = 0;
  }
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
        uint32_t target_duration = cur_target.DurationUS();

        ProgressionType pgrs = progression(time_passed, target_duration);
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

          if (cur_target.Loop() == kNoLoop) targets_.pop();
          target_base_time_ = 0;

          // There maybe some overtime.
          uint32_t overtime = time_passed - target_duration;
          // If it is less than one frame interval, it will be automatically corrected.
          // So we only need to track the whole-frame overshoots.
          overshoot_us_ += overtime - overtime % frame_interval_us_;
        } else {
          if (new_frame_ != nullptr) {
            // Blend frame if requested.
            if (blender_frame_ != nullptr) {
              switch (blend_mode_) {
                case BlendMode::BASIC:
                  blender_frame_->UpdateOverlay(std::move(new_frame_), pgrs_to_alpha(pgrs));
                  break;

                case BlendMode::SMOOTH_2X:
                  blender_frame_->UpdateOverlay(std::move(new_frame_), AlphaMask2X(pgrs));
                  break;

                case BlendMode::SMOOTH_4X:
                  blender_frame_->UpdateOverlay(std::move(new_frame_), AlphaMask4X(pgrs));
                  break;
                case BlendMode::SMOOTH_4XR:
                  blender_frame_->UpdateOverlay(std::move(new_frame_), AlphaMask4XR(pgrs));
                  break;
                case BlendMode::SMOOTH_4X4R:
                  blender_frame_->UpdateOverlay(std::move(new_frame_), AlphaMask4X4R(pgrs));
                  break;

                default:
                  ESP_LOGE(TAG, "Unknown blend mode");
                  assert(false);
              }
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