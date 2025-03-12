#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"

#include "FreeRTOS.h"
#include "freertos/event_groups.h"

#include "ESPIDF_shim.h"

#include "LSUtils.hpp"
#include "LSTarget.hpp"
#include "LSRenderer.hpp"
#include "LSDriver.hpp"

#define RUN_AT_80MHZ 1

namespace {

inline constexpr char TAG[] = "Main";
namespace LS = ::zw_esp8266::lightshow;

inline constexpr LS::StripSizeType STRIP_SIZE = 111;
inline constexpr uint8_t TARGET_FPS = 80;

inline constexpr uint16_t RENDER_TASK_STACK = 1200;
inline constexpr UBaseType_t RENDER_TASK_PRIORITY = 5;

esp_err_t _lightshow() {
  ESP_LOGI(TAG, "Creating LightShow renderer...");
  ASSIGN_OR_RETURN(auto renderer, LS::Renderer::Create(STRIP_SIZE, TARGET_FPS));

  // You can pre-program transitions before starting the driver
  ESP_LOGI(TAG, "Adding some LightShow targets...");
  ESP_RETURN_ON_ERROR(renderer->Enqueue(LS::UniformColorTarget::Create(1000, {0x32, 0x00, 0x00})));
  ESP_RETURN_ON_ERROR(renderer->Enqueue(LS::UniformColorTarget::Create(1000, {0x00, 0x16, 0x32})));
  ESP_RETURN_ON_ERROR(renderer->Enqueue(LS::UniformColorTarget::Create(1000, {0x16, 0x32, 0x08})));
  ESP_RETURN_ON_ERROR(renderer->Enqueue(LS::UniformColorTarget::Create(1000, {0x00, 0x02, 0x01})));

  ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
           uxTaskGetStackHighWaterMark(NULL));
  ESP_LOGI(TAG, "Setting up LightShow driver...");
  // The jitter buffer is allocated in this API
  ESP_RETURN_ON_ERROR(LS::DriverSetup(LS::CONFIG_WS2812, renderer.get()));
  ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
           uxTaskGetStackHighWaterMark(NULL));

  ESP_LOGI(TAG, "Starting LightShow driver...");
  // This API will create a RTOS task that drives the frame rendering
  // Also starts UART1 TX & HW timer interrupt "task"
  ESP_RETURN_ON_ERROR(LS::DriverStart(RENDER_TASK_STACK, RENDER_TASK_PRIORITY));
  ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
           uxTaskGetStackHighWaterMark(NULL));

  // Transition will immediately start and proceed asynchronously
  vTaskDelay(CONFIG_FREERTOS_HZ / 2);

  // You can also add additional transitions after driver starts
  ESP_LOGI(TAG, "Adding some more LightShow targets...");
  ESP_RETURN_ON_ERROR(renderer->Enqueue(
      LS::DotTarget::Create(4000, {.color = {0x08, 0x16, 0x32}, .glow = 50, .pos_pmr = 300},
                            LS::DotState{.color = {0x00, 0x02, 0x01}, .glow = 50, .pos_pmr = 0})));
  ESP_RETURN_ON_ERROR(renderer->Enqueue(
      LS::DotTarget::Create(5000, {.color = {0x32, 0x16, 0x08}, .glow = 30, .pos_pmr = 1000})));
  ESP_RETURN_ON_ERROR(renderer->Enqueue(
      LS::DotTarget::Create(3000, {.color = {0x16, 0x32, 0x08}, .glow = 80, .pos_pmr = 300})));
  ESP_RETURN_ON_ERROR(renderer->Enqueue(
      LS::DotTarget::Create(2000, {.color = {0x00, 0x24, 0x00}, .glow = 15, .pos_pmr = 100})));
  ESP_RETURN_ON_ERROR(renderer->Enqueue(
      LS::DotTarget::Create(2000, {.color = {0x00, 0x02, 0x01}, .glow = 15, .pos_pmr = 0})));

  // Here is how to wait for the targets
  while (true) {
    if (renderer->WaitFor(LS::RENDERER_FINISH_TARGET | LS::RENDERER_NO_MORE_TARGET, portMAX_DELAY) &
        LS::RENDERER_NO_MORE_TARGET)
      break;
    LS::IOStats stats = LS::DriverStats();
    ESP_LOGI(TAG, "Rendered %d frames (%d underflow, %d near-miss)", stats.frames_rendered,
             stats.underflow_actual, stats.underflow_near_miss);
  }

  ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
           uxTaskGetStackHighWaterMark(NULL));
  ESP_LOGI(TAG, "Stopping LightShow driver...");
  // Stops the rendering task and interrupts
  // Does not release jitter buffer, so `DriverStart()` can be called again.
  ESP_RETURN_ON_ERROR(LS::DriverStop());

  ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
           uxTaskGetStackHighWaterMark(NULL));
  ESP_LOGI(TAG, "Shutdown LightShow driver...");
  // Expect return error code
  LS::DriverSetup(LS::CONFIG_WS2812, nullptr);

  ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
           uxTaskGetStackHighWaterMark(NULL));
  ESP_LOGI(TAG, "Done.");
  return ESP_OK;
}

}  // namespace

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Framebuffer-less WS2812 driver demo");

#if RUN_AT_80MHZ
  vTaskDelay(1);
  esp_set_cpu_freq(ESP_CPU_FREQ_80M);
  vTaskDelay(1);
#endif

  if (_lightshow() != ESP_OK) goto failed;
  return;

failed:
  ESP_LOGE(TAG, "Something failed!");
}
