#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "FreeRTOS.h"
#include "freertos/event_groups.h"

#include "ZWUtils.hpp"

#include "LSUtils.hpp"
#include "LSTarget.hpp"
#include "LSRenderer.hpp"
#include "LSDriver.hpp"

#define HW_TIMER_TEST 0
#define RUN_AT_80MHZ 1

#if HW_TIMER_TEST
#include "driver/hw_timer.h"
#include "esp8266/timer_struct.h"
#include "esp8266/timer_register.h"
#endif

namespace zw::esp8266::lightshow::testing {
namespace {

inline constexpr char TAG[] = "Main";

inline constexpr StripSizeType STRIP_SIZE = 60;
inline constexpr uint8_t TARGET_FPS = 80;
// inline const IOConfig CONFIG_WS2812 = CONFIG_WS2812_CLASSIC();
inline const IOConfig CONFIG_WS2812 = CONFIG_WS2812_NEW();
// inline const IOConfig CONFIG_WS2812 = CONFIG_WS2812_CUSTOM(320, 40, 270);

inline constexpr uint16_t RENDER_TASK_STACK = 1200;
inline constexpr UBaseType_t RENDER_TASK_PRIORITY = 10;

void _lightshow_driver_stats() {
  IOStats stats = DriverStats();
  uint32_t duration_ms = std::max(1U, (uint32_t)(esp_timer_get_time() - stats.start_time) / 1000);
  uint16_t fps10x = stats.frames_rendered * 100 / (duration_ms / 100);
  ESP_LOGI(TAG, "Rendered %d frames in %d ms (%d.%d fps)", stats.frames_rendered, duration_ms,
           fps10x / 10, fps10x % 10);
  ESP_LOGI(TAG, "Underflow occurred in %d frames, and there were %d near-misses",
           stats.underflow_actual, stats.underflow_near_miss);
  uint16_t idle_pml = stats.idle_wait / duration_ms;
  ESP_LOGI(TAG, "Driver idle waited %d ms (%d.%d%%) and busy waited %d ms", stats.idle_wait / 1000,
           idle_pml / 10, idle_pml % 10, stats.busy_wait / 1000);
#if ISR_DEVELOPMENT
  ESP_LOGI(TAG, "ISR data serving latency (us): [%d, %d]",
           stats.isr_process_latency_low / g_esp_ticks_per_us,
           stats.isr_process_latency_high / g_esp_ticks_per_us);
#endif
}

void _lightshow_wait_for_finish(const Renderer& renderer, int& target_idx) {
  while (true) {
    auto events = renderer.WaitFor(RENDERER_START_TARGET | RENDERER_IDLE_TARGET, portMAX_DELAY);
    if (events & RENDERER_IDLE_TARGET) break;
    ESP_LOGI(TAG, "Target #%d was reached", ++target_idx);
  }
  _lightshow_driver_stats();
  ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
           uxTaskGetStackHighWaterMark(NULL));
  ESP_LOGI(TAG, "----------------------------------------------------------------");
}

esp_err_t _lightshow() {
  ESP_LOGI(TAG, "Creating LightShow renderer...");
  ASSIGN_OR_RETURN(auto renderer,
                   Renderer::Create(STRIP_SIZE, TARGET_FPS, Renderer::BlendMode::SMOOTH_4X4R));

  // You can pre-program transitions before starting the driver
  int target_idx = 0;
  ESP_LOGI(TAG, "Test uniform color LightShow targets...");
  std::unique_ptr<ChainedTarget> chained_target = std::make_unique<ChainedTarget>();
  ESP_RETURN_ON_ERROR(
      chained_target->AppendOrError(UniformColorTarget::Create(5000, {0x02, 0x04, 0x07})));
  ESP_RETURN_ON_ERROR(
      chained_target->AppendOrError(UniformColorTarget::Create(1000, RGB8BPixel::BLACK())));
  ESP_RETURN_ON_ERROR(chained_target->SetLoop(2));
  renderer->Enqueue(std::move(chained_target));
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(1000, {0x32, 0x00, 0x00})));
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(1000, {0x00, 0x16, 0x32})));
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(1000, {0x16, 0x32, 0x08})));
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(1000, {0x00, 0x02, 0x01})));

  ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
           uxTaskGetStackHighWaterMark(NULL));
  ESP_LOGI(TAG, "Setting up LightShow driver...");
  // The jitter buffer is allocated in this API
  ESP_RETURN_ON_ERROR(DriverSetup(CONFIG_WS2812, renderer.get()));
  ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
           uxTaskGetStackHighWaterMark(NULL));

  ESP_LOGI(TAG, "Starting LightShow driver...");
#if ISR_DEVELOPMENT
  // Preallocate frame underflow debug counters
  std::vector<UnderflowCounters> underflow_debug_counters(STRIP_SIZE, UnderflowCounters{});
#endif
  // This API will create a RTOS task that drives the frame rendering
  // Also starts UART1 TX & HW timer interrupt "task"
  ESP_RETURN_ON_ERROR(DriverStart(RENDER_TASK_STACK, RENDER_TASK_PRIORITY
#if ISR_DEVELOPMENT
                                  ,
                                  underflow_debug_counters.data()
#endif
                                      ));
  ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
           uxTaskGetStackHighWaterMark(NULL));

  // Transition will immediately start and proceed asynchronously
  vTaskDelay(CONFIG_FREERTOS_HZ / 2);

  // You can also add additional transitions after driver starts
  ESP_LOGI(TAG, "Test realtime computed color dot LightShow targets...");
  ESP_RETURN_ON_ERROR(renderer->EnqueueOrError(
      ColorDotTarget::Create(4000, {.color = {0x08, 0x16, 0x32}, .glow = 50, .pos_pgrs = PGRS(0.3)},
                             DotState{.color = {0x00, 0x02, 0x01}, .glow = 50, .pos_pgrs = 0})));
  ESP_RETURN_ON_ERROR(renderer->EnqueueOrError(ColorDotTarget::Create(
      5000, {.color = {0x32, 0x08, 0x16}, .glow = 30, .pos_pgrs = PGRS(1)})));
  ESP_RETURN_ON_ERROR(renderer->EnqueueOrError(ColorDotTarget::Create(
      3000, {.color = {0x16, 0x32, 0x08}, .glow = 80, .pos_pgrs = PGRS(0.25)})));
  ESP_RETURN_ON_ERROR(renderer->EnqueueOrError(ColorDotTarget::Create(
      2000, {.color = {0x00, 0x24, 0x00}, .glow = 30, .pos_pgrs = PGRS(0.1)})));
  ESP_RETURN_ON_ERROR(renderer->EnqueueOrError(ColorDotTarget::Create(
      2000, {.color = {0x08, 0x00, 0x24}, .glow = 15, .pos_pgrs = PGRS(0)})));
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(200, {0x08, 0x24, 0x16})));
  // Wait for all targets have been reached
  _lightshow_wait_for_finish(*renderer, target_idx);

  // You can add additional transitions after all previous targets completed for a while
  vTaskDelay(CONFIG_FREERTOS_HZ / 2);

  // Wiper targets testing
  WiperTarget::Config wiper_config;

  ESP_LOGI(TAG, "Test realtime computed spot wipe targets...");
  DriverStats();
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(800, {0x00, 0x02, 0x01})));
  wiper_config =
      WiperTarget::SpotWipeConfig(0.05, {0x24, 0x08, 0x16}, WiperTarget::Direction::LeftToRight);
  wiper_config.render_method = WiperTarget::RenderMethod::Realtime;
  renderer->EnqueueOrError(WiperTarget::Create(8000, wiper_config));
  wiper_config =
      WiperTarget::SpotWipeConfig(0.1, {0x16, 0x08, 0x24}, WiperTarget::Direction::RightToLeft);
  wiper_config.render_method = WiperTarget::RenderMethod::Realtime;
  renderer->EnqueueOrError(WiperTarget::Create(4000, wiper_config));
  // Wait for completion
  _lightshow_wait_for_finish(*renderer, target_idx);

  ESP_LOGI(TAG, "Test precomputed spot wipe targets...");
  DriverStats();
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(800, {0x00, 0x02, 0x01})));
  wiper_config =
      WiperTarget::SpotWipeConfig(0.05, {0x24, 0x08, 0x16}, WiperTarget::Direction::LeftToRight);
  wiper_config.render_method = WiperTarget::RenderMethod::Precomputed;
  renderer->EnqueueOrError(WiperTarget::Create(8000, wiper_config));
  wiper_config =
      WiperTarget::SpotWipeConfig(0.1, {0x16, 0x08, 0x24}, WiperTarget::Direction::RightToLeft);
  wiper_config.render_method = WiperTarget::RenderMethod::Precomputed;
  renderer->EnqueueOrError(WiperTarget::Create(4000, wiper_config));
  // Wait for completion
  _lightshow_wait_for_finish(*renderer, target_idx);

  ESP_LOGI(TAG, "Test realtime computed dot wipe targets...");
  DriverStats();
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(800, {0x00, 0x02, 0x01})));
  wiper_config =
      WiperTarget::DotWipeConfig(0.05, {0x24, 0x08, 0x16}, WiperTarget::Direction::LeftToRight);
  wiper_config.render_method = WiperTarget::RenderMethod::Realtime;
  renderer->EnqueueOrError(WiperTarget::Create(8000, wiper_config));
  wiper_config =
      WiperTarget::DotWipeConfig(0.1, {0x16, 0x08, 0x24}, WiperTarget::Direction::RightToLeft);
  wiper_config.render_method = WiperTarget::RenderMethod::Realtime;
  renderer->EnqueueOrError(WiperTarget::Create(4000, wiper_config));
  // Wait for completion
  _lightshow_wait_for_finish(*renderer, target_idx);

  ESP_LOGI(TAG, "Test precomputed dot wipe targets...");
  DriverStats();
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(800, {0x00, 0x02, 0x01})));
  wiper_config =
      WiperTarget::DotWipeConfig(0.05, {0x24, 0x08, 0x16}, WiperTarget::Direction::LeftToRight);
  wiper_config.render_method = WiperTarget::RenderMethod::Precomputed;
  renderer->EnqueueOrError(WiperTarget::Create(8000, wiper_config));
  wiper_config =
      WiperTarget::DotWipeConfig(0.1, {0x16, 0x08, 0x24}, WiperTarget::Direction::RightToLeft);
  wiper_config.render_method = WiperTarget::RenderMethod::Precomputed;
  renderer->EnqueueOrError(WiperTarget::Create(4000, wiper_config));
  // Wait for completion
  _lightshow_wait_for_finish(*renderer, target_idx);

  ESP_LOGI(TAG, "Test realtime computed color wipe targets...");
  DriverStats();
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(800, {0x00, 0x02, 0x01})));
  wiper_config =
      WiperTarget::ColorWipeConfig(0.15, {0x00, 0x08, 0x32}, WiperTarget::Direction::LeftToRight);
  wiper_config.render_method = WiperTarget::RenderMethod::Realtime;
  renderer->EnqueueOrError(WiperTarget::Create(8000, wiper_config));
  wiper_config =
      WiperTarget::ColorWipeConfig(0.20, {0x32, 0x08, 0x00}, WiperTarget::Direction::RightToLeft);
  wiper_config.render_method = WiperTarget::RenderMethod::Realtime;
  renderer->EnqueueOrError(WiperTarget::Create(4000, wiper_config));
  // Wait for completion
  _lightshow_wait_for_finish(*renderer, target_idx);

  ESP_LOGI(TAG, "Test precomputed color wipe targets...");
  DriverStats();
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(800, {0x00, 0x02, 0x01})));
  wiper_config =
      WiperTarget::ColorWipeConfig(0.15, {0x00, 0x08, 0x32}, WiperTarget::Direction::LeftToRight);
  wiper_config.render_method = WiperTarget::RenderMethod::Precomputed;
  renderer->EnqueueOrError(WiperTarget::Create(8000, wiper_config));
  wiper_config =
      WiperTarget::ColorWipeConfig(0.20, {0x32, 0x08, 0x00}, WiperTarget::Direction::RightToLeft);
  wiper_config.render_method = WiperTarget::RenderMethod::Precomputed;
  renderer->EnqueueOrError(WiperTarget::Create(4000, wiper_config));
  // Wait for completion
  _lightshow_wait_for_finish(*renderer, target_idx);

  ESP_LOGI(TAG, "Test (precomputed) RGB color wheel targets...");
  DriverStats();
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(800, {0x00, 0x02, 0x01})));
  ESP_RETURN_ON_ERROR(renderer->EnqueueOrError(RGBColorWheelTarget::Create(
      2000, {{.width = 60}, .wheel_from = 0, .wheel_to = PGRS(0.2), .intensity = PGRS(0.8)})));
  ESP_RETURN_ON_ERROR(renderer->EnqueueOrError(RGBColorWheelTarget::Create(
      8000,
      {{.width = 60}, .wheel_from = PGRS(0.2), .wheel_to = PGRS(1), .intensity = PGRS(0.5)})));
  ESP_RETURN_ON_ERROR(renderer->EnqueueOrError(RGBColorWheelTarget::Create(
      5000, {{.width = 60}, .wheel_from = PGRS(0), .wheel_to = PGRS(1), .intensity = PGRS(0.5)})));
  ESP_RETURN_ON_ERROR(renderer->EnqueueOrError(RGBColorWheelTarget::Create(
      2000,
      {{.width = 60}, .wheel_from = PGRS(0), .wheel_to = PGRS(0.2), .intensity = PGRS(0.1)})));
  // Wait for completion
  _lightshow_wait_for_finish(*renderer, target_idx);

  // Wiper frame can be blended on top of a color wheel
  // Demonstrate translucent blending capability
  ESP_LOGI(TAG, "Blending a wiper frame on top of a color wheel background...");
  DriverStats();
  wiper_config =
      WiperTarget::SpotWipeConfig(0.05, {0x24, 0x08, 0x16}, WiperTarget::Direction::LeftToRight);
  wiper_config.render_method = WiperTarget::RenderMethod::Precomputed;
  renderer->EnqueueOrError(WiperTarget::Create(8000, wiper_config));
  wiper_config =
      WiperTarget::SpotWipeConfig(0.1, {0x16, 0x08, 0x24}, WiperTarget::Direction::RightToLeft);
  wiper_config.render_method = WiperTarget::RenderMethod::Precomputed;
  renderer->EnqueueOrError(WiperTarget::Create(4000, wiper_config));
  // Wait for completion
  _lightshow_wait_for_finish(*renderer, target_idx);

  ESP_LOGI(TAG, "Test (precomputed) HSV color wheel targets...");
  DriverStats();
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(800, {0x00, 0x02, 0x01})));
  ESP_RETURN_ON_ERROR(renderer->EnqueueOrError(HSVColorWheelTarget::Create(
      2000, {{.width = 60}, .wheel_from = 0, .wheel_to = PGRS(0.2), .intensity = PGRS(0.8)})));
  ESP_RETURN_ON_ERROR(renderer->EnqueueOrError(HSVColorWheelTarget::Create(
      8000,
      {{.width = 60}, .wheel_from = PGRS(0.2), .wheel_to = PGRS(1), .intensity = PGRS(0.5)})));
  ESP_RETURN_ON_ERROR(renderer->EnqueueOrError(HSVColorWheelTarget::Create(
      5000, {{.width = 60}, .wheel_from = PGRS(0), .wheel_to = PGRS(1), .intensity = PGRS(0.5)})));
  ESP_RETURN_ON_ERROR(renderer->EnqueueOrError(HSVColorWheelTarget::Create(
      2000,
      {{.width = 60}, .wheel_from = PGRS(0), .wheel_to = PGRS(0.2), .intensity = PGRS(0.1)})));

  // Wait for completion
  _lightshow_wait_for_finish(*renderer, target_idx);

  // Finishing up the show
  ESP_LOGI(TAG, "Turning off the strip...");
  DriverStats();
  ESP_RETURN_ON_ERROR(
      renderer->EnqueueOrError(UniformColorTarget::Create(800, RGB8BPixel::BLACK())));
  _lightshow_wait_for_finish(*renderer, target_idx);

  ESP_LOGI(TAG, "Stopping LightShow driver...");
  // Stops the rendering task and interrupts
  // Does not release jitter buffer, so `DriverStart()` can be called again.
  ESP_RETURN_ON_ERROR(DriverStop());

  ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
           uxTaskGetStackHighWaterMark(NULL));
  ESP_LOGI(TAG, "Shutdown LightShow driver...");
  // Expect return error code
  DriverSetup(CONFIG_WS2812, nullptr);

  ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
           uxTaskGetStackHighWaterMark(NULL));

#if ISR_DEVELOPMENT
  ESP_LOGI(TAG, "Underflow counters:");
  for (int i = 0; i < STRIP_SIZE; i++) {
    const auto& counters = underflow_debug_counters[i];
    if (counters.actual | counters.near_miss) {
      ESP_LOGI(TAG, "Pixel #%04d: actual: %d, near-miss: %d", i, counters.actual,
               counters.near_miss);
    }
  }
#endif

  ESP_LOGI(TAG, "Done.");
  return ESP_OK;
}

#if HW_TIMER_TEST

inline constexpr uint16_t HW_TIMER_TEST_MIN = 10;
inline constexpr uint16_t HW_TIMER_TEST_INTERVAL = 10;
inline constexpr uint16_t HW_TIMER_TEST_COUNT = 20;

inline constexpr uint8_t FRC_TICK_PER_US = (TIMER_BASE_CLK >> 4) / 1000000;

uint16_t hw_timer_test_idx_;
uint32_t hw_timer_start_[HW_TIMER_TEST_COUNT];
volatile uint32_t hw_timer_intr_[HW_TIMER_TEST_COUNT];

void _hw_timer_test_handler(void* param) {
  uint32_t ccount, ccompare;

  portENTER_CRITICAL();
  frc1.ctrl.en = 0;
  soc_clear_int_mask(1 << ETS_FRC_TIMER1_INUM);

  ccount = soc_get_ccount();
  ccompare = soc_get_ccompare();
  portEXIT_CRITICAL();

  hw_timer_intr_[hw_timer_test_idx_] =
      (int32_t)(ccount - (ccompare - _xt_tick_divisor)) / g_esp_ticks_per_us;
}

void _hw_timer_intr_test_setup() {
  portENTER_CRITICAL();
  // Disable and reconfigure FRC1
  frc1.ctrl.val = 0x00;
  frc1.ctrl.div = TIMER_CLKDIV_16;
  // Already set by val
  // frc1.ctrl.intr_type = TIMER_EDGE_INT;

  // Attach the FRC1 ISR
  _xt_isr_mask(1 << ETS_FRC_TIMER1_INUM);
  _xt_isr_attach(ETS_FRC_TIMER1_INUM, _hw_timer_test_handler, NULL);
  soc_clear_int_mask(1 << ETS_FRC_TIMER1_INUM);
  TM1_EDGE_INT_ENABLE();
  _xt_isr_unmask(1 << ETS_FRC_TIMER1_INUM);
  portEXIT_CRITICAL();
}

void _hw_timer_intr_test_perform() {
  uint32_t ccount, ccompare;

  for (int i = 0; i < HW_TIMER_TEST_COUNT; i++) {
    hw_timer_start_[i] = 0;
    hw_timer_intr_[i] = 0;
  }

  ESP_LOGI(TAG, "Testing HW timer interrupt accuracy...");
  for (hw_timer_test_idx_ = 0; hw_timer_test_idx_ < HW_TIMER_TEST_COUNT; hw_timer_test_idx_++) {
    // Record the start time of this timer
    portENTER_CRITICAL();
    ccount = soc_get_ccount();
    ccompare = soc_get_ccompare();
    portEXIT_CRITICAL();
    hw_timer_start_[hw_timer_test_idx_] =
        (int32_t)(ccount - (ccompare - _xt_tick_divisor)) / g_esp_ticks_per_us;

    // Set the end time, arm timer
    uint32_t load_val =
        FRC_TICK_PER_US * (HW_TIMER_TEST_MIN + hw_timer_test_idx_ * HW_TIMER_TEST_INTERVAL);
    portENTER_CRITICAL();
    frc1.load.data = load_val;
    frc1.ctrl.en = 0x01;
    portEXIT_CRITICAL();

    // Wait for completion
    while (hw_timer_intr_[hw_timer_test_idx_] == 0) {
      vTaskDelay(1);
    }
  }

  ESP_LOGI(TAG, "Testing Report:");
  for (int i = 0; i < HW_TIMER_TEST_COUNT; i++) {
    ESP_LOGI(TAG, "Deadline %dus, elapsed %dus", HW_TIMER_TEST_MIN + i * HW_TIMER_TEST_INTERVAL,
             hw_timer_intr_[i] - hw_timer_start_[i]);
  }
}

#endif  // HW_TIMER_TEST

}  // namespace
}  // namespace zw::esp8266::lightshow::testing

using namespace zw::esp8266::lightshow::testing;

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Framebuffer-less WS2812 driver demo");

#if HW_TIMER_TEST
  _hw_timer_intr_test_setup();
  _hw_timer_intr_test_perform();
#endif

#if RUN_AT_80MHZ
  vTaskDelay(1);
  esp_set_cpu_freq(ESP_CPU_FREQ_80M);
  vTaskDelay(1);
#endif

#if HW_TIMER_TEST
  _hw_timer_intr_test_perform();
#endif

  if (_lightshow() != ESP_OK) goto failed;
  return;

failed:
  ESP_LOGE(TAG, "Something failed!");
}