#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "FreeRTOS.h"
#include "freertos/event_groups.h"

#include "ESPIDF_shim.h"

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

namespace {

inline constexpr char TAG[] = "Main";
namespace LS = ::zw_esp8266::lightshow;

inline constexpr LS::StripSizeType STRIP_SIZE = 666;
inline constexpr uint8_t TARGET_FPS = 40;
// inline const LS::IOConfig CONFIG_WS2812 = LS::CONFIG_WS2812_CLASSIC();
inline const LS::IOConfig CONFIG_WS2812 = LS::CONFIG_WS2812_NEW();
// inline const LS::IOConfig CONFIG_WS2812 = LS::CONFIG_WS2812_CUSTOM(280, 50, 270);

inline constexpr uint16_t RENDER_TASK_STACK = 1200;
inline constexpr UBaseType_t RENDER_TASK_PRIORITY = 10;

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
  ESP_RETURN_ON_ERROR(LS::DriverSetup(CONFIG_WS2812, renderer.get()));
  ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
           uxTaskGetStackHighWaterMark(NULL));

  ESP_LOGI(TAG, "Starting LightShow driver...");
#if ISR_DEVELOPMENT
  // Preallocate frame underflow debug counters
  std::vector<LS::UnderflowCounters> underflow_debug_counters(STRIP_SIZE, LS::UnderflowCounters{});
#endif
  // This API will create a RTOS task that drives the frame rendering
  // Also starts UART1 TX & HW timer interrupt "task"
  ESP_RETURN_ON_ERROR(LS::DriverStart(RENDER_TASK_STACK, RENDER_TASK_PRIORITY
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
  ESP_LOGI(TAG, "Adding some more LightShow targets...");
  ESP_RETURN_ON_ERROR(renderer->Enqueue(LS::ColorDotTarget::Create(
      4000, {.color = {0x08, 0x16, 0x32}, .glow = 50, .pos_pgrs = LS::PGRS(0.3)},
      LS::DotState{.color = {0x00, 0x02, 0x01}, .glow = 50, .pos_pgrs = 0})));
  ESP_RETURN_ON_ERROR(renderer->Enqueue(LS::ColorDotTarget::Create(
      5000, {.color = {0x32, 0x16, 0x08}, .glow = 30, .pos_pgrs = LS::PGRS(1)})));
  ESP_RETURN_ON_ERROR(renderer->Enqueue(LS::ColorDotTarget::Create(
      3000, {.color = {0x16, 0x32, 0x08}, .glow = 80, .pos_pgrs = LS::PGRS(0.25)})));
  ESP_RETURN_ON_ERROR(renderer->Enqueue(LS::ColorDotTarget::Create(
      2000, {.color = {0x00, 0x24, 0x00}, .glow = 15, .pos_pgrs = LS::PGRS(0.1)})));
  ESP_RETURN_ON_ERROR(renderer->Enqueue(LS::ColorDotTarget::Create(
      2000, {.color = {0x00, 0x02, 0x01}, .glow = 15, .pos_pgrs = LS::PGRS(0)})));

  // Here is how to wait for the targets
  int target_idx = 0;
  while (true) {
    auto events =
        renderer->WaitFor(LS::RENDERER_START_TARGET | LS::RENDERER_IDLE_TARGET, portMAX_DELAY);
    if (events & LS::RENDERER_IDLE_TARGET) break;
    ESP_LOGI(TAG, "Target #%d was reached", ++target_idx);
  }

  LS::IOStats stats = LS::DriverStats();
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
  LS::DriverSetup(CONFIG_WS2812, nullptr);

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
