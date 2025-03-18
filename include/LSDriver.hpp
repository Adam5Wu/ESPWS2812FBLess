// Light-show driver
#ifndef ZWESP8266_LSDRIVER
#define ZWESP8266_LSDRIVER

#include <cstdint>
#include <memory>
#include <optional>

#include "esp_err.h"

#include "LSRenderer.hpp"

// Turn on extra debugging features.
#define ISR_DEVELOPMENT 0

namespace zw_esp8266::lightshow {

inline constexpr uint16_t kMinJitterBudget = 200;
inline constexpr uint16_t kMaxJitterBudget = 20000;

inline constexpr uint16_t kMinResetTime = 30;  // The interrupt resolution limit
inline constexpr uint16_t kMaxResetTime = 2000;

inline constexpr uint32_t kMinBaudRate = 9600;
inline constexpr uint32_t kMaxBaudRate = 4000000;

inline constexpr uint16_t kDefaultTaskStack = 1200;
inline constexpr UBaseType_t kDefaultTaskPriority = 10;

struct IOConfig {
  // Baud rate for serial TX line
  // Refer to the device datasheet to determine appropriate value.
  uint32_t baud_rate;

  // The standard reset time to signal a reset (i.e. starting a new frame).
  // Refer to the device datasheet for appropriate value.
  uint16_t std_reset_us;

  // The lower bound reset time of the strip.
  // Usually lower than the standard reset time, and may vary across
  // different device models or even manufacturing batches.
  // A proper value could significantly reduce visual defects (flickering)
  // in the event of a frame underflow.
  uint16_t min_reset_us;

  // The maximum length of scheduling jitter to accommodate.
  //
  // During a scheduling jitter, the code that renders new pixel data
  // is preempted, and we consume pre-computed data in the buffer.
  // If that buffer is depleted, we have a "frame underflow", and
  // must discard the rest of the frame's data because there is no way
  // to "resume". This may result in small visual defects, but probably
  // not very noticeable if the underflow doesn't happen frequently.
  //
  // The large the budget, the larger buffer will be allocated, and
  // the less probability a frame underflow will happen.
  uint16_t jitter_budget_us;
  
  // Pixel format
  RGB8BLayout pixel_format;

  // Invert serial line logic
  // Set according to how the device data line is driven
  // - True if driven directly by the TX pin;
  // - False if driven via an external N-channel MOS.
  bool invert_logic;

  // Map pixel data bits to serial bits.
  // If `invert_logic` is set, the data must be pre-inverted.
  // Note that UART sends less significant bit (LSB) first.
  uint8_t data_map[4];
};

inline constexpr IOConfig CONFIG_WS2812_TEMPLATE = {
    .baud_rate = 3200000,      // 1.25ns per WS2812 bit (4bits)
    .std_reset_us = 50,        // "Classic" WS2811 and WS2812b reset time
    .min_reset_us = 40,
    .jitter_budget_us = 1200,  // Absorbs ~1.2ms scheduling jitter
    .pixel_format = RGB8BDefaultLayout,
    .invert_logic = true,
    .data_map = {
        // UART sends less significant bit (LSB) first
        // Bits are inverted, and leading and trailing bits are excluded
        /* "00" = 1 000 100 0*/ 0b110111,
        /*          >>> >>>       <<<<<<*/
        /* "01" = 1 000 111 0*/ 0b000111,
        /* "10" = 1 110 100 0*/ 0b110100,
        /* "11" = 1 110 111 0*/ 0b000100,
    },
};

inline IOConfig CONFIG_WS2812_CLASSIC() { return CONFIG_WS2812_TEMPLATE; }

inline IOConfig CONFIG_WS2812_CUSTOM(uint16_t std_reset_us, uint16_t min_reset_us = 0,
                                     std::optional<uint16_t> jitter_budget_us = std::nullopt) {
  IOConfig config = CONFIG_WS2812_TEMPLATE;
  config.std_reset_us = std_reset_us;
  config.min_reset_us = min_reset_us;
  if (jitter_budget_us) config.jitter_budget_us = *jitter_budget_us;
  return config;
}
// The newer WS2812 revision has a >300us reset time
inline IOConfig CONFIG_WS2812_NEW() { return CONFIG_WS2812_CUSTOM(320, 50); }

struct IOStats {
  uint32_t frames_rendered;
  uint32_t underflow_actual;
  uint32_t underflow_near_miss;
#if ISR_DEVELOPMENT
  uint16_t isr_process_latency_high;
  uint16_t isr_process_latency_low;
#endif
};

#if ISR_DEVELOPMENT
struct __attribute__((packed)) UnderflowCounters {
  uint16_t actual;
  uint16_t near_miss;
};
#endif

// Perform the basic set up before starting the driver
// Pass `renderer=nullptr` to release resources of a previous set up.
esp_err_t DriverSetup(const IOConfig& config, Renderer* renderer);

// Starts the driver
// Two tasks are started:
// - An interrupt "task", driven by UART1 TX buffer depletion and HW timer.
//   * It pulls the ring buffer at (1 ~ 25)*reset_us interval, until a new
//     frame's data starts to appear;
//   * When rendering the frame, it will continuously fetch pixel data from
//     the ring buffer, convert it to level data and send into UART1 TX FIFO;
//   * At the end of every frame, a reset_us delay is enforced.
// - A "user-level" task, which keeps pulling new frames from the renderer
//   and pumping the pixel data of each frame to the ring buffer.
//   * When the ring buffer is full, it will wait for the interrupt to
//     consume the data and signal resuming data pumping;
//   * When there is no frame to deliver, e.g. extra time in between two
//     frames, the task will pull the next frame at 1 tick interval.
//   - Hence, the tick frequency of the RTOS is one of the determining factor
//     of the realizable FPS. (The other two factors are: strip size and
//     reset time, i.e. the time needed to transmit a whole frame).
//   - So if you have a small strip and wants to render transitions at higher
//     than 100fps, you will have to increase the RTOS tick frequency.
esp_err_t DriverStart(uint16_t task_stack = kDefaultTaskStack,
                      UBaseType_t task_priority = kDefaultTaskPriority
#if ISR_DEVELOPMENT
                      ,
                      UnderflowCounters* frame_underflow_debug = nullptr
#endif
);

// Stops the driver
// Does not release resources, so `DriverStart()` can be called again.
esp_err_t DriverStop();

// Fetch and clear the stats
IOStats DriverStats();

}  // namespace zw_esp8266::lightshow

#endif  // ZWESP8266_LSDRIVER