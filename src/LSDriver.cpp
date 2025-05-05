#include "LSDriver.hpp"

#include "esp8266/rom_functions.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/hw_timer.h"
#include "esp8266/uart_struct.h"
#include "esp8266/uart_register.h"
#include "esp8266/timer_struct.h"
#include "esp8266/timer_register.h"

#include "ZW_IDFLTH.h"

#include "LSPixel.hpp"
#include "LSFrame.hpp"
#include "LSRenderer.hpp"

// For quick debug / comparison
// #undef CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF

#ifndef CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF
#include "freertos/ringbuf.h"
#endif

namespace zw::esp8266::lightshow {
namespace {

inline constexpr char TAG[] = "LSDriver";

// Each pixel has (8 bit / color) * (3 colors) = 24 bits;
// And every 2 bits got coverted into one (6 bit) byte on UART.
// (Sent as 8 bits, including a lead bit and stop bit.)
using UARTPixel = uint8_t[8 * 3 / 2];

// Set 5 pixels (60 bytes) TX FIFO threshold
inline constexpr uint8_t UART_TX_FIFO_THRESHOLD =
  sizeof(UARTPixel) * CONFIG_ESP2812FBLESS_TX_BUFFER_THRESHOLD;
// The FRC1 counter can only hold a 23 bit value
inline constexpr uint32_t FRC1_COUNTER_LIMIT = (1 << 23) - 1;
// The FRC1 timer interrupt has a resolution limit of ~30us
inline constexpr uint8_t FRC1_RESOLUTION_LIMIT = 30;
// The overhead of the ISR in microseconds (at 80MHz)
// Assume ISR jitter of ~10us, plus an extra ~5us processing cost
inline constexpr uint8_t ISR_OVERHEAD_80MHZ = 15;

IOConfig config_ = {};
Renderer* renderer_ = nullptr;

// Lookup table for the time to deplete the TX buffer
uint16_t buffer_depletion_time_[UART_TX_FIFO_THRESHOLD + 1];

#ifdef CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF

// The number of bytes in FIFO that are "open" when TX interrupt is triggered
inline constexpr uint8_t FIFO_OPENSPACE = UART_FIFO_LEN - UART_TX_FIFO_THRESHOLD;
inline constexpr uint8_t PIXEL_BLOCK_SIZE = FIFO_OPENSPACE / sizeof(UARTPixel);

using PixelBlockType = RGB8BFlatType[PIXEL_BLOCK_SIZE];
using BlockIdxType = int16_t;

inline constexpr BlockIdxType MAX_PIXEL_BLOCK_COUNT = 200;

#define PIXEL_BLOCK_READY_FOR_WRITE BIT(0)

struct {
  PixelBlockType* data = nullptr;
  EventGroupHandle_t events = NULL;
  BlockIdxType block_count = 0;
  volatile BlockIdxType read_block = -1;  // -1 means not ready for read
  volatile BlockIdxType write_block = 0;  // -1 means not ready for write
} pixel_buffer_;

PixelBlockType* IRAM_ATTR _recv_pixel_block_from_isr() {
  BlockIdxType read_block = pixel_buffer_.read_block;
  return read_block >= 0 ? &pixel_buffer_.data[read_block] : nullptr;
}

void IRAM_ATTR _return_pixel_block_from_isr() {
  BlockIdxType return_block = pixel_buffer_.read_block;
  BlockIdxType write_block = pixel_buffer_.write_block;

  if (write_block < 0) {
    pixel_buffer_.write_block = write_block = return_block;
    BaseType_t context_switch = pdFALSE;
    xEventGroupSetBitsFromISR(pixel_buffer_.events, PIXEL_BLOCK_READY_FOR_WRITE, &context_switch);
    if (context_switch) portYIELD_FROM_ISR();
  }
  BlockIdxType next_block = (return_block + 1) % pixel_buffer_.block_count;
  pixel_buffer_.read_block = (next_block == write_block) ? -1 : next_block;
}

#else

// Ring buffer for absorbing scheduling jitters
RingbufHandle_t ringbuf_ = NULL;
StripSizeType ringbuf_size = 0;

#endif  // CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF

struct {
  // How many bytes per frame
  // NOTE: the data is in "compact" color component representation
  StripSizeType frame_size;
  // Which byte are we currently receiving
  StripSizeType frame_pos;
  // How much time (us) to leave as a buffer when HW timer wakeup with frame
  // underflow sentinel set, before we consider the underflow happened.
  uint8_t isr_overhead;

  // Extra bits for future use
  uint8_t unused : 2;
  // Tracking the underflow state of the frame
  // 0 = normal
  // 1 = underflow sentinel
  // 2 = underflow (discarding frame)
  uint8_t underflow_state : 2;
  // Tracking the idle state of the frame
  // 0 = normal (receiving data)
  // 1~15 = Idling
  uint8_t idle_state : 4;
#if ISR_DEVELOPMENT
  // If not nullptr, points to a counter array that tracks where in the
  // frame does each underflow and near-miss happen.
  UnderflowCounters* underflow_counters;
#endif
} state_;

IOStats stats_;

// This driver uses TIMER_CLKDIV_16.
inline constexpr uint8_t FRC_TICK_PER_US = (TIMER_BASE_CLK >> 4) / 1000000;

void _hw_timer_rearm(uint32_t usec) {
  // First filter out excessive values (to avoid overflowing next step)
  assert(usec <= FRC1_COUNTER_LIMIT);
  // We don't check the lower bound because the interrupt will trigger
  // regardless of how low the value, just that the timing will not be
  // accurate; the caller can consult FRC1_RESOLUTION_LIMIT themselves.

  // The number of timer ticks to count down
  uint32_t load_val = FRC_TICK_PER_US * usec;
  // Then check against the actual hardware constraint.
  assert(load_val <= FRC1_COUNTER_LIMIT);

  portENTER_CRITICAL();
  frc1.load.data = load_val;
  frc1.ctrl.en = 0x01;
  portEXIT_CRITICAL();
}

void IRAM_ATTR _uart_send_color_component(uint8_t data) {
  uart1.fifo.rw_byte = config_.data_map[(data >> 6) & 0b11];
  uart1.fifo.rw_byte = config_.data_map[(data >> 4) & 0b11];
  uart1.fifo.rw_byte = config_.data_map[(data >> 2) & 0b11];
  uart1.fifo.rw_byte = config_.data_map[(data >> 0) & 0b11];
}

enum class ISRCaller : uint8_t {
  HW_TIMER = 0,
  UART = 1,
};

// Augment some *static* context to the interrupt handler.
union InterruptContext {
  void* raw;
  struct {
    ISRCaller caller;
  };
};
static_assert(sizeof(InterruptContext) == sizeof(void*));

void IRAM_ATTR _unified_intr_handler(void* param) {
#if ISR_DEVELOPMENT
  uint32_t isr_start_ = soc_get_ccount();
#endif

  InterruptContext context{.raw = param};

  // Take care of the logistics first
  portENTER_CRITICAL();
  // This is a one-shot handler logic
  // Re-triggering will be done via either explicit re-arming
  // of UART FIFO empty, or a one-shot hw_timer.
  switch (context.caller) {
    case ISRCaller::HW_TIMER: {
      frc1.ctrl.en = 0;
      // Extra safety that seems to be not needed
      // frc1.load.data = 0x100;  // Ensure interrupt not firing
      // soc_clear_int_mask(1 << ETS_FRC_TIMER1_INUM);
    } break;

    case ISRCaller::UART: {
      uart1.int_ena.val &= ~UART_TXFIFO_EMPTY_INT_ENA_M;
      uart1.int_clr.val |= UART_TXFIFO_EMPTY_INT_CLR_M;
      // Extra safety that seems to be not needed
      // soc_clear_int_mask(1 << ETS_UART_INUM);  // Clear pending interrupt
    } break;
  }
  portEXIT_CRITICAL();

  switch (state_.idle_state) {
    case 0:  // We are receiving data
      goto receive_data;

    default: {   // 1+ We are idling
    check_idle:  // Express entry point from frame underflow discard completion
#ifdef CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF
      BlockIdxType blocks_available =
          (pixel_buffer_.read_block < 0)
              ? 0
              : ((pixel_buffer_.write_block > pixel_buffer_.read_block)
                     ? (pixel_buffer_.write_block - pixel_buffer_.read_block)
                 : (pixel_buffer_.write_block < 0)
                     ? pixel_buffer_.block_count
                     : (pixel_buffer_.block_count - pixel_buffer_.read_block +
                        pixel_buffer_.write_block));
      bool can_start =
          (blocks_available >= pixel_buffer_.block_count) ||
          (blocks_available * PIXEL_BLOCK_SIZE * sizeof(RGB8BFlatType) > (state_.frame_size / 2));
#else
      UBaseType_t items_waiting;
      vRingbufferGetInfo(ringbuf_, NULL, NULL, NULL, &items_waiting);
      bool can_start = (items_waiting >= ringbuf_size) || (items_waiting > state_.frame_size / 2);
#endif  // CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF
      if (can_start) {
        // We are on, switch to receiving mode.
        // ets_printf("* Frame Start!\n");
        ++stats_.frames_rendered;
        state_.idle_state = 0;
        goto receive_data;
      }
      // Wake up after some idle time to check readiness again.
      // The wait linearly increases from 1x to 25x reset time.
      // The max wait time ranges from 750us to 30ms.
      uint16_t idle_wait = config_.std_reset_us * state_.idle_state;
      _hw_timer_rearm(idle_wait);
      stats_.idle_wait += idle_wait;
      if (state_.idle_state < 15) ++state_.idle_state;
    }
  }
  // Nothing else to do, next event scheduled.
  return;

receive_data:
  // How many bytes remains in the buffer
  // NOTE: the data is in "verbose" level representation
  uint8_t tx_fifo_rem = uart1.status.txfifo_cnt;

#if ISR_DEVELOPMENT
  if ((state_.frame_pos > 0) && (tx_fifo_rem == 0)) ++stats_.isr_late_wakeup;
#endif

  // How many bytes are we still expecting for this frame
  // NOTE: the data is in "compact" color component representation
  StripSizeType frame_rem = state_.frame_size - state_.frame_pos;

  if (frame_rem == 0) {
    // We have reached the end of the frame.
    state_.frame_pos = 0;  // Reset position for the next frame
    ++state_.idle_state;   // Signal frame idle
    // How long until the remaining data finishes sending?
    uint16_t send_completion =
        buffer_depletion_time_[std::min(tx_fifo_rem, UART_TX_FIFO_THRESHOLD)];
    // Wait for the send completion plus reset time.
    // Will wake up in "frame idling" state.
    _hw_timer_rearm(send_completion + config_.std_reset_us);
    // Nothing else to do, next event scheduled.
    return;
  }

  // Find how many bytes can we enqueue
  // NOTE: the data is in "verbose" level representation
  uint8_t tx_fifo_free = UART_FIFO_LEN - tx_fifo_rem;
  // This is the amount of data we can handle now
  // NOTE: the data is in "compact" color component representation
  size_t data_to_receive = std::min(frame_rem, (StripSizeType)(tx_fifo_free >> 2));

#ifdef CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF
  // We want to receive whole pixels to avoid displaying bad color in case of underflow
  data_to_receive -= data_to_receive % sizeof(RGB8BFlatType);

  // Process one pixel block at a time
  bool ready_to_receive = (data_to_receive >= (PIXEL_BLOCK_SIZE * sizeof(RGB8BFlatType))) ||
                          (data_to_receive == frame_rem);
#else
  bool ready_to_receive = data_to_receive > 0;
#endif

  if (ready_to_receive) {
    // Try to receive some data
#ifdef CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF
    auto pixel_data = (uint8_t*)_recv_pixel_block_from_isr();
    size_t data_len = std::min((size_t)(PIXEL_BLOCK_SIZE * sizeof(RGB8BFlatType)), data_to_receive);
#else
    size_t data_len = 0;
    auto pixel_data = (uint8_t*)xRingbufferReceiveUpToFromISR(ringbuf_, &data_len, data_to_receive);
#endif

    if (pixel_data == nullptr) {
      // There is no data, check the frame underflow
      switch (state_.underflow_state) {
        case 0: {  // This is the first out-of-data occurrence
          // The time needed to send remaining data + reset time - isr_overhead
          uint16_t underflow_threshold =
              buffer_depletion_time_[std::min(tx_fifo_rem, UART_TX_FIFO_THRESHOLD)] +
              config_.min_reset_us - state_.isr_overhead;
          ++state_.underflow_state;
          // Wake up when the threshold is passed, hopefully some data will arrive.
          _hw_timer_rearm(underflow_threshold);
          stats_.busy_wait += underflow_threshold;
        } break;

        case 1:  // The sentinel is up, and unfortunately we have a real underflow
          // ets_printf("* Frame Underflow! %d/%d\n", state_.frame_pos, state_.frame_size);
          // Entering discard mode
          ++state_.underflow_state;
          ++stats_.underflow_actual;
#if ISR_DEVELOPMENT
          if (state_.underflow_counters != nullptr)
            ++state_.underflow_counters[state_.frame_pos / sizeof(RGB8BFlatType)].actual;
#endif
          [[fallthrough]];

        default:  // We are in discard mode
          // Wake up after reset time and try to discard the reset of the frame.
          _hw_timer_rearm(config_.std_reset_us);
          stats_.busy_wait += config_.std_reset_us;
      }
      // Nothing else to do, next event scheduled.
      return;
    }

    // We got data!
    state_.frame_pos += data_len;

    // Check the frame underflow
    switch (state_.underflow_state) {
      case 1:  // Successfully recovered before underflow!
        ++stats_.underflow_near_miss;
#if ISR_DEVELOPMENT
        if (state_.underflow_counters != nullptr)
          ++state_.underflow_counters[(state_.frame_pos - data_len) / sizeof(RGB8BFlatType)]
                .near_miss;
#endif

        [[fallthrough]];

      case 0:  // We are in normal receive mode
        state_.underflow_state = 0;
        // We can proceed with sending frame data
        break;

      default: {  // We are in discard mode
#ifdef CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF
#if PIXEL_BLOCK_SEGMENTS > 1
      discard_data:
#endif
        _return_pixel_block_from_isr();
#if PIXEL_BLOCK_SEGMENTS > 1
        // Continue to receive and discard until target hit, or ring buffer is empty.
        if ((data_to_receive -= data_len) > 0) {
          frame_rem -= data_len;
          if (data_to_receive >= PIXEL_BLOCK_SIZE * sizeof(RGB8BFlatType) ||
              data_to_receive == frame_rem) {
            pixel_data = (uint8_t*)_recv_pixel_block_from_isr();
            if (pixel_data != nullptr) {
              data_len = std::min(data_len, data_to_receive);
              state_.frame_pos += data_len;
              goto discard_data;
            }
          }
        }
#endif  // PIXEL_BLOCK_SEGMENTS

#else
        BaseType_t context_switch = pdFALSE;
        vRingbufferReturnItemFromISR(ringbuf_, pixel_data, &context_switch);
        if (context_switch) portYIELD_FROM_ISR();
#endif  // CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF

        // If we have completely discarded the frame...
        if (state_.frame_pos == state_.frame_size) {
          state_.frame_pos = 0;        // Reset position for the next frame
          state_.underflow_state = 0;  // Reset underflow state
          // Since we have already exceeded the reset time we can start
          // working on the next frame (if available) immediately!
          state_.idle_state = 1;
          goto check_idle;
        }
        // Otherwise, continue to discard the rest of the frame.
        // (Or, if there is no data to read, wait for more data.)
        goto receive_data;
      }
    }

#if ISR_DEVELOPMENT
    uint32_t isr_serve_data_ = soc_get_ccount();
    stats_.isr_process_latency_high =
        std::max(stats_.isr_process_latency_high, (uint16_t)(isr_serve_data_ - isr_start_));
    stats_.isr_process_latency_low = std::min(
        stats_.isr_process_latency_low ? stats_.isr_process_latency_low : (uint16_t)UINT16_MAX,
        (uint16_t)(isr_serve_data_ - isr_start_));
#endif

#if defined(CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF) && PIXEL_BLOCK_SEGMENTS > 1
  send_data:
#endif
    // Produce UART data and send them out
    for (size_t i = 0; i < data_len; ++i) {
      _uart_send_color_component(pixel_data[i]);
    }

    // Done sending, return the buffer
#ifdef CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF

    _return_pixel_block_from_isr();

#if PIXEL_BLOCK_SEGMENTS > 1
    // Continue to pipe data through until target hit, or ring buffer is empty.
    if ((data_to_receive -= data_len) > 0) {
      frame_rem -= data_len;
      if (data_to_receive >= PIXEL_BLOCK_SIZE * sizeof(RGB8BFlatType) ||
          data_to_receive == frame_rem) {
        pixel_data = (uint8_t*)_recv_pixel_block_from_isr();
        if (pixel_data != nullptr) {
          data_len = std::min(data_len, data_to_receive);
          state_.frame_pos += data_len;
          goto send_data;
        }
      }
    }
#endif  // PIXEL_BLOCK_SEGMENTS

#else

    BaseType_t context_switch = pdFALSE;
    vRingbufferReturnItemFromISR(ringbuf_, pixel_data, &context_switch);
    if (context_switch) portYIELD_FROM_ISR();

#endif  // CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF
  }

  // Call us again when the current batch is mostly done.
  portENTER_CRITICAL();
  // Extra safety that seems to be not needed
  // uart1.int_clr.val |= UART_TXFIFO_EMPTY_INT_CLR_M;
  uart1.int_ena.val |= UART_TXFIFO_EMPTY_INT_ENA_M;
  portEXIT_CRITICAL();
}

void _uart_intr_init() {
  portENTER_CRITICAL();
  // Disable and clear all UART1 interrupts
  uart1.int_ena.val &= ~UART_INTR_MASK;
  uart1.int_clr.val |= UART_INTR_MASK;

  // Disable and clear UART0 interrupts as well
  // Both share the same ISR and we don't want interferences
  uart0.int_ena.val &= ~UART_INTR_MASK;
  uart0.int_clr.val |= UART_INTR_MASK;

  // Configure and enable TX FIFO empty interrupt
  uart1.conf1.txfifo_empty_thrhd = UART_TX_FIFO_THRESHOLD;
  uart1.int_ena.val = UART_TXFIFO_EMPTY_INT_ENA_M;

  // Attach UART ISR
  _xt_isr_mask(1 << ETS_UART_INUM);
  InterruptContext context{.caller = ISRCaller::UART};
  _xt_isr_attach(ETS_UART_INUM, _unified_intr_handler, context.raw);
  _xt_isr_unmask(1 << ETS_UART_INUM);
  portEXIT_CRITICAL();
}

void _uart_intr_finit() {
  portENTER_CRITICAL();
  // Disable and clear all UART1 interrupts
  uart1.int_ena.val &= ~UART_INTR_MASK;
  uart1.int_clr.val |= UART_INTR_MASK;

  // Detach the UART ISR
  _xt_isr_mask(1 << ETS_UART_INUM);
  _xt_isr_attach(ETS_UART_INUM, NULL, NULL);
  portEXIT_CRITICAL();
}

void _hw_timer_intr_init() {
  portENTER_CRITICAL();
  // Disable and reconfigure FRC1
  frc1.ctrl.val = 0x00;
  frc1.ctrl.div = TIMER_CLKDIV_16;
  // Already set by val
  // frc1.ctrl.intr_type = TIMER_EDGE_INT;

  // Attach the FRC1 ISR
  _xt_isr_mask(1 << ETS_FRC_TIMER1_INUM);
  InterruptContext context{.caller = ISRCaller::HW_TIMER};
  _xt_isr_attach(ETS_FRC_TIMER1_INUM, _unified_intr_handler, context.raw);
  soc_clear_int_mask(1 << ETS_FRC_TIMER1_INUM);
  TM1_EDGE_INT_ENABLE();
  _xt_isr_unmask(1 << ETS_FRC_TIMER1_INUM);
  portEXIT_CRITICAL();
}

void _hw_timer_finit() {
  portENTER_CRITICAL();
  // Disable and clear all FRC1 interrupts
  frc1.ctrl.val = 0;
  soc_clear_int_mask(1 << ETS_FRC_TIMER1_INUM);

  // Detach the FRC1 ISR
  _xt_isr_mask(1 << ETS_FRC_TIMER1_INUM);
  TM1_EDGE_INT_DISABLE();
  _xt_isr_attach(ETS_FRC_TIMER1_INUM, NULL, NULL);
  portEXIT_CRITICAL();
}

xTaskHandle render_engine_task_ = NULL;

void _render_engine(void* param) {
  while (true) {
    Frame* frame = renderer_->RenderFrame();
    if (frame != nullptr) {
      // ESP_LOGI(TAG, "%s", frame->DebugString().c_str());
      PixelWithStatus data;
#ifdef CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF

      PixelBlockType* cur_block = nullptr;
      uint8_t block_pos = 0;
      // uint16_t frame_pos = 0;
      while (true) {
        data = frame->GetPixelData();
        // ++frame_pos;
        if (data.end_of_frame || block_pos >= PIXEL_BLOCK_SIZE) {
          if (cur_block != nullptr) {
            BlockIdxType ready_block = pixel_buffer_.write_block;
            BlockIdxType next_block = (ready_block + 1) % pixel_buffer_.block_count;
            portENTER_CRITICAL();
            BlockIdxType read_block = pixel_buffer_.read_block;
            if (read_block < 0) pixel_buffer_.read_block = read_block = ready_block;
            pixel_buffer_.write_block = (next_block == read_block) ? -1 : next_block;
            portEXIT_CRITICAL();
            cur_block = nullptr;
          }
          if (data.end_of_frame) break;
        }

        if (cur_block == nullptr) {
          BlockIdxType write_block = pixel_buffer_.write_block;
          // The "ready" bit may be left set from a previous ISR block return when the write
          // block was depleted, but hasn't being waited for. So there maybe an extra spin,
          // which seems to be the most cost-effective fix... :P
          while (write_block < 0) {
            xEventGroupWaitBits(pixel_buffer_.events, PIXEL_BLOCK_READY_FOR_WRITE, true, true,
                                portMAX_DELAY);
            write_block = pixel_buffer_.write_block;
          }
          cur_block = &pixel_buffer_.data[write_block];
          block_pos = 0;
        }
        data.pixel.Transcribe(config_.pixel_format, (*cur_block)[block_pos++]);
      }

#else

      RGB8BFlatType hw_pixel;
      while (!(data = frame->GetPixelData()).end_of_frame) {
        data.pixel.Transcribe(config_.pixel_format, hw_pixel);
        // Will wait forever, don't expect failure
        xRingbufferSend(ringbuf_, &hw_pixel, sizeof(RGB8BFlatType), portMAX_DELAY);
      }

#endif  // CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF
    }
    // ESP_LOGI(TAG, "=> Heap: %d; Stack: %d", esp_get_free_heap_size(),
    //          uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(frame ? 0 : 1);
  }
}

}  // namespace

esp_err_t DriverSetup(const IOConfig& config, Renderer* renderer) {
  // Release facilities from previous set up
  renderer_ = nullptr;

#ifdef CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF

  if (pixel_buffer_.data != nullptr) {
    free(pixel_buffer_.data);
    pixel_buffer_.data = nullptr;
  }
  if (pixel_buffer_.events != NULL) {
    vEventGroupDelete(pixel_buffer_.events);
    pixel_buffer_.events = NULL;
  }

#else

  if (ringbuf_ != NULL) {
    vRingbufferDelete(ringbuf_);
    ringbuf_ = NULL;
  }

#endif  // CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF

  if (renderer == nullptr) {
    ESP_LOGE(TAG, "No LightShow renderer");
    return ESP_ERR_INVALID_ARG;
  }

  // Validate timing configs
  if (config.jitter_budget_us < kMinJitterBudget || config.jitter_budget_us > kMaxJitterBudget) {
    ESP_LOGE(TAG, "Invalid jitter budget of %d us", config.jitter_budget_us);
    return ESP_ERR_INVALID_ARG;
  }
  if (config.std_reset_us < kMinResetTime || config.std_reset_us > kMaxResetTime) {
    ESP_LOGE(TAG, "Invalid standard reset time of %d us", config.std_reset_us);
    return ESP_ERR_INVALID_ARG;
  }
  if (config.min_reset_us > config.std_reset_us) {
    ESP_LOGE(TAG, "Invalid minimum reset time of %d us", config.min_reset_us);
    return ESP_ERR_INVALID_ARG;
  }
  if (config.min_reset_us > config.std_reset_us) {
    ESP_LOGE(TAG, "Minimum reset time (%d us) cannot exceed standard reset time (%d us)",
             config.min_reset_us, config.std_reset_us);
    return ESP_ERR_INVALID_ARG;
  }

  // Check if the renderer setup is realistic.
  StripSizeType pixels = renderer->StripSize();
  if (pixels == 0) {
    ESP_LOGE(TAG, "No pixel to render (strip size = 0)");
    return ESP_ERR_INVALID_STATE;
  }
  uint32_t target_frame_us = renderer->FrameInterval();

  uint32_t frame_uart_bits = sizeof(UARTPixel) * 8 * pixels;
  if (config.baud_rate < kMinBaudRate || config.baud_rate > kMaxBaudRate) {
    ESP_LOGE(TAG, "Invalid baud rate of %d", config.baud_rate);
    return ESP_ERR_INVALID_ARG;
  }
  uint32_t frame_data_time_us = (uint64_t)frame_uart_bits * 10000 / (config.baud_rate / 100);
  uint32_t frame_time_us = frame_data_time_us + config.std_reset_us;

  // Rough estimation of FPS.
  uint32_t est_fps = 1000000 / frame_time_us;
  ESP_LOGI(TAG, "LightShow frame %d pixels, max FPS ~= %d", pixels, est_fps);
  if (target_frame_us < frame_time_us) {
    ESP_LOGW(TAG, "Renderer has an infeasible target FPS");
  }

  uart_config_t uart_config = {
      .baud_rate = (int)config.baud_rate,
      .data_bits = UART_DATA_6_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
  };

  ESP_RETURN_ON_ERROR(uart_param_config(UART_NUM_1, &uart_config));
  ESP_RETURN_ON_ERROR(
      uart_set_line_inverse(UART_NUM_1, config.invert_logic ? UART_INVERSE_TXD : 0));

  // Create ring buffer
  uint16_t pixel_time_us = (sizeof(UARTPixel) * 8000000U) / config.baud_rate;
  StripSizeType dejitter_pixels = (config.jitter_budget_us + pixel_time_us - 1) / pixel_time_us;

#ifdef CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF

  pixel_buffer_.block_count = (dejitter_pixels + PIXEL_BLOCK_SIZE - 1) / PIXEL_BLOCK_SIZE;
  if (pixel_buffer_.block_count > MAX_PIXEL_BLOCK_COUNT) {
    ESP_LOGE(TAG, "Too many pixel buffer blocks (%d)", pixel_buffer_.block_count);
    return ESP_ERR_INVALID_STATE;
  }
  ESP_LOGI(TAG, "LightShow jitter budget %d us (%d*%d pixels, %d bytes)", config.jitter_budget_us,
           pixel_buffer_.block_count, PIXEL_BLOCK_SIZE,
           pixel_buffer_.block_count * sizeof(PixelBlockType));
  pixel_buffer_.data = (PixelBlockType*)malloc(pixel_buffer_.block_count * sizeof(PixelBlockType));
  if (pixel_buffer_.data == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate LightShow pixel buffer");
    return ESP_ERR_NO_MEM;
  }
  pixel_buffer_.events = xEventGroupCreate();
  if (pixel_buffer_.events == NULL) {
    ESP_LOGE(TAG, "Failed to allocate LightShow pixel buffer events");
    return ESP_ERR_NO_MEM;
  }

#else

  ringbuf_size = dejitter_pixels * sizeof(RGB8BFlatType);
  ESP_LOGI(TAG, "LightShow jitter budget %d us (%d pixels, %d bytes)", config.jitter_budget_us,
           dejitter_pixels, ringbuf_size);
  ringbuf_ = xRingbufferCreate(ringbuf_size, RINGBUF_TYPE_BYTEBUF);
  if (ringbuf_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate LightShow ring buffer for %d pixels (%d bytes)",
             dejitter_pixels, ringbuf_size);
    return ESP_ERR_NO_MEM;
  }

#endif  // CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF

  // Precompute the amount of time to send out N bytes
  for (uint8_t i = 0; i <= UART_TX_FIFO_THRESHOLD; i++) {
    // Round up to the next us to ensure the send actually completes.
    buffer_depletion_time_[i] = (i * 8000000U + config.baud_rate - 1) / config.baud_rate;
  }

  // Save the setup for internal use.
  config_ = config;
  renderer_ = renderer;
  return ESP_OK;
}

esp_err_t DriverStart(uint16_t task_stack, UBaseType_t task_priority
#if ISR_DEVELOPMENT
                      ,
                      UnderflowCounters* frame_underflow_debug
#endif
) {
  if (renderer_ == nullptr) {
    ESP_LOGE(TAG, "Need to set up LightShow driver first");
    return ESP_ERR_INVALID_STATE;
  }
  if (render_engine_task_ != NULL) {
    ESP_LOGE(TAG, "LightShow driver already started");
    return ESP_ERR_INVALID_STATE;
  }

  // Reset jitter pixel buffer
#ifdef CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF

  pixel_buffer_.read_block = -1;
  pixel_buffer_.write_block = 0;

#else

  while (true) {
    UBaseType_t items_waiting;
    vRingbufferGetInfo(ringbuf_, NULL, NULL, NULL, &items_waiting);
    if (items_waiting == 0) break;

    size_t discard_size;
    void* discard_data = xRingbufferReceive(ringbuf_, &discard_size, 0);
    ESP_LOGD(TAG, "Discarding %d bytes from ring buffer", discard_size);
    vRingbufferReturnItem(ringbuf_, discard_data);
  }

#endif  // CONFIG_ESP2812FBLESS_CUSTOM_RINGBUF

  // Initialize internal states.
  state_ = {};
  state_.frame_size = renderer_->StripSize() * sizeof(RGB8BFlatType);
  state_.isr_overhead = ISR_OVERHEAD_80MHZ;
  if (g_esp_ticks_per_us > 80) state_.isr_overhead /= 2;
  state_.idle_state = 1;  // Start in end of frame state
#if ISR_DEVELOPMENT
  state_.underflow_counters = frame_underflow_debug;
#endif
  // Refresh the stats counters.
  DriverStats();

  // Initialize hardware facilities
  _hw_timer_intr_init();
  _uart_intr_init();

  // Create the render engine task
  if (xTaskCreate(_render_engine, "ls-engine", task_stack, NULL, task_priority,
                  &render_engine_task_) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to create LightShow render engine task");
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

esp_err_t DriverStop() {
  if (render_engine_task_ == NULL) {
    ESP_LOGE(TAG, "LightShow driver not yet started");
    return ESP_ERR_INVALID_STATE;
  }

  // Shutdown hardware facilities
  _uart_intr_finit();
  _hw_timer_finit();

  vTaskDelete(render_engine_task_);
  render_engine_task_ = NULL;
  return ESP_OK;
}

IOStats DriverStats() {
  uint64_t cur_time = esp_timer_get_time();

  portENTER_CRITICAL();
  IOStats result = stats_;
  stats_ = {};
  stats_.start_time = cur_time;
  portEXIT_CRITICAL();
  return result;
}

}  // namespace zw::esp8266::lightshow
