#include "LSDriver.hpp"

#include "esp8266/rom_functions.h"
#include "esp_log.h"
#include "esp_system.h"

#include "FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/hw_timer.h"
#include "esp8266/uart_struct.h"
#include "esp8266/uart_register.h"
#include "esp8266/timer_struct.h"
#include "esp8266/timer_register.h"

#include "ESPIDF_shim.h"

// The IDF ring buffer is generic, but not very efficient.
// By default use a more efficient custom ring buffer.
#define USE_IDF_RINGBUF 0

#if USE_IDF_RINGBUF
#include "freertos/ringbuf.h"
#endif

namespace zw_esp8266::lightshow {
namespace {

inline constexpr char TAG[] = "LSDriver";

// Each pixel has (8 bit / color) * (3 colors) = 24 bits;
// And every 2 bits got coverted into one (6 bit) byte on UART.
// (Sent as 8 bits, including a lead bit and stop bit.)
using UARTPixel = uint8_t[8 * 3 / 2];

// Use a 12 bytes threshold (the SDK default is 10)
inline constexpr uint8_t UART_TX_FIFO_THRESHOLD = sizeof(UARTPixel);
// The FRC1 counter can only hold a 23 bit value
inline constexpr uint32_t FRC1_COUNTER_LIMIT = (1 << 23) - 1;

inline constexpr uint16_t FRAME_UNDERFLOW_MARGIN = 10;
inline constexpr uint16_t HW_TIMER_COUNTER_MIN = 10;
inline constexpr uint16_t RENDER_ENGINE_PRIORITY = 6;

IOConfig config_ = {};
Renderer* renderer_ = nullptr;

// Lookup table for the time to deplete the TX buffer
uint16_t buffer_depletion_time_[UART_TX_FIFO_THRESHOLD + 1];

#if USE_IDF_RINGBUF

// Ring buffer for absorbing scheduling jitters
RingbufHandle_t ringbuf_ = NULL;

#else

inline constexpr uint8_t PIXEL_BLOCK_SIZE =
    (UART_FIFO_LEN - UART_TX_FIFO_THRESHOLD) / sizeof(UARTPixel);
static_assert(PIXEL_BLOCK_SIZE <= 20);

using PixelBlock = RGB8BPixel[PIXEL_BLOCK_SIZE];
using BlockIdxType = int16_t;

inline constexpr BlockIdxType MAX_PIXEL_BLOCK_COUNT = 100;

#define PIXEL_BLOCK_READY_FOR_WRITE BIT(0)

struct {
  PixelBlock* data = nullptr;
  EventGroupHandle_t events = NULL;
  BlockIdxType block_count = 0;
  volatile BlockIdxType read_block = -1;  // -1 means not ready for read
  volatile BlockIdxType write_block = 0;  // -1 means not ready for write
} pixel_buffer_;

PixelBlock* IRAM_ATTR _recv_pixel_block_from_isr() {
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

#endif  // USE_IDF_RINGBUF

struct {
  // How many bytes per frame
  // NOTE: the data is in "compact" color component representation
  StripSizeType frame_size;
  // Which byte are we currently receiving
  StripSizeType frame_pos;
  // Tracking the underflow state of the frame
  // 0 = normal
  // 1 = underflow sentinel
  // 2 = underflow (discarding frame)
  uint8_t frame_underflow;
  // Tracking the idle state of the frame
  // 0 = normal (receiving data)
  // 1 = frame ended (mandatory reset)
  // 2+ = Idling
  uint8_t frame_idle;
} state_ = {};

IOStats stats_;

void _hw_timer_rearm(uint32_t usec) {
  // First filter out excessive values (to avoid overflowing next step)
  assert(usec < FRC1_COUNTER_LIMIT);
  // The number of timer ticks = (80MHz / div) * t
  // This driver uses TIMER_CLKDIV_16.
  uint32_t load_val = ((TIMER_BASE_CLK >> 4) / 1000000) * usec;
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

void IRAM_ATTR _uart_intr_handler(void* param) {
  InterruptContext context{.raw = param};

  // Take care of the logistics first
  portENTER_CRITICAL();
  // This is a one-shot handler logic
  // Re-triggering will be done via either explicit re-arming
  // of UART FIFO empty, or a one-shot hw_timer.
  switch (context.caller) {
    case ISRCaller::HW_TIMER: {
      frc1.ctrl.en = 0;
      soc_clear_int_mask(1 << ETS_FRC_TIMER1_INUM);
    } break;

    case ISRCaller::UART: {
      uart1.int_ena.val &= ~UART_TXFIFO_EMPTY_INT_ENA_M;
      uart1.int_clr.val |= UART_TXFIFO_EMPTY_INT_CLR_M;
    } break;
  }
  portEXIT_CRITICAL();

check_idle:
  switch (state_.frame_idle) {
    case 0:  // We are receiving data
      goto receive_data;

    case 1:  // We are at an end of frame
      // ets_printf("* Frame End!\n");
      // Schedule a one-shot hw_timer event to wait for frame reset.
      _hw_timer_rearm(config_.reset_us);
      ++state_.frame_idle;  // We may enter idling after the wait
      break;

    default: {  // 2+ We are idling
#if USE_IDF_RINGBUF
      // Check if we have data in ring buffer
      UBaseType_t items_waiting;
      vRingbufferGetInfo(ringbuf_, NULL, NULL, NULL, &items_waiting);
      bool can_start = items_waiting >= std::min(UART_FIFO_LEN >> 3, (int)state_.frame_size);
#else
      bool can_start = pixel_buffer_.read_block >= 0;
#endif  // USE_IDF_RINGBUF
      if (can_start) {
        // We are on, switch to receiving mode.
        // ets_printf("* Frame Start!\n");
        ++stats_.frames_rendered;
        state_.frame_idle = 0;
        goto receive_data;
      }
      // Schedule a one-shot hw_timer event to wait for data
      // We keep increasing the wait time until 50x reset time
      // ... which ranges from 0.5ms to 50ms.
      _hw_timer_rearm(state_.frame_idle * config_.reset_us);
      if (state_.frame_idle < 25) ++state_.frame_idle;
    }
  }
  // Nothing else to do, next event scheduled.
  return;

receive_data:
  // How many bytes remains in the buffer
  // NOTE: the data is in "verbose" level representation
  uint8_t tx_fifo_rem = uart1.status.txfifo_cnt;
  // How many bytes are we still expecting for this frame
  // NOTE: the data is in "compact" color component representation
  StripSizeType frame_rem = state_.frame_size - state_.frame_pos;

  if (frame_rem == 0) {
    //  We have reached the end of the frame.
    uint16_t send_completion =
        buffer_depletion_time_[std::min(tx_fifo_rem, UART_TX_FIFO_THRESHOLD)];
    // Schedule a one-shot hw_timer event to wait for data sending to complete
    _hw_timer_rearm(send_completion);
    state_.frame_pos = 0;  // Reset position for the next frame
    ++state_.frame_idle;   // Signal end of frame
    // Nothing else to do, next event scheduled.
    return;
  }

  // Find how many bytes can we enqueue
  // NOTE: the data is in "verbose" level representation
  uint8_t tx_fifo_free = UART_FIFO_LEN - tx_fifo_rem;
  // This is the amount of data we can handle now
  // NOTE: the data is in "compact" color component representation
  size_t data_to_receive = std::min(frame_rem, (StripSizeType)(tx_fifo_free >> 2));

#if USE_IDF_RINGBUF
  bool ready_to_receive = data_to_receive > 0;
#else
  // We can only process a whole pixel block at a time
  bool ready_to_receive = (data_to_receive >= (PIXEL_BLOCK_SIZE * sizeof(RGB8BPixel))) ||
                          (data_to_receive == frame_rem);
#endif

  if (ready_to_receive) {
    // Try to receive some data
#if USE_IDF_RINGBUF
    size_t data_len = 0;
    auto pixel_data = (uint8_t*)xRingbufferReceiveUpToFromISR(ringbuf_, &data_len, data_to_receive);
#else
    auto pixel_data = (uint8_t*)_recv_pixel_block_from_isr();
    size_t data_len = std::min((size_t)(PIXEL_BLOCK_SIZE * sizeof(RGB8BPixel)), data_to_receive);
#endif

    if (pixel_data == nullptr) {
      // There is no data, check the frame underflow
      switch (state_.frame_underflow) {
        case 0: {  // This is the first out-of-data occurrence
          // The time needed to send remaining data + 75% of reset time
          uint16_t underflow_threshold =
              buffer_depletion_time_[std::min(tx_fifo_rem, UART_TX_FIFO_THRESHOLD)] +
              config_.reset_us - FRAME_UNDERFLOW_MARGIN;
          // Schedule a one-shot hw_timer event to wait for data
          // Hopefully some data will arrive before the deadline!
          _hw_timer_rearm(underflow_threshold);
          ++state_.frame_underflow;
        } break;

        case 1:  // The sentinel is up, and unfortunately we have a real underflow
          // ets_printf("* Frame Underflow! %d/%d\n", state_.frame_pos, state_.frame_size);
          // Entering discard mode
          ++stats_.underflow_actual;
          ++state_.frame_underflow;
          [[fallthrough]];

        default:  // We are in discard mode,
          // Schedule a one-shot hw_timer event to wait for rest of the frame.
          _hw_timer_rearm(config_.reset_us);
      }
      // Nothing else to do, next event scheduled.
      return;
    }

    // We got data!
    state_.frame_pos += data_len;

    // Check the frame underflow
    switch (state_.frame_underflow) {
      case 1:  // Successfully recovered before underflow!
        ++stats_.underflow_near_miss;
        [[fallthrough]];

      case 0:  // We are in normal receive mode
        state_.frame_underflow = 0;
        // We can proceed with sending frame data
        break;

      default: {  // We are in discard mode
#if USE_IDF_RINGBUF
        // There is nothing to send, just return the buffer
        BaseType_t context_switch = pdFALSE;
        vRingbufferReturnItemFromISR(ringbuf_, pixel_data, &context_switch);
        if (context_switch) portYIELD_FROM_ISR();
#else
        _return_pixel_block_from_isr();
#endif

        // If we have completed discarding the frame...
        if (state_.frame_pos == state_.frame_size) {
          state_.frame_pos = 0;        // Reset position for the next frame
          state_.frame_underflow = 0;  // Reset underflow state
          // It is possible this was the last frame and we are entering idle state.
          state_.frame_idle = 2;
          // Since we have already exceeded the reset time we can start
          // working on the next frame immediately!
          goto check_idle;
        }
        // Otherwise, continue wait for rest of the frame.
        _hw_timer_rearm(config_.reset_us);
        // Nothing else to do, next event scheduled.
        return;
      }
    }

    // Produce UART data and send them out
    for (size_t i = 0; i < data_len; ++i) {
      _uart_send_color_component(pixel_data[i]);
    }

    // Done sending, return the buffer
#if USE_IDF_RINGBUF
    BaseType_t context_switch = pdFALSE;
    vRingbufferReturnItemFromISR(ringbuf_, pixel_data, &context_switch);
    if (context_switch) portYIELD_FROM_ISR();
#else
    _return_pixel_block_from_isr();
#endif
  }

  // Call us again when the current batch is mostly done.
  portENTER_CRITICAL();
  // Should not need to clear again
  // c uart1.int_clr.val |= UART_TXFIFO_EMPTY_INT_CLR_M;
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
  _xt_isr_attach(ETS_UART_INUM, _uart_intr_handler, context.raw);
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
  _xt_isr_attach(ETS_FRC_TIMER1_INUM, _uart_intr_handler, context.raw);
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
#if USE_IDF_RINGBUF
      while (!(data = frame->GetPixelData()).end_of_frame) {
        // Will wait forever, don't expect failure
        xRingbufferSend(ringbuf_, &data.pixel, sizeof(data.pixel), portMAX_DELAY);
      }
#else
      PixelBlock* cur_block = nullptr;
      uint8_t block_pos;
      do {
        if (!cur_block) {
          BlockIdxType write_block = pixel_buffer_.write_block;
          // Not sure why sometimes this has to spin 2 or 3 times...
          while (write_block < 0) {
            xEventGroupWaitBits(pixel_buffer_.events, PIXEL_BLOCK_READY_FOR_WRITE, true, true,
                                portMAX_DELAY);
            write_block = pixel_buffer_.write_block;
            // if (write_block < 0) ESP_LOGI(TAG, "??");
          }
          // assert(write_block >= 0);
          cur_block = &pixel_buffer_.data[write_block];
          block_pos = 0;
        }
        (*cur_block)[block_pos++] = (data = frame->GetPixelData()).pixel;
        if (data.end_of_frame || block_pos >= PIXEL_BLOCK_SIZE) {
          BlockIdxType ready_block = pixel_buffer_.write_block;
          BlockIdxType next_block = (ready_block + 1) % pixel_buffer_.block_count;
          portENTER_CRITICAL();
          BlockIdxType read_block = pixel_buffer_.read_block;
          if (read_block < 0) pixel_buffer_.read_block = read_block = ready_block;
          pixel_buffer_.write_block = (next_block == read_block) ? -1 : next_block;
          portEXIT_CRITICAL();
          cur_block = nullptr;
        }
      } while (!data.end_of_frame);
#endif  // USE_IDF_RINGBUF
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

#if USE_IDF_RINGBUF
  if (ringbuf_ != NULL) {
    vRingbufferDelete(ringbuf_);
    ringbuf_ = NULL;
  }
#else
  if (pixel_buffer_.data != nullptr) {
    free(pixel_buffer_.data);
    pixel_buffer_.data = nullptr;
  }
  if (pixel_buffer_.events != NULL) {
    vEventGroupDelete(pixel_buffer_.events);
    pixel_buffer_.events = NULL;
  }
#endif  // USE_IDF_RINGBUF

  if (renderer == nullptr) {
    ESP_LOGE(TAG, "No LightShow renderer");
    return ESP_ERR_INVALID_ARG;
  }

  // Check if the renderer setup is realistic.
  StripSizeType pixels = renderer->StripSize();
  uint32_t target_frame_us = renderer->FrameInterval();

  uint32_t frame_uart_bits = sizeof(UARTPixel) * 8 * pixels;
  uint32_t frame_data_time_us =
      (frame_uart_bits * 1000000ULL + config.baud_rate - 1) / config.baud_rate;
  uint32_t frame_time_us = frame_data_time_us + config.reset_us;

  // Rough estimation of FPS.
  uint32_t est_fps = 1000000 / frame_time_us;
  ESP_LOGI(TAG, "LightShow frame %d pixels, max FPS ~= %d", pixels, est_fps);
  if (target_frame_us < frame_time_us) {
    ESP_LOGW(TAG, "Renderer has an infeasible target FPS");
  }

  // Validate other timing configs
  if (config.jitter_budget_us < kMinJitterBudget || config.jitter_budget_us > kMaxJitterBudget) {
    ESP_LOGE(TAG, "Invalid jitter budget of %d us", config.jitter_budget_us);
    return ESP_ERR_INVALID_ARG;
  }
  if (config.reset_us < kMinResetTime || config.reset_us > kMaxResetTime) {
    ESP_LOGE(TAG, "Invalid reset time of %d us", config.reset_us);
    return ESP_ERR_INVALID_ARG;
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

#if USE_IDF_RINGBUF

  StripSizeType ringbuf_size = dejitter_pixels * sizeof(RGB8BPixel);
  ESP_LOGI(TAG, "LightShow jitter budget %d us (%d pixels, %d bytes)", config.jitter_budget_us,
           dejitter_pixels, ringbuf_size);
  ringbuf_ = xRingbufferCreate(ringbuf_size, RINGBUF_TYPE_BYTEBUF);
  if (ringbuf_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate LightShow ring buffer for %d pixels (%d bytes)",
             dejitter_pixels, ringbuf_size);
    return ESP_ERR_NO_MEM;
  }

#else

  pixel_buffer_.block_count = (dejitter_pixels + PIXEL_BLOCK_SIZE - 1) / PIXEL_BLOCK_SIZE;
  if (pixel_buffer_.block_count > MAX_PIXEL_BLOCK_COUNT) {
    ESP_LOGE(TAG, "Too many pixel buffer blocks (%d)", pixel_buffer_.block_count);
    return ESP_ERR_INVALID_STATE;
  }
  ESP_LOGI(TAG, "LightShow jitter budget %d us (%d*%d pixels, %d bytes)", config.jitter_budget_us,
           pixel_buffer_.block_count, PIXEL_BLOCK_SIZE,
           pixel_buffer_.block_count * sizeof(PixelBlock));
  pixel_buffer_.data = (PixelBlock*)malloc(pixel_buffer_.block_count * sizeof(PixelBlock));
  if (pixel_buffer_.data == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate LightShow pixel buffer");
    return ESP_ERR_NO_MEM;
  }
  pixel_buffer_.events = xEventGroupCreate();
  if (pixel_buffer_.events == NULL) {
    ESP_LOGE(TAG, "Failed to allocate LightShow pixel buffer events");
    return ESP_ERR_NO_MEM;
  }

#endif  // USE_IDF_RINGBUF

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

esp_err_t DriverStart(uint16_t task_stack, UBaseType_t task_priority) {
  if (renderer_ == nullptr) {
    ESP_LOGE(TAG, "Need to set up LightShow driver first");
    return ESP_ERR_INVALID_STATE;
  }
  if (render_engine_task_ != NULL) {
    ESP_LOGE(TAG, "LightShow driver already started");
    return ESP_ERR_INVALID_STATE;
  }

  // Reset jitter pixel buffer
#if USE_IDF_RINGBUF
  while (true) {
    UBaseType_t items_waiting;
    vRingbufferGetInfo(ringbuf_, NULL, NULL, NULL, &items_waiting);
    if (items_waiting == 0) break;

    size_t discard_size;
    void* discard_data = xRingbufferReceive(ringbuf_, &discard_size, 0);
    ESP_LOGD(TAG, "Discarding %d bytes from ring buffer", discard_size);
    vRingbufferReturnItem(ringbuf_, discard_data);
  }
#else
  pixel_buffer_.read_block = -1;
  pixel_buffer_.write_block = 0;
#endif  // USE_IDF_RINGBUF

  // Initialize internal states.
  state_ = {};
  state_.frame_size = renderer_->StripSize() * sizeof(RGB8BPixel);
  state_.frame_idle = 1;  // Start in end of frame state

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
  portENTER_CRITICAL();
  IOStats result = stats_;
  stats_ = {};
  portEXIT_CRITICAL();
  return result;
}

}  // namespace zw_esp8266::lightshow
