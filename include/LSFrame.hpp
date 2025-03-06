// Light-show frame
#ifndef ZWESP8266_LSFRAME
#define ZWESP8266_LSFRAME

#include <string>

#include "LSPixel.hpp"

namespace zw_esp8266::lightshow {

struct PixelWithStatus {
  RGB8BPixel pixel;
  bool end_of_frame;
};

using StripSizeType = uint16_t;

// Represents a "frame" of light-show data, i.e. an array of pixels.
// Note that this may not be a literal array in memory.
class Frame {
 public:
  const StripSizeType size;

  virtual ~Frame() = default;

  enum class FrameType {
    kUniformColor,
    kColorDot,
  };
  // Get the type of the frame (works with `-fno-rtti`).
  virtual FrameType Type() const = 0;

  // Obtain a sequence of pixels
  // Returns nullptr when reached the end of the frame.
  virtual PixelWithStatus GetPixelData() = 0;

  // Reset the frame so it can be re-rendered.
  virtual void Reset() { index_ = 0; }

#ifndef NDEBUG
  // String representation of the frame for debugging.
  virtual std::string DebugString() const = 0;
#endif

 protected:
  StripSizeType index_ = 0;

  Frame(StripSizeType strip_size) : size(strip_size) {}
};

// A frame that displays a single color
class UniformColorFrame : public Frame {
 public:
  const RGB8BPixel color;

  UniformColorFrame(StripSizeType strip_size, RGB8BPixel frame_color)
      : Frame(strip_size), color(frame_color) {}

  FrameType Type() const override { return FrameType::kUniformColor; }
  PixelWithStatus GetPixelData() override {
    return {.pixel = color, .end_of_frame = index_++ >= size};
  }

#ifndef NDEBUG
  std::string DebugString() const override {
    return "UniformColor[" + std::to_string(size) + "] = " + std::to_string(color.r) + ", " +
           std::to_string(color.g) + ", " + std::to_string(color.b);
  }
#endif
};

struct DotState {
  // Position of the object in the strip [0, 1000], where
  // 0 is the beginning of the strip and 1000 is the end.
  uint16_t pos_pmr;
  RGB8BPixel color;
};

// A frame that displays a colored dot
class ColorDotFrame : public Frame {
 public:
  const DotState dot;
  const RGB8BPixel bgcolor;

  ColorDotFrame(StripSizeType strip_size, const DotState& dot_state, RGB8BPixel background)
      : Frame(strip_size), dot(dot_state), bgcolor(background) {
    Init();
  }

  FrameType Type() const override { return FrameType::kColorDot; }
  PixelWithStatus GetPixelData() override {
    StripSizeType scan_pos = index_++;
    if (scan_pos == start_pos) return {.pixel = dpix_[0], .end_of_frame = false};
    if (scan_pos < size && scan_pos == start_pos + 1)
      return {.pixel = dpix_[1], .end_of_frame = false};
    return {.pixel = bgcolor, .end_of_frame = scan_pos >= size};
  }

#ifndef NDEBUG
  std::string DebugString() const override {
    return "ColorDot[" + std::to_string(size) + "] = " + std::to_string(dot.color.r) + ", " +
           std::to_string(dot.color.g) + ", " + std::to_string(dot.color.b) + " @ " +
           std::to_string(dot.pos_pmr / 10) + "." + std::to_string(dot.pos_pmr % 10) + "%";
  }
#endif

 private:
  StripSizeType start_pos;
  RGB8BPixel dpix_[2];

  void Init() {
    start_pos = (size - 1) * dot.pos_pmr / 1000;
    uint8_t tpct = ((size - 1) * dot.pos_pmr / 10) % 100;
    if (tpct) {
      dpix_[0].u[0] = bgcolor.u[0] + (int16_t)(dot.color.u[0] - bgcolor.u[0]) * (100 - tpct) / 100;
      dpix_[0].u[1] = bgcolor.u[1] + (int16_t)(dot.color.u[1] - bgcolor.u[1]) * (100 - tpct) / 100;
      dpix_[0].u[2] = bgcolor.u[2] + (int16_t)(dot.color.u[2] - bgcolor.u[2]) * (100 - tpct) / 100;
      dpix_[1].u[0] = bgcolor.u[0] + (int16_t)(dot.color.u[0] - bgcolor.u[0]) * tpct / 100;
      dpix_[1].u[1] = bgcolor.u[1] + (int16_t)(dot.color.u[1] - bgcolor.u[1]) * tpct / 100;
      dpix_[1].u[2] = bgcolor.u[2] + (int16_t)(dot.color.u[2] - bgcolor.u[2]) * tpct / 100;
    } else {
      dpix_[0] = dot.color;
      dpix_[1] = bgcolor;
    }
  }
};

}  // namespace zw_esp8266::lightshow

#endif  // ZWESP8266_LSFRAME