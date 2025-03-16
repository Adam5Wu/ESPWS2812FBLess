// Light-show frame
#ifndef ZWESP8266_LSFRAME
#define ZWESP8266_LSFRAME

#include <vector>
#include <memory>
#include <utility>
#include <string>

#include "LSUtils.hpp"
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

  Frame(const Frame&) = delete;
  Frame& operator=(const Frame&) = delete;

  enum class FrameType {
    kUniformColor,
    kBlender,
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

// A frame that blends multiple frames
class BlenderFrame : public Frame {
 public:
  BlenderFrame(std::unique_ptr<Frame> base_frame);

  FrameType Type() const override { return FrameType::kBlender; }
  void UpdateOverlay(std::unique_ptr<Frame> blend_frame, uint8_t alpha);
  PixelWithStatus GetPixelData() override;

  void Reset() override {
    Frame::Reset();
    base_frame_->Reset();
    if (blend_frame_ != nullptr) blend_frame_->Reset();
  }

#ifndef NDEBUG
  std::string DebugString() const override {
    return "Blender[" + base_frame_->DebugString() + "]: " + blend_frame_->DebugString() + " @" +
           std::to_string(alpha_);
  }
#endif

 private:
  std::unique_ptr<Frame> base_frame_;
  std::unique_ptr<Frame> blend_frame_;
  uint8_t alpha_;
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
    return "UniformColor: " + std::to_string(color.r) + ", " + std::to_string(color.g) + ", " +
           std::to_string(color.b);
  }
#endif
};

struct DotState {
  RGB8BPixel color;
  // "Glow" of the dot in 1/10 of a pixels.
  // When glow is 0, a dot is exactly 1 pixel wide.
  // (Hence the maximum dot size will be 26.5 pixels.)
  //
  // Non-zero glow will add to the dot's total "size", which is rendered
  // using a ~3.16-sigma gaussian distribution (i.e. sigma ~= size/6.32).
  uint8_t glow;
  // Position of the object in the strip measured by progression (12-bit),
  // where 0 is the beginning of the strip and 4096 is the end.
  // The dot will be rendered at the center, hance its "glows" will be
  // cut-off when it is near the ends of the strip.
  ProgressionType pos_pgrs;
};

// A frame that displays a colored dot
class ColorDotFrame : public Frame {
 public:
  const DotState dot;
  const RGB8BPixel bgcolor;

  ColorDotFrame(StripSizeType strip_size, const DotState& dot, RGB8BPixel bgcolor);

  FrameType Type() const override { return FrameType::kColorDot; }
  PixelWithStatus GetPixelData() override;

#ifndef NDEBUG
  std::string DebugString() const override {
    return "ColorDot[" + std::to_string(dot.glow / 10 + 1) + "." + std::to_string(dot.glow % 10) +
           "]: " + std::to_string(dot.color.r) + ", " + std::to_string(dot.color.g) + ", " +
           std::to_string(dot.color.b) + " @ PGRS=" + std::to_string(dot.pos_pgrs);
  }
#endif

 private:
  StripSizeType start_pos_, end_pos_;
  float dot_pos_;        // The real position of the dot on the strip
  float two_sigma_sqr_;  // Deterministic component of the Gaussian PDF exponent
};

}  // namespace zw_esp8266::lightshow

#endif  // ZWESP8266_LSFRAME