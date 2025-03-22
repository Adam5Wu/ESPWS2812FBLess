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
  union {
    RGB888 rgb;
    RGB8BPixel pixel;
    AlphaBlendPixel blend_pixel;
  };
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
    kComputedColorDot,
    kBlendedColorDot,
  };
  // Get the type of the frame (works with `-fno-rtti`).
  virtual FrameType Type() const = 0;

  // Whether the frame returns translucent pixel data.
  // A translucent frame returns `RGBA8BBlendPixel` instead of `RGB8BPixel`.
  // The pixels are blended to the base frame by the render.
  virtual bool IsTranslucent() const { return false; }

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
  PixelWithStatus GetPixelData() override;

  void Reset() override {
    Frame::Reset();
    base_frame_->Reset();
    if (blend_frame_ != nullptr) blend_frame_->Reset();
  }

  // Replace current blend frame with a new frame, and new alpha
  // Will also reset rendering location.
  // Note that if the blend_frame is translucent, alpha is ignored.
  void UpdateOverlay(std::unique_ptr<Frame> blend_frame, uint8_t alpha);

#ifndef NDEBUG
  std::string DebugString() const override {
    return "Blender[" + base_frame_->DebugString() + "]: " + blend_frame_->DebugString() + " @" +
           std::to_string(alpha_);
  }
#endif

  const Frame* base_frame() const { return base_frame_.get(); }
  const Frame* blend_frame() const { return blend_frame_.get(); }

 private:
  std::unique_ptr<Frame> base_frame_;
  std::unique_ptr<Frame> blend_frame_;
  bool translucent_blend_;
  uint8_t alpha_;
};

// A frame that displays a single color
class UniformColorFrame : public Frame {
 public:
  const RGB888 color;

  UniformColorFrame(StripSizeType strip_size, RGB888 frame_color)
      : Frame(strip_size), color(frame_color) {}

  FrameType Type() const override { return FrameType::kUniformColor; }
  PixelWithStatus GetPixelData() override {
    return PixelWithStatus{.rgb = color, .end_of_frame = ++index_ > size};
  }

#ifndef NDEBUG
  std::string DebugString() const override {
    return "UniformColor: " + std::to_string(color.r) + ", " + std::to_string(color.g) + ", " +
           std::to_string(color.b);
  }
#endif
};

struct DotState {
  RGB888 color;
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

class ColorDotFrame : public Frame {
 public:
  const DotState dot;

 protected:
  ColorDotFrame(StripSizeType strip_size, const DotState& dot) : Frame(strip_size), dot(dot) {}
};

// A frame that displays a colored dot.
// The dot pixels are rendered by real-time floating point arithmetics.
class ComputedColorDotFrame : public ColorDotFrame {
 public:
  const RGB888 bgcolor;

  ComputedColorDotFrame(StripSizeType strip_size, const DotState& dot, RGB888 bgcolor);

  FrameType Type() const override { return FrameType::kComputedColorDot; }
  PixelWithStatus GetPixelData() override;

#ifndef NDEBUG
  std::string DebugString() const override {
    return "ComputedColorDot[" + std::to_string(dot.glow / 10 + 1) + "." +
           std::to_string(dot.glow % 10) + "]: " + std::to_string(dot.color.r) + ", " +
           std::to_string(dot.color.g) + ", " + std::to_string(dot.color.b) +
           " @ PGRS=" + std::to_string(dot.pos_pgrs);
  }
#endif

 private:
  StripSizeType start_pos_, end_pos_;
  float dot_pos_;        // The real position of the dot on the strip
  float two_sigma_sqr_;  // Deterministic component of the Gaussian PDF exponent
};

// A frame that displays a colored dot.
// The dot are pre-computed RGBA8BBlenderPixel for more efficient rendering.
class BlendedColorDotFrame : public ColorDotFrame {
 public:
  BlendedColorDotFrame(StripSizeType strip_size, const DotState& dot);

  FrameType Type() const override { return FrameType::kBlendedColorDot; }
  bool IsTranslucent() const override { return true; }
  PixelWithStatus GetPixelData() override;

#ifndef NDEBUG
  std::string DebugString() const override {
    return "BlendedColorDot[" + std::to_string(dot.glow / 10 + 1) + "." +
           std::to_string(dot.glow % 10) + "]: " + std::to_string(dot.color.r) + ", " +
           std::to_string(dot.color.g) + ", " + std::to_string(dot.color.b) +
           " @ PGRS=" + std::to_string(dot.pos_pgrs);
  }
#endif

 private:
  StripSizeType start_pos_, end_pos_;
  std::vector<AlphaBlendPixel> blend_pixels_;
};

}  // namespace zw_esp8266::lightshow

#endif  // ZWESP8266_LSFRAME