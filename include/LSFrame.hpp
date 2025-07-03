// Light-show frame
#ifndef ZWLIGHTSHOW_FRAME
#define ZWLIGHTSHOW_FRAME

#include <vector>
#include <memory>
#include <utility>
#include <string>

#include "LSUtils.hpp"
#include "LSPixel.hpp"

namespace zw::esp8266::lightshow {

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
    kCustom,
    kUniformColor,
    kBlender,
    kColorDot,
    kWiper,
    kRGBColorWheel,
    kHSVColorWheel,
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

  // Replace current blend frame with a new frame, and new alpha.
  // Will also reset rendering location.
  // Note that if the blend_frame is translucent, alpha is ignored.
  void UpdateOverlay(std::unique_ptr<Frame> blend_frame, const std::vector<uint8_t>& alpha_mask);

  inline void UpdateOverlay(std::unique_ptr<Frame> blend_frame, uint8_t alpha) {
    UpdateOverlay(std::move(blend_frame), std::vector<uint8_t>{alpha});
  }

#ifndef NDEBUG
  std::string DebugString() const override {
    std::string alpha_str;
    for (uint8_t alpha : alpha_mask_) alpha_str.append(std::to_string(alpha)).push_back(',');
    alpha_str.pop_back();
    return "Blender[" + base_frame_->DebugString() + "]: " + blend_frame_->DebugString() + " @" +
           (alpha_mask_.size() == 1 ? alpha_str : "{" + alpha_str + "}");
  }
#endif

  const Frame* base_frame() const { return base_frame_.get(); }
  const Frame* blend_frame() const { return blend_frame_.get(); }

 private:
  std::unique_ptr<Frame> base_frame_;
  std::unique_ptr<Frame> blend_frame_;
  std::vector<uint8_t> alpha_mask_;
  bool translucent_blend_;
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
  std::string DebugString() const override { return "UniformColor: " + to_string(color); }
#endif
};

// A dot is a pixel drawn on a certain position of a strip.
//
// |------------------====*====----------------|
//  ^--- start  glow---^  ^--- dot     end ---^
struct DotState {
  RGB888 color;
  // "Glow" of the dot in 1/10 of a pixels.
  // When glow is 0, a dot is exactly 1 pixel wide.
  // (Hence the maximum dot size will be 26.5 pixels.)
  //
  // Non-zero glow will add to the dot's total "size", which is rendered
  // using a ~3.16-sigma gaussian distribution (i.e. sigma ~= size/6.32).
  uint8_t glow;
  // Position of the dot on the strip measured by progression (12-bit),
  // where 0 is the beginning of the strip and PGRS_FULL is the end.
  // The dot will be rendered *at its center*, hence its "glows" will be
  // cut-off when it is near the ends of the strip, but the dot itself
  // will always be on the strip.
  ProgressionType pos_pgrs;
};

// An abstract frame for rendering a color dot.
class ColorDotFrame : public Frame {
 public:
  const DotState dot;
  const RGB888 bgcolor;

  ColorDotFrame(StripSizeType strip_size, const DotState& dot, RGB888 bgcolor);

  FrameType Type() const override { return FrameType::kColorDot; }
  PixelWithStatus GetPixelData() override;

#ifndef NDEBUG
  std::string DebugString() const override {
    return "ColorDot[" + std::to_string(dot.glow / 10 + 1) + "." + std::to_string(dot.glow % 10) +
           "@" + std::to_string(dot.pos_pgrs) + "]: " + to_string(dot.color) +
           "; bg=" + to_string(bgcolor);
  }
#endif

 protected:
  // The real position of the dot on the strip
  float dot_pos_;
  // The number of pixels of a half dot (including glow)
  float half_dot_size_;
  // The range of pixels on the strip affected by the dot
  StripSizeType start_pos_, end_pos_;
  // Deterministic component of the Gaussian PDF exponent
  float two_sigma_sqr_;
};

// A "wiper" separates a strip into three parts: left, right, and in between them a
// predefined width of transitional interval, aka. the "wiper blade".
//
// |-------------========--------------------|
//  ^--- start    ^--- wiper blade   end ---^
struct WiperProp {
  // The color of the blade, and to its left and right.
  // The alpha channel allows expression of translucency, hence a "dot" can
  // be expressed as having a solid `color` and transparent `l_color` and `r_color`.
  RGBA8888 color, l_color, r_color;
  // Width of the wiper blade, expressed as the portion of the strip length.
  // A 1 means the blade's width is equal to the entire strip.
  // Note that the width *could* be larger than 1, in which case a part of the
  // wiper could occupy the entire strip, depending on the position.
  float width;
};

struct WiperState : WiperProp {
  // Position of the wiper on the strip measured by progression (12-bit).
  // *Different from the dot*, the wiper position accounts for the blade width.
  // So, 0 means:
  //             =======|------------------------------|
  // wiper blade ---^    ^--- strip start      end ---^
  // And PGRS_FULL means:
  //                    |------------------------------|=======
  //                     ^--- strip start      end ---^    ^--- wiper blade
  ProgressionType pos_pgrs;
};

// An abstract frame for rendering a wiper.
class WiperFrame : public Frame {
 public:
  const WiperState wiper;

  // This callback determines how the blade part is rendered.
  // - The input is a progression value denoting the position on the blade.
  //   0 is the left edge and PGRS_FULL is the right.
  // - The return is also a progression for computing the pixel:
  //   * If the current position (i.e. input) is <= PGRS_MIDWAY, the pixel
  //     will be value-blended (all 4 channels) between `l_color` and `color`,
  //     where 0 means all `l_color` and PGRS_FULL means all `color`;
  //   * Otherwise, value-blend between `r_color` and `color`, where 0 means
  //     all `r_color` and PGRS_FULL means all `color`;
  using BladeGenerator = ProgressionType (*)(ProgressionType);

  WiperFrame(StripSizeType strip_size, const WiperState& wiper, BladeGenerator blade_func);

  FrameType Type() const override { return FrameType::kWiper; }
  bool IsTranslucent() const override { return true; }
  PixelWithStatus GetPixelData() override;

#ifndef NDEBUG
  std::string DebugString() const override {
    return "Wiper[" + std::to_string(uint16_t(wiper.width * 100)) + "%@" +
           std::to_string(wiper.pos_pgrs) + "]: " + to_string(wiper.color) +
           "; l=" + to_string(wiper.l_color) + "; r=" + to_string(wiper.r_color);
  }
#endif

 protected:
  const BladeGenerator gen_func_;
  // The real position of the wiper blade *center* on the strip
  // Note that this can go off the strip (e.g. negative, or > strip size)
  float blade_center_;
  // The number of pixels of a half blade
  float half_blade_size_;
  // The range of pixels on the strip affected by the blade
  // Why they are not of type `StripSizeType`? When blade position is at the extremes
  // (blade moved off-strip), we could have negative positions!
  int16_t start_pos_, end_pos_;

  // Return the pixel on the blade at given position.
  virtual AlphaBlendPixel GetBladePixel(StripSizeType idx) const = 0;
};

class ComputedWiperFrame : public WiperFrame {
 public:
  ComputedWiperFrame(StripSizeType strip_size, const WiperState& wiper, BladeGenerator blade_func)
      : WiperFrame(strip_size, wiper, blade_func) {}

 protected:
  AlphaBlendPixel GetBladePixel(StripSizeType idx) const override;
};

class CachedWiperFrame : public ComputedWiperFrame {
 public:
  CachedWiperFrame(StripSizeType strip_size, const WiperState& wiper, BladeGenerator blade_func);

 private:
  std::vector<AlphaBlendPixel> blend_pixels_;

  AlphaBlendPixel GetBladePixel(StripSizeType idx) const override;
};

struct ColorWheelProp {
  // Perimeter of the color wheel in pixels.
  // Note that the width *could* be larger than the strip size, in which case
  // only a part of the wheel can be displayed on the strip at a time.
  StripSizeType width;
};

struct ColorWheelState : ColorWheelProp {
  // The color wheel's origin (on its perimeter) measured by progression (12-bit).
  // Note that regardless of the value a color wheel frame will always fill
  // the entire strip. The difference is only which pixel shows what color.
  ProgressionType pos_pgrs;
  // The intensity of the color wheel
  ProgressionType intensity;
};

// RGB color wheel.
// Each color component is spaced evenly "out-of-phase" on the wheel.
class RGBColorWheelFrame : public Frame {
 public:
  const ColorWheelState state;

  // This callback determines how the wheel "spins".
  // - The input is a progression value denoting distance the wheel travels
  //   from its starting point.
  // - The return is also a progression for computing the color component value.
  using WheelGenerator = ProgressionType (*)(ProgressionType);

  RGBColorWheelFrame(StripSizeType strip_size, const ColorWheelState& state,
                     WheelGenerator wheel_func);

  FrameType Type() const override { return FrameType::kRGBColorWheel; }
  bool IsTranslucent() const override { return false; }
  PixelWithStatus GetPixelData() override;

#ifndef NDEBUG
  std::string DebugString() const override {
    return "RGBColorWheel[" + std::to_string(state.width) + "]@" + std::to_string(state.pos_pgrs);
  }
#endif

 protected:
  const WheelGenerator gen_func_;
  // The precomputed pixel data of the wheel
  // Its size is the smaller of "wheel width" and strip size.
  std::vector<RGB888> wheel_pixels_;

  // Return the pixel on the wheel at given position.
  RGB888 GetWheelPixel(float wheel_pos) const;
};

// HSV color wheel.
class HSVColorWheelFrame : public Frame {
 public:
  const ColorWheelState state;

  HSVColorWheelFrame(StripSizeType strip_size, const ColorWheelState& state);

  FrameType Type() const override { return FrameType::kHSVColorWheel; }
  bool IsTranslucent() const override { return false; }
  PixelWithStatus GetPixelData() override;

#ifndef NDEBUG
  std::string DebugString() const override {
    return "HSVColorWheel[" + std::to_string(state.width) + "]@" + std::to_string(state.pos_pgrs);
  }
#endif

 protected:
  // The precomputed pixel data of the wheel
  // Its size is the smaller of "wheel width" and strip size.
  std::vector<RGB888> wheel_pixels_;

  // Return the pixel on the wheel at given position.
  RGB888 GetWheelPixel(float wheel_pos) const;
};

}  // namespace zw::esp8266::lightshow

#endif  // ZWLIGHTSHOW_FRAME