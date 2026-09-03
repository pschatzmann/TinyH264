#pragma once
#include <cstring>

#include "StdAllocator.h"
#include "common/h264_buffer.h"
#include "encoder/h264_color_convert.h"
#include "encoder/h264_encoder.h"
#include "encoder/h264_hw_encoder_p4.h"
#ifdef ESP32
#include "PSRAMAllocatorESP32.h"
#endif

// TinyH264Encoder: a minimal, header-only H.264 Baseline Profile (CAVLC)
// encoder for microcontrollers, matching TinyH264Decoder's own scope and
// design center - see docs/encoding.md for full
// scope (I_16x16/I_4x4 intra, P_16x16/P_Skip inter with automatic
// Intra-in-P-slice fallback, single-reference-frame P-frames, simple
// rate control).
//
// Usage - encodeFrame() is the only public encode entry point: configure
// picture size/QP policy once via setSize()/setQp() (or setTargetBitrate()
// for rate control instead of a fixed QP), then call encodeFrame() once
// per picture, in order - it decides I-frame vs. P-frame automatically
// (see its own doc comment for exactly when):
//
//   using namespace tinyh264;
//
//   TinyH264Encoder<> encoder;
//   uint8_t bitstream[32768];
//   encoder.setSize(width, height);
//   encoder.setQp(26);  // or setTargetBitrate() instead, for rate control
//   for (...) {
//     size_t n = encoder.encodeFrame(y, u, v, bitstream, sizeof(bitstream));
//     if (n == 0) { /* buffer too small, width/height not a multiple of
//                      16, or setSize() was never called - see
//                      encodeFrame()'s own doc comment */ }
//     // bitstream[0..n) is now a complete Annex-B NAL unit (or units),
//     // decodable by this library's own TinyH264Decoder or any
//     // conformant H.264 decoder (verified against real ffmpeg - see
//     // test/native/test_encode_iframe.cpp/test_encode_pframe.cpp).
//   }
//
// setStride()/setPackedStride() override the source row stride(s) for a
// padded source buffer (both default to tightly packed, derived from
// setSize()'s width - see each one's own doc comment).
// encodeFrameRgb888()/encodeFrameRgb666()/encodeFrameRgb565()/
// encodeFrameYuv422() are the same idea for RGB/packed-YUV422 source data
// instead of separate YUV planes.
//
// There is deliberately no public way to force an I-frame or P-frame on a
// specific call - setKeyframeInterval() (periodic) and setSize() (a
// resolution change always forces an I-frame) are the only ways to
// influence *when* a keyframe happens; see Encoder's own file header
// comment (encoder/h264_encoder.h) for why.
//
// The Allocator template parameter (default StdAllocator<uint8_t>, see
// StdAllocator.h) is used for the picture buffers this class holds
// internally (its own closed-loop reconstruction plus the single P-frame
// reference - see encoder/h264_macroblock_encode.h's MbEncodeContext doc
// comment for why an encoder needs one at all) - exactly the same
// PSRAM-placement mechanism TinyH264Decoder offers:
//   TinyH264Encoder<PSRAMAllocatorESP32<uint8_t>> encoder;
// An out-of-memory allocation failure surfaces as a 0 return from
// encodeFrame() (and its color-format overloads) or from begin(),
// instead of crashing - see StdAllocator.h's file comment.
//
// See h264_config.h for the resolution budget (H264_MAX_WIDTH/
// H264_MAX_HEIGHT, shared with TinyH264Decoder).

namespace tinyh264 {

/**
 * Public-facing encoder API: wraps the internal Encoder<Allocator> (see
 * encoder/h264_encoder.h) behind encodeFrame() and its color-format
 * siblings, which take raw pixel data and produce a complete Annex-B
 * bitstream, without the caller needing to know anything about NAL
 * units, slices, or macroblocks. `Allocator` controls where the
 * reconstructed-picture buffers are allocated - see the file comment
 * above for the PSRAM usage example.
 */
template <typename Allocator = StdAllocator<uint8_t>>
class TinyH264Encoder {
 public:
  /**
   * Constructs a TinyH264Encoder, optionally pre-configuring picture
   * width/height and periodic keyframe interval in one step instead of
   * calling setSize()/setKeyframeInterval() separately afterward - see
   * Encoder's own constructor comment (encoder/h264_encoder.h). All
   * three default to 0 (unconfigured/no periodic keyframe), matching a
   * default-constructed TinyH264Encoder's previous behavior exactly -
   * `TinyH264Encoder<> encoder;` still compiles and behaves the same as
   * before this constructor existed.
   */
  TinyH264Encoder(int width = 0, int height = 0, int keyframeInterval = 0)
      : encoder_(width, height, keyframeInterval) {
    width_ = width;
    height_ = height;
    keyframeInterval_ = keyframeInterval;
  }

  /**
   * True if this build can actually use ESP32-P4's dedicated hardware
   * H.264 encoder - i.e. this is being compiled for the `esp32p4`
   * target (chip revision < 3.0 - see
   * `encoder/h264_hw_encoder_p4.h`'s file comment for why that specific
   * silicon revision). Unlike an earlier version of this feature, this
   * is available from a plain Arduino IDE/`arduino-cli` sketch build
   * too - the hardware driver talks directly to the H.264 core/DMA
   * registers and only genuinely generic ESP-IDF infrastructure
   * (`esp_intr_alloc`/`esp_cache_msync`/FreeRTOS), none of which needs
   * Espressif's own `esp_h264`/`esp_video` components. A compile-time
   * answer (no device is touched to check this), so it's safe to call
   * before setUseHardware().
   */
  static constexpr bool hardwareAvailable() {
#ifdef TINYH264_HW_ENCODER_P4_AVAILABLE
    return true;
#else
    return false;
#endif
  }

  /**
   * Switches `encodeFrame()` (and its color-format overloads) between
   * this library's own software Baseline/CAVLC pipeline (the default)
   * and ESP32-P4's dedicated hardware H.264 encoder
   * (`encoder/h264_hw_encoder_p4.h::HwEncoderP4`). Returns false and
   * leaves the previous mode in effect if `enable` is true but
   * `hardwareAvailable()` is false.
   *
   * The hardware path is a genuinely different implementation, with a
   * deliberately narrower feature set than the software one - see
   * `HwEncoderP4`'s own file header for the full scope/validation-status
   * disclaimer (this is the one part of the library that hasn't been
   * validated the way everything else has - no pixel-diff oracle exists
   * for real hardware output). While hardware mode is active:
   * - `setQp()` sets the one fixed QP used for the whole stream - the
   *   hardware driver doesn't support `setTargetBitrate()`-style rate
   *   control in this version (unlike the software path, there's no
   *   `-1`/auto sentinel; hardware mode requires an explicit `setQp()`
   *   call with a real 0-51 value, and fails cleanly - `encodeFrame()`
   *   returning 0 - if none was ever given).
   * - `setKeyframeInterval()` becomes the hardware's own GOP size.
   * - `setMotionSearchRange()`/`setMotionSearchAlgorithm()`/
   *   `setAllOptimizationsActive()` and `setTargetBitrate()` are
   *   software-encoder-specific and silently ignored in hardware mode.
   * - Unlike the software path, there's no per-call I/P decision to
   *   reason about - the hardware decides GOP placement entirely on its
   *   own once opened.
   *
   * The hardware device is opened lazily (on the next `encodeFrame()`-
   * family call, once `setSize()`/`setQp()` have established real
   * values) rather than by this call itself.
   */
  bool setUseHardware(bool enable) {
    if (enable && !hardwareAvailable()) return false;
    if (useHardware_ != enable) hwConfigDirty_ = true;
    useHardware_ = enable;
    return true;
  }

  /// Whether `encodeFrame()` (and its color-format overloads) currently
  /// route through ESP32-P4's hardware encoder - see setUseHardware().
  bool useHardware() const { return useHardware_; }

  /**
   * Encodes one picture, automatically deciding I-frame vs. P-frame -
   * call this once per picture, in order, from raw YUV 4:2:0 planar
   * source data (`srcY`/`srcU`/`srcV` - separate plane pointers, matching
   * TinyH264Decoder's own y()/u()/v() convention). Uses the width/
   * height/stride/qp configured via setSize()/setStride()/setQp() (see
   * each one's own comment for the tightly-packed-stride/rate-control
   * defaults if you don't call them) instead of taking them as
   * parameters - call setSize() at least once first, or this returns 0.
   * Becomes an I-frame (SPS + PPS + IDR slice) on the first call,
   * whenever setSize() changes width/height from a prior call (a
   * P-frame can't represent a resolution change), or when
   * setKeyframeInterval() was configured and enough frames have passed
   * since the last one; every other call becomes a P-frame (motion-
   * compensated against the previous picture, no SPS/PPS resent).
   * Returns the number of bytes written to `dst`, or 0 if width/height/
   * qp are invalid, `dst`'s capacity was too small, or a picture buffer
   * allocation failed (out of memory - see StdAllocator.h) - same size-
   * checked-return convention as TinyH264Decoder's to*() converters -
   * nothing usable is left in `dst` in that case; call again with a
   * bigger buffer).
   */
  size_t encodeFrame(const uint8_t* srcY, const uint8_t* srcU,
                      const uint8_t* srcV, uint8_t* dst, size_t dstCapacity) {
    if (useHardware_) {
      int strideY = strideY_ > 0 ? strideY_ : width_;
      int strideC = strideC_ > 0 ? strideC_ : width_ / 2;
      return encodeHwPlanar(srcY, strideY, srcU, srcV, strideC, dst,
                             dstCapacity);
    }
    return encoder_.encodeFrame(srcY, srcU, srcV, dst, dstCapacity);
  }

  /**
   * Same as encodeFrame(), for RGB888 source data (3 bytes/pixel, R/G/B
   * order - matches TinyH264Decoder::toRGB888()'s convention) instead of
   * separate YUV planes. Converts internally to YUV 4:2:0 (see
   * encoder/h264_color_convert.h) before encoding - a real, if modest,
   * precision loss on top of the usual DCT quantization, not a lossless
   * passthrough. Row stride defaults to `width*3` (tightly packed)
   * unless overridden via setPackedStride().
   */
  size_t encodeFrameRgb888(const uint8_t* rgb, uint8_t* dst,
                            size_t dstCapacity) {
    if (useHardware_) {
      int stride = packedStride_ > 0 ? packedStride_ : width_ * 3;
      if (!prepareHwScratch()) return 0;
      convertRgb888ToYuv420(rgb, stride, width_, height_, hwScratchY(),
                             width_, hwScratchU(), hwScratchV(), width_ / 2);
      return encodeHwScratch(dst, dstCapacity);
    }
    return encoder_.encodeFrameRgb888(rgb, dst, dstCapacity);
  }

  /**
   * Same as encodeFrame(), for RGB666 source data (3 bytes/pixel, each
   * byte's 6 significant bits left-justified in bits 7:2 - matches
   * TinyH264Decoder::toRGB666()'s convention). Row stride defaults to
   * `width*3` (tightly packed) unless overridden via setPackedStride().
   */
  size_t encodeFrameRgb666(const uint8_t* rgb666, uint8_t* dst,
                            size_t dstCapacity) {
    if (useHardware_) {
      int stride = packedStride_ > 0 ? packedStride_ : width_ * 3;
      if (!prepareHwScratch()) return 0;
      convertRgb666ToYuv420(rgb666, stride, width_, height_, hwScratchY(),
                             width_, hwScratchU(), hwScratchV(), width_ / 2);
      return encodeHwScratch(dst, dstCapacity);
    }
    return encoder_.encodeFrameRgb666(rgb666, dst, dstCapacity);
  }

  /**
   * Same as encodeFrame(), for RGB565 source data (uint16_t/pixel, 5-6-5
   * packed - matches TinyH264Decoder::toRGB565()'s convention). Row
   * stride (in uint16_t entries) defaults to `width` (tightly packed)
   * unless overridden via setPackedStride().
   */
  size_t encodeFrameRgb565(const uint16_t* rgb565, uint8_t* dst,
                            size_t dstCapacity) {
    if (useHardware_) {
      int stride = packedStride_ > 0 ? packedStride_ : width_;
      if (!prepareHwScratch()) return 0;
      convertRgb565ToYuv420(rgb565, stride, width_, height_, hwScratchY(),
                             width_, hwScratchU(), hwScratchV(), width_ / 2);
      return encodeHwScratch(dst, dstCapacity);
    }
    return encoder_.encodeFrameRgb565(rgb565, dst, dstCapacity);
  }

  /**
   * Same as encodeFrame(), for YUYV-order packed YUV 4:2:2 source data
   * (Y0 U0 Y1 V0 per horizontal pixel pair - the common camera-module
   * convention, e.g. OV2640/OV7670 output). Row stride defaults to
   * `width*2` (tightly packed) unless overridden via setPackedStride().
   */
  size_t encodeFrameYuv422(const uint8_t* yuyv, uint8_t* dst,
                            size_t dstCapacity) {
    if (useHardware_) {
      int stride = packedStride_ > 0 ? packedStride_ : width_ * 2;
      if (!prepareHwScratch()) return 0;
      convertYuyv422ToYuv420(yuyv, stride, width_, height_, hwScratchY(),
                              width_, hwScratchU(), hwScratchV(), width_ / 2);
      return encodeHwScratch(dst, dstCapacity);
    }
    return encoder_.encodeFrameYuv422(yuyv, dst, dstCapacity);
  }

  /**
   * Configures rate control: leave `qp` at its default (or call
   * `setQp(-1)` explicitly) to let the encoder pick its own QP each
   * frame, adapted toward `bitsPerSecond` at `fps` frames/sec, instead
   * of a fixed QP. A real-time-appropriate feedback controller, not a
   * two-pass/lookahead one - see Encoder::setTargetBitrate()'s own
   * comment (encoder/h264_encoder.h) for exactly how it adapts. Must be
   * called at least once before ever encoding with `qp == -1`; safe to
   * call again later to retarget mid-sequence.
   */
  void setTargetBitrate(int bitsPerSecond, double fps) {
    encoder_.setTargetBitrate(bitsPerSecond, fps);
  }
  // Note: setTargetBitrate() only affects the software encoder path -
  // see setUseHardware()'s own comment for why the hardware path
  // requires a fixed setQp() instead.

  /**
   * The QP actually used by the most recent encodeFrame()-family call -
   * the only way to find out what rate control chose when `qp == -1`.
   */
  int lastQp() const { return encoder_.lastQp(); }

  /**
   * Configures automatic periodic keyframes for encodeFrame() (and its
   * color-format overloads) - a GOP size (keyframes land at picture 0,
   * `frames`, `2*frames`, ...). See Encoder::setKeyframeInterval()'s own
   * comment (encoder/h264_encoder.h) for exactly what it does and why
   * `frames` should come from your actual frame rate, not a guessed
   * constant. `frames <= 0` (the default) disables this.
   */
  void setKeyframeInterval(int frames) {
    encoder_.setKeyframeInterval(frames);
    if (keyframeInterval_ != frames) hwConfigDirty_ = true;
    keyframeInterval_ = frames;
  }

  /**
   * Pre-establishes picture width/height for encodeFrame() (and its
   * color-format overloads) - see Encoder::setSize()'s own comment
   * (encoder/h264_encoder.h) for exactly what this does. Can be called
   * before or after begin(); required before the first encodeFrame()
   * call.
   */
  void setSize(int width, int height) {
    encoder_.setSize(width, height);
    if (width != width_ || height != height_) hwConfigDirty_ = true;
    width_ = width;
    height_ = height;
  }

  /**
   * Overrides the Y/C plane row strides encodeFrame() uses - only needed
   * for a padded source buffer; see Encoder::setStride()'s own comment
   * for the tightly-packed default this replaces.
   */
  void setStride(int strideY, int strideC) {
    encoder_.setStride(strideY, strideC);
    strideY_ = strideY;
    strideC_ = strideC;
  }

  /**
   * Overrides the packed-row stride encodeFrameRgb888()/
   * encodeFrameRgb666()/encodeFrameRgb565()/encodeFrameYuv422() use -
   * see Encoder::setPackedStride()'s own comment for the units (format-
   * dependent) and tightly-packed default this replaces.
   */
  void setPackedStride(int stride) {
    encoder_.setPackedStride(stride);
    packedStride_ = stride;
  }

  /**
   * Overrides the `qp` encodeFrame() (and its color-format overloads)
   * use - see Encoder::setQp()'s own comment (default -1: rate control
   * via setTargetBitrate()). In hardware mode (see setUseHardware()),
   * `qp` is the one fixed QP used for the whole stream - hardware mode
   * has no rate-control sentinel, so a negative `qp` there makes
   * encodeFrame() fail (return 0) until a real 0-51 value is set.
   */
  void setQp(int qp) {
    encoder_.setQp(qp);
    if (qp_ != qp) hwConfigDirty_ = true;
    qp_ = qp;
  }

  /**
   * Overrides the +/-pixel window the motion search checks per
   * P-macroblock - see Encoder::setMotionSearchRange()'s own comment
   * (default 8) for the speed/compression tradeoff. Search cost is
   * O(range^2); a smaller range encodes faster but can't represent
   * motion larger than `range` pixels/frame (encoded via a bigger
   * residual instead, not a correctness issue).
   */
  void setMotionSearchRange(int range) { encoder_.setMotionSearchRange(range); }
  /// The current motion search range - see setMotionSearchRange().
  int motionSearchRange() const { return encoder_.motionSearchRange(); }

  /**
   * Selects the motion search algorithm - see
   * Encoder::setMotionSearchAlgorithm()'s own comment (encoder/
   * h264_encoder.h) for the full tradeoff. Defaults to
   * `MotionSearchAlgorithm::Exhaustive` (a full, guaranteed-best-match
   * search within the window - this project's original, still-default
   * behavior); `MotionSearchAlgorithm::Fast` (a Diamond Search) checks far
   * fewer candidates but can settle on a locally-good, not globally-best,
   * match on some content - a real compression/speed tradeoff, not a
   * free win, so it's opt-in:
   *
   *   encoder.setMotionSearchAlgorithm(MotionSearchAlgorithm::Fast);
   */
  void setMotionSearchAlgorithm(MotionSearchAlgorithm algorithm) {
    encoder_.setMotionSearchAlgorithm(algorithm);
  }
  /// The current motion search algorithm - see setMotionSearchAlgorithm().
  MotionSearchAlgorithm motionSearchAlgorithm() const {
    return encoder_.motionSearchAlgorithm();
  }

  /**
   * Single switch for every optional, opt-in performance optimization -
   * see Encoder::setAllOptimizationsActive()'s own comment (encoder/
   * h264_encoder.h) for exactly what it does and doesn't touch (currently
   * just setMotionSearchAlgorithm(); not setMotionSearchRange(), and not
   * the always-on fixes that have no tradeoff to opt into):
   *
   *   encoder.setAllOptimizationsActive(true);  // faster, real compression
   *                                              // tradeoff on some content
   */
  void setAllOptimizationsActive(bool active) {
    encoder_.setAllOptimizationsActive(active);
  }

  /**
   * Runtime alternative to `#define H264_MAX_WIDTH ...`/`#define
   * H264_MAX_HEIGHT ...` before `#include`ing this header - overrides
   * the picture-buffer/metadata-table/color-conversion-scratch
   * allocation ceiling for this encoder instance (see
   * Encoder::setMaxDimension(), encoder/h264_encoder.h, for the full
   * explanation). A setSize()/encodeFrame() call for a picture bigger
   * than this fails and returns 0. Call before the first encode to size
   * buffers for your actual content up front instead of the compile-time
   * H264_MAX_WIDTH/H264_MAX_HEIGHT default (h264_config.h) - e.g.
   * `encoder.setMaxDimension(176, 144);` in setup().
   */
  void setMaxDimension(int maxWidth, int maxHeight) {
    encoder_.setMaxDimension(maxWidth, maxHeight);
  }
  /// The current allocation-ceiling width - see setMaxDimension().
  int maxWidth() const { return encoder_.maxWidth(); }
  /// The current allocation-ceiling height - see setMaxDimension().
  int maxHeight() const { return encoder_.maxHeight(); }

  /**
   * Reserves this encoder's picture buffers up front instead of the
   * default allocate-on-first-encode behavior - see Encoder::begin()'s
   * own comment (encoder/h264_encoder.h) for exactly what this does, why
   * it's optional, and what `reserveColorConversionScratch` controls.
   * Also resets any prior stream state, so it's safe to call again to
   * start a fresh, unrelated sequence. Call once, typically from
   * setup(), if you want any allocation failure to surface
   * deterministically before encoding starts rather than mid-stream.
   * Returns false if allocation failed - out of memory, not a crash -
   * see StdAllocator.h.
   */
  bool begin(bool reserveColorConversionScratch = false) {
    return encoder_.begin(reserveColorConversionScratch);
  }

  /**
   * Releases this encoder's picture/scratch buffers and resets it to a
   * fresh, just-constructed stream state - see Encoder::end()'s own
   * comment for exactly what this does (and what it deliberately leaves
   * untouched - setTargetBitrate()/setKeyframeInterval()/setQp()/
   * setStride()/setPackedStride() configuration). Not required before
   * destruction; only useful for reclaiming memory before that, e.g.
   * between unrelated sequences, while this object is still alive. Safe
   * to call begin()/encodeFrame() again afterward (after a fresh
   * setSize() call, since end() does reset the established width/height).
   */
  void end() {
    encoder_.end();
#ifdef TINYH264_HW_ENCODER_P4_AVAILABLE
    hw_.close();
    hwScratch_.release();
    hwConfigDirty_ = true;
#endif
  }

  /**
   * The most recently encoded picture, reconstructed exactly as decoding
   * the just-produced bitstream back would give (deblocking filter
   * included) - useful for measuring this encoder's own quality (e.g.
   * PSNR against the source) without a separate decode pass, or for
   * previewing what was just encoded.
   */
  int width() const { return encoder_.frame().width; }
  int height() const { return encoder_.frame().height; }
  const uint8_t* y() const { return encoder_.frame().y(); }
  const uint8_t* u() const { return encoder_.frame().u(); }
  const uint8_t* v() const { return encoder_.frame().v(); }
  int strideY() const { return encoder_.frame().strideY; }
  int strideUV() const { return encoder_.frame().strideC; }

 private:
  Encoder<Allocator> encoder_;

  // Cached copies of setSize()/setStride()/setPackedStride()/setQp()/
  // setKeyframeInterval() - encoder_ (Encoder<Allocator>) has no getters
  // for its own equivalents, and the hardware path needs to read them
  // back to (re)open HwEncoderP4 with matching configuration.
  int width_ = 0;
  int height_ = 0;
  int strideY_ = 0;
  int strideC_ = 0;
  int packedStride_ = 0;
  int qp_ = -1;
  int keyframeInterval_ = 0;
  bool useHardware_ = false;
  // True whenever open()-affecting configuration (size, qp, keyframe
  // interval) changed since the hardware encoder was last opened -
  // checked by ensureHardwareReady() to decide whether to reopen.
  // Declared unconditionally since setSize()/setQp()/
  // setKeyframeInterval() all write to it unconditionally too -
  // harmless dead state on a build where the hardware path itself isn't
  // available.
  bool hwConfigDirty_ = true;

#ifdef TINYH264_HW_ENCODER_P4_AVAILABLE
  HwEncoderP4 hw_;
  Buffer<uint8_t, Allocator> hwScratch_;

  bool ensureHardwareReady() {
    if (width_ <= 0 || height_ <= 0 || qp_ < 0) return false;
    if (hw_.isOpen() && hw_.width() == width_ && hw_.height() == height_ &&
        !hwConfigDirty_) {
      return true;
    }
    if (!hw_.open(width_, height_, qp_, keyframeInterval_)) return false;
    hwConfigDirty_ = false;
    return true;
  }

  bool prepareHwScratch() {
    if (width_ <= 0 || height_ <= 0) return false;
    size_t ySize = (size_t)width_ * height_;
    size_t cSize = (size_t)(width_ / 2) * (height_ / 2);
    size_t needed = ySize + 2 * cSize;
    if (!hwScratch_.empty() && hwScratch_.size() != needed) {
      hwScratch_.release();
    }
    return hwScratch_.allocate(needed);
  }

  uint8_t* hwScratchY() { return hwScratch_.data(); }
  uint8_t* hwScratchU() {
    return hwScratch_.data() + (size_t)width_ * height_;
  }
  uint8_t* hwScratchV() {
    return hwScratchU() + (size_t)(width_ / 2) * (height_ / 2);
  }

  // Encodes hwScratch_ (already filled with tightly-packed YUV420 by a
  // color-format overload's convert*ToYuv420() call) via the hardware
  // encoder - shared tail end of encodeFrameRgb888()/Rgb666()/Rgb565()/
  // Yuv422()'s hardware path.
  size_t encodeHwScratch(uint8_t* dst, size_t dstCapacity) {
    if (!ensureHardwareReady()) return 0;
    return hw_.encode(hwScratchY(), width_, hwScratchU(), hwScratchV(),
                       width_ / 2, dst, dstCapacity);
  }

  // Encodes separate Y/U/V planes (encodeFrame()'s own source shape)
  // directly via the hardware encoder - no scratch copy needed, since
  // HwEncoderP4::encode() itself takes strided Y/U/V planes (it does
  // its own internal conversion to the packed pixel format the
  // hardware requires).
  size_t encodeHwPlanar(const uint8_t* srcY, int strideY, const uint8_t* srcU,
                        const uint8_t* srcV, int strideC, uint8_t* dst,
                        size_t dstCapacity) {
    if (!ensureHardwareReady()) return 0;
    return hw_.encode(srcY, strideY, srcU, srcV, strideC, dst, dstCapacity);
  }
#else
  // Unreachable stubs (useHardware_ can never be true without
  // TINYH264_HW_ENCODER_P4_AVAILABLE - see setUseHardware()) that exist
  // only so encodeFrame()/the color-format overloads' `if (useHardware_)`
  // branches still compile on a build where the hardware path itself
  // isn't available.
  bool prepareHwScratch() { return false; }
  size_t encodeHwScratch(uint8_t*, size_t) { return 0; }
  size_t encodeHwPlanar(const uint8_t*, int, const uint8_t*, const uint8_t*,
                        int, uint8_t*, size_t) {
    return 0;
  }
  uint8_t* hwScratchY() { return nullptr; }
  uint8_t* hwScratchU() { return nullptr; }
  uint8_t* hwScratchV() { return nullptr; }
#endif  // TINYH264_HW_ENCODER_P4_AVAILABLE
};

}  // namespace tinyh264
