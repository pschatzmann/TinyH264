#pragma once
#include <cstring>

#include "common/MemoryResource.h"
#include "StdAllocator.h"
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
// influence *when* a keyframe happens; see SoftwareEncoder's own file
// header comment (encoder/h264_encoder.h) for why.
//
// The Allocator template parameter (default StdAllocator<uint8_t>, see
// StdAllocator.h) is used for the picture buffers this class holds
// internally (its own closed-loop reconstruction plus the single P-frame
// reference - see encoder/h264_macroblock_encode.h's MbEncodeContext doc
// comment for why an encoder needs one at all). TinyH264Encoder itself
// stays templated on Allocator for a stable public API, but internally
// builds an AllocatorMemoryResource<Allocator> (see MemoryResource.h)
// and hands that down to the non-templated SoftwareEncoder/HwEncoderP4 it
// wraps - exactly the same PSRAM-placement mechanism TinyH264Decoder
// offers:
//   TinyH264Encoder<PSRAMAllocatorESP32<uint8_t>> encoder;
// An out-of-memory allocation failure surfaces as a 0 return from
// encodeFrame() (and its color-format overloads) or from begin(),
// instead of crashing - see StdAllocator.h's file comment.
//
// See h264_config.h for the resolution budget (H264_MAX_WIDTH/
// H264_MAX_HEIGHT, shared with TinyH264Decoder).

namespace tinyh264 {

/**
 * Public-facing encoder API: wraps a single concreteEncoder_ instance
 * behind encodeFrame() and its color-format siblings, which take raw
 * pixel data and produce a complete Annex-B bitstream, without the
 * caller needing to know anything about NAL units, slices, or
 * macroblocks. `Allocator` controls where the reconstructed-picture
 * buffers are allocated - see the file comment above for the PSRAM usage
 * example.
 *
 * Holds no hardware-specific logic of its own: on a build where ESP32-P4's
 * hardware encoder is available, concreteEncoder_'s type is HwEncoderP4
 * (encoder/h264_hw_encoder_p4.h) - itself a SoftwareEncoder subclass that
 * tries real hardware first and falls back to its own inherited software
 * implementation - so this class only ever needs to forward calls
 * straight through, regardless of which concrete type it holds.
 */
template <typename Allocator = StdAllocator<uint8_t>>
class TinyH264Encoder {
 public:
  /**
   * Constructs a TinyH264Encoder, optionally pre-configuring picture
   * width/height and periodic keyframe interval in one step instead of
   * calling setSize()/setKeyframeInterval() separately afterward - see
   * SoftwareEncoder's own constructor comment (encoder/h264_encoder.h).
   * All three default to 0 (unconfigured/no periodic keyframe), matching
   * a default-constructed TinyH264Encoder's previous behavior exactly -
   * `TinyH264Encoder<> encoder;` still compiles and behaves the same as
   * before this constructor existed.
   */
  TinyH264Encoder(int width = 0, int height = 0, int keyframeInterval = 0)
      : concreteEncoder_(memRes_, width, height, keyframeInterval) {}

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
   * disclaimer. While hardware mode is active:
   * - `setQp()` sets the one fixed QP used for the whole stream - the
   *   hardware driver doesn't support `setTargetBitrate()`-style rate
   *   control in this version (unlike the software path, there's no
   *   `-1`/auto sentinel; hardware mode requires an explicit `setQp()`
   *   call with a real 0-51 value, and fails cleanly - `encodeFrame()`
   *   returning 0 - if none was ever given).
   * - `setKeyframeInterval()` becomes the hardware's own GOP size.
   * - `setMotionSearchRange()`/`setMotionSearchAlgorithm()`/
   *   `setAllOptimizationsActive()`/`setTargetBitrate()` are software-
   *   encoder-specific - each now returns `false` while hardware mode
   *   is active (still applied to the inherited software fallback, just
   *   not to hardware itself - see each one's own comment).
   * - Unlike the software path, there's no per-call I/P decision to
   *   reason about - the hardware decides GOP placement entirely on its
   *   own once opened.
   *
   * The hardware device is opened lazily (on the next `encodeFrame()`-
   * family call, once `setSize()`/`setQp()` have established real
   * values) rather than by this call itself.
   */
  bool setUseHardware(bool enable) {
#ifdef TINYH264_HW_ENCODER_P4_AVAILABLE
    return concreteEncoder_.setUseHardware(enable);
#else
    if (enable) {
      H264LOG.warn("TinyH264Encoder::setUseHardware(true) requested but this build has no hardware encoder support");
    }
    return !enable;
#endif
  }

  /// Whether `encodeFrame()` (and its color-format overloads) currently
  /// route through ESP32-P4's hardware encoder - see setUseHardware().
  bool useHardware() const {
#ifdef TINYH264_HW_ENCODER_P4_AVAILABLE
    return concreteEncoder_.useHardware();
#else
    return false;
#endif
  }

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
    return concreteEncoder_.encodeFrame(srcY, srcU, srcV, dst, dstCapacity);
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
    return concreteEncoder_.encodeFrameRgb888(rgb, dst, dstCapacity);
  }

  /**
   * Same as encodeFrame(), for RGB666 source data (3 bytes/pixel, each
   * byte's 6 significant bits left-justified in bits 7:2 - matches
   * TinyH264Decoder::toRGB666()'s convention). Row stride defaults to
   * `width*3` (tightly packed) unless overridden via setPackedStride().
   */
  size_t encodeFrameRgb666(const uint8_t* rgb666, uint8_t* dst,
                            size_t dstCapacity) {
    return concreteEncoder_.encodeFrameRgb666(rgb666, dst, dstCapacity);
  }

  /**
   * Same as encodeFrame(), for RGB565 source data (uint16_t/pixel, 5-6-5
   * packed - matches TinyH264Decoder::toRGB565()'s convention). Row
   * stride (in uint16_t entries) defaults to `width` (tightly packed)
   * unless overridden via setPackedStride().
   */
  size_t encodeFrameRgb565(const uint16_t* rgb565, uint8_t* dst,
                            size_t dstCapacity) {
    return concreteEncoder_.encodeFrameRgb565(rgb565, dst, dstCapacity);
  }

  /**
   * Same as encodeFrame(), for YUYV-order packed YUV 4:2:2 source data
   * (Y0 U0 Y1 V0 per horizontal pixel pair - the common camera-module
   * convention, e.g. OV2640/OV7670 output). Row stride defaults to
   * `width*2` (tightly packed) unless overridden via setPackedStride().
   */
  size_t encodeFrameYuv422(const uint8_t* yuyv, uint8_t* dst,
                            size_t dstCapacity) {
    return concreteEncoder_.encodeFrameYuv422(yuyv, dst, dstCapacity);
  }

  /**
   * Configures rate control: leave `qp` at its default (or call
   * `setQp(-1)` explicitly) to let the encoder pick its own QP each
   * frame, adapted toward `bitsPerSecond` at `fps` frames/sec, instead
   * of a fixed QP. A real-time-appropriate feedback controller, not a
   * two-pass/lookahead one - see SoftwareEncoder::setTargetBitrate()'s
   * own comment (encoder/h264_encoder.h) for exactly how it adapts. Must
   * be called at least once before ever encoding with `qp == -1`; safe
   * to call again later to retarget mid-sequence. Returns `false` while
   * hardware mode is active (see setUseHardware()) - the call still
   * takes effect for the inherited software fallback, but hardware
   * itself has no rate-control mode to apply it to.
   */
  bool setTargetBitrate(int bitsPerSecond, double fps) {
    return concreteEncoder_.setTargetBitrate(bitsPerSecond, fps);
  }

  /**
   * The QP actually used by the most recent encodeFrame()-family call -
   * the only way to find out what rate control chose when `qp == -1`.
   */
  int lastQp() const { return concreteEncoder_.lastQp(); }

  /**
   * Configures automatic periodic keyframes for encodeFrame() (and its
   * color-format overloads) - a GOP size (keyframes land at picture 0,
   * `frames`, `2*frames`, ...). See SoftwareEncoder::setKeyframeInterval()'s
   * own comment (encoder/h264_encoder.h) for exactly what it does and
   * why `frames` should come from your actual frame rate, not a guessed
   * constant. `frames <= 0` (the default) disables this. In hardware
   * mode (see setUseHardware()), this becomes the hardware's own GOP
   * size instead.
   */
  void setKeyframeInterval(int frames) {
    concreteEncoder_.setKeyframeInterval(frames);
  }

  /**
   * Pre-establishes picture width/height for encodeFrame() (and its
   * color-format overloads) - see SoftwareEncoder::setSize()'s own
   * comment (encoder/h264_encoder.h) for exactly what this does. Can be
   * called before or after begin(); required before the first
   * encodeFrame() call.
   */
  void setSize(int width, int height) {
    concreteEncoder_.setSize(width, height);
  }

  /**
   * Overrides the Y/C plane row strides encodeFrame() uses - only needed
   * for a padded source buffer; see SoftwareEncoder::setStride()'s own
   * comment for the tightly-packed default this replaces.
   */
  void setStride(int strideY, int strideC) {
    concreteEncoder_.setStride(strideY, strideC);
  }

  /**
   * Overrides the packed-row stride encodeFrameRgb888()/
   * encodeFrameRgb666()/encodeFrameRgb565()/encodeFrameYuv422() use -
   * see SoftwareEncoder::setPackedStride()'s own comment for the units
   * (format-dependent) and tightly-packed default this replaces.
   */
  void setPackedStride(int stride) {
    concreteEncoder_.setPackedStride(stride);
  }

  /**
   * Overrides the `qp` encodeFrame() (and its color-format overloads)
   * use - see SoftwareEncoder::setQp()'s own comment (default -1: rate
   * control via setTargetBitrate()). In hardware mode (see
   * setUseHardware()), `qp` is the one fixed QP used for the whole
   * stream - hardware mode has no rate-control sentinel, so a negative
   * `qp` there makes encodeFrame() fail (return 0) until a real 0-51
   * value is set.
   */
  void setQp(int qp) { concreteEncoder_.setQp(qp); }

  /**
   * Overrides the +/-pixel window the motion search checks per
   * P-macroblock - see SoftwareEncoder::setMotionSearchRange()'s own
   * comment (default 8) for the speed/compression tradeoff. Search cost
   * is O(range^2); a smaller range encodes faster but can't represent
   * motion larger than `range` pixels/frame (encoded via a bigger
   * residual instead, not a correctness issue). Returns `false` while
   * hardware mode is active - motion search is software-only.
   */
  bool setMotionSearchRange(int range) {
    return concreteEncoder_.setMotionSearchRange(range);
  }
  /// The current motion search range - see setMotionSearchRange().
  int motionSearchRange() const {
    return concreteEncoder_.motionSearchRange();
  }

  /**
   * Selects the motion search algorithm - see
   * SoftwareEncoder::setMotionSearchAlgorithm()'s own comment (encoder/
   * h264_encoder.h) for the full tradeoff. Defaults to
   * `MotionSearchAlgorithm::Exhaustive` (a full, guaranteed-best-match
   * search within the window - this project's original, still-default
   * behavior); `MotionSearchAlgorithm::Fast` (a Diamond Search) checks far
   * fewer candidates but can settle on a locally-good, not globally-best,
   * match on some content - a real compression/speed tradeoff, not a
   * free win, so it's opt-in:
   *
   *   encoder.setMotionSearchAlgorithm(MotionSearchAlgorithm::Fast);
   *
   * Returns `false` while hardware mode is active - motion search is
   * software-only.
   */
  bool setMotionSearchAlgorithm(MotionSearchAlgorithm algorithm) {
    return concreteEncoder_.setMotionSearchAlgorithm(algorithm);
  }
  /// The current motion search algorithm - see setMotionSearchAlgorithm().
  MotionSearchAlgorithm motionSearchAlgorithm() const {
    return concreteEncoder_.motionSearchAlgorithm();
  }

  /**
   * Single switch for every optional, opt-in performance optimization -
   * see SoftwareEncoder::setAllOptimizationsActive()'s own comment
   * (encoder/h264_encoder.h) for exactly what it does and doesn't touch
   * (currently just setMotionSearchAlgorithm(); not
   * setMotionSearchRange(), and not the always-on fixes that have no
   * tradeoff to opt into):
   *
   *   encoder.setAllOptimizationsActive(true);  // faster, real compression
   *                                              // tradeoff on some content
   *
   * Returns `false` while hardware mode is active - see
   * setMotionSearchAlgorithm().
   */
  bool setAllOptimizationsActive(bool active) {
    return concreteEncoder_.setAllOptimizationsActive(active);
  }

  /**
   * Runtime alternative to `#define H264_MAX_WIDTH ...`/`#define
   * H264_MAX_HEIGHT ...` before `#include`ing this header - overrides
   * the picture-buffer/metadata-table/color-conversion-scratch
   * allocation ceiling for this encoder instance (see
   * SoftwareEncoder::setMaxDimension(), encoder/h264_encoder.h, for the
   * full explanation). A setSize()/encodeFrame() call for a picture
   * bigger than this fails and returns 0. Call before the first encode
   * to size buffers for your actual content up front instead of the
   * compile-time H264_MAX_WIDTH/H264_MAX_HEIGHT default (h264_config.h)
   * - e.g. `encoder.setMaxDimension(176, 144);` in setup().
   */
  void setMaxDimension(int maxWidth, int maxHeight) {
    concreteEncoder_.setMaxDimension(maxWidth, maxHeight);
  }
  /// The current allocation-ceiling width - see setMaxDimension().
  int maxWidth() const { return concreteEncoder_.maxWidth(); }
  /// The current allocation-ceiling height - see setMaxDimension().
  int maxHeight() const { return concreteEncoder_.maxHeight(); }

  /**
   * Reserves this encoder's picture buffers up front instead of the
   * default allocate-on-first-encode behavior - see
   * SoftwareEncoder::begin()'s own comment (encoder/h264_encoder.h) for
   * exactly what this does, why it's optional, and what
   * `reserveColorConversionScratch` controls. Also resets any prior
   * stream state, so it's safe to call again to start a fresh, unrelated
   * sequence. Call once, typically from setup(), if you want any
   * allocation failure to surface deterministically before encoding
   * starts rather than mid-stream. Returns false if allocation failed -
   * out of memory, not a crash - see StdAllocator.h.
   */
  bool begin(bool reserveColorConversionScratch = false) {
    return concreteEncoder_.begin(reserveColorConversionScratch);
  }

  /**
   * Releases this encoder's picture/scratch buffers and resets it to a
   * fresh, just-constructed stream state - see
   * SoftwareEncoder::end()'s own comment for exactly what this does (and
   * what it deliberately leaves untouched - setTargetBitrate()/
   * setKeyframeInterval()/setQp()/setStride()/setPackedStride()
   * configuration). In hardware mode, also closes the hardware device -
   * see HwEncoderP4::end(). Not required before destruction; only useful
   * for reclaiming memory before that, e.g. between unrelated sequences,
   * while this object is still alive. Safe to call begin()/encodeFrame()
   * again afterward (after a fresh setSize() call, since end() does
   * reset the established width/height).
   */
  void end() { concreteEncoder_.end(); }

  /**
   * The most recently encoded picture, reconstructed exactly as decoding
   * the just-produced bitstream back would give (deblocking filter
   * included) - useful for measuring this encoder's own quality (e.g.
   * PSNR against the source) without a separate decode pass, or for
   * previewing what was just encoded. Reflects the *software* pipeline's
   * own closed-loop reconstruction - in hardware mode, hardware doesn't
   * feed its own output back through this, so these stay whatever the
   * inherited SoftwareEncoder last reconstructed (typically empty,
   * unless hardware fell back to software at least once).
   */
  int width() const { return concreteEncoder_.frame().width; }
  int height() const { return concreteEncoder_.frame().height; }
  const uint8_t* y() const { return concreteEncoder_.frame().y(); }
  const uint8_t* u() const { return concreteEncoder_.frame().u(); }
  const uint8_t* v() const { return concreteEncoder_.frame().v(); }
  int strideY() const { return concreteEncoder_.frame().strideY; }
  int strideUV() const { return concreteEncoder_.frame().strideC; }

 private:
  // Declared before concreteEncoder_: members initialize in declaration
  // order, and concreteEncoder_ needs memRes_ already constructed to
  // build from.
  AllocatorMemoryResource<Allocator> memRes_;
  // Every public method above calls straight into this one instance -
  // no separate SoftwareEncoder/HwEncoderP4 pair and no pointer
  // indirection to pick between them, since only one concrete type is
  // ever constructed per build (this #ifdef) and it never changes at
  // runtime. No hardware-specific logic lives in this class at all -
  // it's entirely inside HwEncoderP4 itself, see
  // encoder/h264_hw_encoder_p4.h.
#ifdef TINYH264_HW_ENCODER_P4_AVAILABLE
  HwEncoderP4 concreteEncoder_;
#else
  SoftwareEncoder concreteEncoder_;
#endif
};

}  // namespace tinyh264
