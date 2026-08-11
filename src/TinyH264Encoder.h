#pragma once
#include <memory>
#include "encoder/h264_encoder.h"
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
// The Allocator template parameter (default std::allocator<uint8_t>) is
// used for the picture buffers this class holds internally (its own
// closed-loop reconstruction plus the single P-frame reference - see
// encoder/h264_macroblock_encode.h's MbEncodeContext doc comment for why
// an encoder needs one at all) - exactly the same PSRAM-placement
// mechanism TinyH264Decoder offers:
//   TinyH264Encoder<PSRAMAllocatorESP32<uint8_t>> encoder;
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
template <typename Allocator = std::allocator<uint8_t>>
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
      : encoder_(width, height, keyframeInterval) {}

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
   * qp are invalid or `dst`'s capacity was too small (same size-checked-
   * return convention as TinyH264Decoder's to*() converters - nothing
   * usable is left in `dst` in that case; call again with a bigger
   * buffer).
   */
  size_t encodeFrame(const uint8_t* srcY, const uint8_t* srcU,
                      const uint8_t* srcV, uint8_t* dst, size_t dstCapacity) {
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
  }

  /**
   * Pre-establishes picture width/height for encodeFrame() (and its
   * color-format overloads) - see Encoder::setSize()'s own comment
   * (encoder/h264_encoder.h) for exactly what this does. Can be called
   * before or after begin(); required before the first encodeFrame()
   * call.
   */
  void setSize(int width, int height) { encoder_.setSize(width, height); }

  /**
   * Overrides the Y/C plane row strides encodeFrame() uses - only needed
   * for a padded source buffer; see Encoder::setStride()'s own comment
   * for the tightly-packed default this replaces.
   */
  void setStride(int strideY, int strideC) {
    encoder_.setStride(strideY, strideC);
  }

  /**
   * Overrides the packed-row stride encodeFrameRgb888()/
   * encodeFrameRgb666()/encodeFrameRgb565()/encodeFrameYuv422() use -
   * see Encoder::setPackedStride()'s own comment for the units (format-
   * dependent) and tightly-packed default this replaces.
   */
  void setPackedStride(int stride) { encoder_.setPackedStride(stride); }

  /**
   * Overrides the `qp` encodeFrame() (and its color-format overloads)
   * use - see Encoder::setQp()'s own comment (default -1: rate control
   * via setTargetBitrate()).
   */
  void setQp(int qp) { encoder_.setQp(qp); }

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
   */
  void begin(bool reserveColorConversionScratch = false) {
    encoder_.begin(reserveColorConversionScratch);
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
  void end() { encoder_.end(); }

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
};

}  // namespace tinyh264
