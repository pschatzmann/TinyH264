#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../common/Logger.h"
#include "../common/MemoryResource.h"
#include "h264_bitwriter.h"
#include "h264_color_convert.h"
#include "h264_macroblock_encode.h"
#include "h264_macroblock_encode_inter.h"
#include "h264_nal_writer.h"
#include "h264_sps_pps_writer.h"
#include "../common/h264_frame.h"
#include "../common/h264_mb_info.h"
#include "../common/h264_nal_types.h"
#include "h264_config.h"
#include "../decoder/h264_deblock.h"
#include "../decoder/h264_sps_pps.h"

/*
 * Header-only. Top-level encoder driving loop - the encoder-side
 * counterpart to decoder/h264_decoder.h's Decoder class.
 *
 * encodeFrame() (and its color-format overloads encodeFrameRgb888()/
 * encodeFrameRgb666()/encodeFrameRgb565()/encodeFrameYuv422()) is the
 * *only* public encode entry point: it decides I-frame vs. P-frame
 * automatically (no reference yet, a size change via setSize(), or an
 * optional setKeyframeInterval() all force an I-frame; everything else
 * becomes a P-frame against the single most-recently-encoded picture) -
 * just call it once per picture, in order, and don't think about the I/P
 * distinction at all. Picture geometry/stride/QP policy are configured
 * once via setSize()/setStride()/setPackedStride()/setQp() instead of
 * being passed to every call (see each setter's own comment) - so every
 * encodeFrame()-family call takes only source pointer(s) plus a
 * destination buffer.
 *
 * There is deliberately no public way to force an I-frame or a P-frame
 * on a specific call anymore (an earlier version of this class exposed
 * encodeIFrame()/encodePFrame() as public "lower-level primitives" for
 * that) - setKeyframeInterval() (periodic) and setSize() (a resolution
 * change always forces an I-frame, since a P-frame can't represent one)
 * are the only remaining ways to influence *when* a keyframe happens.
 * The I-frame/P-frame encoding logic itself still exists, just as a
 * private implementation detail encodeFrame() dispatches to internally
 * (see the private section below) - kept, and kept correct, exactly as
 * before; only the public surface shrank.
 *
 * One deliberate cross-directory dependency: this file (only this file,
 * not h264_macroblock_encode.h) includes decoder/h264_deblock.h and
 * decoder/h264_sps_pps.h to run the *decoder's own, already ffmpeg-
 * verified* deblocking filter on the reconstructed picture before
 * returning - reimplementing it would be both wasteful and a real
 * correctness risk (the whole point of a closed-loop encoder is that its
 * own reconstruction matches what a real decoder produces, and this
 * project already has a verified deblocking filter). See the private
 * encodeIFrame()'s closing deblockPicture() call.
 */

namespace tinyh264 {

/**
 * Encodes one Baseline-profile CAVLC picture at a time into a real H.264
 * Annex-B stream. Backed by a MemoryResource (../MemoryResource.h)
 * passed to the constructor, exactly like decoder::SoftwareDecoder, for
 * the same reason: the two Frame objects this class holds (the
 * closed-loop reconstruction plus the single P-frame reference - see
 * h264_macroblock_encode.h's MbEncodeContext doc comment for why an
 * encoder needs one at all) can be placed in PSRAM via a custom
 * allocator on boards that have it (see PSRAMAllocatorESP32.h, wrapped
 * via AllocatorMemoryResource - the mechanism TinyH264Encoder<Allocator>
 * uses to build the MemoryResource this class actually receives). An
 * allocation failure (out of memory) is reported as a 0 return from
 * encodeFrame() (and its color-format overloads) rather than crashing -
 * see StdAllocator.h's file comment for the mechanics.
 */
class SoftwareEncoder {
 public:
  /**
   * Constructs a SoftwareEncoder backed by `memRes` (propagated to every
   * Frame/Buffer/MbInfoTable member this class holds), optionally
   * pre-configuring picture width/height (see setSize()'s own comment)
   * and periodic keyframe interval (see setKeyframeInterval()'s own
   * comment) in one step instead of calling both setters separately
   * afterward - convenient when they're already known at construction
   * time (e.g. a fixed-resolution camera feed). `width`/`height` default
   * to 0 (unconfigured); setSize() (or this constructor's `width`/
   * `height`) must establish a real size before the first encodeFrame()
   * call, or it returns 0.
   */
  SoftwareEncoder(MemoryResource& memRes, int width = 0, int height = 0,
                   int keyframeInterval = 0)
      : frame_(memRes), mbInfo_(memRes), yuvY_(memRes), yuvU_(memRes),
        yuvV_(memRes), refFrame_(memRes) {
    setSize(width, height);
    setKeyframeInterval(keyframeInterval);
  }

  /// Virtual so a subclass (see encoder/h264_hw_encoder_p4.h::HwEncoderP4)
  /// can be destroyed correctly through a base-class pointer - required
  /// once this class has any virtual method at all, even though nothing
  /// in this codebase currently deletes a SoftwareEncoder that way.
  virtual ~SoftwareEncoder() = default;

  /**
   * Encodes one picture, automatically deciding I-frame vs. P-frame -
   * the only public encode entry point (see this file's own header
   * comment): call this once per picture, in order. Becomes an I-frame
   * (SPS + PPS + IDR slice) when there's no reference yet (the first
   * call), the size set via setSize() differs from the established
   * sequence (a resolution change - a P-frame can't represent that, so
   * this is not optional), or setKeyframeInterval() was configured and
   * enough frames have passed since the last one; every other call
   * becomes a P-frame (motion-compensated against the previous picture,
   * no SPS/PPS resent). Requires setSize() to have been called at least
   * once first (returns 0 otherwise, same as an invalid size); `srcY`/
   * `srcU`/`srcV` row strides default to tightly packed (`strideY ==`
   * the width from setSize(), `strideC == width/2`) unless overridden
   * via setStride(), and `qp` defaults to -1 (rate control via
   * setTargetBitrate() - fails cleanly, returning 0, if that was never
   * called) unless overridden via setQp(). Returns the number of bytes
   * written to `dst`, or 0 on any failure (invalid size, invalid qp,
   * `dstCapacity` too small - mirrors TinyH264Decoder's to*()
   * converters' size-checked-return convention; nothing usable is left
   * in `dst` in the too-small case, though the internal reconstructed-
   * picture state may still have been updated - call again with a
   * bigger buffer rather than trying to resume).
   */
  virtual size_t encodeFrame(const uint8_t* srcY, const uint8_t* srcU,
                              const uint8_t* srcV, uint8_t* dst,
                              size_t dstCapacity) {
    if (width_ <= 0 || height_ <= 0) {
      H264LOG.error("encodeFrame: no valid size configured (call setSize() first)");
      return 0;
    }
    int strideY = defaultStrideY_ > 0 ? defaultStrideY_ : width_;
    int strideC = defaultStrideC_ > 0 ? defaultStrideC_ : width_ / 2;
    return encodeFrameExplicit(srcY, strideY, srcU, srcV, strideC, width_,
                                height_, defaultQp_, dst, dstCapacity);
  }

  /**
   * Same as encodeFrame(), for RGB888 source data (3 bytes/pixel, R/G/B
   * order - matches TinyH264Decoder::toRGB888()'s convention) instead of
   * separate YUV planes - converts internally to YUV 4:2:0 (see
   * convertRgb888ToYuv420(), h264_color_convert.h) before encoding, a
   * real, if modest, precision loss on top of the usual DCT quantization,
   * not a lossless passthrough. `rgbStride` defaults to `width*3`
   * (tightly packed) unless overridden via setPackedStride().
   */
  virtual size_t encodeFrameRgb888(const uint8_t* rgb, uint8_t* dst,
                                    size_t dstCapacity) {
    if (width_ <= 0 || height_ <= 0) {
      H264LOG.error("encodeFrameRgb888: no valid size configured (call setSize() first)");
      return 0;
    }
    int stride = defaultPackedStride_ > 0 ? defaultPackedStride_ : width_ * 3;
    return encodeFrameRgb888Explicit(rgb, stride, width_, height_, defaultQp_,
                                      dst, dstCapacity);
  }

  /**
   * Same as encodeFrame(), for RGB666 source data (3 bytes/pixel, each
   * byte's 6 significant bits left-justified in bits 7:2 - matches
   * TinyH264Decoder::toRGB666()'s convention). `rgbStride` defaults to
   * `width*3` (tightly packed) unless overridden via setPackedStride().
   */
  virtual size_t encodeFrameRgb666(const uint8_t* rgb666, uint8_t* dst,
                                    size_t dstCapacity) {
    if (width_ <= 0 || height_ <= 0) {
      H264LOG.error("encodeFrameRgb666: no valid size configured (call setSize() first)");
      return 0;
    }
    int stride = defaultPackedStride_ > 0 ? defaultPackedStride_ : width_ * 3;
    return encodeFrameRgb666Explicit(rgb666, stride, width_, height_,
                                      defaultQp_, dst, dstCapacity);
  }

  /**
   * Same as encodeFrame(), for RGB565 source data (uint16_t/pixel, 5-6-5
   * packed - matches TinyH264Decoder::toRGB565()'s convention).
   * `rgbStride` (in uint16_t entries) defaults to `width` (tightly
   * packed) unless overridden via setPackedStride().
   */
  virtual size_t encodeFrameRgb565(const uint16_t* rgb565, uint8_t* dst,
                                    size_t dstCapacity) {
    if (width_ <= 0 || height_ <= 0) {
      H264LOG.error("encodeFrameRgb565: no valid size configured (call setSize() first)");
      return 0;
    }
    int stride = defaultPackedStride_ > 0 ? defaultPackedStride_ : width_;
    return encodeFrameRgb565Explicit(rgb565, stride, width_, height_,
                                      defaultQp_, dst, dstCapacity);
  }

  /**
   * Same as encodeFrame(), for YUYV-order packed YUV 4:2:2 source data
   * (Y0 U0 Y1 V0 per horizontal pixel pair - the common camera-module
   * convention, e.g. OV2640/OV7670 output). `yuyvStride` defaults to
   * `width*2` (tightly packed) unless overridden via setPackedStride().
   */
  virtual size_t encodeFrameYuv422(const uint8_t* yuyv, uint8_t* dst,
                                    size_t dstCapacity) {
    if (width_ <= 0 || height_ <= 0) {
      H264LOG.error("encodeFrameYuv422: no valid size configured (call setSize() first)");
      return 0;
    }
    int stride = defaultPackedStride_ > 0 ? defaultPackedStride_ : width_ * 2;
    return encodeFrameYuv422Explicit(yuyv, stride, width_, height_,
                                      defaultQp_, dst, dstCapacity);
  }

  /**
   * Configures rate control: `bitsPerSecond` / `fps` gives a target
   * average size per frame, in bytes, that encodeFrame() (and its
   * color-format overloads) aim for whenever the configured `qp` is -1
   * (setQp()'s default, if setQp() was never called). A simple,
   * real-time-appropriate feedback controller (not a two-pass/lookahead
   * one - consistent with this encoder's whole "correctness and
   * simplicity over maximal efficiency" design center): after each
   * rate-controlled frame, the *next* frame's QP is nudged up (smaller/
   * coarser) if the frame just produced came out over target, or down
   * (larger/finer) if under - see updateRateControl()'s own comment for
   * the exact step sizes. Must be called at least once before ever
   * leaving `qp` at -1; can be called again later to retarget mid-
   * sequence (e.g. a bandwidth change) - takes effect starting with the
   * next rate-controlled call. Virtual and bool-returning so a subclass
   * that doesn't support rate control (see encoder/h264_hw_encoder_p4.h::
   * HwEncoderP4) can report that back to the caller instead of silently
   * ignoring the call - always true here.
   */
  virtual bool setTargetBitrate(int bitsPerSecond, double fps) {
    int bytes = (int)(bitsPerSecond / fps / 8.0);
    targetFrameBytes_ = bytes > 0 ? bytes : 1;
    return true;
  }

  /**
   * The QP actually used by the most recent encodeFrame()-family call -
   * always meaningful, but the only way to find out what rate control
   * chose when that call used `qp == -1`.
   */
  int lastQp() const { return lastQp_; }

  /**
   * Configures automatic periodic keyframes for encodeFrame() (and its
   * color-format overloads): a GOP size (the standard meaning, e.g.
   * ffmpeg's `-g`) - every `frames`-th picture becomes an I-frame even
   * though a valid reference already exists, instead of always P-frame
   * after the first picture, landing at picture 0, `frames`,
   * `2*frames`, ... `frames <= 0` (the default) disables this -
   * encodeFrame() then only re-keys on the cases that are never
   * optional (no reference yet, a resolution change). Real streaming
   * setups typically want a periodic keyframe anyway, so a decoder
   * joining mid-stream (or recovering from a lost/corrupted frame
   * upstream) has somewhere to resync - a fixed interval doesn't know
   * your actual frame rate, so pick `frames` as (seconds-between-
   * keyframes * your fps), not a guessed constant.
   */
  void setKeyframeInterval(int frames) { keyframeInterval_ = frames; }

  /**
   * Pre-establishes this encoder's picture width/height (must each be a
   * multiple of 16 - frame_cropping isn't implemented yet) so
   * encodeFrame() (and its color-format overloads) know what to encode
   * without taking width/height as a parameter every call. Writes
   * directly into the same width_/height_ this class already tracks
   * internally to detect resolution changes - calling setSize() again
   * with a different value is exactly how you tell encodeFrame() to
   * start a new resolution (forcing an I-frame, since a P-frame can't
   * represent that). begin() doesn't reset it, so setSize() can be
   * called either before or after begin(). Not validated here (an
   * invalid size set here just makes the next encodeFrame() call return
   * 0, same as encodeFrame() would report for any other failure).
   */
  void setSize(int width, int height) {
    width_ = width;
    height_ = height;
  }

  /**
   * Overrides the Y/C plane row strides encodeFrame() passes through -
   * only needed if your source buffer has row padding (e.g. a camera
   * driver's frame buffer aligned wider than the actual picture);
   * without this, `strideY` defaults to the width from setSize() and
   * `strideC` to half that (tightly packed, the common case). Call
   * setSize() first - this doesn't independently validate width/height.
   */
  void setStride(int strideY, int strideC) {
    defaultStrideY_ = strideY;
    defaultStrideC_ = strideC;
  }

  /**
   * Overrides the single packed-row stride encodeFrameRgb888()/
   * encodeFrameRgb666()/encodeFrameRgb565()/encodeFrameYuv422() pass
   * through - only needed for a padded source buffer, same rationale as
   * setStride() above (which is for the plain-YUV encodeFrame() instead).
   * Units match whichever format overload you actually call (bytes for
   * RGB888/RGB666/YUV422, uint16_t entries for RGB565 - see each one's
   * own doc comment); if you use more than one packed format with this
   * same Encoder instance, call this again before switching, since the
   * same stored value is shared across all of them. Without this, each
   * overload defaults to its own tightly-packed stride derived from the
   * width set via setSize().
   */
  void setPackedStride(int stride) { defaultPackedStride_ = stride; }

  /**
   * Overrides the `qp` encodeFrame() (and its color-format overloads)
   * use - 0-51 for a fixed QP, or -1 (the default, if this is never
   * called) to use rate control via setTargetBitrate() every call.
   */
  void setQp(int qp) { defaultQp_ = qp; }

  /**
   * Overrides the +/-pixel window motionSearch16x16() (see
   * h264_macroblock_encode_inter.h) searches for each P-macroblock's
   * motion vector - defaults to 8 (this project's original, still-
   * default search window). Search cost is O(range^2)
   * ((2*range+1)^2 candidate positions, each a 256-pixel SAD - see
   * docs/optimizations.md's "Encoding" chapter for measured
   * numbers, e.g. ~730ms/P-frame at the default range=8 on real ESP32/
   * RP2040 hardware at QCIF): a smaller range trades this away
   * proportionally faster, at the cost of not finding motion vectors
   * larger than `range` pixels/frame - real motion beyond that still
   * gets encoded correctly, just via a larger residual instead of a
   * matching MV (worse compression on fast-moving content, not a
   * correctness issue). Clamped to >= 1; values are otherwise
   * unconstrained (a larger-than-8 range is legal too, just slower and
   * rarely useful since real content-to-content motion beyond 8 pixels/
   * frame at typical frame rates is uncommon). Virtual and bool-returning
   * so a subclass that doesn't support motion search at all (see
   * encoder/h264_hw_encoder_p4.h::HwEncoderP4) can report that back to
   * the caller instead of silently ignoring the call - always true here.
   */
  virtual bool setMotionSearchRange(int range) {
    if (range < 1) range = 1;
    motionSearchRange_ = range;
    return true;
  }
  /// The current motion search range - see setMotionSearchRange().
  int motionSearchRange() const { return motionSearchRange_; }

  /**
   * Selects which algorithm motion-estimates each P-macroblock's motion
   * vector - `MotionSearchAlgorithm::Exhaustive` (the default, unchanged
   * since this project's first P-frame milestone) is motionSearch16x16()
   * (h264_macroblock_encode_inter.h): a full `(2*range+1)^2`-candidate
   * search, guaranteed to find the true best-SAD match within the window.
   * `MotionSearchAlgorithm::Fast` is motionSearch16x16Fast(): a classic
   * Diamond Search that checks far fewer candidates (~15-30 on typical
   * content vs. 289 at the default range=8) but is a *local* search - it
   * can settle on a locally-good match that isn't the true global-best
   * SAD (a real, data-dependent quality/compression tradeoff, not just a
   * speed dial - see docs/optimizations.md's "Encoding" chapter,
   * "Fast search algorithm"). Defaults to Exhaustive so existing
   * behavior/bit-exactness is unchanged unless a caller opts in. Virtual
   * and bool-returning so a subclass that doesn't support motion search
   * at all (see encoder/h264_hw_encoder_p4.h::HwEncoderP4) can report
   * that back to the caller instead of silently ignoring the call -
   * always true here.
   */
  virtual bool setMotionSearchAlgorithm(MotionSearchAlgorithm algorithm) {
    motionSearchAlgorithm_ = algorithm;
    return true;
  }
  /// The current motion search algorithm - see setMotionSearchAlgorithm().
  MotionSearchAlgorithm motionSearchAlgorithm() const {
    return motionSearchAlgorithm_;
  }

  /**
   * Single switch for every *optional, opt-in* performance optimization
   * this encoder exposes - currently just setMotionSearchAlgorithm()
   * (`true` selects `MotionSearchAlgorithm::Fast`, `false` selects
   * `MotionSearchAlgorithm::Exhaustive`, the default). Does *not* touch
   * setMotionSearchRange(): that's a continuous speed/compression dial,
   * not an on/off optimization, and combining it with `Fast` measured
   * almost no additional benefit over `Fast` alone anyway (see
   * docs/optimizations.md's "Encoding" chapter) - so this
   * intentionally leaves it at whatever the caller already configured.
   * Also does not touch anything permanently applied with no tradeoff
   * (the SAD branch-elimination fix, the duplicate motion-compensation/
   * transform elimination) - those aren't optional, so there's nothing
   * to switch. A convenience for callers who just want "make this faster"
   * without tracking each optional optimization's own setter individually
   * as more are added over time - equivalent to calling
   * setMotionSearchAlgorithm(MotionSearchAlgorithm::Fast) directly today,
   * but insulates a caller from needing to know that's currently the only
   * one. See docs/encoding.md and docs/optimizations.md for the
   * real compression/quality tradeoff `Fast` carries - this is not a free
   * win, so it stays opt-in either way it's reached. Virtual and
   * bool-returning for the same reason as setMotionSearchAlgorithm()
   * (which this forwards to) - always true here.
   */
  virtual bool setAllOptimizationsActive(bool active) {
    return setMotionSearchAlgorithm(active ? MotionSearchAlgorithm::Fast
                                            : MotionSearchAlgorithm::Exhaustive);
  }

  /**
   * Overrides the H264_MAX_WIDTH/H264_MAX_HEIGHT compile-time defaults
   * (h264_config.h) for this Encoder instance's own picture-buffer,
   * per-macroblock-metadata, and color-conversion-scratch allocation
   * ceiling - an alternative to a `#define H264_MAX_WIDTH ...`/
   * `#define H264_MAX_HEIGHT ...` before `#include`ing this header, for
   * callers that would rather configure this at runtime (e.g. once in
   * setup()) than via the preprocessor. Propagates to frame_, refFrame_,
   * the macroblock metadata table (see Frame::setMaxDimension()/
   * MbInfoTable::setMaxDimension()), and - if already allocated - the
   * yuvY_/yuvU_/yuvV_ color-conversion scratch buffers, released here so
   * ensureYuvScratchAllocated() reallocates them at the new size on next
   * use (it reads maxWidth_/maxHeight_ directly, so this is the only
   * propagation they need). Not just a smaller bound, either direction
   * works, since nothing downstream of
   * this assumes a fixed compile-time size anymore (H264_MAX_NAL_SIZE is
   * the only remaining genuinely compile-time-fixed limit here). A
   * setSize()/encodeFrame() call for a picture bigger than this ceiling
   * fails and returns 0, same as exceeding the old compile-time
   * H264_MAX_WIDTH/H264_MAX_HEIGHT always did. If picture buffers are
   * already allocated, this releases and reclaims them so the next
   * encode reallocates at the new size - safe to call at any time, not
   * just before the first encode.
   */
  void setMaxDimension(int maxWidth, int maxHeight) {
    maxWidth_ = maxWidth;
    maxHeight_ = maxHeight;
    frame_.setMaxDimension(maxWidth, maxHeight);
    refFrame_.setMaxDimension(maxWidth, maxHeight);
    mbInfo_.setMaxDimension(maxWidth, maxHeight);
    if (!yuvY_.empty()) {
      yuvY_.release();
      yuvU_.release();
      yuvV_.release();
    }
  }
  /// The current allocation-ceiling width - see setMaxDimension().
  int maxWidth() const { return maxWidth_; }
  /// The current allocation-ceiling height - see setMaxDimension().
  int maxHeight() const { return maxHeight_; }

  /**
   * The most recently encoded picture, reconstructed exactly as decoding
   * the just-produced bitstream back would give (closed loop) - useful
   * for measuring this encoder's own quality (e.g. PSNR against the
   * source) without needing a separate decode pass.
   */
  const Frame& frame() const { return frame_; }

  /**
   * Reserves this encoder's picture buffers (frame_ and refFrame_, each
   * up to their maxWidth()/maxHeight() maximum - see setMaxDimension()/
   * h264_frame.h) up front, instead of the default allocate-on-
   * first-encode behavior (Frame::ensureAllocated(), otherwise first
   * triggered by the first encodeFrame() call). Pass
   * `reserveColorConversionScratch = true` to also reserve the
   * yuvY_/yuvU_/yuvV_ scratch buffers the encodeFrameRgb888()/
   * encodeFrameRgb666()/encodeFrameRgb565()/encodeFrameYuv422()
   * overloads need (otherwise left unallocated until one of those is
   * actually called, same as before this method existed - see
   * ensureYuvScratchAllocated()). Entirely optional - every
   * encodeFrame()-family call still allocates lazily on its own if this
   * was never called - but useful on an embedded target that wants any
   * allocation failure to surface deterministically during setup()
   * rather than mid-stream. Also resets any prior stream state (as if
   * this Encoder had just been constructed - see end()) *except*
   * width_/height_ (deliberately - see setSize()'s own comment: it
   * writes into the same fields, and should work whether called before
   * or after begin()), so it's safe to call again to start a fresh,
   * unrelated sequence. Safe to call more than once
   * (Frame::ensureAllocated() is itself idempotent). Returns false if any
   * allocation failed (out of memory) - see StdAllocator.h; whichever
   * buffers succeeded are left allocated (harmless - end() releases
   * them, or a retried begin() call just attempts the rest again).
   */
  bool begin(bool reserveColorConversionScratch = false) {
    bool ok = frame_.ensureAllocated();
    ok = refFrame_.ensureAllocated() && ok;
    if (reserveColorConversionScratch) ok = ensureYuvScratchAllocated() && ok;
    haveReference_ = false;
    frameNum_ = 0;
    framesSinceKeyframe_ = 0;
    if (!ok) {
      H264LOG.error("SoftwareEncoder::begin: one or more picture buffer allocations failed");
    }
    return ok;
  }

  /**
   * Releases every picture/scratch buffer this encoder holds (frame_,
   * refFrame_, and the color-conversion scratch buffers if they were
   * ever allocated, via Frame::release()/vector::shrink_to_fit()) and
   * resets this encoder's stream state (reference-picture bookkeeping,
   * established width/height, frame_num) back to how a freshly
   * constructed Encoder starts - the counterpart to begin(), for callers
   * that want to reclaim this encoder's resident memory (~76KB+ for
   * frame_+refFrame_ alone at QCIF - see docs/memory-budget.md) before
   * starting an unrelated sequence, or before
   * doing something else memory-hungry, rather than keeping it allocated
   * for the rest of the program's lifetime. Deliberately leaves
   * configuration (setTargetBitrate()'s target, setKeyframeInterval()'s
   * interval, setQp()/setStride()/setPackedStride()'s values) untouched
   * - those are settings, not per-stream state, and a caller that
   * configured them before end() shouldn't have that silently undone.
   * Not required before destruction - the members' own destructors free
   * everything regardless - only useful for freeing memory *before*
   * that, while this object is still alive. Safe to call
   * begin()/encodeFrame() again afterward to start over (after a fresh
   * setSize() call, since end() does reset width_/height_).
   */
  virtual void end() {
    frame_.release();
    refFrame_.release();
    yuvY_.release();
    yuvU_.release();
    yuvV_.release();
    haveReference_ = false;
    frameNum_ = 0;
    width_ = height_ = 0;
    ppsBaseQp_ = 0;
    framesSinceKeyframe_ = 0;
  }

 private:
  /*
   * ---------------------------------------------------------------------
   * Explicit-parameter I-frame/P-frame implementation. This used to be
   * this class's public API (encodeIFrame()/encodePFrame() and their
   * color-format overloads, plus an explicit-parameter encodeFrame()
   * dispatcher) - moved here, unchanged in behavior, once the only
   * public entry points became the defaults-driven encodeFrame()-family
   * methods above. Still exactly what makes an I-frame an I-frame and a
   * P-frame a P-frame; only the public surface shrank.
   * ---------------------------------------------------------------------
   */

  /**
   * Encodes one I-frame from raw YUV 4:2:0 planar source data (three
   * separate planes/strides) into `dst`, a complete Annex-B byte stream
   * (SPS + PPS + IDR slice NALs, each with a 4-byte start code).
   * `width`/`height` must each be a multiple of 16 (frame_cropping_flag
   * isn't implemented yet - see writeSpsRbsp()); `qp` (0-51, lower =
   * higher quality/larger output) is applied uniformly across the whole
   * picture (no per-macroblock QP adaptation within one frame). Pass
   * `qp = -1` to use rate control instead (see setTargetBitrate()) - the
   * encoder picks its own QP, adapted frame-to-frame toward the
   * configured target size; call setTargetBitrate() at least once
   * before ever passing -1, or this returns 0. Returns the number of
   * bytes written to `dst`, or 0 if `width`/`height` aren't valid, `qp`
   * is invalid (neither -1 nor 0-51, or -1 without a configured target),
   * the picture buffer allocation failed (out of memory - see
   * StdAllocator.h), or `dstCapacity` was too small (mirrors
   * TinyH264Decoder's to*() converters' size-checked-return convention;
   * nothing is written to `dst` in the too-small case, though the
   * internal reconstructed-picture state may still have been updated -
   * call again with a
   * bigger buffer rather than trying to resume).
   */
  size_t encodeIFrame(const uint8_t* srcY, int srcStrideY, const uint8_t* srcU,
                       const uint8_t* srcV, int srcStrideC, int width,
                       int height, int qp, uint8_t* dst, size_t dstCapacity) {
    if (width <= 0 || height <= 0 || (width % 16) != 0 || (height % 16) != 0) {
      H264LOG.error("encodeIFrame: invalid size %dx%d (must be >0 and a multiple of 16)",
                     width, height);
      return 0;
    }
    if (width > maxWidth_ || height > maxHeight_) {
      H264LOG.error("encodeIFrame: %dx%d exceeds the allocation ceiling %dx%d - see setMaxDimension()",
                     width, height, maxWidth_, maxHeight_);
      return 0;
    }
    if (!resolveQp(&qp)) return 0;
    if (!frame_.setSize(width, height)) {
      H264LOG.error("encodeIFrame: frame_.setSize(%d,%d) failed", width, height);
      return 0;
    }
    int mbWidth = width / 16, mbHeight = height / 16;
    if (!mbInfo_.reset(mbWidth, mbHeight)) {
      H264LOG.error("encodeIFrame: mbInfo_.reset(%d,%d) failed", mbWidth, mbHeight);
      return 0;
    }
    width_ = width;
    height_ = height;

    size_t o = 0;

    uint8_t hdrRbsp[64];
    BitWriter spsW(hdrRbsp, sizeof(hdrRbsp));
    writeSpsRbsp(spsW, width, height, /*levelIdc=*/30, /*maxNumRefFrames=*/1);
    if (spsW.error()) {
      H264LOG.error("encodeIFrame: SPS RBSP overflowed its scratch buffer");
      return 0;
    }
    size_t n = writeNalUnit(dst + o, dstCapacity - o, /*nalRefIdc=*/3,
                             kNalSps, hdrRbsp, spsW.bytesWritten());
    if (n == 0) {
      H264LOG.error("encodeIFrame: dst too small for the SPS NAL (dstCapacity=%zu)", dstCapacity);
      return 0;
    }
    o += n;

    BitWriter ppsW(hdrRbsp, sizeof(hdrRbsp));
    writePpsRbsp(ppsW, qp);
    ppsBaseQp_ = qp;
    if (ppsW.error()) {
      H264LOG.error("encodeIFrame: PPS RBSP overflowed its scratch buffer");
      return 0;
    }
    n = writeNalUnit(dst + o, dstCapacity - o, /*nalRefIdc=*/3, kNalPps,
                      hdrRbsp, ppsW.bytesWritten());
    if (n == 0) {
      H264LOG.error("encodeIFrame: dst too small for the PPS NAL (dstCapacity=%zu, used=%zu)",
                     dstCapacity, o);
      return 0;
    }
    o += n;

    BitWriter sliceW(sliceScratch_, sizeof(sliceScratch_));
    writeSliceHeaderIdr(sliceW);

    MbEncodeContext ctx;
    ctx.frame = &frame_;
    ctx.mbInfo = &mbInfo_;
    ctx.chromaQpIndexOffset = 0;  // matches writePpsRbsp()'s fixed choice
    ctx.sliceId = 0;
    ctx.srcY = srcY;
    ctx.srcStrideY = srcStrideY;
    ctx.srcU = srcU;
    ctx.srcV = srcV;
    ctx.srcStrideC = srcStrideC;

    int qpRunning = qp;
    for (int mbY = 0; mbY < mbHeight; mbY++) {
      for (int mbX = 0; mbX < mbWidth; mbX++) {
        mbInfo_.beginMb(mbX, mbY, ctx.sliceId);
        ctx.mbX = mbX;
        ctx.mbY = mbY;

        /*
         * I_16x16-vs-I_4x4 mode decision (see shouldUseIntra4x4()'s own
         * comment for the heuristic): chooseIntra16x16Mode() is called
         * here purely to get a real SAD figure to compare against - it
         * also writes its winning prediction into frame_, which is
         * harmless whichever way the decision goes (encodeMacroblockIntra16x16()
         * redoes the same, deterministic call internally if I_16x16 wins,
         * cheaply; encodeMacroblockIntra4x4()'s own per-block prediction
         * calls overwrite this trial prediction naturally as they run if
         * I_4x4 wins - no explicit rollback needed either way).
         */
        bool leftAvail = mbInfo_.leftAvailable(mbX, mbY, ctx.sliceId);
        bool topAvail = mbInfo_.topAvailable(mbX, mbY, ctx.sliceId);
        bool topLeftAvail = mbInfo_.topLeftAvailable(mbX, mbY, ctx.sliceId);
        chooseIntra16x16Mode(ctx, leftAvail, topAvail, topLeftAvail);
        int i16x16Sad = sadLuma16x16(ctx);

        if (shouldUseIntra4x4(ctx, i16x16Sad)) {
          qpRunning = encodeMacroblockIntra4x4(sliceW, ctx, qpRunning, qp);
        } else {
          qpRunning = encodeMacroblockIntra16x16(sliceW, ctx, qpRunning, qp);
        }
        if (sliceW.error()) {
          H264LOG.error("encodeIFrame: slice scratch buffer overflowed at mb(%d,%d)",
                         mbX, mbY);
          return 0;
        }
      }
    }
    sliceW.rbspTrailingBits();
    if (sliceW.error()) {
      H264LOG.error("encodeIFrame: slice scratch buffer overflowed writing trailing bits");
      return 0;
    }

    n = writeNalUnit(dst + o, dstCapacity - o, /*nalRefIdc=*/3, kNalSliceIdr,
                      sliceScratch_, sliceW.bytesWritten());
    if (n == 0) {
      H264LOG.error("encodeIFrame: dst too small for the IDR slice NAL (dstCapacity=%zu, used=%zu)",
                     dstCapacity, o);
      return 0;
    }
    o += n;

    /*
     * Matches this class's own fixed, minimal PPS (writePpsRbsp()): CAVLC
     * doesn't touch any Pps field deblockPicture() doesn't also need
     * defaulted correctly, and deblocking_filter_control_present_flag ==
     * 0 means every macroblock's disableDeblockIdc/alphaC0OffsetDiv2/
     * betaOffsetDiv2 (MacroblockInfo, left at their zero defaults by
     * encodeMacroblockIntra16x16()) already represent "deblocking
     * enabled, no offset" correctly without this class needing to touch
     * them itself.
     */
    Pps pps;
    pps.chromaQpIndexOffset = 0;
    deblockPicture(frame_, mbInfo_, pps);

    // Out of memory (the very first refFrame_.copyFrom() call, before any
    // prior successful call has left it allocated - see h264_decoder.h's
    // analogous copyFrom() comment): the encoded bitstream in `dst` is
    // already valid and fully written, but this picture can't be used as
    // a reference for the next encodePFrame() call - report the same
    // failure a bad width/height/qp would (return 0), rather than
    // returning a byte count with no usable reference behind it.
    if (!refFrame_.copyFrom(frame_)) {
      H264LOG.error("encodeIFrame: refFrame_.copyFrom() failed - no usable reference for the next encodePFrame()");
      return 0;
    }
    haveReference_ = true;
    frameNum_ = 1;  // next call's frame_num - this IDR itself used 0

    updateRateControl(o);
    return o;
  }

  /**
   * Encodes one P-frame from raw YUV 4:2:0 planar source data against
   * the single most-recently-encoded picture as its sole motion-
   * compensation reference - see h264_macroblock_encode_inter.h's file
   * header for the full scope (P_16x16/P_Skip only, single reference,
   * integer-pel-only motion search). Writes *only* a P-slice NAL - no
   * SPS/PPS. `width`/`height` are implied from the last encodeIFrame()
   * call (must match - this function has no way to change picture
   * dimensions mid-sequence without a new SPS, which needs a fresh
   * encodeIFrame() call instead). Returns 0 if no prior encodeIFrame()
   * has established a reference yet, or (same as encodeIFrame()) if
   * `dstCapacity` was too small or the reference-picture allocation
   * failed (out of memory - see StdAllocator.h). Same `qp = -1`
   * rate-control sentinel as encodeIFrame().
   */
  size_t encodePFrame(const uint8_t* srcY, int srcStrideY, const uint8_t* srcU,
                       const uint8_t* srcV, int srcStrideC, int qp,
                       uint8_t* dst, size_t dstCapacity) {
    if (!haveReference_) {
      H264LOG.error("encodePFrame: no reference picture yet (call encodeFrame() to produce an I-frame first)");
      return 0;
    }
    if (!resolveQp(&qp)) return 0;
    int width = width_, height = height_;
    // frame_ is already allocated whenever haveReference_ is true (the
    // prior encodeIFrame() call already succeeded at this exact
    // setSize()), so this can't actually fail here - checked anyway for
    // consistency with setSize()'s new bool contract.
    if (!frame_.setSize(width, height)) {
      H264LOG.error("encodePFrame: frame_.setSize(%d,%d) failed", width, height);
      return 0;
    }
    int mbWidth = width / 16, mbHeight = height / 16;
    // mbInfo_ is likewise already allocated whenever haveReference_ is
    // true (the prior encodeIFrame() call's own reset() already
    // succeeded) - checked anyway for the same consistency reason.
    if (!mbInfo_.reset(mbWidth, mbHeight)) {
      H264LOG.error("encodePFrame: mbInfo_.reset(%d,%d) failed", mbWidth, mbHeight);
      return 0;
    }

    BitWriter sliceW(sliceScratch_, sizeof(sliceScratch_));
    writeSliceHeaderP(sliceW, frameNum_, qp, ppsBaseQp_);

    MbEncodeContext ctx;
    ctx.frame = &frame_;
    ctx.mbInfo = &mbInfo_;
    ctx.chromaQpIndexOffset = 0;
    ctx.sliceId = 0;
    ctx.srcY = srcY;
    ctx.srcStrideY = srcStrideY;
    ctx.srcU = srcU;
    ctx.srcV = srcV;
    ctx.srcStrideC = srcStrideC;

    int qpRunning = qp;
    int totalMbs = mbWidth * mbHeight;
    int pendingSkipRun = 0;
    for (int mbAddr = 0; mbAddr < totalMbs; mbAddr++) {
      int mbX = mbAddr % mbWidth, mbY = mbAddr / mbWidth;
      mbInfo_.beginMb(mbX, mbY, ctx.sliceId);
      ctx.mbX = mbX;
      ctx.mbY = mbY;

      int16_t bestMv[2];
      int interSad = motionSearchAlgorithm_ == MotionSearchAlgorithm::Fast
                          ? motionSearch16x16Fast(ctx, refFrame_, bestMv,
                                                   motionSearchRange_)
                          : motionSearch16x16(ctx, refFrame_, bestMv,
                                               motionSearchRange_);
      int16_t skipMvVal[2];
      skipMv(ctx, skipMvVal);

      /*
       * P_Skip is only valid (per clause 8.4.1.1's implicit "zero
       * residual" meaning) when the motion-estimated best match *is*
       * the inferred skip MV *and* that MV's residual actually quantizes
       * to zero. Only worth computing that residual at all when the MVs
       * already match (mvEqSkip) - see computeInterResidual()'s own
       * comment for why this is the *same* computation
       * wouldHaveZeroResidual() used to redo a second time whenever the
       * answer was "no": that residual is exactly what finishInterMacroblock()
       * below needs if this macroblock turns out to be a real P_L0_16x16
       * (not skip, not Intra) instead, so it's kept rather than discarded.
       * Checked first and short-circuits the Intra-vs-Inter decision below
       * entirely (same as before this trial was made reusable) - a
       * genuine zero-bit skip is never worse than either alternative, so
       * there's nothing to compare it against, and skip macroblocks never
       * pay for the intra trial below either.
       */
      bool mvEqSkip = bestMv[0] == skipMvVal[0] && bestMv[1] == skipMvVal[1];
      MacroblockInfo trialMb;
      int32_t trialLumaBlocks[16][16];
      int32_t trialChromaBlocks[2][4][16];
      int32_t trialChromaDcGrid[2][4];
      bool isSkip = false;
      if (mvEqSkip) {
        isSkip = computeInterResidual(ctx, refFrame_, skipMvVal[0],
                                       skipMvVal[1], qpRunning, trialMb,
                                       trialLumaBlocks, trialChromaBlocks,
                                       trialChromaDcGrid);
      }
      if (isSkip) {
        /*
         * Zero residual: ctx.frame already holds the correct final
         * reconstruction (computeInterResidual()'s motion compensation,
         * unmodified since adding an all-zero residual is a no-op) and
         * trialMb already has the right mv/refIdx (fillPartitionMv() for
         * skipMvVal) and cbpLuma==cbpChroma==0 - just relabel it P_Skip
         * instead of redoing skipMv()+motionCompensate16x16() a second
         * time via encodeMacroblockPSkip().
         */
        MacroblockInfo& mb = mbInfo_.at(mbX, mbY);
        mb = trialMb;
        mb.type = kMbPSkip;
        mb.qpY = (int8_t)qpRunning;
        pendingSkipRun++;
        continue;
      }

      /*
       * Not skip: decide between an Intra-macroblock fallback (clause
       * 7.3.5's mb_type >= 5) and P_L0_16x16 - see
       * shouldUseIntraInPSlice()'s own comment (h264_macroblock_encode_inter.h)
       * for the heuristic. chooseIntra16x16Mode() is a real trial (same
       * "harmless either way" reasoning as encodeIFrame()'s own I_16x16-
       * vs-I_4x4 decision above) - but note it only overwrites ctx.frame's
       * *luma* plane (see its own comment), which matters below: if this
       * macroblock reuses the computeInterResidual() trial above, only
       * luma (not chroma) needs restoring afterward.
       */
      bool leftAvail = mbInfo_.leftAvailable(mbX, mbY, ctx.sliceId);
      bool topAvail = mbInfo_.topAvailable(mbX, mbY, ctx.sliceId);
      bool topLeftAvail = mbInfo_.topLeftAvailable(mbX, mbY, ctx.sliceId);
      chooseIntra16x16Mode(ctx, leftAvail, topAvail, topLeftAvail);
      int intraSad = sadLuma16x16(ctx);

      sliceW.ue((uint32_t)pendingSkipRun);
      pendingSkipRun = 0;

      if (shouldUseIntraInPSlice(intraSad, interSad)) {
        if (shouldUseIntra4x4(ctx, intraSad)) {
          qpRunning = encodeMacroblockIntra4x4(sliceW, ctx, qpRunning, qp,
                                                /*mbTypeOffset=*/5);
        } else {
          qpRunning = encodeMacroblockIntra16x16(sliceW, ctx, qpRunning, qp,
                                                  /*mbTypeOffset=*/5);
        }
      } else if (mvEqSkip) {
        /*
         * computeInterResidual() above already computed bestMv's (==
         * skipMvVal's) residual and found it non-zero - reuse it instead
         * of paying for the full transform/quantize pass a second time
         * (see finishInterMacroblock()'s own comment). chooseIntra16x16Mode()
         * just clobbered ctx.frame's luma plane with its own trial
         * prediction though (chroma untouched - see the comment above),
         * so luma needs re-establishing before finishInterMacroblock()'s
         * reconstruction step runs against it.
         */
        int px = ctx.mbX * 16, py = ctx.mbY * 16;
        motionCompLuma(ctx.frame->yRow(py) + px, ctx.frame->strideY,
                        refFrame_, px, py, 16, 16, bestMv[0], bestMv[1]);
        qpRunning = finishInterMacroblock(
            sliceW, ctx, trialMb, trialLumaBlocks, trialChromaBlocks,
            trialChromaDcGrid, bestMv[0], bestMv[1], qpRunning, qp);
      } else {
        qpRunning = encodeMacroblockInter16x16(sliceW, ctx, refFrame_,
                                                bestMv[0], bestMv[1],
                                                qpRunning, qp);
      }
      if (sliceW.error()) {
        H264LOG.error("encodePFrame: slice scratch buffer overflowed at mb(%d,%d)",
                       ctx.mbX, ctx.mbY);
        return 0;
      }
    }
    /*
     * Trailing skip run: matches decode's slice_data() loop, which exits
     * once mbAddr reaches totalMbs *inside* the skip-processing block,
     * before ever reading another mb_skip_run/macroblock_layer() - so a
     * picture ending in skips needs exactly one final mb_skip_run and
     * nothing after it.
     */
    if (pendingSkipRun > 0) sliceW.ue((uint32_t)pendingSkipRun);

    sliceW.rbspTrailingBits();
    if (sliceW.error()) {
      H264LOG.error("encodePFrame: slice scratch buffer overflowed writing trailing bits");
      return 0;
    }

    size_t o = writeNalUnit(dst, dstCapacity, /*nalRefIdc=*/3,
                             kNalSliceNonIdr, sliceScratch_,
                             sliceW.bytesWritten());
    if (o == 0) {
      H264LOG.error("encodePFrame: dst too small for the P-slice NAL (dstCapacity=%zu)", dstCapacity);
      return 0;
    }

    Pps pps;
    pps.chromaQpIndexOffset = 0;
    deblockPicture(frame_, mbInfo_, pps);

    // refFrame_ is already allocated whenever haveReference_ is true (see
    // encodeIFrame()'s own copyFrom() call, which already succeeded to
    // get here) - checked anyway for consistency with copyFrom()'s new
    // bool contract; same "bitstream already written, but don't trust
    // this call's output" rationale as encodeIFrame()'s copyFrom() check.
    if (!refFrame_.copyFrom(frame_)) {
      H264LOG.error("encodePFrame: refFrame_.copyFrom() failed - no usable reference for the next call");
      return 0;
    }
    frameNum_ = (frameNum_ + 1) & 0xFF;  // 8 bits (log2_max_frame_num_minus4==4)

    updateRateControl(o);
    return o;
  }

  /**
   * Same as encodeIFrame(), for RGB888 source data - converts into this
   * class's own YUV420 scratch buffers (see convertRgb888ToYuv420(),
   * h264_color_convert.h) before delegating to encodeIFrame().
   */
  size_t encodeIFrameRgb888(const uint8_t* rgb, int rgbStride, int width,
                             int height, int qp, uint8_t* dst,
                             size_t dstCapacity) {
    if (!prepareYuvScratch(width, height)) return 0;
    convertRgb888ToYuv420(rgb, rgbStride, width, height, yuvY_.data(), width, yuvU_.data(),
                           yuvV_.data(), width / 2);
    return encodeIFrame(yuvY_.data(), width, yuvU_.data(), yuvV_.data(), width / 2, width, height,
                         qp, dst, dstCapacity);
  }

  /// Same as encodeIFrame(), for RGB666 source data.
  size_t encodeIFrameRgb666(const uint8_t* rgb666, int rgbStride, int width,
                             int height, int qp, uint8_t* dst,
                             size_t dstCapacity) {
    if (!prepareYuvScratch(width, height)) return 0;
    convertRgb666ToYuv420(rgb666, rgbStride, width, height, yuvY_.data(), width,
                           yuvU_.data(), yuvV_.data(), width / 2);
    return encodeIFrame(yuvY_.data(), width, yuvU_.data(), yuvV_.data(), width / 2, width, height,
                         qp, dst, dstCapacity);
  }

  /// Same as encodeIFrame(), for RGB565 source data.
  size_t encodeIFrameRgb565(const uint16_t* rgb565, int rgbStride, int width,
                             int height, int qp, uint8_t* dst,
                             size_t dstCapacity) {
    if (!prepareYuvScratch(width, height)) return 0;
    convertRgb565ToYuv420(rgb565, rgbStride, width, height, yuvY_.data(), width,
                           yuvU_.data(), yuvV_.data(), width / 2);
    return encodeIFrame(yuvY_.data(), width, yuvU_.data(), yuvV_.data(), width / 2, width, height,
                         qp, dst, dstCapacity);
  }

  /// Same as encodeIFrame(), for YUYV-order packed YUV 4:2:2 source data.
  size_t encodeIFrameYuv422(const uint8_t* yuyv, int yuyvStride, int width,
                             int height, int qp, uint8_t* dst,
                             size_t dstCapacity) {
    if (!prepareYuvScratch(width, height)) return 0;
    convertYuyv422ToYuv420(yuyv, yuyvStride, width, height, yuvY_.data(), width,
                            yuvU_.data(), yuvV_.data(), width / 2);
    return encodeIFrame(yuvY_.data(), width, yuvU_.data(), yuvV_.data(), width / 2, width, height,
                         qp, dst, dstCapacity);
  }

  /**
   * Same as encodePFrame(), for RGB888 source data (see
   * encodeIFrameRgb888()'s own comment for the format/conversion
   * details - identical here, just against the established sequence's
   * dimensions instead of a width/height parameter).
   */
  size_t encodePFrameRgb888(const uint8_t* rgb, int rgbStride, int qp,
                             uint8_t* dst, size_t dstCapacity) {
    if (!haveReference_) {
      H264LOG.error("encodePFrameRgb888: no reference picture yet (call encodeFrame() to produce an I-frame first)");
      return 0;
    }
    if (!prepareYuvScratch(width_, height_)) return 0;
    convertRgb888ToYuv420(rgb, rgbStride, width_, height_, yuvY_.data(),
                           width_, yuvU_.data(), yuvV_.data(), width_ / 2);
    return encodePFrame(yuvY_.data(), width_, yuvU_.data(), yuvV_.data(),
                         width_ / 2, qp, dst, dstCapacity);
  }

  /// Same as encodePFrame(), for RGB666 source data.
  size_t encodePFrameRgb666(const uint8_t* rgb666, int rgbStride, int qp,
                             uint8_t* dst, size_t dstCapacity) {
    if (!haveReference_) {
      H264LOG.error("encodePFrameRgb666: no reference picture yet (call encodeFrame() to produce an I-frame first)");
      return 0;
    }
    if (!prepareYuvScratch(width_, height_)) return 0;
    convertRgb666ToYuv420(rgb666, rgbStride, width_, height_, yuvY_.data(),
                           width_, yuvU_.data(), yuvV_.data(), width_ / 2);
    return encodePFrame(yuvY_.data(), width_, yuvU_.data(), yuvV_.data(),
                         width_ / 2, qp, dst, dstCapacity);
  }

  /// Same as encodePFrame(), for RGB565 source data.
  size_t encodePFrameRgb565(const uint16_t* rgb565, int rgbStride, int qp,
                             uint8_t* dst, size_t dstCapacity) {
    if (!haveReference_) {
      H264LOG.error("encodePFrameRgb565: no reference picture yet (call encodeFrame() to produce an I-frame first)");
      return 0;
    }
    if (!prepareYuvScratch(width_, height_)) return 0;
    convertRgb565ToYuv420(rgb565, rgbStride, width_, height_, yuvY_.data(),
                           width_, yuvU_.data(), yuvV_.data(), width_ / 2);
    return encodePFrame(yuvY_.data(), width_, yuvU_.data(), yuvV_.data(),
                         width_ / 2, qp, dst, dstCapacity);
  }

  /// Same as encodePFrame(), for YUYV-order packed YUV 4:2:2 source data.
  size_t encodePFrameYuv422(const uint8_t* yuyv, int yuyvStride, int qp,
                             uint8_t* dst, size_t dstCapacity) {
    if (!haveReference_) {
      H264LOG.error("encodePFrameYuv422: no reference picture yet (call encodeFrame() to produce an I-frame first)");
      return 0;
    }
    if (!prepareYuvScratch(width_, height_)) return 0;
    convertYuyv422ToYuv420(yuyv, yuyvStride, width_, height_, yuvY_.data(),
                            width_, yuvU_.data(), yuvV_.data(), width_ / 2);
    return encodePFrame(yuvY_.data(), width_, yuvU_.data(), yuvV_.data(),
                         width_ / 2, qp, dst, dstCapacity);
  }

  /**
   * The auto I/P dispatcher every public encodeFrame()-family method
   * resolves its stored defaults and delegates to: I-frame when
   * needsKeyframe() says so, P-frame otherwise. Not a new encoding path
   * of its own - purely a dispatch layer over the private
   * encodeIFrame()/encodePFrame() above.
   */
  size_t encodeFrameExplicit(const uint8_t* srcY, int srcStrideY,
                              const uint8_t* srcU, const uint8_t* srcV,
                              int srcStrideC, int width, int height, int qp,
                              uint8_t* dst, size_t dstCapacity) {
    if (needsKeyframe(width, height)) {
      size_t n = encodeIFrame(srcY, srcStrideY, srcU, srcV, srcStrideC, width,
                               height, qp, dst, dstCapacity);
      if (n > 0) framesSinceKeyframe_ = 0;
      return n;
    }
    size_t n = encodePFrame(srcY, srcStrideY, srcU, srcV, srcStrideC, qp, dst,
                             dstCapacity);
    if (n > 0) framesSinceKeyframe_++;
    return n;
  }

  /// Same dispatch as encodeFrameExplicit(), for RGB888 source data.
  size_t encodeFrameRgb888Explicit(const uint8_t* rgb, int rgbStride,
                                    int width, int height, int qp,
                                    uint8_t* dst, size_t dstCapacity) {
    if (needsKeyframe(width, height)) {
      size_t n = encodeIFrameRgb888(rgb, rgbStride, width, height, qp, dst,
                                     dstCapacity);
      if (n > 0) framesSinceKeyframe_ = 0;
      return n;
    }
    size_t n = encodePFrameRgb888(rgb, rgbStride, qp, dst, dstCapacity);
    if (n > 0) framesSinceKeyframe_++;
    return n;
  }

  /// Same dispatch as encodeFrameExplicit(), for RGB666 source data.
  size_t encodeFrameRgb666Explicit(const uint8_t* rgb666, int rgbStride,
                                    int width, int height, int qp,
                                    uint8_t* dst, size_t dstCapacity) {
    if (needsKeyframe(width, height)) {
      size_t n = encodeIFrameRgb666(rgb666, rgbStride, width, height, qp, dst,
                                     dstCapacity);
      if (n > 0) framesSinceKeyframe_ = 0;
      return n;
    }
    size_t n = encodePFrameRgb666(rgb666, rgbStride, qp, dst, dstCapacity);
    if (n > 0) framesSinceKeyframe_++;
    return n;
  }

  /// Same dispatch as encodeFrameExplicit(), for RGB565 source data.
  size_t encodeFrameRgb565Explicit(const uint16_t* rgb565, int rgbStride,
                                    int width, int height, int qp,
                                    uint8_t* dst, size_t dstCapacity) {
    if (needsKeyframe(width, height)) {
      size_t n = encodeIFrameRgb565(rgb565, rgbStride, width, height, qp, dst,
                                     dstCapacity);
      if (n > 0) framesSinceKeyframe_ = 0;
      return n;
    }
    size_t n = encodePFrameRgb565(rgb565, rgbStride, qp, dst, dstCapacity);
    if (n > 0) framesSinceKeyframe_++;
    return n;
  }

  /**
   * Same dispatch as encodeFrameExplicit(), for YUYV-order packed YUV
   * 4:2:2 source data.
   */
  size_t encodeFrameYuv422Explicit(const uint8_t* yuyv, int yuyvStride,
                                    int width, int height, int qp,
                                    uint8_t* dst, size_t dstCapacity) {
    if (needsKeyframe(width, height)) {
      size_t n = encodeIFrameYuv422(yuyv, yuyvStride, width, height, qp, dst,
                                     dstCapacity);
      if (n > 0) framesSinceKeyframe_ = 0;
      return n;
    }
    size_t n = encodePFrameYuv422(yuyv, yuyvStride, qp, dst, dstCapacity);
    if (n > 0) framesSinceKeyframe_++;
    return n;
  }

  /**
   * Whether encodeFrameExplicit() (or one of its color-format siblings)
   * should produce an I-frame for a `width`x`height` picture right now -
   * see encodeFrame()'s own doc comment for the three cases (no
   * reference yet, a resolution change, or a configured keyframe
   * interval elapsed). `keyframeInterval_` is a GOP size (the standard
   * meaning, e.g. ffmpeg's `-g`): keyframes land exactly
   * `keyframeInterval_` frames apart (0, N, 2N, ... for
   * `keyframeInterval_ == N`), not N+1 - `framesSinceKeyframe_` counts
   * P-frames encoded since the last keyframe, so the Nth P-frame (index
   * N-1, since counting starts at 0 for the first P-frame after a
   * keyframe) is the last one before the next keyframe is due; comparing
   * against `keyframeInterval_ - 1` (not `keyframeInterval_`) is what
   * gets the spacing exactly right - verified against a real
   * encodeFrame()-only sequence in test/native/test_encode_autoframe.cpp
   * (keyframeInterval 3 landing keyframes at picture 0, 3, 6, 9, not
   * 0, 4, 8).
   */
  bool needsKeyframe(int width, int height) const {
    if (!haveReference_ || width != width_ || height != height_) return true;
    return keyframeInterval_ > 0 &&
           framesSinceKeyframe_ >= keyframeInterval_ - 1;
  }

 protected:
  /**
   * Bounds-checks width/height (the same maxWidth_/maxHeight_ budget
   * encodeIFrame() itself checks - see setMaxDimension()) and lazily
   * allocates yuvY_/yuvU_/yuvV_ at their maximum size on first use - shared by
   * every encodeIFrame*() color-conversion overload above. Unlike
   * sliceScratch_ (always needed, so always a plain fixed array),
   * yuvY_/yuvU_/yuvV_ back a set of *optional* convenience overloads -
   * callers who only ever use the plain YUV-planes encodeFrame()
   * shouldn't unconditionally pay ~38KB of static RAM for conversion
   * buffers they never touch. Same one-time-allocate-then-reuse idiom as
   * Frame::ensureAllocated() (h264_frame.h): a `Buffer<uint8_t>`
   * allocated once, not a per-call allocation in the hot path, and using
   * the same MemoryResource so it can be placed in PSRAM alongside
   * frame_ on boards that have it. Protected (not private): a subclass
   * that needs to hand hardware a planar YUV420 source too (see
   * encoder/h264_hw_encoder_p4.h::HwEncoderP4) reuses this same scratch
   * rather than keeping a second copy.
   */
  bool prepareYuvScratch(int width, int height) {
    if (width <= 0 || height <= 0 || (width % 16) != 0 ||
        (height % 16) != 0) {
      H264LOG.error("prepareYuvScratch: invalid size %dx%d (must be >0 and a multiple of 16)",
                     width, height);
      return false;
    }
    if (width > maxWidth_ || height > maxHeight_) {
      H264LOG.error("prepareYuvScratch: %dx%d exceeds the allocation ceiling %dx%d - see setMaxDimension()",
                     width, height, maxWidth_, maxHeight_);
      return false;
    }
    return ensureYuvScratchAllocated();
  }

 private:

  /**
   * The allocation half of prepareYuvScratch() above, split out so
   * begin() can also trigger it eagerly (see begin()'s own comment)
   * without duplicating the resize logic or its width/height validation
   * (irrelevant when called from begin(), which has no picture yet).
   * Returns false - without crashing - if the allocator (see
   * StdAllocator.h) couldn't satisfy one of the three allocations.
   */
  bool ensureYuvScratchAllocated() {
    if (!yuvY_.empty()) return true;
    bool ok = yuvY_.allocate((size_t)maxWidth_ * maxHeight_) &&
              yuvU_.allocate((size_t)(maxWidth_ / 2) * (maxHeight_ / 2)) &&
              yuvV_.allocate((size_t)(maxWidth_ / 2) * (maxHeight_ / 2));
    if (!ok) {
      yuvY_.release();
      yuvU_.release();
      yuvV_.release();
      return false;
    }
    return true;
  }

  /**
   * Resolves the `qp` an encodeIFrame()/encodePFrame() call should
   * actually use: a real 0-51 value passes through unchanged (and is
   * remembered for lastQp()); `-1` requests rate control, replaced with
   * the current `adaptiveQp_` estimate (must have been primed by
   * setTargetBitrate() first - `targetFrameBytes_ <= 0` means it never
   * was, so this fails rather than silently encoding at a meaningless
   * default). Returns false (nothing written to `*qp`, caller should
   * fail the whole encode call) for any other invalid value.
   */
  bool resolveQp(int* qp) {
    if (*qp == -1) {
      if (targetFrameBytes_ <= 0) {
        H264LOG.error("resolveQp: qp=-1 (rate control) requested but setTargetBitrate() was never called");
        return false;
      }
      *qp = adaptiveQp_;
    } else if (*qp < 0 || *qp > 51) {
      H264LOG.error("resolveQp: invalid qp=%d (must be 0-51, or -1 for rate control)", *qp);
      return false;
    }
    lastQp_ = *qp;
    return true;
  }

  /**
   * Rate control's feedback step, run after every frame actually
   * encoded with `qp = -1` (encodeIFrame()/encodePFrame() call this
   * unconditionally - it's a no-op when rate control was never
   * configured, `targetFrameBytes_ <= 0`, so an explicit-QP caller pays
   * nothing for this). A hysteresis-banded proportional controller:
   * `actualBytes` well over target nudges `adaptiveQp_` up (coarser,
   * smaller future frames), well under nudges it down; small errors
   * (within 10% of target) are left alone rather than chasing every
   * frame's natural content-driven size variation, and the per-frame
   * step is capped at +/-2 QP to avoid visible quality oscillation from
   * one frame to the next - both are real, if not empirically-optimal-
   * tuned-against-a-rate-distortion-curve, choices consistent with this
   * encoder's whole "simple and real-time-appropriate, not maximally
   * efficient" design center (same spirit as shouldUseIntra4x4()'s
   * kI4x4Bias, h264_macroblock_encode.h). Verified empirically (not
   * just "does it compile") against a real multi-frame sequence - see
   * test/native/test_encode_ratecontrol.cpp - that actual total output
   * size converges toward the configured target over a handful of
   * frames rather than diverging or oscillating without bound.
   */
  void updateRateControl(size_t actualBytes) {
    if (targetFrameBytes_ <= 0) return;
    int64_t error = (int64_t)actualBytes - targetFrameBytes_;
    int step = 0;
    if (error > targetFrameBytes_ / 4) {
      step = 2;
    } else if (error > targetFrameBytes_ / 10) {
      step = 1;
    } else if (error < -targetFrameBytes_ / 4) {
      step = -2;
    } else if (error < -targetFrameBytes_ / 10) {
      step = -1;
    }
    adaptiveQp_ += step;
    if (adaptiveQp_ < 0) adaptiveQp_ = 0;
    if (adaptiveQp_ > 51) adaptiveQp_ = 51;
  }

  Frame frame_;
  MbInfoTable mbInfo_;
  uint8_t sliceScratch_[H264_MAX_NAL_SIZE];

 protected:
  // Declared here (not grouped with the rest of this class's protected
  // section further below) so their position relative to frame_/mbInfo_/
  // refFrame_ - all memRes-constructed via this class's own constructor
  // initializer list, in this exact order - doesn't change; moving them
  // elsewhere would silently reorder construction (-Wreorder).
  Buffer<uint8_t> yuvY_;
  Buffer<uint8_t> yuvU_;
  Buffer<uint8_t> yuvV_;

 private:
  /*
   * P-frame state: the single reference picture (this milestone's only
   * reference - see h264_macroblock_encode_inter.h), whether one exists
   * yet (encodePFrame() refuses to run without it), the frame_num to use
   * for the *next* encodePFrame() call, the picture dimensions
   * established by the last encodeIFrame() call (encodePFrame() has no
   * dimensions of its own), and the QP writePpsRbsp() was originally
   * called with (writeSliceHeaderP() needs it to compute slice_qp_delta).
   */
  Frame refFrame_;
  bool haveReference_ = false;
  int frameNum_ = 0;

 protected:
  /// Picture width/height established by setSize() - see
  /// needsKeyframe() (resolution change forces an I-frame) and
  /// encoder/h264_hw_encoder_p4.h::HwEncoderP4, which reads these
  /// directly to (re)open the hardware encoder at matching dimensions
  /// instead of a separate cached copy.
  int width_ = 0, height_ = 0;

 private:
  int ppsBaseQp_ = 0;

  /*
   * Rate control state (see setTargetBitrate()/resolveQp()/
   * updateRateControl()): targetFrameBytes_ <= 0 means "not configured,
   * qp = -1 is refused"; adaptiveQp_ is the running QP estimate, adjusted
   * after each rate-controlled frame; lastQp_ records whatever QP the
   * most recent call actually used (explicit or rate-controlled) for
   * lastQp()'s benefit.
   */
  int targetFrameBytes_ = 0;
  int adaptiveQp_ = 26;
  int lastQp_ = 26;

 protected:
  /// keyframeInterval_ <= 0 means "never insert a periodic keyframe"
  /// (the default - encodeFrame() still always re-keys on a fresh
  /// sequence or a resolution change, those aren't optional) - see
  /// needsKeyframe()/setKeyframeInterval(). Also read directly by
  /// encoder/h264_hw_encoder_p4.h::HwEncoderP4 as its own GOP size.
  int keyframeInterval_ = 0;

 private:
  int framesSinceKeyframe_ = 0;

 protected:
  /*
   * Defaults for the public encodeFrame()-family methods (see
   * setStride()/setPackedStride()/setQp()): defaultStrideY_/
   * defaultStrideC_/defaultPackedStride_ <= 0 means "derive a tightly-
   * packed stride from width_ instead" (the common no-row-padding case);
   * defaultQp_ == -1 means "use rate control" (encodeIFrame()/
   * encodePFrame()'s own existing qp == -1 meaning, so this needs no
   * separate sentinel). width_/height_ themselves (see setSize()) reuse
   * the fields already declared above rather than duplicating them here.
   * Protected: encoder/h264_hw_encoder_p4.h::HwEncoderP4 reads these
   * directly to resolve the same effective stride/QP its inherited
   * software fallback would use.
   */
  int defaultStrideY_ = -1;
  int defaultStrideC_ = -1;
  int defaultPackedStride_ = -1;
  int defaultQp_ = -1;

 private:
  int maxWidth_ = H264_MAX_WIDTH;    ///< allocation-ceiling width, see setMaxDimension()
  int maxHeight_ = H264_MAX_HEIGHT;  ///< allocation-ceiling height, see setMaxDimension()
  int motionSearchRange_ = 8;  ///< +/-pixel search window, see setMotionSearchRange()
  MotionSearchAlgorithm motionSearchAlgorithm_ =
      MotionSearchAlgorithm::Exhaustive;  ///< see setMotionSearchAlgorithm()
};

}  // namespace tinyh264
