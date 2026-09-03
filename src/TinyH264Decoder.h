#pragma once
#include "MemoryResource.h"
#include "StdAllocator.h"
#include "decoder/h264_decoder.h"
#include "decoder/h264_rgb.h"
#ifdef ESP32
#include "PSRAMAllocatorESP32.h"
#endif

/*
 * TinyH264Decoder: a minimal, header-only H.264 Baseline Profile (CAVLC)
 * decoder for microcontrollers. This is the public-facing API, in the same
 * `tinyh264` namespace as everything under decoder/h264_*.h (implementation
 * detail, but kept in one consistent namespace throughout the library).
 *
 * Usage:
 *   using namespace tinyh264;
 *
 *   void onFrame(TinyH264Decoder<> &decoder, void *userData) {
 *     // decoder.width()/height()/y()/u()/v()/strideY()/strideUV()
 *   }
 *   TinyH264Decoder<> decoder;
 *   decoder.setCallback(onFrame);
 *   decoder.write(annexBBuffer, size);  // one or more complete NAL units;
 *                                       // onFrame() runs once per decoded
 *                                       // picture before write() returns.
 *   if (decoder.hasError()) { ... }     // bitstream error, unsupported
 *                                       // feature, or an out-of-memory
 *                                       // allocation failure - see Status
 *                                       // below - during that write() call
 *
 * The Allocator template parameter (default StdAllocator<uint8_t>, see
 * StdAllocator.h) is used for the picture buffers (see h264_frame.h) -
 * pass a custom allocator to place them in PSRAM or a dedicated pool
 * instead of the default heap, e.g. on an ESP32-S3 with PSRAM:
 *   TinyH264Decoder<PSRAMAllocatorESP32<uint8_t>> decoder;
 * TinyH264Decoder itself stays templated on Allocator for a stable
 * public API, but internally builds an AllocatorMemoryResource<Allocator>
 * (see MemoryResource.h) and hands that down to the non-templated
 * SoftwareDecoder it wraps. Any Allocator used here should follow the
 * same "return nullptr, don't throw" contract StdAllocator/PSRAMAllocatorESP32 do (see
 * StdAllocator.h's file comment for why) so an out-of-memory condition
 * surfaces as Status::kAllocationError instead of crashing.
 *
 * See h264_config.h for the resolution/memory budget (H264_MAX_WIDTH/
 * H264_MAX_HEIGHT default to QCIF, sized for a plain ESP32 without PSRAM).
 */

namespace tinyh264 {

/**
 * Public-facing decoder API: wraps the internal SoftwareDecoder (see
 * decoder/h264_decoder.h) behind a small push-data-get-callback interface
 * suitable for feeding NAL data straight off a file, PROGMEM buffer, or
 * network socket without the caller needing to know anything about NAL
 * units, slices, or macroblocks. `Allocator` controls where the decoded
 * picture buffers (Frame::y/u/v) are allocated - see the file comment
 * above for the PSRAM usage example.
 */
template <typename Allocator = StdAllocator<uint8_t>>
class TinyH264Decoder {
 public:
  /// Builds this decoder's AllocatorMemoryResource<Allocator> and hands
  /// it to the SoftwareDecoder it wraps - see the file comment above.
  TinyH264Decoder() : decoder_(memRes_) {}

  /// Outcome of the most recent write()/decodeNext() call.
  enum class Status {
    kFrameReady,       ///< a new picture is ready via width()/height()/y()/u()/v()
    kNeedMoreData,     ///< call write() with more data and keep decoding
    kUnsupported,      ///< stream uses an unimplemented feature
    kError,            ///< corrupt/truncated bitstream
    kAllocationError,  ///< a picture buffer allocation failed (out of memory) - see StdAllocator.h; treat this as terminal for the current decoder instance the same way kError is
  };

  /**
   * Called once per decoded picture, from within write(). `decoder` is the
   * same object write() was called on (so width()/y()/u()/v()/etc. are
   * valid inside the callback); `userData` is whatever was passed to
   * setCallback().
   */
  using FrameCallback = void (*)(TinyH264Decoder& decoder, void* userData);

  /**
   * Registers the function to invoke once per decoded picture during
   * write(). Pass nullptr to disable (the default); `userData` is passed
   * through unchanged to each callback invocation.
   */
  void setCallback(FrameCallback cb, void* userData = nullptr) {
    callback_ = cb;
    userData_ = userData;
  }

  /**
   * Sets the runtime-active maximum number of stored reference pictures,
   * clamped to [1, H264_MAX_REF_FRAMES] (the compile-time upper bound,
   * see h264_config.h - defaults to 3, matching ffmpeg's own default
   * -preset). Defaults to H264_MAX_REF_FRAMES if never called. Lower
   * this to reduce memory use (each reference picture costs one full
   * frame buffer, ~38KB at QCIF) if you know your stream needs fewer;
   * raise H264_MAX_REF_FRAMES itself (via -D or #define before including
   * this header) if you need more than the compile-time default allows.
   */
  void setMaxRefFrames(int n) { decoder_.setMaxRefFrames(n); }

  /// The current runtime-active maximum reference-picture count.
  int maxRefFrames() const { return decoder_.maxRefFrames(); }

  /**
   * Runtime alternative to `#define H264_MAX_WIDTH ...`/`#define
   * H264_MAX_HEIGHT ...` before `#include`ing this header - overrides
   * the picture-buffer/metadata-table allocation ceiling for this
   * decoder instance (see Decoder::setMaxDimension(), h264_decoder.h,
   * for the full explanation). A stream whose SPS declares a resolution
   * bigger than this is rejected as kUnsupported. Call before the first
   * decode to size buffers for your actual content up front instead of
   * the compile-time H264_MAX_WIDTH/H264_MAX_HEIGHT default (h264_config.h) -
   * e.g. `decoder.setMaxDimension(256, 192);` in setup().
   */
  void setMaxDimension(int maxWidth, int maxHeight) {
    decoder_.setMaxDimension(maxWidth, maxHeight);
  }
  /// The current allocation-ceiling width - see setMaxDimension().
  int maxWidth() const { return decoder_.maxWidth(); }
  /// The current allocation-ceiling height - see setMaxDimension().
  int maxHeight() const { return decoder_.maxHeight(); }

  /**
   * Reserves this decoder's picture buffers up front instead of the
   * default allocate-on-first-decode behavior - see Decoder::begin()'s
   * own comment (decoder/h264_decoder.h) for exactly what this does and
   * why it's optional (write()/decodeNext() still allocate lazily on
   * their own if this was never called). Call once, typically from
   * setup(), if you want any allocation failure to surface
   * deterministically before the stream starts rather than mid-decode.
   * Returns false (and sets lastStatus() to kAllocationError) if
   * allocation failed - out of memory, not a crash - see StdAllocator.h.
   */
  bool begin() {
    bool ok = decoder_.begin();
    lastStatus_ = ok ? Status::kNeedMoreData : Status::kAllocationError;
    return ok;
  }

  /**
   * Releases this decoder's picture buffers and resets it to a fresh,
   * just-constructed state (no cached SPS/PPS, no reference pictures) -
   * see Decoder::end()'s own comment for exactly what this does. Not
   * required before destruction; only useful for reclaiming memory
   * before that, e.g. between unrelated streams, while this object is
   * still alive. Safe to call begin()/write() again afterward to start
   * over.
   */
  void end() {
    decoder_.end();
    lastStatus_ = Status::kNeedMoreData;
    nextFrameIndex_ = 0;
    frameIndex_ = 0;
  }

  /**
   * Feeds one buffer of Annex-B data (NAL units delimited by 00 00 01 / 00
   * 00 00 01 start codes) and immediately decodes everything it can from
   * it, invoking the frame callback (if set, via setCallback()) once per
   * decoded picture. The buffer must remain valid for the duration of this
   * call. Returns the status of the *last* thing that happened while
   * draining the buffer: kFrameReady if the last NAL produced a picture,
   * kNeedMoreData once the buffer is fully drained with nothing pending,
   * or kUnsupported/kError/kAllocationError if decoding stopped early
   * because of one - same value as lastStatus() would give right after
   * this call.
   */
  Status write(const uint8_t* data, size_t size) {
    decoder_.setInput(data, size);
    while (true) {
      DecodeStatus raw = decoder_.next();
      if (raw == DecodeStatus::kOk) {
        lastStatus_ = Status::kFrameReady;
        onFrameReady();
        if (callback_) callback_(*this, userData_);
        continue;
      }
      if (raw == DecodeStatus::kNeedMoreData) {
        /*
         * This alone doesn't mean "buffer drained" - it's also returned
         * after processing a non-picture NAL (SPS/PPS/SEI/...), which
         * just means "keep calling next()". Only stop once the NAL
         * reader itself has run out of start codes.
         */
        if (decoder_.inputExhausted()) {
          lastStatus_ = Status::kNeedMoreData;
          break;
        }
        continue;
      }
      lastStatus_ = toStatus(raw);  // kError / kUnsupported / kAllocationError
      break;
    }
    return lastStatus_;
  }

  /**
   * Steps the decoder by exactly one NAL unit, for callers that want to
   * drive decoding manually instead of using the write()+callback loop.
   * Does not invoke the frame callback - check the return value (or
   * lastStatus()) for kFrameReady instead.
   */
  Status decodeNext() {
    lastStatus_ = toStatus(decoder_.next());
    if (lastStatus_ == Status::kFrameReady) onFrameReady();
    return lastStatus_;
  }

  /// The Status returned by the most recent write() or decodeNext() call.
  Status lastStatus() const { return lastStatus_; }
  /// True if the most recent call ended in kError, kUnsupported, or kAllocationError.
  bool hasError() const {
    return lastStatus_ == Status::kError || lastStatus_ == Status::kUnsupported ||
           lastStatus_ == Status::kAllocationError;
  }

  /**
   * The frame rate declared by the stream itself - SPS vui_parameters()'s
   * timing_info (num_units_in_tick/time_scale), i.e. what the encoder
   * says the source video's frame rate is, not anything timed/measured
   * by this decoder. Valid once at least one picture has been decoded
   * (uses the SPS active for the most recently decoded picture's
   * slices); returns 0.0f if that SPS never set vui_parameters_present_flag/
   * timing_info_present_flag - a common, spec-legal case (many encoders,
   * including this library's own TinyH264Encoder, never emit VUI at all),
   * not an error. See Sps::frameRate() (decoder/h264_sps_pps.h) for the
   * exact conversion.
   */
  float fps() const { return (float)decoder_.sps().frameRate(); }

  /**
   * Sets the frame rate to fall back to when the stream itself doesn't
   * declare one - i.e. when fps() returns 0.0f (no VUI timing_info, a
   * common, spec-legal case - see fps()'s own comment; this library's own
   * TinyH264Encoder never emits VUI). Used by presentationTimeMs() below.
   * Ignored whenever fps() is nonzero - the stream's own declared rate
   * always wins. Defaults to 0.0f (no fallback).
   */
  void setDefaultFps(float fps) { defaultFps_ = fps; }
  /// The current fallback frame rate - see setDefaultFps().
  float defaultFps() const { return defaultFps_; }

  /**
   * The time, in milliseconds since the first decoded picture, at which
   * the most recently decoded picture should be displayed - valid after
   * the frame callback fires (or decodeNext() returns kFrameReady), same
   * as width()/y()/etc. above.
   *
   * This works out to simply `frameIndex * 1000 / fps`: Baseline Profile
   * (the only profile this decoder supports) has no B-frames, so pictures
   * are never reordered between decode and display - the Nth picture
   * decoded is always the Nth picture displayed. That means presentation
   * time can be derived purely from a running frame count and the frame
   * rate, without needing any timestamp carried in the bitstream itself
   * (H.264 slice headers don't carry one anyway - that's what a
   * container format like MP4/RTP normally supplies).
   *
   * The frame rate used is fps() if the stream declared one, otherwise
   * defaultFps() (see setDefaultFps()) as a fallback. Returns 0 if
   * neither is known (both are 0.0f) - in that case every frame reports
   * time 0, since there's no rate to derive an interval from.
   */
  uint32_t presentationTimeMs() const {
    float rate = fps() > 0.0f ? fps() : defaultFps_;
    if (rate <= 0.0f) return 0;
    return (uint32_t)((double)frameIndex_ * 1000.0 / (double)rate + 0.5);
  }

  /*
   * Valid after the frame callback fires (or decodeNext() returns
   * kFrameReady). YUV 4:2:0 planar, 8-bit.
   */
  int width() const { return decoder_.frame().width; }   ///< coded picture width in luma samples
  int height() const { return decoder_.frame().height; } ///< coded picture height in luma samples
  const uint8_t* y() const { return decoder_.frame().y(); } ///< luma plane, strideY() bytes/row
  const uint8_t* u() const { return decoder_.frame().u(); } ///< Cb plane, strideUV() bytes/row
  const uint8_t* v() const { return decoder_.frame().v(); } ///< Cr plane, strideUV() bytes/row
  int strideY() const { return decoder_.frame().strideY; } ///< bytes per row of y()
  int strideUV() const { return decoder_.frame().strideC; } ///< bytes per row of u()/v()

  /**
   * Sets an output scale factor applied by the toRGB565()/toRGB666()/
   * toRGB888()/toYUV420() conversion methods below (both whole-frame and
   * windowed/tiled overloads) - useful when the decoded resolution is
   * smaller than the target display (e.g. decoding at a lower resolution
   * to fit a memory-constrained board's heap - see docs/memory-budget.md -
   * but wanting to fill a bigger physical screen). Does not change how
   * the bitstream is decoded, y()/u()/v()/width()/height()/getY()/getU()/
   * getV() - only the conversion methods' *output* dimensions and pixel
   * sampling. Values above 1.0 upscale, below 1.0 downscale; the default,
   * 1.0, means "no scaling" and is a true no-op (widthScaled()/heightScaled()
   * exactly equal width()/height(), and every conversion method's output
   * is byte-identical to before this method existed - see
   * test/native/test_scale.cpp). Scaling uses nearest-neighbor sampling
   * (integer ratio, no interpolation) - matches this library's general
   * simplicity-over-quality conversion philosophy (see decoder/h264_rgb.h).
   */
  void setScaleFactor(float factor) { scaleFactor_ = factor; }
  /// The current output scale factor - see setScaleFactor().
  float scaleFactor() const { return scaleFactor_; }

  /**
   * Controls whether toRGB565() (both whole-frame and windowed/tiled
   * overloads) byte-swaps each output uint16_t before returning it.
   * RGB565 is a 16-bit value, but the wire format most SPI/8080-parallel
   * TFT controllers (ILI9341, ST7789, ...) expect is big-endian (high
   * byte first) - the opposite of how a uint16_t is laid out in memory
   * on every MCU this library targets (ESP32, RP2040 - both
   * little-endian). If a display driver (or your own SPI code) writes
   * toRGB565()'s buffer to the panel as raw bytes without separately
   * correcting for this, the R/B channels and brightness come out
   * scrambled - a very common "colors are wrong" symptom that has
   * nothing to do with the YUV->RGB conversion itself being wrong.
   * Defaults to true (swap) since that's what a raw memcpy/DMA straight
   * to an SPI panel needs; set false if you're instead handing the
   * buffer to a display library (e.g. TFT_eSPI's pushImage()) that
   * already does its own byte-order correction - swapping twice would
   * undo the fix. Does not affect toRGB666()/toRGB888()/toYUV420(),
   * which are byte-oriented (no multi-byte word, so no endianness to
   * get wrong).
   */
  void setByteSwap(bool enable) { byteSwap_ = enable; }
  /// The current toRGB565() byte-swap setting - see setByteSwap().
  bool byteSwap() const { return byteSwap_; }

  /**
   * Scaled picture width/height - what the toXXX() conversion methods'
   * whole-frame overloads actually produce, and what a windowed/tiled
   * toXXX() call's x/y/dx/dy are measured against (see setScaleFactor()).
   * Always even (4:2:0 chroma needs it), rounded from
   * width()/height() * scaleFactor(). Equal to width()/height() exactly
   * when scaleFactor() is 1.0 (the default) - width()/height() are
   * already guaranteed even (H.264 macroblocks are 16x16), so no
   * rounding actually occurs in that case.
   */
  int widthScaled() const { return scaledDimension(width()); }
  /// See widthScaled().
  int heightScaled() const { return scaledDimension(height()); }

  /*
   * Single-sample convenience accessors, for callers that want one pixel
   * at a time rather than raw plane/stride pointer math - e.g. a simple
   * "eyeball the frame" sanity check, or a naive per-pixel post-process.
   * Not bounds-checked (no exceptions/aborts on out-of-range px/py,
   * matching this library's no-hidden-cost philosophy elsewhere) - caller
   * must ensure 0 <= px < width() and 0 <= py < height(). Parameters are
   * named px/py rather than x/y specifically so they don't shadow this
   * class's own y() accessor when read inside these method bodies.
   */

  /// Luma (brightness) sample at pixel (px, py), full resolution.
  uint8_t getY(int px, int py) const { return y()[(size_t)py * strideY() + px]; }
  /**
   * Cb (blue-difference chroma) sample at pixel (px, py) - internally
   * indexes at (px/2, py/2), since chroma is subsampled 2x in each
   * dimension (4:2:0). Multiple adjacent (px,py) therefore read the same
   * underlying sample, by design (H.264/YUV420 samples color once per
   * 2x2 luma block, not once per pixel - see getV()/toRGB565()).
   */
  uint8_t getU(int px, int py) const {
    return u()[(size_t)(py >> 1) * strideUV() + (px >> 1)];
  }
  /**
   * Cr (red-difference chroma) sample at pixel (px, py) - see getU() for
   * the subsampling note (applies identically here).
   */
  uint8_t getV(int px, int py) const {
    return v()[(size_t)(py >> 1) * strideUV() + (px >> 1)];
  }

  /**
   * Converts the most recently decoded picture to RGB565 (16-bit,
   * 5-6-5 packed) into a caller-provided buffer - the format most
   * embedded display libraries (TFT_eSPI, Adafruit_GFX, LovyanGFX, ...)
   * expect. Each uint16_t is byte-swapped before being written unless
   * setByteSwap(false) was called - see setByteSwap() for why the
   * default is swap-on (most SPI/parallel TFT panels want big-endian
   * pixels; every MCU this library targets is little-endian). Output is
   * widthScaled()*heightScaled() (see setScaleFactor() - equal to
   * width()*height() at the default scale factor of 1.0). `dstCapacity`
   * is the buffer's size in uint16_t entries (not bytes); if it's
   * smaller than widthScaled()*heightScaled(), nothing is written and
   * this returns 0 - checked explicitly rather than trusting the caller
   * sized `dst` correctly, since a raw pointer carries no size
   * information of its own and a mismatch here would otherwise silently
   * corrupt memory past the buffer. On success, returns the number of
   * uint16_t entries written. Never allocates on success; write straight
   * into your display's own framebuffer if it has one, rather than
   * through an intermediate copy. See decoder/h264_rgb.h for the exact
   * YUV-to-RGB conversion used (ITU-R BT.601 limited range, cross-checked
   * against ffmpeg's own conversion to within 1 LSB per channel). Also
   * see toRGB666()/toRGB888() for 18-bit and full 24-bit color instead.
   */
  size_t toRGB565(uint16_t* dst, size_t dstCapacity) const {
    size_t needed = (size_t)widthScaled() * (size_t)heightScaled();
    if (dstCapacity < needed) return 0;
    convertFrameToRgb565(decoder_.frame(), widthScaled(), heightScaled(), dst);
    if (byteSwap_) swapBytes16(dst, needed);
    return needed;
  }

  /**
   * Converts a `dx` x `dy` rectangle of the most recently decoded
   * picture, starting at (x, y), to RGB565 into a caller-provided
   * buffer (packed with no row padding - dst's row stride is exactly
   * dx). `x`/`y`/`dx`/`dy` are measured against the *scaled* picture
   * (widthScaled() x heightScaled(), see setScaleFactor()), not the native
   * decode resolution. `dstCapacity` is checked against dx*dy the same
   * way as the whole-frame overload above - returns 0 without writing if
   * too small, or dx*dy on success. Splits the whole-frame conversion
   * into smaller pieces - e.g. to push a decoded picture to a display in
   * tiles (matching how many small-display drivers already expect
   * windowed pushColors() calls), or to spread the conversion work
   * across multiple loop() iterations instead of one large blocking
   * call. `x`/`y` must be even (chroma is subsampled 2x - see
   * decoder/h264_rgb.h) and, together with dx/dy, must stay within
   * widthScaled()/heightScaled() - that part is not bounds-checked (only the
   * destination buffer size is).
   */
  size_t toRGB565(int x, int y, int dx, int dy, uint16_t* dst,
                  size_t dstCapacity) const {
    size_t needed = (size_t)dx * (size_t)dy;
    if (dstCapacity < needed) return 0;
    convertFrameToRgb565(decoder_.frame(), widthScaled(), heightScaled(), x, y, dx,
                         dy, dst);
    if (byteSwap_) swapBytes16(dst, needed);
    return needed;
  }

  /**
   * Converts the most recently decoded picture to RGB666 (18-bit color:
   * 3 bytes per pixel, R/G/B order, each byte's 6 significant bits
   * left-justified in bits 7:2) into a caller-provided buffer -
   * `dstCapacity` (in uint8_t entries) must be at least
   * widthScaled()*heightScaled()*3 (see setScaleFactor()), checked the same
   * way as toRGB565() (returns 0 without writing if too small, or the
   * number of bytes written on success). The wire format real
   * 18-bit-interface TFT controllers (e.g. ILI9488 in 18-bit mode)
   * expect - see decoder/h264_rgb.h for exactly why this bit layout and
   * how it was verified.
   */
  size_t toRGB666(uint8_t* dst, size_t dstCapacity) const {
    size_t needed = (size_t)widthScaled() * (size_t)heightScaled() * 3;
    if (dstCapacity < needed) return 0;
    convertFrameToRgb666(decoder_.frame(), widthScaled(), heightScaled(), dst);
    return needed;
  }

  /**
   * Windowed/tiled counterpart of toRGB666() - see toRGB565(x,y,dx,dy,...)
   * for the tiling rationale, the scaled-coordinate-space semantics, and
   * the x/y-must-be-even constraint. `dstCapacity` must be at least
   * dx*dy*3.
   */
  size_t toRGB666(int x, int y, int dx, int dy, uint8_t* dst,
                  size_t dstCapacity) const {
    size_t needed = (size_t)dx * (size_t)dy * 3;
    if (dstCapacity < needed) return 0;
    convertFrameToRgb666(decoder_.frame(), widthScaled(), heightScaled(), x, y, dx,
                         dy, dst);
    return needed;
  }

  /**
   * Converts the most recently decoded picture to RGB888 (24-bit color:
   * 3 bytes per pixel, R/G/B order, full 8-bit precision per channel -
   * matches ffmpeg's `rgb24`) into a caller-provided buffer -
   * `dstCapacity` (in uint8_t entries) must be at least
   * widthScaled()*heightScaled()*3 (see setScaleFactor()), checked the same
   * way as toRGB565() (returns 0 without writing if too small, or the
   * number of bytes written on success).
   */
  size_t toRGB888(uint8_t* dst, size_t dstCapacity) const {
    size_t needed = (size_t)widthScaled() * (size_t)heightScaled() * 3;
    if (dstCapacity < needed) return 0;
    convertFrameToRgb888(decoder_.frame(), widthScaled(), heightScaled(), dst);
    return needed;
  }

  /**
   * Windowed/tiled counterpart of toRGB888() - see toRGB565(x,y,dx,dy,...)
   * for the tiling rationale, the scaled-coordinate-space semantics, and
   * the x/y-must-be-even constraint. `dstCapacity` must be at least
   * dx*dy*3.
   */
  size_t toRGB888(int x, int y, int dx, int dy, uint8_t* dst,
                  size_t dstCapacity) const {
    size_t needed = (size_t)dx * (size_t)dy * 3;
    if (dstCapacity < needed) return 0;
    convertFrameToRgb888(decoder_.frame(), widthScaled(), heightScaled(), x, y, dx,
                         dy, dst);
    return needed;
  }

  /**
   * Copies the most recently decoded picture into a caller-provided
   * buffer as one tightly-packed YUV 4:2:0 planar block (standard
   * "I420" layout: Y bytes, then U bytes, then V bytes, no row padding -
   * unlike y()/u()/v() above, which point into this decoder's own
   * internal frame storage, are never scaled, and are only valid until
   * the next picture is decoded). Useful for handing a whole decoded
   * frame to something that wants one contiguous buffer - writing it to
   * storage, sending it over a socket, or queuing it for another
   * thread/task to process later - rather than three separate plane
   * pointers. Output is widthScaled()*heightScaled() (see setScaleFactor()).
   * `dstCapacity` (in uint8_t entries) must be at least
   * widthScaled()*heightScaled() + 2*(widthScaled()/2)*(heightScaled()/2); returns 0
   * without writing if too small, or the number of bytes written on
   * success - the same size-checking pattern as
   * toRGB565()/toRGB666()/toRGB888(). Never allocates. See
   * decoder/h264_frame.h for the exact layout.
   */
  size_t toYUV420(uint8_t* dst, size_t dstCapacity) const {
    int w = widthScaled(), h = heightScaled();
    size_t needed = (size_t)w * h + 2 * (size_t)(w / 2) * (size_t)(h / 2);
    if (dstCapacity < needed) return 0;
    convertFrameToYuv420(decoder_.frame(), w, h, dst);
    return needed;
  }

  /**
   * Windowed/tiled counterpart of toYUV420() - packs just a `dx` x `dy`
   * rectangle starting at (x, y) instead of the whole picture, the same
   * tiling rationale and scaled-coordinate-space semantics as
   * toRGB565(x,y,dx,dy,...). `x`/`y` must be even (chroma is subsampled
   * 2x) and, together with dx/dy, must stay within
   * widthScaled()/heightScaled() - not bounds-checked (only the destination
   * buffer size is). `dstCapacity` must be at least dx*dy +
   * 2*(dx/2)*(dy/2).
   */
  size_t toYUV420(int x, int y, int dx, int dy, uint8_t* dst,
                  size_t dstCapacity) const {
    size_t needed = (size_t)dx * dy + 2 * (size_t)(dx / 2) * (size_t)(dy / 2);
    if (dstCapacity < needed) return 0;
    convertFrameToYuv420(decoder_.frame(), widthScaled(), heightScaled(), x, y, dx,
                         dy, dst);
    return needed;
  }

 protected:
  /**
   * Byte-swaps each of `count` uint16_t entries in place - see
   * setByteSwap(). A plain 8-bit shuffle rather than a builtin byte-swap
   * intrinsic, to stay portable across every target this header-only
   * library compiles on (Xtensa/ARM/desktop) without a per-platform
   * `#ifdef`.
   */
  static void swapBytes16(uint16_t* buf, size_t count) {
    for (size_t i = 0; i < count; i++) {
      uint16_t v = buf[i];
      buf[i] = (uint16_t)((v << 8) | (v >> 8));
    }
  }

  /**
   * Maps the internal Decoder's DecodeStatus onto this class's public
   * Status enum (kept separate so the internal enum can evolve without
   * changing the public API's values).
   */
  static Status toStatus(DecodeStatus s) {
    switch (s) {
      case DecodeStatus::kOk:
        return Status::kFrameReady;
      case DecodeStatus::kNeedMoreData:
        return Status::kNeedMoreData;
      case DecodeStatus::kUnsupported:
        return Status::kUnsupported;
      case DecodeStatus::kAllocationError:
        return Status::kAllocationError;
      case DecodeStatus::kError:
      default:
        return Status::kError;
    }
  }

  /**
   * Rounds `nativeDimension * scaleFactor_` to the nearest integer, then
   * down to the nearest even number (4:2:0 chroma needs an even
   * dimension) - see widthScaled()/heightScaled(). Floored to 2 rather than 0
   * so a very small scale factor still produces a valid (if degenerate)
   * picture instead of a zero-sized one.
   */
  int scaledDimension(int nativeDimension) const {
    int scaled = (int)(nativeDimension * scaleFactor_ + 0.5f);
    if (scaled < 2) scaled = 2;
    return scaled & ~1;
  }

  /**
   * Records the display-order index of the picture that was just decoded,
   * for presentationTimeMs() - called once per kFrameReady, from both
   * write() (before the callback fires) and decodeNext().
   */
  void onFrameReady() { frameIndex_ = nextFrameIndex_++; }

  // Declared before decoder_: members initialize in declaration order,
  // and decoder_ needs memRes_ already constructed to build from.
  AllocatorMemoryResource<Allocator> memRes_;
  SoftwareDecoder decoder_;
  FrameCallback callback_ = nullptr;
  void* userData_ = nullptr;
  Status lastStatus_ = Status::kNeedMoreData;
  float scaleFactor_ = 1.0f;
  bool byteSwap_ = true;
  float defaultFps_ = 0.0f;
  uint32_t frameIndex_ = 0;      ///< display-order index of the most recently decoded picture
  uint32_t nextFrameIndex_ = 0;  ///< index that will be assigned to the next decoded picture
};

}  // namespace tinyh264
