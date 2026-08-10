#pragma once
#include <memory>
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
 *   if (decoder.hasError()) { ... }     // bitstream error or unsupported
 *                                       // feature during that write() call
 *
 * The Allocator template parameter (default std::allocator<uint8_t>) is
 * used for the picture buffers (see h264_frame.h) - pass a custom
 * allocator to place them in PSRAM or a dedicated pool instead of the
 * default heap, e.g. on an ESP32-S3 with PSRAM:
 *   TinyH264Decoder<PSRAMAllocatorESP32<uint8_t>> decoder;
 *
 * See h264_config.h for the resolution/memory budget (H264_MAX_WIDTH/
 * H264_MAX_HEIGHT default to QCIF, sized for a plain ESP32 without PSRAM).
 */

namespace tinyh264 {

/**
 * Public-facing decoder API: wraps the internal Decoder<Allocator> (see
 * decoder/h264_decoder.h) behind a small push-data-get-callback interface
 * suitable for feeding NAL data straight off a file, PROGMEM buffer, or
 * network socket without the caller needing to know anything about NAL
 * units, slices, or macroblocks. `Allocator` controls where the decoded
 * picture buffers (Frame::y/u/v) are allocated - see the file comment
 * above for the PSRAM usage example.
 */
template <typename Allocator = std::allocator<uint8_t>>
class TinyH264Decoder {
 public:
  /// Outcome of the most recent write()/decodeNext() call.
  enum class Status {
    kFrameReady,    ///< a new picture is ready via width()/height()/y()/u()/v()
    kNeedMoreData,  ///< call write() with more data and keep decoding
    kUnsupported,   ///< stream uses an unimplemented feature
    kError,         ///< corrupt/truncated bitstream
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
   * Reserves this decoder's picture buffers up front instead of the
   * default allocate-on-first-decode behavior - see Decoder::begin()'s
   * own comment (decoder/h264_decoder.h) for exactly what this does and
   * why it's optional (write()/decodeNext() still allocate lazily on
   * their own if this was never called). Call once, typically from
   * setup(), if you want any allocation failure to surface
   * deterministically before the stream starts rather than mid-decode.
   */
  void begin() { decoder_.begin(); }

  /**
   * Releases this decoder's picture buffers and resets it to a fresh,
   * just-constructed state (no cached SPS/PPS, no reference pictures) -
   * see Decoder::end()'s own comment for exactly what this does. Not
   * required before destruction; only useful for reclaiming memory
   * before that, e.g. between unrelated streams, while this object is
   * still alive. Safe to call begin()/write() again afterward to start
   * over.
   */
  void end() { decoder_.end(); }

  /**
   * Feeds one buffer of Annex-B data (NAL units delimited by 00 00 01 / 00
   * 00 00 01 start codes) and immediately decodes everything it can from
   * it, invoking the frame callback (if set, via setCallback()) once per
   * decoded picture. The buffer must remain valid for the duration of this
   * call. Returns the status of the *last* thing that happened while
   * draining the buffer: kFrameReady if the last NAL produced a picture,
   * kNeedMoreData once the buffer is fully drained with nothing pending,
   * or kUnsupported/kError if decoding stopped early because of one -
   * same value as lastStatus() would give right after this call.
   */
  Status write(const uint8_t* data, size_t size) {
    decoder_.setInput(data, size);
    while (true) {
      DecodeStatus raw = decoder_.next();
      if (raw == DecodeStatus::kOk) {
        lastStatus_ = Status::kFrameReady;
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
      lastStatus_ = toStatus(raw);  // kError / kUnsupported
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
  Status decodeNext() { return lastStatus_ = toStatus(decoder_.next()); }

  /// The Status returned by the most recent write() or decodeNext() call.
  Status lastStatus() const { return lastStatus_; }
  /// True if the most recent call ended in kError or kUnsupported.
  bool hasError() const {
    return lastStatus_ == Status::kError || lastStatus_ == Status::kUnsupported;
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
   * 5-6-5 packed, little-endian) into a caller-provided buffer - the
   * format most embedded display libraries (TFT_eSPI, Adafruit_GFX,
   * LovyanGFX, ...) expect. `dstCapacity` is the buffer's size in
   * uint16_t entries (not bytes); if it's smaller than width()*height(),
   * nothing is written and this returns 0 - checked explicitly rather
   * than trusting the caller sized `dst` correctly, since a raw pointer
   * carries no size information of its own and a mismatch here would
   * otherwise silently corrupt memory past the buffer. On success,
   * returns the number of uint16_t entries written (width()*height()).
   * Never allocates on success; write straight into your display's own
   * framebuffer if it has one, rather than through an intermediate copy.
   * See decoder/h264_rgb.h for the exact YUV-to-RGB conversion used
   * (ITU-R BT.601 limited range, cross-checked against ffmpeg's own
   * conversion to within 1 LSB per channel). Also see toRGB666()/
   * toRGB888() for 18-bit and full 24-bit color instead.
   */
  size_t toRGB565(uint16_t* dst, size_t dstCapacity) const {
    size_t needed = (size_t)width() * (size_t)height();
    if (dstCapacity < needed) return 0;
    convertFrameToRgb565(decoder_.frame(), dst);
    return needed;
  }

  /**
   * Converts a `dx` x `dy` rectangle of the most recently decoded
   * picture, starting at (x, y), to RGB565 into a caller-provided
   * buffer (packed with no row padding - dst's row stride is exactly
   * dx). `dstCapacity` is checked against dx*dy the same way as the
   * whole-frame overload above - returns 0 without writing if too
   * small, or dx*dy on success. Splits the whole-frame conversion into
   * smaller pieces - e.g.
   * to push a decoded picture to a display in tiles (matching how many
   * small-display drivers already expect windowed pushColors() calls),
   * or to spread the conversion work across multiple loop() iterations
   * instead of one large blocking call. `x`/`y` must be even (chroma is
   * subsampled 2x - see decoder/h264_rgb.h) and, together with
   * dx/dy, must stay within width()/height() - that part is not
   * bounds-checked (only the destination buffer size is).
   */
  size_t toRGB565(int x, int y, int dx, int dy, uint16_t* dst,
                  size_t dstCapacity) const {
    size_t needed = (size_t)dx * (size_t)dy;
    if (dstCapacity < needed) return 0;
    convertFrameToRgb565(decoder_.frame(), x, y, dx, dy, dst);
    return needed;
  }

  /**
   * Converts the most recently decoded picture to RGB666 (18-bit color:
   * 3 bytes per pixel, R/G/B order, each byte's 6 significant bits
   * left-justified in bits 7:2) into a caller-provided buffer -
   * `dstCapacity` (in uint8_t entries) must be at least
   * width()*height()*3, checked the same way as toRGB565() (returns 0
   * without writing if too small, or the number of bytes written on
   * success). The wire format real 18-bit-interface TFT controllers
   * (e.g. ILI9488 in 18-bit mode) expect - see decoder/h264_rgb.h for
   * exactly why this bit layout and how it was verified.
   */
  size_t toRGB666(uint8_t* dst, size_t dstCapacity) const {
    size_t needed = (size_t)width() * (size_t)height() * 3;
    if (dstCapacity < needed) return 0;
    convertFrameToRgb666(decoder_.frame(), dst);
    return needed;
  }

  /**
   * Windowed/tiled counterpart of toRGB666() - see toRGB565(x,y,dx,dy,...)
   * for the tiling rationale and the x/y-must-be-even constraint.
   * `dstCapacity` must be at least dx*dy*3.
   */
  size_t toRGB666(int x, int y, int dx, int dy, uint8_t* dst,
                  size_t dstCapacity) const {
    size_t needed = (size_t)dx * (size_t)dy * 3;
    if (dstCapacity < needed) return 0;
    convertFrameToRgb666(decoder_.frame(), x, y, dx, dy, dst);
    return needed;
  }

  /**
   * Converts the most recently decoded picture to RGB888 (24-bit color:
   * 3 bytes per pixel, R/G/B order, full 8-bit precision per channel -
   * matches ffmpeg's `rgb24`) into a caller-provided buffer -
   * `dstCapacity` (in uint8_t entries) must be at least
   * width()*height()*3, checked the same way as toRGB565() (returns 0
   * without writing if too small, or the number of bytes written on
   * success).
   */
  size_t toRGB888(uint8_t* dst, size_t dstCapacity) const {
    size_t needed = (size_t)width() * (size_t)height() * 3;
    if (dstCapacity < needed) return 0;
    convertFrameToRgb888(decoder_.frame(), dst);
    return needed;
  }

  /**
   * Windowed/tiled counterpart of toRGB888() - see toRGB565(x,y,dx,dy,...)
   * for the tiling rationale and the x/y-must-be-even constraint.
   * `dstCapacity` must be at least dx*dy*3.
   */
  size_t toRGB888(int x, int y, int dx, int dy, uint8_t* dst,
                  size_t dstCapacity) const {
    size_t needed = (size_t)dx * (size_t)dy * 3;
    if (dstCapacity < needed) return 0;
    convertFrameToRgb888(decoder_.frame(), x, y, dx, dy, dst);
    return needed;
  }

  /**
   * Copies the most recently decoded picture into a caller-provided
   * buffer as one tightly-packed YUV 4:2:0 planar block (standard
   * "I420" layout: Y bytes, then U bytes, then V bytes, no row padding -
   * unlike y()/u()/v() above, which point into this decoder's own
   * internal frame storage and are only valid until the next picture is
   * decoded). Useful for handing a whole decoded frame to something that
   * wants one contiguous buffer - writing it to storage, sending it over
   * a socket, or queuing it for another thread/task to process later -
   * rather than three separate plane pointers. `dstCapacity` (in
   * uint8_t entries) must be at least width()*height() +
   * 2*(width()/2)*(height()/2); returns 0 without writing if too small,
   * or the number of bytes written on success - the same size-checking
   * pattern as toRGB565()/toRGB666()/toRGB888(). Never allocates. See
   * decoder/h264_frame.h for the exact layout.
   */
  size_t toYUV420(uint8_t* dst, size_t dstCapacity) const {
    int w = width(), h = height();
    size_t needed = (size_t)w * h + 2 * (size_t)(w / 2) * (size_t)(h / 2);
    if (dstCapacity < needed) return 0;
    convertFrameToYuv420(decoder_.frame(), dst);
    return needed;
  }

  /**
   * Windowed/tiled counterpart of toYUV420() - packs just a `dx` x `dy`
   * rectangle starting at (x, y) instead of the whole picture, the same
   * tiling rationale as toRGB565(x,y,dx,dy,...). `x`/`y` must be even
   * (chroma is subsampled 2x) and, together with dx/dy, must stay within
   * width()/height() - not bounds-checked (only the destination buffer
   * size is). `dstCapacity` must be at least dx*dy + 2*(dx/2)*(dy/2).
   */
  size_t toYUV420(int x, int y, int dx, int dy, uint8_t* dst,
                  size_t dstCapacity) const {
    size_t needed = (size_t)dx * dy + 2 * (size_t)(dx / 2) * (size_t)(dy / 2);
    if (dstCapacity < needed) return 0;
    convertFrameToYuv420(decoder_.frame(), x, y, dx, dy, dst);
    return needed;
  }

 protected:
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
      case DecodeStatus::kError:
      default:
        return Status::kError;
    }
  }

  Decoder<Allocator> decoder_;
  FrameCallback callback_ = nullptr;
  void* userData_ = nullptr;
  Status lastStatus_ = Status::kNeedMoreData;
};

}  // namespace tinyh264
