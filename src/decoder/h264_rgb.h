#pragma once
#include <stdint.h>
#include "../common/h264_frame.h"
#include "../common/h264_transform.h"  // clip255

/*
 * Header-only. Converts a decoded YUV 4:2:0 planar picture to RGB565,
 * RGB666, or RGB888 - the pixel formats embedded display libraries
 * (TFT_eSPI, Adafruit_GFX, LovyanGFX, ...) commonly expect. Not part of
 * the decode path itself (H.264 is natively YUV; nothing in
 * decoder/h264_*.h depends on this file) - an optional conversion step
 * for callers that want to hand decoded frames straight to a display.
 *
 * Uses the ITU-R BT.601 *limited-range* YUV-to-RGB conversion (Y in
 * [16,235], Cb/Cr in [16,240]) - the convention real H.264 encoders emit
 * by default (this decoder doesn't parse VUI video_full_range_flag, so
 * limited range is assumed unconditionally, matching the common case).
 * The integer coefficients (298/409/100/208/516, fixed-point scaled by
 * 256) were independently re-derived from the published BT.601 inverse
 * matrix and then cross-checked against ffmpeg's own `-pix_fmt rgb565le`/
 * `-pix_fmt rgb24` conversion of a real decoded frame (see
 * test/native/test_rgb565.cpp, test_rgb888.cpp) rather than trusted from
 * memory alone - confirmed to match ffmpeg to within 1 LSB per 8-bit
 * channel on every pixel of that frame (not bit-exact: ffmpeg's swscale
 * uses 16-bit-precision conversion tables, impractical memory-wise for
 * this library's targets, so a cheaper 8-bit fixed-point formula is used
 * instead - the same one widely used in embedded/mobile YUV->RGB
 * conversion generally). The one shared `yuvToRgb8()` helper below
 * computes this once per pixel; RGB565/RGB666/RGB888 differ only in how
 * they pack the resulting 8-bit R/G/B into the destination buffer.
 *
 * RGB666 output convention (no ffmpeg pixel format to cross-check
 * against, since ffmpeg doesn't have a raw 18-bit-per-pixel format): 3
 * bytes per pixel, each byte holding its 6-bit component *left-justified*
 * in bits 7:2 (bits 1:0 zero) - the wire format real 18-bit-parallel/SPI
 * TFT controllers (e.g. ILI9488 in 18-bit interface mode) expect, since
 * those controllers simply don't drive/read the bottom 2 data lines.
 * Verified by self-consistency instead (test_rgb666.cpp): every RGB666
 * output byte equals the corresponding RGB888 output byte with its
 * bottom 2 bits cleared (`& 0xFC`), since both are computed from the same
 * yuvToRgb8() values and RGB666 is exactly that 8-bit value with 2 bits
 * of precision discarded.
 */

namespace tinyh264 {

/**
 * Computes the 8-bit R/G/B values for one YUV sample (ITU-R BT.601
 * limited range - see file comment). Shared by every pixel-format
 * converter below; only the final packing into the destination buffer
 * differs between RGB565/RGB666/RGB888.
 */
inline void yuvToRgb8(int y, int u, int v, uint8_t* r, uint8_t* g, uint8_t* b) {
  int c = y - 16;
  int d = u - 128;
  int e = v - 128;
  *r = clip255((298 * c + 409 * e + 128) >> 8);
  *g = clip255((298 * c - 100 * d - 208 * e + 128) >> 8);
  *b = clip255((298 * c + 516 * d + 128) >> 8);
}

/**
 * Converts a `width` x `height` rectangle of a YUV 4:2:0 planar frame,
 * starting at (originX, originY) in the source planes, to RGB565 into a
 * caller-provided buffer. `dst` must have room for width*height
 * uint16_t entries, laid out with *no* row padding (dst's own row
 * stride is exactly `width`, distinct from the source planes'
 * strideY/strideC) - this function never allocates, matching this
 * library's philosophy of not silently adding another full-frame-sized
 * buffer for callers who don't need it.
 *
 * Defaults to the whole frame (originX = originY = 0) when called with
 * just the frame's own width/height. Pass a smaller width/height and a
 * non-zero origin to convert only a sub-rectangle instead - e.g. to push
 * a decoded picture to a display in tiles (matching how many
 * small-display drivers - ILI9341, ST7789, ... - already expect
 * windowed pushColors() calls), or to spread the conversion work across
 * multiple loop() iterations instead of one large blocking call.
 *
 * `originX`/`originY` must be even: chroma is subsampled 2x, so an odd
 * origin would split one 2x2 luma block's shared chroma sample across
 * two separate calls, shifting chroma alignment by half a sample at the
 * tile boundary. Not otherwise bounds-checked - caller must ensure the
 * requested rectangle stays within the source frame.
 */
inline void convertYuv420ToRgb565(const uint8_t* yPlane, int strideY,
                                   const uint8_t* uPlane, const uint8_t* vPlane,
                                   int strideC, int width, int height,
                                   uint16_t* dst, int originX = 0,
                                   int originY = 0) {
  for (int row = 0; row < height; row++) {
    int py = originY + row;
    const uint8_t* yRow = yPlane + (size_t)py * strideY;
    const uint8_t* uRow = uPlane + (size_t)(py >> 1) * strideC;
    const uint8_t* vRow = vPlane + (size_t)(py >> 1) * strideC;
    uint16_t* dstRow = dst + (size_t)row * width;
    for (int col = 0; col < width; col++) {
      int px = originX + col;
      uint8_t r, g, b;
      yuvToRgb8(yRow[px], uRow[px >> 1], vRow[px >> 1], &r, &g, &b);
      dstRow[col] =
          (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    }
  }
}

/**
 * Same as convertYuv420ToRgb565(), packing into RGB666 instead: 3 bytes
 * per pixel (R, G, B order), each byte holding its 6-bit component
 * left-justified in bits 7:2 - see the file comment for the exact wire
 * convention and why. `dst` must have room for width*height*3 uint8_t
 * entries, packed with no row padding.
 */
inline void convertYuv420ToRgb666(const uint8_t* yPlane, int strideY,
                                   const uint8_t* uPlane, const uint8_t* vPlane,
                                   int strideC, int width, int height,
                                   uint8_t* dst, int originX = 0,
                                   int originY = 0) {
  for (int row = 0; row < height; row++) {
    int py = originY + row;
    const uint8_t* yRow = yPlane + (size_t)py * strideY;
    const uint8_t* uRow = uPlane + (size_t)(py >> 1) * strideC;
    const uint8_t* vRow = vPlane + (size_t)(py >> 1) * strideC;
    uint8_t* dstRow = dst + (size_t)row * width * 3;
    for (int col = 0; col < width; col++) {
      int px = originX + col;
      uint8_t r, g, b;
      yuvToRgb8(yRow[px], uRow[px >> 1], vRow[px >> 1], &r, &g, &b);
      uint8_t* p = dstRow + col * 3;
      p[0] = (uint8_t)(r & 0xFC);
      p[1] = (uint8_t)(g & 0xFC);
      p[2] = (uint8_t)(b & 0xFC);
    }
  }
}

/**
 * Same as convertYuv420ToRgb565(), packing into RGB888 instead: 3 bytes
 * per pixel (R, G, B order - matches ffmpeg's `rgb24`), full 8-bit
 * precision per channel. `dst` must have room for width*height*3
 * uint8_t entries, packed with no row padding.
 */
inline void convertYuv420ToRgb888(const uint8_t* yPlane, int strideY,
                                   const uint8_t* uPlane, const uint8_t* vPlane,
                                   int strideC, int width, int height,
                                   uint8_t* dst, int originX = 0,
                                   int originY = 0) {
  for (int row = 0; row < height; row++) {
    int py = originY + row;
    const uint8_t* yRow = yPlane + (size_t)py * strideY;
    const uint8_t* uRow = uPlane + (size_t)(py >> 1) * strideC;
    const uint8_t* vRow = vPlane + (size_t)(py >> 1) * strideC;
    uint8_t* dstRow = dst + (size_t)row * width * 3;
    for (int col = 0; col < width; col++) {
      int px = originX + col;
      uint8_t* p = dstRow + col * 3;
      yuvToRgb8(yRow[px], uRow[px >> 1], vRow[px >> 1], &p[0], &p[1], &p[2]);
    }
  }
}

/**
 * Convenience overload taking a decoded Frame directly (see
 * TinyH264Decoder::toRGB565() for the public-API wrapper) - whole frame.
 */
template <typename Allocator>
inline void convertFrameToRgb565(const Frame<Allocator>& frame, uint16_t* dst) {
  convertYuv420ToRgb565(frame.y(), frame.strideY, frame.u(),
                         frame.v(), frame.strideC, frame.width,
                         frame.height, dst);
}

/**
 * Convenience overload taking a decoded Frame directly - a sub-rectangle
 * (see TinyH264Decoder::toRGB565() for the public-API wrapper, and the
 * width/height overload above for the origin/alignment constraints).
 */
template <typename Allocator>
inline void convertFrameToRgb565(const Frame<Allocator>& frame, int originX,
                                  int originY, int width, int height,
                                  uint16_t* dst) {
  convertYuv420ToRgb565(frame.y(), frame.strideY, frame.u(),
                         frame.v(), frame.strideC, width, height, dst,
                         originX, originY);
}

/**
 * Scaled counterpart of the windowed convertFrameToRgb565() above - `x`/
 * `y`/`width`/`height` describe the requested tile *in the scaled output
 * picture's coordinate space* (scaledWidth x scaledHeight overall, see
 * TinyH264Decoder::setScaleFactor()/widthScaled()/heightScaled()), not the
 * decoded picture's own native resolution. Each output pixel is
 * nearest-neighbor sampled from the decoded picture (integer ratio,
 * srcX = outputX * frame.width / scaledWidth, clamped to the last valid
 * source column/row - no floating point, no interpolation, matching
 * this project's simplicity-over-quality conversion philosophy
 * elsewhere). When scaledWidth/scaledHeight equal frame.width/
 * frame.height (scale factor 1.0, the default), this delegates straight
 * to the existing unscaled path above - zero behavior or performance
 * change for callers who never touch setScaleFactor(). `dst` must have
 * room for width*height uint16_t entries; never allocates.
 */
template <typename Allocator>
inline void convertFrameToRgb565(const Frame<Allocator>& frame,
                                  int scaledWidth, int scaledHeight, int x,
                                  int y, int width, int height,
                                  uint16_t* dst) {
  if (scaledWidth == frame.width && scaledHeight == frame.height) {
    convertFrameToRgb565(frame, x, y, width, height, dst);
    return;
  }
  int srcWidth = frame.width;
  int srcHeight = frame.height;
  for (int row = 0; row < height; row++) {
    int py = (int)((int64_t)(y + row) * srcHeight / scaledHeight);
    if (py >= srcHeight) py = srcHeight - 1;
    const uint8_t* yRow = frame.yRow(py);
    const uint8_t* uRow = frame.uRow(py >> 1);
    const uint8_t* vRow = frame.vRow(py >> 1);
    uint16_t* dstRow = dst + (size_t)row * width;
    for (int col = 0; col < width; col++) {
      int px = (int)((int64_t)(x + col) * srcWidth / scaledWidth);
      if (px >= srcWidth) px = srcWidth - 1;
      uint8_t r, g, b;
      yuvToRgb8(yRow[px], uRow[px >> 1], vRow[px >> 1], &r, &g, &b);
      dstRow[col] =
          (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    }
  }
}

/**
 * Whole-picture convenience overload of the scaled/windowed
 * convertFrameToRgb565() above - converts the entire scaledWidth x
 * scaledHeight output picture. `dst` must have room for
 * scaledWidth*scaledHeight uint16_t entries.
 */
template <typename Allocator>
inline void convertFrameToRgb565(const Frame<Allocator>& frame,
                                  int scaledWidth, int scaledHeight,
                                  uint16_t* dst) {
  convertFrameToRgb565(frame, scaledWidth, scaledHeight, 0, 0, scaledWidth,
                        scaledHeight, dst);
}

/**
 * Convenience overload taking a decoded Frame directly (see
 * TinyH264Decoder::toRGB666()) - whole frame.
 */
template <typename Allocator>
inline void convertFrameToRgb666(const Frame<Allocator>& frame, uint8_t* dst) {
  convertYuv420ToRgb666(frame.y(), frame.strideY, frame.u(),
                         frame.v(), frame.strideC, frame.width,
                         frame.height, dst);
}

/**
 * Convenience overload taking a decoded Frame directly - a sub-rectangle
 * (see TinyH264Decoder::toRGB666()).
 */
template <typename Allocator>
inline void convertFrameToRgb666(const Frame<Allocator>& frame, int originX,
                                  int originY, int width, int height,
                                  uint8_t* dst) {
  convertYuv420ToRgb666(frame.y(), frame.strideY, frame.u(),
                         frame.v(), frame.strideC, width, height, dst,
                         originX, originY);
}

/**
 * Scaled counterpart of the windowed convertFrameToRgb666() above - see
 * convertFrameToRgb565()'s scaled overload for the exact semantics
 * (`x`/`y`/`width`/`height` in scaled-output space, nearest-neighbor
 * sampling, unscaled fast path when scaledWidth/scaledHeight match the
 * native picture). `dst` must have room for width*height*3 uint8_t
 * entries.
 */
template <typename Allocator>
inline void convertFrameToRgb666(const Frame<Allocator>& frame,
                                  int scaledWidth, int scaledHeight, int x,
                                  int y, int width, int height,
                                  uint8_t* dst) {
  if (scaledWidth == frame.width && scaledHeight == frame.height) {
    convertFrameToRgb666(frame, x, y, width, height, dst);
    return;
  }
  int srcWidth = frame.width;
  int srcHeight = frame.height;
  for (int row = 0; row < height; row++) {
    int py = (int)((int64_t)(y + row) * srcHeight / scaledHeight);
    if (py >= srcHeight) py = srcHeight - 1;
    const uint8_t* yRow = frame.yRow(py);
    const uint8_t* uRow = frame.uRow(py >> 1);
    const uint8_t* vRow = frame.vRow(py >> 1);
    uint8_t* dstRow = dst + (size_t)row * width * 3;
    for (int col = 0; col < width; col++) {
      int px = (int)((int64_t)(x + col) * srcWidth / scaledWidth);
      if (px >= srcWidth) px = srcWidth - 1;
      uint8_t r, g, b;
      yuvToRgb8(yRow[px], uRow[px >> 1], vRow[px >> 1], &r, &g, &b);
      uint8_t* p = dstRow + col * 3;
      p[0] = (uint8_t)(r & 0xFC);
      p[1] = (uint8_t)(g & 0xFC);
      p[2] = (uint8_t)(b & 0xFC);
    }
  }
}

/**
 * Whole-picture convenience overload of the scaled/windowed
 * convertFrameToRgb666() above - `dst` must have room for
 * scaledWidth*scaledHeight*3 uint8_t entries.
 */
template <typename Allocator>
inline void convertFrameToRgb666(const Frame<Allocator>& frame,
                                  int scaledWidth, int scaledHeight,
                                  uint8_t* dst) {
  convertFrameToRgb666(frame, scaledWidth, scaledHeight, 0, 0, scaledWidth,
                        scaledHeight, dst);
}

/**
 * Convenience overload taking a decoded Frame directly (see
 * TinyH264Decoder::toRGB888()) - whole frame.
 */
template <typename Allocator>
inline void convertFrameToRgb888(const Frame<Allocator>& frame, uint8_t* dst) {
  convertYuv420ToRgb888(frame.y(), frame.strideY, frame.u(),
                         frame.v(), frame.strideC, frame.width,
                         frame.height, dst);
}

/**
 * Convenience overload taking a decoded Frame directly - a sub-rectangle
 * (see TinyH264Decoder::toRGB888()).
 */
template <typename Allocator>
inline void convertFrameToRgb888(const Frame<Allocator>& frame, int originX,
                                  int originY, int width, int height,
                                  uint8_t* dst) {
  convertYuv420ToRgb888(frame.y(), frame.strideY, frame.u(),
                         frame.v(), frame.strideC, width, height, dst,
                         originX, originY);
}

/**
 * Scaled counterpart of the windowed convertFrameToRgb888() above - see
 * convertFrameToRgb565()'s scaled overload for the exact semantics
 * (`x`/`y`/`width`/`height` in scaled-output space, nearest-neighbor
 * sampling, unscaled fast path when scaledWidth/scaledHeight match the
 * native picture). `dst` must have room for width*height*3 uint8_t
 * entries.
 */
template <typename Allocator>
inline void convertFrameToRgb888(const Frame<Allocator>& frame,
                                  int scaledWidth, int scaledHeight, int x,
                                  int y, int width, int height,
                                  uint8_t* dst) {
  if (scaledWidth == frame.width && scaledHeight == frame.height) {
    convertFrameToRgb888(frame, x, y, width, height, dst);
    return;
  }
  int srcWidth = frame.width;
  int srcHeight = frame.height;
  for (int row = 0; row < height; row++) {
    int py = (int)((int64_t)(y + row) * srcHeight / scaledHeight);
    if (py >= srcHeight) py = srcHeight - 1;
    const uint8_t* yRow = frame.yRow(py);
    const uint8_t* uRow = frame.uRow(py >> 1);
    const uint8_t* vRow = frame.vRow(py >> 1);
    uint8_t* dstRow = dst + (size_t)row * width * 3;
    for (int col = 0; col < width; col++) {
      int px = (int)((int64_t)(x + col) * srcWidth / scaledWidth);
      if (px >= srcWidth) px = srcWidth - 1;
      uint8_t* p = dstRow + col * 3;
      yuvToRgb8(yRow[px], uRow[px >> 1], vRow[px >> 1], &p[0], &p[1], &p[2]);
    }
  }
}

/**
 * Whole-picture convenience overload of the scaled/windowed
 * convertFrameToRgb888() above - `dst` must have room for
 * scaledWidth*scaledHeight*3 uint8_t entries.
 */
template <typename Allocator>
inline void convertFrameToRgb888(const Frame<Allocator>& frame,
                                  int scaledWidth, int scaledHeight,
                                  uint8_t* dst) {
  convertFrameToRgb888(frame, scaledWidth, scaledHeight, 0, 0, scaledWidth,
                        scaledHeight, dst);
}

}  // namespace tinyh264
