#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../common/h264_transform.h"  // clip255()

/*
 * Header-only. Packed-pixel-format -> YUV 4:2:0 planar conversion, for
 * encodeFrame*() callers whose source data isn't already three separate
 * Y/U/V planes (e.g. a camera module's native YUV422/RGB565 output, or an
 * RGB888/RGB666 framebuffer). This is the *forward* direction (encoder
 * input) - the mirror image of decoder/h264_rgb.h's YUV->RGB output
 * conversion, not a reuse of it (a genuinely different operation, so it
 * lives in encoder/ rather than common/ - nothing on the decode side
 * needs RGB/YUV422 -> YUV420 conversion).
 *
 * The RGB<->YUV integer coefficients (66/129/25, -38/-74/112, 112/-94/-18,
 * fixed-point scaled by 256) are the standard ITU-R BT.601 limited-range
 * *forward* matrix - the algebraic inverse of decoder/h264_rgb.h's
 * yuvToRgb8() (298/409/100/208/516), which was itself independently
 * re-derived and cross-checked against real ffmpeg output; round-trip
 * self-consistency (convert known RGB -> this file's YUV420 -> back
 * through yuvToRgb8()) and a real ffmpeg `-pix_fmt rgb24` -> `-pix_fmt
 * yuv420p` cross-check are this file's own verification (see
 * test/native/test_color_convert.cpp) rather than trusted from memory.
 *
 * Chroma is derived from a 2x2 (RGB888/666/565) or 1x2 (YUV422, already
 * horizontally subsampled) block average before the RGB->YUV matrix is
 * applied, not by subsampling a single corner pixel - cheap (a handful
 * of extra adds) and meaningfully reduces color aliasing at chroma
 * edges compared to picking one representative pixel.
 */

namespace tinyh264 {

/// BT.601 limited-range forward luma - see file comment for the matrix.
inline uint8_t rgb8ToY(int r, int g, int b) {
  return clip255(((66 * r + 129 * g + 25 * b + 128) >> 8) + 16);
}

/// BT.601 limited-range forward chroma - see file comment for the matrix.
inline void rgb8ToUv(int r, int g, int b, uint8_t* u, uint8_t* v) {
  *u = clip255(((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128);
  *v = clip255(((112 * r - 94 * g - 18 * b + 128) >> 8) + 128);
}

/**
 * Converts an RGB888 (3 bytes/pixel, R/G/B order - matches decoder's
 * toRGB888() convention) `width` x `height` image into YUV 4:2:0 planar,
 * into caller-provided Y/U/V planes (never allocates). `rgbStride` is in
 * bytes (>= width*3); `dstStrideY`/`dstStrideC` are the destination
 * planes' own row strides. `width`/`height` need not be even for this
 * function itself, but the encoder they feed requires multiples of 16 -
 * see TinyH264Encoder::encodeFrameRgb888().
 */
inline void convertRgb888ToYuv420(const uint8_t* rgb, int rgbStride,
                                   int width, int height, uint8_t* dstY,
                                   int dstStrideY, uint8_t* dstU,
                                   uint8_t* dstV, int dstStrideC) {
  for (int y = 0; y < height; y++) {
    const uint8_t* row = rgb + (size_t)y * rgbStride;
    uint8_t* yRow = dstY + (size_t)y * dstStrideY;
    for (int x = 0; x < width; x++) {
      const uint8_t* p = row + x * 3;
      yRow[x] = rgb8ToY(p[0], p[1], p[2]);
    }
  }
  for (int cy = 0; cy < height / 2; cy++) {
    uint8_t* uRow = dstU + (size_t)cy * dstStrideC;
    uint8_t* vRow = dstV + (size_t)cy * dstStrideC;
    for (int cx = 0; cx < width / 2; cx++) {
      int sr = 0, sg = 0, sb = 0;
      for (int dy = 0; dy < 2; dy++) {
        const uint8_t* p =
            rgb + (size_t)(cy * 2 + dy) * rgbStride + (size_t)(cx * 2) * 3;
        sr += p[0] + p[3];
        sg += p[1] + p[4];
        sb += p[2] + p[5];
      }
      rgb8ToUv((sr + 2) / 4, (sg + 2) / 4, (sb + 2) / 4, &uRow[cx], &vRow[cx]);
    }
  }
}

/**
 * Converts an RGB666 (3 bytes/pixel, each byte's 6 significant bits
 * left-justified in bits 7:2 - matches decoder's toRGB666() convention)
 * image into YUV 4:2:0 planar. Each byte is expanded back to a full
 * 8-bit value by replicating its top 6 bits into the 2 empty low bits
 * (`(v & 0xFC) | (v >> 6)`, the standard bit-replication upscale - the
 * same technique convertRgb565ToYuv420() uses for 5/6-bit channels)
 * before applying the same matrix convertRgb888ToYuv420() uses.
 */
inline void convertRgb666ToYuv420(const uint8_t* rgb666, int rgbStride,
                                   int width, int height, uint8_t* dstY,
                                   int dstStrideY, uint8_t* dstU,
                                   uint8_t* dstV, int dstStrideC) {
  auto expand6 = [](uint8_t v) -> int { return (v & 0xFC) | (v >> 6); };
  for (int y = 0; y < height; y++) {
    const uint8_t* row = rgb666 + (size_t)y * rgbStride;
    uint8_t* yRow = dstY + (size_t)y * dstStrideY;
    for (int x = 0; x < width; x++) {
      const uint8_t* p = row + x * 3;
      yRow[x] = rgb8ToY(expand6(p[0]), expand6(p[1]), expand6(p[2]));
    }
  }
  for (int cy = 0; cy < height / 2; cy++) {
    uint8_t* uRow = dstU + (size_t)cy * dstStrideC;
    uint8_t* vRow = dstV + (size_t)cy * dstStrideC;
    for (int cx = 0; cx < width / 2; cx++) {
      int sr = 0, sg = 0, sb = 0;
      for (int dy = 0; dy < 2; dy++) {
        const uint8_t* p = rgb666 + (size_t)(cy * 2 + dy) * rgbStride +
                            (size_t)(cx * 2) * 3;
        sr += expand6(p[0]) + expand6(p[3]);
        sg += expand6(p[1]) + expand6(p[4]);
        sb += expand6(p[2]) + expand6(p[5]);
      }
      rgb8ToUv((sr + 2) / 4, (sg + 2) / 4, (sb + 2) / 4, &uRow[cx], &vRow[cx]);
    }
  }
}

/**
 * Converts an RGB565 (uint16_t/pixel, 5-6-5 packed - matches decoder's
 * toRGB565() convention: `(r&0xF8)<<8 | (g&0xFC)<<3 | b>>3`) image into
 * YUV 4:2:0 planar. Each 5/6-bit channel is expanded to 8 bits by
 * replicating its top bits into the low bits it doesn't have
 * (`(v5<<3)|(v5>>2)` / `(v6<<2)|(v6>>4)` - the standard bit-replication
 * upscale, which maps the channel's full 0..31/0..63 range evenly across
 * 0..255 rather than leaving the low bits at 0). `rgbStride` is in
 * uint16_t entries (>= width), not bytes.
 */
inline void convertRgb565ToYuv420(const uint16_t* rgb565, int rgbStride,
                                   int width, int height, uint8_t* dstY,
                                   int dstStrideY, uint8_t* dstU,
                                   uint8_t* dstV, int dstStrideC) {
  auto r8 = [](uint16_t px) -> int {
    int v = (px >> 11) & 0x1F;
    return (v << 3) | (v >> 2);
  };
  auto g8 = [](uint16_t px) -> int {
    int v = (px >> 5) & 0x3F;
    return (v << 2) | (v >> 4);
  };
  auto b8 = [](uint16_t px) -> int {
    int v = px & 0x1F;
    return (v << 3) | (v >> 2);
  };
  for (int y = 0; y < height; y++) {
    const uint16_t* row = rgb565 + (size_t)y * rgbStride;
    uint8_t* yRow = dstY + (size_t)y * dstStrideY;
    for (int x = 0; x < width; x++) {
      yRow[x] = rgb8ToY(r8(row[x]), g8(row[x]), b8(row[x]));
    }
  }
  for (int cy = 0; cy < height / 2; cy++) {
    uint8_t* uRow = dstU + (size_t)cy * dstStrideC;
    uint8_t* vRow = dstV + (size_t)cy * dstStrideC;
    for (int cx = 0; cx < width / 2; cx++) {
      int sr = 0, sg = 0, sb = 0;
      for (int dy = 0; dy < 2; dy++) {
        const uint16_t* row = rgb565 + (size_t)(cy * 2 + dy) * rgbStride;
        for (int dx = 0; dx < 2; dx++) {
          uint16_t px = row[cx * 2 + dx];
          sr += r8(px);
          sg += g8(px);
          sb += b8(px);
        }
      }
      rgb8ToUv((sr + 2) / 4, (sg + 2) / 4, (sb + 2) / 4, &uRow[cx], &vRow[cx]);
    }
  }
}

/**
 * Converts a YUYV-order packed YUV 4:2:2 (Y0 U0 Y1 V0 per horizontal
 * pixel pair - the common camera-module convention, e.g. OV2640/OV7670)
 * image into YUV 4:2:0 planar. Luma is a direct copy (4:2:2 already has
 * full-resolution luma); chroma is already horizontally subsampled the
 * same way 4:2:0 needs (one U/V pair per 2 horizontal pixels) but has
 * *full* vertical chroma resolution, so this function's only real work
 * is averaging each vertically-adjacent row pair's U/V down to 4:2:0's
 * halved vertical resolution. `yuyvStride` is in bytes (>= width*2).
 */
inline void convertYuyv422ToYuv420(const uint8_t* yuyv, int yuyvStride,
                                    int width, int height, uint8_t* dstY,
                                    int dstStrideY, uint8_t* dstU,
                                    uint8_t* dstV, int dstStrideC) {
  for (int y = 0; y < height; y++) {
    const uint8_t* row = yuyv + (size_t)y * yuyvStride;
    uint8_t* yRow = dstY + (size_t)y * dstStrideY;
    for (int x = 0; x < width; x++) {
      yRow[x] = row[x * 2];  // Y0 at even byte offsets, Y1 at odd - either
                              // way it's byte 2*x within the YUYV quad
    }
  }
  for (int cy = 0; cy < height / 2; cy++) {
    const uint8_t* row0 = yuyv + (size_t)(cy * 2) * yuyvStride;
    const uint8_t* row1 = yuyv + (size_t)(cy * 2 + 1) * yuyvStride;
    uint8_t* uRow = dstU + (size_t)cy * dstStrideC;
    uint8_t* vRow = dstV + (size_t)cy * dstStrideC;
    for (int cx = 0; cx < width / 2; cx++) {
      int u0 = row0[cx * 4 + 1], v0 = row0[cx * 4 + 3];
      int u1 = row1[cx * 4 + 1], v1 = row1[cx * 4 + 3];
      uRow[cx] = (uint8_t)((u0 + u1 + 1) >> 1);
      vRow[cx] = (uint8_t)((v0 + v1 + 1) >> 1);
    }
  }
}

}  // namespace tinyh264
