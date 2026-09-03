#pragma once
#include <stdint.h>
#include "../common/h264_frame.h"
#include "../common/h264_transform.h"  // clip255

/*
 * Header-only. Inter prediction support: motion vector median prediction
 * (clause 8.4.1.3) and motion compensation / sub-pel interpolation
 * (clause 8.4.2.2). Luma uses quarter-sample precision (6-tap FIR for
 * half-sample positions, then bilinear averaging for quarter-sample
 * positions); chroma (4:2:0) uses eighth-sample bilinear interpolation
 * directly, reusing the luma MV value unscaled (clause 8.4.1.4: for 4:2:0,
 * non-interlaced, mvCLX == mvLX).
 */

namespace tinyh264 {

/**
 * Clamped (edge-replicated) full-pel sample fetch - motion vectors can
 * legally point outside the picture, and the spec requires the picture
 * border to be treated as extended by replicating the edge samples
 * (clause 8.4.2.2.1, "the sample value at that location shall be
 * substituted for the value at the boundary").
 */
inline int clampedSample(const uint8_t* plane, int stride, int width,
                          int height, int x, int y) {
  if (x < 0) x = 0;
  if (x >= width) x = width - 1;
  if (y < 0) y = 0;
  if (y >= height) y = height - 1;
  return plane[(size_t)y * stride + x];
}

/**
 * The 6-tap FIR filter kernel [1,-5,20,20,-5,1] used for every half-pel
 * luma interpolation, clause 8.4.2.2.1.
 */
inline int tap6(int a, int b, int c, int d, int e, int f) {
  return a - 5 * b + 20 * c + 20 * d - 5 * e + f;
}

/**
 * Raw (unclipped, unrounded) horizontal 6-tap sum centered between (x,y)
 * and (x+1,y) - the "b" half-pel position's pre-normalization value.
 */
inline int rawHalfH(const uint8_t* plane, int stride, int w, int h, int x,
                     int y) {
  return tap6(clampedSample(plane, stride, w, h, x - 2, y),
              clampedSample(plane, stride, w, h, x - 1, y),
              clampedSample(plane, stride, w, h, x, y),
              clampedSample(plane, stride, w, h, x + 1, y),
              clampedSample(plane, stride, w, h, x + 2, y),
              clampedSample(plane, stride, w, h, x + 3, y));
}
/**
 * Raw (unclipped, unrounded) vertical 6-tap sum centered between (x,y)
 * and (x,y+1) - the "h" half-pel position's pre-normalization value.
 */
inline int rawHalfV(const uint8_t* plane, int stride, int w, int h, int x,
                     int y) {
  return tap6(clampedSample(plane, stride, w, h, x, y - 2),
              clampedSample(plane, stride, w, h, x, y - 1),
              clampedSample(plane, stride, w, h, x, y),
              clampedSample(plane, stride, w, h, x, y + 1),
              clampedSample(plane, stride, w, h, x, y + 2),
              clampedSample(plane, stride, w, h, x, y + 3));
}
/**
 * Normalized, clipped horizontal half-pel luma sample "b" ((sum+16)>>5,
 * clause 8.4.2.2.1).
 */
inline int halfH(const uint8_t* plane, int stride, int w, int h, int x,
                  int y) {
  return clip255((rawHalfH(plane, stride, w, h, x, y) + 16) >> 5);
}
/// Normalized, clipped vertical half-pel luma sample "h" ((sum+16)>>5).
inline int halfV(const uint8_t* plane, int stride, int w, int h, int x,
                  int y) {
  return clip255((rawHalfV(plane, stride, w, h, x, y) + 16) >> 5);
}
/**
 * Rounded average of two samples - the bilinear step used for quarter-pel
 * luma positions once the nearest half-pel neighbor(s) are known.
 */
inline int avg2(int a, int b) { return (a + b + 1) >> 1; }

/**
 * Center ("j") half-pel luma sample: a second 6-tap pass over the
 * *unrounded* horizontal half-pel sums at 6 consecutive rows (equivalent
 * to doing it over the vertical sums instead - the spec's formula is
 * symmetric), clause 8.4.2.2.1.
 */
inline int centerJ(const uint8_t* plane, int stride, int w, int h, int x,
                    int y) {
  int s0 = rawHalfH(plane, stride, w, h, x, y - 2);
  int s1 = rawHalfH(plane, stride, w, h, x, y - 1);
  int s2 = rawHalfH(plane, stride, w, h, x, y);
  int s3 = rawHalfH(plane, stride, w, h, x, y + 1);
  int s4 = rawHalfH(plane, stride, w, h, x, y + 2);
  int s5 = rawHalfH(plane, stride, w, h, x, y + 3);
  return clip255((tap6(s0, s1, s2, s3, s4, s5) + 512) >> 10);
}

/**
 * Interpolates one luma sample at full-pel base (x0,y0) plus quarter-pel
 * fractional offset (fracX,fracY in 0..3), clause 8.4.2.2.1 / Figure 8-4's
 * labeled quarter/half-sample positions (G=integer, b/h/j=half-pel,
 * a/c/d/n/f/i/k/q/e/g/p/r=quarter-pel, named per the spec figure and
 * referenced by those single-letter names in the comments below).
 */
inline uint8_t interpLumaSample(const uint8_t* plane, int stride, int w,
                                 int h, int x0, int y0, int fracX,
                                 int fracY) {
  if (fracX == 0 && fracY == 0) {
    return (uint8_t)clampedSample(plane, stride, w, h, x0, y0);
  }
  int G = clampedSample(plane, stride, w, h, x0, y0);
  if (fracY == 0) {
    int b = halfH(plane, stride, w, h, x0, y0);
    if (fracX == 2) return (uint8_t)b;
    int H = clampedSample(plane, stride, w, h, x0 + 1, y0);
    return (uint8_t)(fracX == 1 ? avg2(G, b) : avg2(b, H));
  }
  if (fracX == 0) {
    int hh = halfV(plane, stride, w, h, x0, y0);
    if (fracY == 2) return (uint8_t)hh;
    int M = clampedSample(plane, stride, w, h, x0, y0 + 1);
    return (uint8_t)(fracY == 1 ? avg2(G, hh) : avg2(hh, M));
  }
  if (fracX == 2 && fracY == 2) {
    return (uint8_t)centerJ(plane, stride, w, h, x0, y0);
  }
  int b = halfH(plane, stride, w, h, x0, y0);          // top row, horiz half
  int s = halfH(plane, stride, w, h, x0, y0 + 1);       // bottom row, horiz half
  int hCol = halfV(plane, stride, w, h, x0, y0);        // left col, vert half
  int m = halfV(plane, stride, w, h, x0 + 1, y0);       // right col, vert half
  if (fracX == 2) {
    /*
     * f (fracY==1): average the top-row half b with center j.
     * q (fracY==3): average the bottom-row half s with center j.
     */
    int j = centerJ(plane, stride, w, h, x0, y0);
    return (uint8_t)avg2(fracY == 1 ? b : s, j);
  }
  if (fracY == 2) {
    /*
     * i (fracX==1): average the left-col half h with center j.
     * k (fracX==3): average the right-col half m with center j.
     */
    int j = centerJ(plane, stride, w, h, x0, y0);
    return (uint8_t)avg2(fracX == 1 ? hCol : m, j);
  }
  /*
   * Diagonal quarter positions e,g,p,r: average the nearest horizontal-half
   * and vertical-half samples.
   */
  if (fracX == 1 && fracY == 1) return (uint8_t)avg2(hCol, b);  // e
  if (fracX == 3 && fracY == 1) return (uint8_t)avg2(b, m);     // g
  if (fracX == 1 && fracY == 3) return (uint8_t)avg2(hCol, s);  // p
  return (uint8_t)avg2(m, s);                                   // r
}

/**
 * Motion-compensates a wxh luma block from `ref` into `dst` (dst has its
 * own stride, e.g. the current frame's luma stride at some MB offset),
 * clause 8.4.2.2. mvX/mvY are in quarter-luma-sample units; originX/originY
 * are the block's integer top-left position in the current picture.
 */
inline void motionCompLuma(uint8_t* dst, int dstStride, const Frame& ref,
                            int originX, int originY, int blockW, int blockH,
                            int mvX, int mvY) {
  int fullX = originX + (mvX >> 2);
  int fullY = originY + (mvY >> 2);
  int fracX = mvX & 3;
  int fracY = mvY & 3;
  for (int y = 0; y < blockH; y++) {
    for (int x = 0; x < blockW; x++) {
      dst[y * dstStride + x] =
          interpLumaSample(ref.y(), ref.strideY, ref.width, ref.height,
                            fullX + x, fullY + y, fracX, fracY);
    }
  }
}

/**
 * Motion-compensates a wxh chroma block (bilinear, eighth-pel), for one
 * plane (Cb or Cr), clause 8.4.2.2.2. mvX/mvY are the *luma* MV
 * (quarter-pel); for 4:2:0 non-interlaced this is reused directly as an
 * eighth-pel chroma MV (clause 8.4.1.4: mvCLX == mvLX, since ChromaArrayType
 * == 1 makes the horizontal/vertical scaling factors both 2, matching the
 * quarter-to-eighth-pel unit change exactly).
 */
inline void motionCompChroma(uint8_t* dst, int dstStride,
                              const uint8_t* refPlane, int refStride,
                              int refW, int refH, int originX, int originY,
                              int blockW, int blockH, int mvX, int mvY) {
  int fullX = originX + (mvX >> 3);
  int fullY = originY + (mvY >> 3);
  int fracX = mvX & 7;
  int fracY = mvY & 7;
  for (int y = 0; y < blockH; y++) {
    for (int x = 0; x < blockW; x++) {
      int A = clampedSample(refPlane, refStride, refW, refH, fullX + x,
                             fullY + y);
      int B = clampedSample(refPlane, refStride, refW, refH, fullX + x + 1,
                             fullY + y);
      int C = clampedSample(refPlane, refStride, refW, refH, fullX + x,
                             fullY + y + 1);
      int D = clampedSample(refPlane, refStride, refW, refH, fullX + x + 1,
                             fullY + y + 1);
      int v = ((8 - fracX) * (8 - fracY) * A + fracX * (8 - fracY) * B +
                (8 - fracX) * fracY * C + fracX * fracY * D + 32) >>
               6;
      dst[y * dstStride + x] = (uint8_t)v;
    }
  }
}

// --- Motion vector prediction (clause 8.4.1.3) ------------------------

/**
 * Median of three integers, computed without branching on the middle
 * value directly (sum minus the min minus the max) - used component-wise
 * by predictMv() for the neighbor-A/B/C motion vector median.
 */
inline int median3(int a, int b, int c) {
  return a + b + c - (a < b ? (a < c ? a : c) : (b < c ? b : c)) -
         (a > b ? (a > c ? a : c) : (b > c ? b : c));
}

/**
 * The three spatial neighbor motion vectors (A=left, B=top, C=top-right
 * or top-left fallback) feeding clause 8.4.1.3's median predictor, plus
 * per-neighbor availability flags. Built by the caller
 * (predictMvGeneral() in h264_macroblock_inter.h) from the current
 * partition's actual neighbor lookup before calling predictMv().
 */
struct MvPredictorInput {
  bool haveA = false, haveB = false, haveC = false;
  int16_t mvA[2] = {0, 0}, mvB[2] = {0, 0}, mvC[2] = {0, 0};
};

/**
 * Plain component-wise median of A/B/C (clause 8.4.1.3.1's general case).
 * Unavailable/Intra neighbors contribute (0,0), per clause 8.4.1.3.2. The
 * "exactly one neighbor has a matching ref_idx" and 16x8/8x16 directional
 * shortcuts that can bypass this median entirely are resolved by the
 * caller (predictMvGeneral() in h264_macroblock_inter.h) before falling
 * back to this function.
 */
inline void predictMv(const MvPredictorInput& in, int16_t* outMv) {
  int ax = in.haveA ? in.mvA[0] : 0, ay = in.haveA ? in.mvA[1] : 0;
  int bx = in.haveB ? in.mvB[0] : 0, by = in.haveB ? in.mvB[1] : 0;
  int cx = in.haveC ? in.mvC[0] : 0, cy = in.haveC ? in.mvC[1] : 0;
  outMv[0] = (int16_t)median3(ax, bx, cx);
  outMv[1] = (int16_t)median3(ay, by, cy);
}

}  // namespace tinyh264
