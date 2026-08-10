#pragma once
#include <stdint.h>
#include "h264_tables.h"

// Header-only. Inverse quantization + inverse transform, ITU-T H.264
// clauses 8.5.9 (4x4 residual scaling), 8.5.10 (luma DC Hadamard + scaling),
// 8.5.11 (chroma DC Hadamard + scaling), 8.5.12.2 (4x4 core inverse
// transform). Only the flat (no scaling-list) case is implemented, which is
// the only case a Baseline-profile stream can signal.

namespace tinyh264 {

/// Table 8-15 (chroma_qp_index_offset already applied and clamped to
/// 0..51 by the caller): maps QPI -> QPC for 8-bit video.
static const uint8_t kChromaQpTable[52] = {
    0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 29, 30,
    31, 32, 32, 33, 34, 34, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38,
    39, 39, 39, 39,
};

/// Derives QP_C from QP_Y (clause 8.5.8): clamps QP_Y + the PPS's
/// chroma_qp_index_offset into 0..51, then looks up Table 8-15.
inline int chromaQp(int lumaQp, int chromaQpIndexOffset) {
  int qpi = lumaQp + chromaQpIndexOffset;
  if (qpi < 0) qpi = 0;
  if (qpi > 51) qpi = 51;
  return kChromaQpTable[qpi];
}

/// Scales a 4x4 block of quantized transform coefficients back to the
/// residual domain, in place (raster order, 16 entries), clause 8.5.9 -
/// the "flat scaling list" (weightScale == 16 everywhere) case, the only
/// one a Baseline-profile stream can signal. If `skipDC` is true,
/// position 0 is left untouched (used for I_16x16 luma AC blocks and all
/// chroma AC blocks, where the DC term instead comes from the separately
/// Hadamard-transformed DC block via dequantLumaDC4x4()/
/// dequantChromaDC2x2()).
inline void dequant4x4(int32_t* c, int qp, bool skipDC) {
  int m = qp % 6;
  int shift = qp / 6;
  for (int idx = 0; idx < 16; idx++) {
    if (skipDC && idx == 0) continue;
    int row = idx >> 2, col = idx & 3;
    int cls = (row & 1) + (col & 1);
    int32_t scale = (int32_t)kNormAdjust4x4[m][cls] * 16;
    if (shift >= 4) {
      c[idx] = c[idx] * scale * (1 << (shift - 4));
    } else {
      c[idx] = (c[idx] * scale + (1 << (3 - shift))) >> (4 - shift);
    }
  }
}

/// 4x4 Walsh-Hadamard transform used for I_16x16 luma DC coefficients, in
/// place (raster order), clause 8.5.10. No rounding: the DC-specific
/// dequant step below (dequantLumaDC4x4()) handles scaling.
inline void hadamard4x4(int32_t* block) {
  int32_t tmp[16];
  for (int i = 0; i < 4; i++) {
    int32_t d0 = block[i + 0 * 4], d1 = block[i + 1 * 4];
    int32_t d2 = block[i + 2 * 4], d3 = block[i + 3 * 4];
    int32_t e0 = d0 + d2, e1 = d0 - d2, e2 = d1 - d3, e3 = d1 + d3;
    tmp[i + 0 * 4] = e0 + e3;
    tmp[i + 1 * 4] = e1 + e2;
    tmp[i + 2 * 4] = e1 - e2;
    tmp[i + 3 * 4] = e0 - e3;
  }
  for (int j = 0; j < 4; j++) {
    int32_t d0 = tmp[j * 4 + 0], d1 = tmp[j * 4 + 1];
    int32_t d2 = tmp[j * 4 + 2], d3 = tmp[j * 4 + 3];
    int32_t e0 = d0 + d2, e1 = d0 - d2, e2 = d1 - d3, e3 = d1 + d3;
    block[j * 4 + 0] = e0 + e3;
    block[j * 4 + 1] = e1 + e2;
    block[j * 4 + 2] = e1 - e2;
    block[j * 4 + 3] = e0 - e3;
  }
}

/// Scales an already-Hadamard-transformed (via hadamard4x4()) 4x4 luma DC
/// block in place, clause 8.5.10. Result feeds into the corresponding
/// AC block's position 0 (see decodeLumaBlockAc() in h264_macroblock.h).
inline void dequantLumaDC4x4(int32_t* f, int qp) {
  int m = qp % 6;
  int shift = qp / 6;
  int32_t scale = (int32_t)kNormAdjust4x4[m][0] * 16;
  for (int i = 0; i < 16; i++) {
    if (shift >= 6) {
      f[i] = f[i] * scale * (1 << (shift - 6));
    } else {
      f[i] = (f[i] * scale + (1 << (5 - shift))) >> (6 - shift);
    }
  }
}

/// 2x2 Walsh-Hadamard transform for chroma DC coefficients (4:2:0), in
/// place, raster order {c00, c01, c10, c11}, clause 8.5.11.
inline void hadamard2x2(int32_t* f) {
  int32_t c00 = f[0], c01 = f[1], c10 = f[2], c11 = f[3];
  f[0] = c00 + c01 + c10 + c11;
  f[1] = c00 - c01 + c10 - c11;
  f[2] = c00 + c01 - c10 - c11;
  f[3] = c00 - c01 - c10 + c11;
}

/// Scales an already-Hadamard-transformed (via hadamard2x2()) 2x2 chroma
/// DC block in place, clause 8.5.11, using the chroma QP (see chromaQp()).
inline void dequantChromaDC2x2(int32_t* f, int chromaQp) {
  int m = chromaQp % 6;
  int shift = chromaQp / 6;
  int32_t scale = (int32_t)kNormAdjust4x4[m][0] * 16;
  for (int i = 0; i < 4; i++) {
    f[i] = (f[i] * scale * (1 << shift)) >> 5;
  }
}

/// 4x4 inverse core transform, in place (raster order), clause 8.5.12.2.
/// Includes the final (+32) >> 6 rounding/normalization, so the output is
/// directly the spatial-domain residual ready for addResidual4x4().
inline void idct4x4(int32_t* block) {
  int32_t tmp[16];
  for (int i = 0; i < 4; i++) {
    int32_t d0 = block[i + 0 * 4], d1 = block[i + 1 * 4];
    int32_t d2 = block[i + 2 * 4], d3 = block[i + 3 * 4];
    int32_t e0 = d0 + d2, e1 = d0 - d2;
    int32_t e2 = (d1 >> 1) - d3, e3 = d1 + (d3 >> 1);
    tmp[i + 0 * 4] = e0 + e3;
    tmp[i + 1 * 4] = e1 + e2;
    tmp[i + 2 * 4] = e1 - e2;
    tmp[i + 3 * 4] = e0 - e3;
  }
  for (int j = 0; j < 4; j++) {
    int32_t d0 = tmp[j * 4 + 0], d1 = tmp[j * 4 + 1];
    int32_t d2 = tmp[j * 4 + 2], d3 = tmp[j * 4 + 3];
    int32_t e0 = d0 + d2, e1 = d0 - d2;
    int32_t e2 = (d1 >> 1) - d3, e3 = d1 + (d3 >> 1);
    block[j * 4 + 0] = (e0 + e3 + 32) >> 6;
    block[j * 4 + 1] = (e1 + e2 + 32) >> 6;
    block[j * 4 + 2] = (e1 - e2 + 32) >> 6;
    block[j * 4 + 3] = (e0 - e3 + 32) >> 6;
  }
}

/// Clip1_Y/Clip1_C (clause 8.5.12.1, and generally "Clip1" throughout the
/// spec for 8-bit video): clamps a signed sample value into the valid
/// [0,255] pixel range.
inline uint8_t clip255(int32_t v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}

/// Adds a 4x4 residual block (raster order, as produced by idct4x4()) to
/// an 8-bit prediction block already sitting at dst (stride `stride`),
/// clipping to [0,255] - the "prediction + residual" reconstruction step
/// common to clause 8.3 (intra) and 8.4 (inter) sample reconstruction.
inline void addResidual4x4(uint8_t* dst, int stride, const int32_t* residual) {
  for (int y = 0; y < 4; y++) {
    for (int x = 0; x < 4; x++) {
      dst[y * stride + x] =
          clip255((int32_t)dst[y * stride + x] + residual[y * 4 + x]);
    }
  }
}

}  // namespace tinyh264
