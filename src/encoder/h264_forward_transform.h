#pragma once
#include <stdint.h>
#include "../common/h264_tables.h"
#include "../common/h264_transform.h"

// Header-only. Forward (encoder-side) transform + quantization, the
// counterpart to h264_transform.h's inverse/dequant path. ITU-T H.264
// clauses 8.5.4/8.5.6/8.5.11 are informative (only the decoder's inverse
// is normative), so an encoder has real design freedom here - this file's
// design choice is to reuse h264_transform.h's *already-verified*
// (pixel-exact against real ffmpeg decodes, this whole project's
// methodology) hadamard4x4()/hadamard2x2() directly for the DC paths
// (both are their own inverse) and quantize against the exact same
// kNormAdjust4x4 table dequant4x4()/dequantLumaDC4x4()/
// dequantChromaDC2x2() already use, rather than porting a whole separate
// quantization table from scratch. The core 4x4 transform
// (forwardDct4x4()) is a genuine exception - it's *not* self-inverse, so
// it's ported from real x264 source (common/dct.c, sub4x4_dct()) instead.
//
// A naive "just algebraically invert the dequant formula" derivation of
// each quantize*() function's scale factor is NOT trustworthy on its
// own here: two of the three (quantizeBlock4x4(), quantizeLumaDC4x4())
// initially got a plausible-looking-but-wrong power-of-2 factor this way
// during development, because dequant4x4()/dequantLumaDC4x4()/
// dequantChromaDC2x2()'s own scales were calibrated against a *specific*
// forward-transform gain convention, and it's easy to silently assume
// the wrong one. Every scale factor below was instead pinned down by a
// controlled round-trip test - a spatially-flat block/macroblock, so
// only one transform coefficient is ever nonzero and the exact gain is
// unambiguous - then cross-checked against real image data and (see
// test/native/test_encode_iframe.cpp) a real ffmpeg decode of actual
// encoder output. Trust the empirical calibration notes on each function
// over a fresh re-derivation from the dequant formulas alone.

namespace tinyh264 {

/// Computes the 4x4 residual (src - pred) in place-into-`out`, raster
/// order - the encoder-side inverse of addResidual4x4()
/// (h264_transform.h).
inline void subtractBlock4x4(const uint8_t* src, int srcStride,
                              const uint8_t* pred, int predStride,
                              int32_t* out) {
  for (int y = 0; y < 4; y++) {
    for (int x = 0; x < 4; x++) {
      out[y * 4 + x] =
          (int32_t)src[y * srcStride + x] - (int32_t)pred[y * predStride + x];
    }
  }
}

/// Forward 4x4 core transform, in place (raster order) - the standard
/// H.264 encoder-side integer DCT approximation, same butterfly
/// coefficients as x264's sub4x4_dct() (common/dct.c), the well-
/// established reference implementation. The *axis order* of the two
/// 1-D passes below deliberately does NOT match x264's own indexing
/// (x264's first pass reads a row and scatters into a column of `tmp`,
/// an implicit-transpose trick paired with x264's own idct convention) -
/// it's instead written to match idct4x4()'s (h264_transform.h) column-
/// then-row axis order exactly (first pass reads/writes column i, second
/// pass reads/writes row i), which is what this codebase's *own*,
/// already ffmpeg-verified idct4x4() expects of its algebraic inverse.
/// (Row-then-column vs column-then-row give the same 2-D transform
/// mathematically, but the intermediate `tmp` layout - and therefore
/// which pairing correctly round-trips - depends on matching axis order
/// with whichever idct4x4 you pair it with; mixing x264's convention
/// with this file's own idct4x4 produced a real, empirically-caught
/// scale/transpose mismatch during development - see the derivation
/// note in quantizeBlock4x4() below and test/native/test_encode_iframe.cpp
/// for the round-trip verification this was checked against.)
inline void forwardDct4x4(int32_t* block) {
  int32_t tmp[16];
  // Pass 1: transform each column (matches idct4x4()'s first pass axis).
  for (int i = 0; i < 4; i++) {
    int32_t d0 = block[i + 0 * 4], d1 = block[i + 1 * 4];
    int32_t d2 = block[i + 2 * 4], d3 = block[i + 3 * 4];
    int32_t s03 = d0 + d3, s12 = d1 + d2;
    int32_t d03 = d0 - d3, d12 = d1 - d2;
    tmp[i + 0 * 4] = s03 + s12;
    tmp[i + 1 * 4] = 2 * d03 + d12;
    tmp[i + 2 * 4] = s03 - s12;
    tmp[i + 3 * 4] = d03 - 2 * d12;
  }
  // Pass 2: transform each row (matches idct4x4()'s second pass axis).
  for (int i = 0; i < 4; i++) {
    int32_t d0 = tmp[i * 4 + 0], d1 = tmp[i * 4 + 1];
    int32_t d2 = tmp[i * 4 + 2], d3 = tmp[i * 4 + 3];
    int32_t s03 = d0 + d3, s12 = d1 + d2;
    int32_t d03 = d0 - d3, d12 = d1 - d2;
    block[i * 4 + 0] = s03 + s12;
    block[i * 4 + 1] = 2 * d03 + d12;
    block[i * 4 + 2] = s03 - s12;
    block[i * 4 + 3] = d03 - 2 * d12;
  }
}

/// Round-to-nearest (ties away from zero) integer division, num / denom
/// with denom > 0 - the building block every quantize*() function below
/// uses to invert its dequant*() counterpart.
inline int32_t roundDivPositive(int32_t num, int32_t denom) {
  if (num >= 0) return (num + denom / 2) / denom;
  return -((-num + denom / 2) / denom);
}

/// Quantizes one forward-transformed (forwardDct4x4()) 4x4 block in
/// place, raster order - paired with dequant4x4() (h264_transform.h) as
/// its round-trip inverse. dequant4x4() computes d ~= c * normAdjust[m][cls]
/// * 2^shift (m = qp%6, shift = qp/6, cls = (row&1)+(col&1)); naively
/// inverting that alone (round(coeff / (normAdjust*2^shift)), no extra
/// factor) undershot by a real, empirically-caught 4x on a controlled
/// round-trip test (a flat 4x4 block: forwardDct4x4() has its own gain of
/// 16x for constant input, and idct4x4()'s final >>6 divides by 64 -
/// their product only nets out to unity gain, matching dequant4x4()'s
/// intended scale, with this extra *4 included) - so the quantized level
/// is round(coeff * 4 / (normAdjust[m][cls] * 2^shift)).
/// `skipDC` mirrors dequant4x4()'s: leaves position 0 untouched (used for
/// I_16x16 luma AC blocks and all chroma AC blocks, whose DC term is
/// coded separately via the Hadamard DC block instead). Returns true if
/// any coefficient (excluding a skipped DC) quantized to non-zero -
/// mirrors mb_type/coded_block_pattern's "does this block need coding"
/// question the macroblock layer needs.
inline bool quantizeBlock4x4(int32_t* c, int qp, bool skipDC) {
  int m = qp % 6;
  int shift = qp / 6;
  bool nonzero = false;
  for (int idx = 0; idx < 16; idx++) {
    if (skipDC && idx == 0) continue;
    int row = idx >> 2, col = idx & 3;
    int cls = (row & 1) + (col & 1);
    int32_t denom = (int32_t)kNormAdjust4x4[m][cls] << shift;
    c[idx] = roundDivPositive(c[idx] * 4, denom);
    if (c[idx] != 0) nonzero = true;
  }
  return nonzero;
}

/// Quantizes an already-Hadamard-transformed (hadamard4x4(), applied to
/// the 16 blocks' own forwardDct4x4() position-0 outputs, per clause
/// 8.5.6) luma DC block in place - the inverse of dequantLumaDC4x4().
/// Empirically calibrated (not hand-derived from dequantLumaDC4x4()'s
/// formula in isolation, which gave a factor that overshot by 4x on a
/// controlled round-trip test - hadamard4x4() applied twice, forward
/// then inverse, has its own combined gain of 16x for any input by
/// construction (H^2 = 16I for this specific unnormalized 4x4 Hadamard),
/// which dequantLumaDC4x4()'s scale must already account for jointly
/// with this function; round(coeff / (normAdjust[m][0] * 2^shift)) -
/// the same denominator dequant4x4() would use at cls=0, no extra
/// numerator scaling - is what a real controlled test (a flat 16x16 MB,
/// where every DC coefficient is identical, isolating pure transform
/// gain from cross-position noise) confirmed reconstructs correctly).
inline bool quantizeLumaDC4x4(int32_t* f, int qp) {
  int m = qp % 6;
  int shift = qp / 6;
  int32_t denom = (int32_t)kNormAdjust4x4[m][0] << shift;
  bool nonzero = false;
  for (int i = 0; i < 16; i++) {
    f[i] = roundDivPositive(f[i], denom);
    if (f[i] != 0) nonzero = true;
  }
  return nonzero;
}

/// Quantizes an already-Hadamard-transformed (hadamard2x2(), applied to
/// the 4 chroma 4x4 blocks' own forwardDct4x4() position-0 outputs, per
/// clause 8.5.11) chroma DC block in place - the inverse of
/// dequantChromaDC2x2(), using the *chroma* QP (see chromaQp(),
/// h264_transform.h). Like quantizeLumaDC4x4(), this factor was pinned
/// down empirically (a controlled flat-8x8-chroma-block round-trip test)
/// rather than trusted from a one-line algebraic comparison of the two
/// dequant scales alone - hadamard2x2() has its own combined forward+
/// inverse gain (H^2 = 4I for the 2x2 case) that has to be accounted for
/// jointly with dequantChromaDC2x2()'s specific >>5 scale, and it's easy
/// to get a plausible-looking factor that's off by exactly a power of 2.
inline bool quantizeChromaDC2x2(int32_t* f, int chromaQp) {
  int m = chromaQp % 6;
  int shift = chromaQp / 6;
  int32_t denom = (int32_t)kNormAdjust4x4[m][0] << shift;
  bool nonzero = false;
  for (int i = 0; i < 4; i++) {
    f[i] = roundDivPositive(f[i] * 2, denom);
    if (f[i] != 0) nonzero = true;
  }
  return nonzero;
}

}  // namespace tinyh264
