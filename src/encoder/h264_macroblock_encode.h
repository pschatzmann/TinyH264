#pragma once
#include <stdint.h>
#include <stdlib.h>  // abs()
#include "h264_bitwriter.h"
#include "h264_cavlc_encode.h"
#include "h264_forward_transform.h"
#include "../common/h264_frame.h"
#include "../common/h264_intra_pred.h"
#include "../common/h264_mb_info.h"
#include "../common/h264_tables.h"
#include "../common/h264_transform.h"

/*
 * Header-only. macroblock_layer() encoding for I_16x16 macroblocks only
 * (this encoder's first milestone - see the project README's "Encoding"
 * section for scope) - the encoder-side counterpart to
 * decoder/h264_macroblock.h's decodeMacroblockIntraWithType() I_16x16
 * branch. Every syntax element written here, and the order it's written
 * in, was checked directly against that function (this file's comments
 * cite the exact lines) - not re-derived from spec text alone - so this
 * project's own decoder (already pixel-exact-verified against real
 * ffmpeg) and real ffmpeg both parse the result. See
 * test/native/test_encode_iframe.cpp for that round-trip verification.
 *
 * Design choice worth calling out: mode decision (which of the 4 I_16x16
 * luma / 4 chroma prediction modes to use) is plain SAD (sum of absolute
 * differences) over the small, fixed candidate set - no rate-distortion
 * optimization, no Lagrangian cost weighing bits against distortion. This
 * matches the project's "simplicity/speed over compression ratio" design
 * center for this encoder (an embedded target, real-time budget) - see
 * the scoping discussion this milestone was built under. A future
 * milestone could add RDO if compression ratio becomes the priority.
 */

namespace tinyh264 {

/**
 * Everything one macroblock's encode needs that stays constant across
 * the whole slice/picture: the picture being reconstructed (== what a
 * real decoder would produce from this encoder's own bitstream - closed-
 * loop, so later macroblocks' intra prediction context matches exactly
 * what decoding this stream back would give), per-picture macroblock
 * metadata (shared MbInfoTable/MacroblockInfo types with the decoder),
 * and the *source* picture actually being encoded (plain plane
 * pointers/strides, read-only). Structurally mirrors decoder::
 * MbDecodeContext (same member names for the fields both need) so the
 * shared lumaNeighborNnz()/chromaNeighborNnz() templates
 * (common/h264_mb_info.h) work unchanged for either direction. Doesn't
 * hold a full Sps/Pps (decoder/h264_sps_pps.h) - this encoder always
 * writes a fixed, minimal PPS (see h264_sps_pps_writer.h), so the one
 * PPS field a macroblock encode actually needs (chroma_qp_index_offset)
 * is carried directly instead of pulling in the decode-side struct.
 */
template <typename Allocator>
struct MbEncodeContext {
  Frame<Allocator>* frame;
  MbInfoTable<Allocator>* mbInfo;
  int chromaQpIndexOffset;
  int sliceId;
  int mbX, mbY;

  const uint8_t* srcY;
  int srcStrideY;
  const uint8_t* srcU;
  const uint8_t* srcV;
  int srcStrideC;
};

/**
 * Sum of absolute differences between the 16x16 luma region already
 * written into `ctx.frame` at this macroblock's position and the source
 * picture at the same position - the mode-decision cost function.
 */
template <typename Allocator>
inline int sadLuma16x16(const MbEncodeContext<Allocator>& ctx) {
  int px = ctx.mbX * 16, py = ctx.mbY * 16;
  int sad = 0;
  for (int y = 0; y < 16; y++) {
    const uint8_t* rec = ctx.frame->yRow(py + y) + px;
    const uint8_t* src = ctx.srcY + (size_t)(py + y) * ctx.srcStrideY + px;
    for (int x = 0; x < 16; x++) sad += abs((int)rec[x] - (int)src[x]);
  }
  return sad;
}

/**
 * Sum of |sample - block average| over one 4x4 *source* block - a cheap
 * mode-decision proxy that needs no prediction/neighbor context at all
 * (unlike sadLuma16x16(), which measures a specific candidate mode's
 * actual error). Used by shouldUseIntra4x4() below as a stand-in for
 * "how well would I_4x4's finer-grained, block-local prediction likely
 * do here", without the cost of actually running I_4x4's real,
 * progressively-dependent per-block prediction just to find out.
 */
inline int blockVarianceSad4x4(const uint8_t* src, int stride) {
  int sum = 0;
  for (int y = 0; y < 4; y++)
    for (int x = 0; x < 4; x++) sum += src[y * stride + x];
  int avg = (sum + 8) >> 4;
  int sad = 0;
  for (int y = 0; y < 4; y++)
    for (int x = 0; x < 4; x++) sad += abs((int)src[y * stride + x] - avg);
  return sad;
}

/**
 * Macroblock-level I_16x16-vs-I_4x4 decision (clause 7.3.5's `mb_type ==
 * 0` vs `1..24` choice for I-slices), run *before* either macroblock
 * encoder actually runs: I_16x16 predicts the whole 16x16 block at once
 * (cheap, coarse - a poor fit for detailed/edge-heavy content), I_4x4
 * predicts+codes each 4x4 block individually (finer-grained, better fit
 * for detail, but real extra signaling cost - up to 16 mode-flag/rem
 * codes plus per-block CAVLC framing overhead I_16x16 doesn't pay). This
 * is a fast, SAD-based heuristic, not full rate-distortion optimization
 * (consistent with this encoder's whole design center - see the file
 * header comment): `i16x16Sad` is the *real* SAD of I_16x16's actual
 * best-mode prediction (the caller already ran chooseIntra16x16Mode()
 * to get it, cheaply reusable rather than duplicating that search here);
 * the I_4x4 side uses blockVarianceSad4x4() summed over all 16 blocks as
 * a proxy, cheaper than running I_4x4's real per-block prediction search
 * just to decide whether to use it. `kI4x4Bias` accounts for I_4x4's
 * real extra bit cost - not empirically tuned against a rate-distortion
 * curve, a documented starting point (see
 * test/native/test_encode_i4x4.cpp for the "does this ever actually
 * choose I_4x4, and does it measurably help on detailed content" check
 * this was verified against).
 */
template <typename Allocator>
inline bool shouldUseIntra4x4(const MbEncodeContext<Allocator>& ctx,
                               int i16x16Sad) {
  int px0 = ctx.mbX * 16, py0 = ctx.mbY * 16;
  int totalVarSad = 0;
  for (int by = 0; by < 4; by++) {
    for (int bx = 0; bx < 4; bx++) {
      totalVarSad += blockVarianceSad4x4(
          ctx.srcY + (size_t)(py0 + by * 4) * ctx.srcStrideY + px0 + bx * 4,
          ctx.srcStrideY);
    }
  }
  const int kI4x4Bias = 24;
  return totalVarSad + kI4x4Bias < i16x16Sad;
}

/**
 * Sum of absolute differences between an 8x8 chroma plane region already
 * written into `plane` and the corresponding source plane region.
 */
inline int sadChroma8x8(const uint8_t* plane, int stride, int px, int py,
                         const uint8_t* src, int srcStride) {
  int sad = 0;
  for (int y = 0; y < 8; y++) {
    const uint8_t* rec = plane + (size_t)(py + y) * stride + px;
    const uint8_t* s = src + (size_t)(py + y) * srcStride + px;
    for (int x = 0; x < 8; x++) sad += abs((int)rec[x] - (int)s[x]);
  }
  return sad;
}

/**
 * Chooses the I_16x16 luma prediction mode (clause 8.3.3) with lowest SAD
 * among the candidates actually valid given neighbor availability
 * (Vertical needs topAvail, Horizontal needs leftAvail, Plane needs both
 * - DC is always valid, it has an explicit "neither available" fallback
 * of 128, see predictIntra16x16() in h264_intra_pred.h). Leaves the
 * chosen mode's prediction actually written into `ctx.frame` (each
 * predictIntra16x16() call only reads neighbor samples, outside the
 * macroblock's own 16x16 target region, and only writes within it - see
 * that function's own body - so re-predicting with the winning mode
 * after trying every candidate is safe and needs no scratch buffer).
 */
template <typename Allocator>
inline int chooseIntra16x16Mode(MbEncodeContext<Allocator>& ctx,
                                 bool leftAvail, bool topAvail,
                                 bool topLeftAvail) {
  int candidates[4];
  int nCand = 0;
  candidates[nCand++] = kI16Dc;
  if (topAvail) candidates[nCand++] = kI16Vertical;
  if (leftAvail) candidates[nCand++] = kI16Horizontal;
  if (topAvail && leftAvail) candidates[nCand++] = kI16Plane;

  int bestMode = kI16Dc, bestSad = -1;
  for (int i = 0; i < nCand; i++) {
    predictIntra16x16(*ctx.frame, ctx.mbX, ctx.mbY, candidates[i], leftAvail,
                       topAvail, topLeftAvail);
    int sad = sadLuma16x16(ctx);
    if (bestSad < 0 || sad < bestSad) {
      bestSad = sad;
      bestMode = candidates[i];
    }
  }
  predictIntra16x16(*ctx.frame, ctx.mbX, ctx.mbY, bestMode, leftAvail,
                     topAvail, topLeftAvail);
  return bestMode;
}

/**
 * Chooses the chroma prediction mode (clause 8.3.4) - one mode shared by
 * both Cb and Cr (clause 7.3.5.1's intra_chroma_pred_mode applies to
 * both planes identically), so SAD is summed across both planes for
 * each candidate before comparing. Same "leave the winner actually
 * predicted" approach as chooseIntra16x16Mode() above, run once per
 * plane.
 */
template <typename Allocator>
inline int chooseChromaMode(MbEncodeContext<Allocator>& ctx, bool leftAvail,
                             bool topAvail, bool topLeftAvail) {
  int candidates[4];
  int nCand = 0;
  candidates[nCand++] = kChromaDc;
  if (topAvail) candidates[nCand++] = kChromaVertical;
  if (leftAvail) candidates[nCand++] = kChromaHorizontal;
  if (topAvail && leftAvail) candidates[nCand++] = kChromaPlane;

  int px = ctx.mbX * 8, py = ctx.mbY * 8;
  int bestMode = kChromaDc, bestSad = -1;
  for (int i = 0; i < nCand; i++) {
    predictIntraChromaPlane(ctx.frame->u(), ctx.frame->strideC, px, py,
                             candidates[i], leftAvail, topAvail, topLeftAvail);
    predictIntraChromaPlane(ctx.frame->v(), ctx.frame->strideC, px, py,
                             candidates[i], leftAvail, topAvail, topLeftAvail);
    int sad = sadChroma8x8(ctx.frame->u(), ctx.frame->strideC, px, py,
                            ctx.srcU, ctx.srcStrideC) +
              sadChroma8x8(ctx.frame->v(), ctx.frame->strideC, px, py,
                            ctx.srcV, ctx.srcStrideC);
    if (bestSad < 0 || sad < bestSad) {
      bestSad = sad;
      bestMode = candidates[i];
    }
  }
  predictIntraChromaPlane(ctx.frame->u(), ctx.frame->strideC, px, py,
                           bestMode, leftAvail, topAvail, topLeftAvail);
  predictIntraChromaPlane(ctx.frame->v(), ctx.frame->strideC, px, py,
                           bestMode, leftAvail, topAvail, topLeftAvail);
  return bestMode;
}

/**
 * Chroma mode decision + transform/quantize for an I-slice macroblock
 * (clause 8.3.4/8.5.11 - identical for I_16x16 and I_NxN, since chroma
 * prediction/coding for intra macroblocks doesn't depend on the luma
 * mode at all) - phase 1 of 2, run *before* mb_type/coded_block_pattern
 * can be written (cbpChroma needs to be known first). Shared by
 * encodeMacroblockIntra16x16() and encodeMacroblockIntra4x4(). Fills
 * `mb.chromaPredMode`/`mb.cbpChroma`; `chromaBlocks`/`chromaDcGrid` are
 * caller-owned scratch, consumed later by writeChromaResidual().
 */
template <typename Allocator>
inline int quantizeChromaIntra(MbEncodeContext<Allocator>& ctx,
                                MacroblockInfo& mb, bool leftAvail,
                                bool topAvail, bool topLeftAvail,
                                int targetQp, int32_t chromaBlocks[2][4][16],
                                int32_t chromaDcGrid[2][4]) {
  int chromaMode = chooseChromaMode(ctx, leftAvail, topAvail, topLeftAvail);
  mb.chromaPredMode = (uint8_t)chromaMode;

  int cQp = chromaQp(targetQp, ctx.chromaQpIndexOffset);
  int pcx = ctx.mbX * 8, pcy = ctx.mbY * 8;
  bool chromaAcNonzero = false;
  bool chromaDcNonzero = false;
  for (int plane = 0; plane < 2; plane++) {
    const uint8_t* srcPlane = plane == 0 ? ctx.srcU : ctx.srcV;
    uint8_t* recPlane = plane == 0 ? ctx.frame->u() : ctx.frame->v();
    for (int cy = 0; cy < 2; cy++) {
      for (int cx = 0; cx < 2; cx++) {
        int idx = cy * 2 + cx;
        subtractBlock4x4(
            srcPlane + (size_t)(pcy + cy * 4) * ctx.srcStrideC + pcx + cx * 4,
            ctx.srcStrideC,
            recPlane + (size_t)(pcy + cy * 4) * ctx.frame->strideC + pcx +
                cx * 4,
            ctx.frame->strideC, chromaBlocks[plane][idx]);
        forwardDct4x4(chromaBlocks[plane][idx]);
        chromaDcGrid[plane][idx] = chromaBlocks[plane][idx][0];
      }
    }
    hadamard2x2(chromaDcGrid[plane]);
    if (quantizeChromaDC2x2(chromaDcGrid[plane], cQp)) chromaDcNonzero = true;
    for (int idx = 0; idx < 4; idx++) {
      if (quantizeBlock4x4(chromaBlocks[plane][idx], cQp, true))
        chromaAcNonzero = true;
    }
  }
  /*
   * clause 9.2.1's cbpChroma convention: 0 = no chroma residual at all,
   * 1 = DC only, 2 = DC+AC (matches decodeMacroblockIntraWithType()'s
   * `mb.cbpChroma >= 1` / `>= 2` gates exactly).
   */
  mb.cbpChroma = chromaAcNonzero ? 2 : (chromaDcNonzero ? 1 : 0);
  return chromaMode;
}

/**
 * Writes chroma DC/AC residual CAVLC data and reconstructs into
 * ctx.frame - phase 2 of 2, run *after* mb_qp_delta (matching
 * decodeMacroblockIntraWithType()'s own field order). Shared the same
 * way as quantizeChromaIntra() above.
 */
template <typename Allocator>
inline void writeChromaResidual(BitWriter& bw, MbEncodeContext<Allocator>& ctx,
                                 MacroblockInfo& mb, int targetQp,
                                 int32_t chromaBlocks[2][4][16],
                                 int32_t chromaDcGrid[2][4]) {
  int cQp = chromaQp(targetQp, ctx.chromaQpIndexOffset);
  int pcx = ctx.mbX * 8, pcy = ctx.mbY * 8;
  int32_t chromaDcRecon[2][4];
  for (int plane = 0; plane < 2; plane++) {
    if (mb.cbpChroma >= 1) {
      /*
       * Chroma DC (2x2) has no zigzag scan (clause 8.5.11) - unlike the
       * 4x4 luma DC/AC blocks, decodeMacroblockIntraWithType() copies
       * residualBlockCavlc()'s 4 coefficients straight into raster
       * position (dst[0..3] = coeff[0..3], no kZigZag4x4 remapping at
       * all) - matched here exactly. A real, caught-by-this-round-trip-
       * test bug: kZigZag4x4[0..3] is {0,1,4,8} - index 8 is out of
       * bounds for this 4-element array, corrupting the encoded chroma
       * DC block with garbage adjacent stack memory.
       */
      encodeResidualBlockCavlc(bw, -1, 4, chromaDcGrid[plane]);
    }
    for (int i = 0; i < 4; i++) chromaDcRecon[plane][i] = chromaDcGrid[plane][i];
    hadamard2x2(chromaDcRecon[plane]);  // see the luma DC comment on
                                         /*
                                          * encodeMacroblockIntra16x16() -
                                          * same missing-inverse-transform
                                          * bug, same fix, chroma side
                                          */
    dequantChromaDC2x2(chromaDcRecon[plane], cQp);
  }

  for (int plane = 0; plane < 2; plane++) {
    uint8_t* recPlane = plane == 0 ? ctx.frame->u() : ctx.frame->v();
    for (int cy = 0; cy < 2; cy++) {
      for (int cx = 0; cx < 2; cx++) {
        int cidx = cy * 2 + cx;
        int32_t dcVal = chromaDcRecon[plane][cidx];
        int px = pcx + cx * 4, py = pcy + cy * 4;
        if (mb.cbpChroma >= 2) {
          int32_t acScan[15];
          for (int k = 0; k < 15; k++)
            acScan[k] = chromaBlocks[plane][cidx][kZigZag4x4[k + 1]];
          int nA = chromaNeighborNnz(ctx, plane, cx, cy, -1, 0);
          int nB = chromaNeighborNnz(ctx, plane, cx, cy, 0, -1);
          int nC = predictNc(nA, nB);
          encodeResidualBlockCavlc(bw, nC, 15, acScan);
          uint32_t totalCoeff = 0;
          for (int k = 0; k < 15; k++) if (acScan[k] != 0) totalCoeff++;
          mb.nnz[16 + plane * 4 + cidx] = (uint8_t)totalCoeff;

          int32_t block[16];
          for (int i = 0; i < 16; i++) block[i] = chromaBlocks[plane][cidx][i];
          dequant4x4(block, cQp, true);
          block[0] = dcVal;
          idct4x4(block);
          addResidual4x4(recPlane + (size_t)py * ctx.frame->strideC + px,
                          ctx.frame->strideC, block);
        } else {
          int32_t block[16] = {0};
          block[0] = dcVal;
          idct4x4(block);
          addResidual4x4(recPlane + (size_t)py * ctx.frame->strideC + px,
                          ctx.frame->strideC, block);
          mb.nnz[16 + plane * 4 + cidx] = 0;
        }
      }
    }
  }
}

/**
 * Encodes one I_16x16 macroblock (mb_type, intra_chroma_pred_mode,
 * mb_qp_delta, luma DC/AC residual, chroma DC/AC residual) into `bw`,
 * and reconstructs it into `ctx.frame`/`ctx.mbInfo` exactly as decoding
 * this same bitstream back would - the closed loop every real H.264
 * encoder needs so later macroblocks' (and later pictures', via
 * `ctx.frame`/`ctx.mbInfo` state a P-frame's `refFrame_` is built from)
 * intra/inter prediction context matches a real decoder's, not the
 * original source. `qpPrev` is the running QP entering this macroblock
 * (mirrors decodeMacroblockIntraWithType()'s `*qpY` in/out parameter);
 * returns the running QP after this macroblock's mb_qp_delta (always
 * coded for I_16x16 - see the `is16x16` term in
 * decodeMacroblockIntraWithType()'s mb_qp_delta condition). `mbTypeOffset`
 * is 0 for an I-slice macroblock (mb_type 1..24, clause 7.3.5) or 5 for
 * an Intra macroblock inside a P-slice (mb_type 6..29, clause 7.4.5's
 * "I-slice numbering + 5" convention) - see
 * Encoder::encodePFrame()'s Intra-fallback path
 * (h264_encoder.h) for the P-slice case.
 */
template <typename Allocator>
inline int encodeMacroblockIntra16x16(BitWriter& bw,
                                       MbEncodeContext<Allocator>& ctx,
                                       int qpPrev, int targetQp,
                                       uint32_t mbTypeOffset = 0) {
  MacroblockInfo& mb = ctx.mbInfo->at(ctx.mbX, ctx.mbY);
  mb = MacroblockInfo();
  mb.type = kMbIntra16x16;

  bool leftAvail = ctx.mbInfo->leftAvailable(ctx.mbX, ctx.mbY, ctx.sliceId);
  bool topAvail = ctx.mbInfo->topAvailable(ctx.mbX, ctx.mbY, ctx.sliceId);
  bool topLeftAvail =
      ctx.mbInfo->topLeftAvailable(ctx.mbX, ctx.mbY, ctx.sliceId);

  int lumaMode = chooseIntra16x16Mode(ctx, leftAvail, topAvail, topLeftAvail);
  mb.intra16x16PredMode = (uint8_t)lumaMode;

  // --- Luma transform/quantize (prediction already sits in ctx.frame) --
  int px0 = ctx.mbX * 16, py0 = ctx.mbY * 16;
  int32_t lumaBlocks[16][16];  // forwardDct4x4() output, then AC-quantized
  int32_t dcGrid[16];          // raster block-grid order (by*4+bx)
  for (int blk = 0; blk < 16; blk++) {
    int bx = kBlk4x4X[blk], by = kBlk4x4Y[blk];
    subtractBlock4x4(
        ctx.srcY + (size_t)(py0 + by * 4) * ctx.srcStrideY + (px0 + bx * 4),
        ctx.srcStrideY, ctx.frame->yRow(py0 + by * 4) + px0 + bx * 4,
        ctx.frame->strideY, lumaBlocks[blk]);
    forwardDct4x4(lumaBlocks[blk]);
    dcGrid[by * 4 + bx] = lumaBlocks[blk][0];
  }
  hadamard4x4(dcGrid);
  quantizeLumaDC4x4(dcGrid, targetQp);

  bool anyAcNonzero = false;
  for (int blk = 0; blk < 16; blk++) {
    if (quantizeBlock4x4(lumaBlocks[blk], targetQp, true)) anyAcNonzero = true;
  }
  mb.cbpLuma = anyAcNonzero ? 15 : 0;

  // --- Chroma mode + transform/quantize --------------------------------
  int32_t chromaBlocks[2][4][16];  // [plane][blockIdx cy*2+cx][coeff]
  int32_t chromaDcGrid[2][4];      // [plane][cy*2+cx] raster
  int chromaMode = quantizeChromaIntra(ctx, mb, leftAvail, topAvail,
                                        topLeftAvail, targetQp, chromaBlocks,
                                        chromaDcGrid);

  // --- mb_type, intra_chroma_pred_mode ----------------------------------
  uint32_t idx = (uint32_t)lumaMode + 4u * mb.cbpChroma +
                 (mb.cbpLuma != 0 ? 12u : 0u);
  bw.ue(mbTypeOffset + 1 + idx);
  bw.ue((uint32_t)chromaMode);

  // --- mb_qp_delta: always coded for I_16x16 ----------------------------
  bw.se(targetQp - qpPrev);
  mb.qpY = (int8_t)targetQp;

  // --- Luma residual: DC block always coded, per-block AC gated on cbpLuma
  int32_t dcScan[16];
  for (int k = 0; k < 16; k++) dcScan[k] = dcGrid[kZigZag4x4[k]];
  int bxA = lumaNeighborNnz(ctx, 0, 0, -1, 0);
  int bxB = lumaNeighborNnz(ctx, 0, 0, 0, -1);
  int nCdc = predictNc(bxA, bxB);
  encodeResidualBlockCavlc(bw, nCdc, 16, dcScan);

  /*
   * Re-run the (already-verified) inverse-Hadamard-then-dequant path so
   * ctx.frame ends up holding exactly what decoding this bitstream back
   * would produce - mirrors decodeMacroblockIntraWithType()'s own
   * dcBlock handling exactly, including the order (hadamard4x4() before
   * dequantLumaDC4x4(), not after): dcGrid here holds *quantized levels*
   * (post quantizeLumaDC4x4()), the same domain decode's dcBlock is in
   * right after its zigzag scatter from CAVLC - hadamard4x4() is self-
   * inverse so the same function undoes the forward transform, but
   * dequantLumaDC4x4()'s integer rounding is position-independent only
   * in the idealized (unrounded) math, not bit-exactly after it - the
   * order genuinely matters once rounding is involved, and this was a
   * real bug during development (silently omitting the hadamard4x4()
   * call entirely here reconstructed only the top-left block correctly,
   * since a Hadamard-domain impulse at position 0 - what a still-
   * quantized-but-not-yet-inverse-transformed DC block looks like for
   * any content - inverse-transforms to a uniform value everywhere, but
   * skipping that step left every other position's *quantized level*,
   * not a proper per-block spatial DC value, sitting at dcRecon[i]).
   */
  int32_t dcRecon[16];
  for (int i = 0; i < 16; i++) dcRecon[i] = dcGrid[i];
  hadamard4x4(dcRecon);
  dequantLumaDC4x4(dcRecon, targetQp);

  for (int blk = 0; blk < 16; blk++) {
    int bx = kBlk4x4X[blk], by = kBlk4x4Y[blk];
    int32_t dc = dcRecon[by * 4 + bx];
    if (mb.cbpLuma != 0) {
      int32_t acScan[15];
      for (int k = 0; k < 15; k++) acScan[k] = lumaBlocks[blk][kZigZag4x4[k + 1]];
      int nA = lumaNeighborNnz(ctx, bx, by, -1, 0);
      int nB = lumaNeighborNnz(ctx, bx, by, 0, -1);
      int nC = predictNc(nA, nB);
      encodeResidualBlockCavlc(bw, nC, 15, acScan);
      uint32_t totalCoeff = 0;
      for (int k = 0; k < 15; k++) if (acScan[k] != 0) totalCoeff++;
      mb.nnz[blk] = (uint8_t)totalCoeff;

      int32_t block[16];
      for (int i = 0; i < 16; i++) block[i] = lumaBlocks[blk][i];
      dequant4x4(block, targetQp, true);
      block[0] = dc;
      idct4x4(block);
      int px = px0 + bx * 4, py = py0 + by * 4;
      addResidual4x4(ctx.frame->yRow(py) + px, ctx.frame->strideY, block);
    } else {
      int32_t block[16] = {0};
      block[0] = dc;
      idct4x4(block);
      int px = px0 + bx * 4, py = py0 + by * 4;
      addResidual4x4(ctx.frame->yRow(py) + px, ctx.frame->strideY, block);
      mb.nnz[blk] = 0;
    }
  }

  writeChromaResidual(bw, ctx, mb, targetQp, chromaBlocks, chromaDcGrid);

  return targetQp;
}

/**
 * Builds the inverse of kCbpIntra4x4 (h264_tables.h): given (cbpLuma,
 * cbpChroma), finds the coded_block_pattern ue(v) code to write. Linear
 * scan of the 48-entry table - cheap, once per I_NxN macroblock, and
 * avoids authoring/verifying a second, separately-transcribed inverse
 * table (a real transcription-error risk this project has cared about
 * elsewhere - see h264_rgb.h's shared yuvToRgb8() comment).
 */
inline uint32_t cbpCodeForIntra4x4(int cbpLuma, int cbpChroma) {
  uint8_t want = (uint8_t)(cbpLuma | (cbpChroma << 4));
  for (uint32_t i = 0; i < 48; i++) {
    if (kCbpIntra4x4[i] == want) return i;
  }
  return 0;  // unreachable: every (cbpLuma 0-15, cbpChroma 0-2) combination
             /*
              * appears exactly once in kCbpIntra4x4 (it's a bijection over
              * 48 = 16*3 values) - verified by test_encode_i4x4.cpp.
              */
}

/**
 * Chooses the Intra_4x4 prediction mode (clause 8.3.1.2) for one 4x4
 * luma block with lowest SAD among the candidates actually valid given
 * `n`'s neighbor availability - each of the 9 modes has its own real
 * precondition (not just "top and/or left", unlike I_16x16): Vertical/
 * DiagDownLeft/VerticalLeft need only top (DiagDownLeft's top-right and
 * VerticalLeft's top-right are auto-substituted by gatherNeighbors4x4()
 * when unavailable); Horizontal/HorizontalUp need only left;
 * DiagDownRight/VerticalRight/HorizontalDown need top *and* left *and*
 * the top-left corner sample specifically (all three read `topLeft`
 * directly, unlike the others); DC is always valid (has its own
 * per-availability fallback, see predictIntra4x4()). Selecting a mode
 * whose precondition isn't met would make predictIntra4x4() read
 * uninitialized `Neighbors4x4` fields - decodeMacroblockIntraWithType()
 * relies on `usePredicted`/mb_type signaling *never* producing such a
 * mode (see its own comment), an invariant an encoder has to uphold
 * itself since nothing else enforces it. `predictedMode` (from
 * predictIntra4x4Mode(), common/h264_mb_info.h) gets a small cost bias
 * - cheaper to signal (1 flag bit vs. flag+3 bits) - a modest, real
 * rate-awareness consistent with this encoder's SAD-not-full-RDO
 * design center. Leaves the winning mode's prediction actually written
 * into `ctx.frame` (same "safe to just re-predict the winner, no
 * scratch buffer needed" reasoning as chooseIntra16x16Mode()).
 */
template <typename Allocator>
inline int chooseIntra4x4Mode(MbEncodeContext<Allocator>& ctx,
                               const Neighbors4x4& n, int px, int py,
                               int predictedMode) {
  int candidates[9];
  int nCand = 0;
  candidates[nCand++] = kI4Dc;
  if (n.haveTop) candidates[nCand++] = kI4Vertical;
  if (n.haveLeft) candidates[nCand++] = kI4Horizontal;
  if (n.haveTop) candidates[nCand++] = kI4DiagDownLeft;
  if (n.haveTop && n.haveLeft && n.haveTopLeft)
    candidates[nCand++] = kI4DiagDownRight;
  if (n.haveTop && n.haveLeft && n.haveTopLeft)
    candidates[nCand++] = kI4VerticalRight;
  if (n.haveTop && n.haveLeft && n.haveTopLeft)
    candidates[nCand++] = kI4HorizontalDown;
  if (n.haveTop) candidates[nCand++] = kI4VerticalLeft;
  if (n.haveLeft) candidates[nCand++] = kI4HorizontalUp;

  int bestMode = kI4Dc, bestCost = -1;
  for (int i = 0; i < nCand; i++) {
    predictIntra4x4(*ctx.frame, px, py, candidates[i], n);
    int sad = 0;
    for (int y = 0; y < 4; y++) {
      const uint8_t* rec = ctx.frame->yRow(py + y) + px;
      const uint8_t* src = ctx.srcY + (size_t)(py + y) * ctx.srcStrideY + px;
      for (int x = 0; x < 4; x++) sad += abs((int)rec[x] - (int)src[x]);
    }
    int cost = sad + (candidates[i] == predictedMode ? 0 : 2);
    if (bestCost < 0 || cost < bestCost) {
      bestCost = cost;
      bestMode = candidates[i];
    }
  }
  predictIntra4x4(*ctx.frame, px, py, bestMode, n);
  return bestMode;
}

/**
 * Encodes one I_NxN (Intra_4x4) macroblock - the encoder-side
 * counterpart of decodeMacroblockIntraWithType()'s `!is16x16` branch.
 * Unlike I_16x16 (whole-macroblock prediction, residual handled per-
 * block afterward), I_4x4 predicts, transforms/quantizes, *and
 * reconstructs* one 4x4 block at a time, strictly in kBlk4x4X/Y (Z-scan)
 * order - each later block's prediction context (both pixel neighbors,
 * via gatherNeighbors4x4(), and the predicted-mode derivation, via
 * predictIntra4x4Mode()) depends on *reconstructed* (prediction +
 * residual, not just prediction) samples/modes of earlier blocks within
 * the same macroblock, exactly matching decode order - so mode
 * decision, quantization, and reconstruction cannot be separated into
 * two passes the way I_16x16's whole-MB-at-once prediction allows.
 * coded_block_pattern (unlike I_16x16, where cbp is folded directly
 * into mb_type) still has to be *known* before mb_type/cbp can be
 * written, though - so this function still runs in two phases: phase 1
 * (this loop) predicts+quantizes+reconstructs all 16 blocks, storing
 * each one's quantized coefficients and chosen mode; phase 2 (after,
 * once cbpLuma is known) writes the actual bitstream syntax using the
 * stored per-block state. `mbTypeOffset` is 0 for an I-slice macroblock
 * (mb_type 0) or 5 for an Intra macroblock inside a P-slice (mb_type 5,
 * clause 7.4.5's "I-slice numbering + 5" convention) - see
 * encodeMacroblockIntra16x16()'s matching parameter and
 * Encoder::encodePFrame()'s Intra-fallback path (h264_encoder.h).
 */
template <typename Allocator>
inline int encodeMacroblockIntra4x4(BitWriter& bw,
                                     MbEncodeContext<Allocator>& ctx,
                                     int qpPrev, int targetQp,
                                     uint32_t mbTypeOffset = 0) {
  MacroblockInfo& mb = ctx.mbInfo->at(ctx.mbX, ctx.mbY);
  mb = MacroblockInfo();
  mb.type = kMbIntra4x4;

  bool leftAvail = ctx.mbInfo->leftAvailable(ctx.mbX, ctx.mbY, ctx.sliceId);
  bool topAvail = ctx.mbInfo->topAvailable(ctx.mbX, ctx.mbY, ctx.sliceId);
  bool topLeftAvail =
      ctx.mbInfo->topLeftAvailable(ctx.mbX, ctx.mbY, ctx.sliceId);
  bool topRightAvail =
      ctx.mbInfo->topRightAvailable(ctx.mbX, ctx.mbY, ctx.sliceId);

  int px0 = ctx.mbX * 16, py0 = ctx.mbY * 16;
  int32_t lumaBlocks[16][16];  // forwardDct4x4() output, then fully quantized
  bool blockNonzero[16];

  // --- Phase 1: predict + quantize + reconstruct, block by block -------
  for (int blk = 0; blk < 16; blk++) {
    int bx = kBlk4x4X[blk], by = kBlk4x4Y[blk];
    int px = px0 + bx * 4, py = py0 + by * 4;

    Neighbors4x4 n = gatherNeighbors4x4(*ctx.frame, px, py, blk, leftAvail,
                                         topAvail, topLeftAvail, topRightAvail);
    int predicted = predictIntra4x4Mode(ctx, bx, by, leftAvail, topAvail);
    int mode = chooseIntra4x4Mode(ctx, n, px, py, predicted);  // writes
                                                                // prediction
    mb.intra4x4PredMode[blk] = (uint8_t)mode;

    subtractBlock4x4(ctx.srcY + (size_t)py * ctx.srcStrideY + px,
                      ctx.srcStrideY, ctx.frame->yRow(py) + px,
                      ctx.frame->strideY, lumaBlocks[blk]);
    forwardDct4x4(lumaBlocks[blk]);
    // skipDC=false: unlike I_16x16, I_NxN has no separate Hadamard DC
    // transform (clause 8.5.6 only applies to Intra16x16) - the full
    // 16-coefficient block is quantized together, matching
    // decodeLumaBlockFull()'s dequant4x4(block, qp, /*skipDC=*/false).
    blockNonzero[blk] = quantizeBlock4x4(lumaBlocks[blk], targetQp, false);

    int32_t block[16];
    for (int i = 0; i < 16; i++) block[i] = lumaBlocks[blk][i];
    dequant4x4(block, targetQp, false);
    idct4x4(block);
    addResidual4x4(ctx.frame->yRow(py) + px, ctx.frame->strideY, block);
  }

  /*
   * coded_block_pattern's luma nibble is one bit per 8x8 *quadrant*
   * (clause 9.2.1), not per individual 4x4 block - kBlk4x4X/Y groups
   * blocks 0-3/4-7/8-11/12-15 into quadrants 0-3 (blk/4), matching
   * decodeMacroblockIntraWithType()'s `int quadrant = blk / 4;` exactly.
   */
  int cbpLuma = 0;
  for (int blk = 0; blk < 16; blk++) {
    if (blockNonzero[blk]) cbpLuma |= (1 << (blk / 4));
  }
  mb.cbpLuma = (uint8_t)cbpLuma;

  // --- Chroma mode + transform/quantize (same as I_16x16's) ------------
  int32_t chromaBlocks[2][4][16];
  int32_t chromaDcGrid[2][4];
  int chromaMode = quantizeChromaIntra(ctx, mb, leftAvail, topAvail,
                                        topLeftAvail, targetQp, chromaBlocks,
                                        chromaDcGrid);

  // --- mb_type, per-block pred-mode signaling, intra_chroma_pred_mode --
  bw.ue(mbTypeOffset);  // mb_type = 0 (I_NxN), or +5 inside a P-slice
  for (int blk = 0; blk < 16; blk++) {
    int bx = kBlk4x4X[blk], by = kBlk4x4Y[blk];
    /*
     * Deterministic re-derivation, not a stored value: predictIntra4x4Mode()
     * only reads *earlier* blocks' mb.intra4x4PredMode entries (already
     * final by phase 1's end) plus mbInfo state that hasn't changed since
     * phase 1, so recomputing here gives byte-identical results to the
     * phase-1 call at strictly lower cost than caching a 16-entry array.
     */
    int predicted = predictIntra4x4Mode(ctx, bx, by, leftAvail, topAvail);
    int mode = mb.intra4x4PredMode[blk];
    if (mode == predicted) {
      bw.flag(true);  // prev_intra4x4_pred_mode_flag
    } else {
      bw.flag(false);
      int rem = mode < predicted ? mode : mode - 1;
      bw.u((uint32_t)rem, 3);  // rem_intra4x4_pred_mode
    }
  }
  bw.ue((uint32_t)chromaMode);

  // --- coded_block_pattern -----------------------------------------------
  bw.ue(cbpCodeForIntra4x4(mb.cbpLuma, mb.cbpChroma));

  /*
   * --- mb_qp_delta: only coded if there's any residual to code (unlike
   * I_16x16, where it's unconditional) - matches
   * decodeMacroblockIntraWithType()'s `if (cbpLuma != 0 || cbpChroma != 0
   * || is16x16)` gate exactly. When neither is true, the running QP for
   * this macroblock simply doesn't change (mb.qpY stays qpPrev) - the
   * quantization work above already used targetQp throughout, but that's
   * harmless here: an all-zero residual dequantizes to all-zero at *any*
   * QP, so which QP was nominally used never shows up in this specific
   * (fully-zero) case. This matters once rate control varies QP
   * per-macroblock - see Encoder::encodeIFrame()'s qpRunning bookkeeping.
   */
  bool hasResidual = mb.cbpLuma != 0 || mb.cbpChroma != 0;
  int qpUsed = hasResidual ? targetQp : qpPrev;
  if (hasResidual) bw.se(targetQp - qpPrev);
  mb.qpY = (int8_t)qpUsed;

  /*
   * --- Luma residual: per-block, gated on this block's quadrant's cbpLuma
   * bit (all 16 blocks' quantized coefficients were already computed in
   * phase 1 - this just writes them and records nnz for future nC context).
   */
  for (int blk = 0; blk < 16; blk++) {
    if (mb.cbpLuma & (1 << (blk / 4))) {
      int bx = kBlk4x4X[blk], by = kBlk4x4Y[blk];
      int32_t scan[16];
      for (int k = 0; k < 16; k++) scan[k] = lumaBlocks[blk][kZigZag4x4[k]];
      int nA = lumaNeighborNnz(ctx, bx, by, -1, 0);
      int nB = lumaNeighborNnz(ctx, bx, by, 0, -1);
      int nC = predictNc(nA, nB);
      encodeResidualBlockCavlc(bw, nC, 16, scan);
      uint32_t totalCoeff = 0;
      for (int k = 0; k < 16; k++) if (scan[k] != 0) totalCoeff++;
      mb.nnz[blk] = (uint8_t)totalCoeff;
    } else {
      mb.nnz[blk] = 0;
    }
  }

  writeChromaResidual(bw, ctx, mb, targetQp, chromaBlocks, chromaDcGrid);

  return qpUsed;
}

}  // namespace tinyh264
