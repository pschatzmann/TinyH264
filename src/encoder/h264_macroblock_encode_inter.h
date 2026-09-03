#pragma once
#include <stdint.h>
#include <stdlib.h>  // abs()
#include "h264_bitwriter.h"
#include "h264_cavlc_encode.h"
#include "h264_forward_transform.h"
#include "h264_macroblock_encode.h"  // MbEncodeContext, quantizeChromaIntra-style
                                      /*
                                       * helpers, writeChromaResidual() (mode-
                                       * independent, reused as-is)
                                       */
#include "../common/h264_frame.h"
#include "../common/h264_mv_predict.h"
#include "../common/h264_tables.h"
#include "../common/h264_transform.h"

/*
 * Header-only. P-slice (Inter) macroblock encoding - the encoder-side
 * counterpart to decoder/h264_macroblock_inter.h. First P-frame milestone
 * scope (mirroring how I_16x16-only came before I_4x4): every macroblock
 * is P_Skip, P_L0_16x16 (one 16x16 partition, one reference picture -
 * the single most-recently-encoded frame, no multi-reference yet), or an
 * Intra macroblock (I_16x16/I_4x4, reusing h264_macroblock_encode.h's
 * encoders directly with mbTypeOffset=5 - see shouldUseIntraInPSlice()
 * below and Encoder::encodePFrame(), h264_encoder.h, for the per-
 * macroblock decision) - no P_16x8/P_8x16/P_8x8 sub-partitioning yet.
 * Motion estimation is a small integer-pel-only full search (see
 * motionSearch16x16()) - no sub-pel refinement, no fast-search algorithm
 * (diamond/hex/etc) - consistent with this encoder's whole "correctness
 * and simplicity first, not maximal compression efficiency" design
 * center (see docs/encoding.md).
 */

namespace tinyh264 {

/**
 * Sum of absolute differences between the current macroblock's 16x16
 * source region and the reference picture at integer-pel offset
 * (dx,dy) from the macroblock's own position - motion search's cost
 * function. Uses clampedSample() (common/h264_motion.h) for the
 * reference fetch, so candidate offsets that would read outside the
 * reference picture are handled the same edge-replicated way real
 * motion compensation (motionCompLuma()) treats them - the search cost
 * and the actual reconstruction cost agree.
 *
 * Bounds-checks the whole 16x16 block *once* up front rather than once
 * per pixel: for the common case (a macroblock/candidate-offset pair
 * that stays fully inside the reference picture - true for most of a
 * full search's ~289 candidates outside the outermost ring of
 * macroblocks), this drops straight into a branch-free inner loop
 * instead of paying clampedSample()'s 4 comparisons per pixel 256 times
 * over. Bit-for-bit identical result either way - this is purely a
 * measured real bottleneck (motionSearch16x16() below calls this ~290
 * times per macroblock; QCIF encode measured 1.8s/P-frame on real
 * ESP32/RP2040 hardware before this fix), not a behavior change.
 */
inline int sad16x16At(const MbEncodeContext& ctx,
                       const Frame& ref, int dx, int dy) {
  int px = ctx.mbX * 16, py = ctx.mbY * 16;
  int sad = 0;
  int rx = px + dx, ry = py + dy;
  if (rx >= 0 && ry >= 0 && rx + 16 <= ref.width && ry + 16 <= ref.height) {
    for (int y = 0; y < 16; y++) {
      const uint8_t* src = ctx.srcY + (size_t)(py + y) * ctx.srcStrideY + px;
      const uint8_t* refRow = ref.y() + (size_t)(ry + y) * ref.strideY + rx;
      for (int x = 0; x < 16; x++) {
        sad += abs((int)src[x] - (int)refRow[x]);
      }
    }
    return sad;
  }
  for (int y = 0; y < 16; y++) {
    const uint8_t* src = ctx.srcY + (size_t)(py + y) * ctx.srcStrideY + px;
    for (int x = 0; x < 16; x++) {
      int rv = clampedSample(ref.y(), ref.strideY, ref.width, ref.height,
                              px + x + dx, py + y + dy);
      sad += abs((int)src[x] - rv);
    }
  }
  return sad;
}

/**
 * Small integer-pel full search over +/-range pixels for the best
 * 16x16 motion vector, SAD-based with a light bias toward smaller
 * displacement (cheaper mvd_l0 to signal, and real motion in typical
 * content is usually small frame-to-frame) - not a rate-distortion-
 * optimal search, a fast/simple one (see file header comment). Writes
 * the winning MV in quarter-pel units (integer-pel search, so the low 2
 * bits are always 0 - sub-pel refinement is a natural future
 * improvement, not implemented yet) into `outMv`; returns that MV's raw
 * SAD (not including the magnitude bias, which is a search-time-only
 * tiebreaker, not part of the reported cost).
 *
 * `range` defaults to 8 (this project's original, still-default search
 * window) - see Encoder::setMotionSearchRange()/
 * TinyH264Encoder::setMotionSearchRange() for the public, runtime-
 * configurable entry point. Search cost is O(range^2) ((2*range+1)^2
 * candidates, each a 256-pixel SAD - see docs/optimizations.md's
 * "Encoding" chapter for measured numbers): halving `range` roughly
 * quarters the candidate count, at the cost of not finding motion
 * vectors larger than `range` pixels/frame - real motion beyond that
 * still gets coded, just via a larger residual instead of a matching
 * MV, so this trades encode speed for compression efficiency on
 * fast-moving content, not correctness.
 */
inline int motionSearch16x16(const MbEncodeContext& ctx,
                              const Frame& ref, int16_t* outMv,
                              int range = 8) {
  int bestSad = sad16x16At(ctx, ref, 0, 0);
  int bestDx = 0, bestDy = 0;
  for (int dy = -range; dy <= range; dy++) {
    for (int dx = -range; dx <= range; dx++) {
      if (dx == 0 && dy == 0) continue;
      int sad = sad16x16At(ctx, ref, dx, dy);
      int cost = sad + (abs(dx) + abs(dy));
      int bestCost = bestSad + (abs(bestDx) + abs(bestDy));
      if (cost < bestCost) {
        bestSad = sad;
        bestDx = dx;
        bestDy = dy;
      }
    }
  }
  outMv[0] = (int16_t)(bestDx * 4);
  outMv[1] = (int16_t)(bestDy * 4);
  return bestSad;
}

/**
 * Which search motionSearch16x16Dispatch() (h264_encoder.h) uses -
 * `Exhaustive` (the default, unchanged since this project's first P-frame
 * milestone) is motionSearch16x16() above; `Fast` is
 * motionSearch16x16Fast() below. See Encoder::setMotionSearchAlgorithm()/
 * TinyH264Encoder::setMotionSearchAlgorithm() for the public entry point.
 */
enum class MotionSearchAlgorithm { Exhaustive, Fast };

/**
 * Classic Diamond Search (Zhu & Ma, 2000) - checks ~15-30 candidates on
 * typical content instead of exhaustive search's `(2*range+1)^2` (289 at
 * the default range=8), at the cost of being a *local* search: it can
 * settle on a locally-good match that isn't the true global-best SAD
 * (data-dependent - see docs/optimizations.md's "Encoding" chapter,
 * "Fast search algorithm" for the original motivation and why this is
 * opt-in, not the default). Same cost function and MV-encoding convention
 * as motionSearch16x16() (a light magnitude bias, quarter-pel output units,
 * bounded to +/-range) so the two are drop-in alternatives for the same
 * call site.
 *
 * Starts at (0,0) (this project's zero-motion prior - real content
 * usually has small frame-to-frame motion, so this is normally already
 * close to the true best match) and alternates two diamond-shaped probe
 * patterns around a moving search center:
 *  - LDSP (Large Diamond Search Pattern, radius 2, 8 points): evaluated
 *    around the current center; if any point beats the center, the center
 *    moves there and LDSP repeats around the new center; if none does,
 *    the search has converged and moves on to SDSP.
 *  - SDSP (Small Diamond Search Pattern, radius 1, 4 points): one final
 *    refinement pass once LDSP converges - this is what lets the result
 *    land on a match LDSP's radius-2 steps would otherwise step over.
 *
 * LDSP is capped at `2*range+1` iterations (each iteration can only move
 * the center by up to 2 pixels toward the range boundary, so this is
 * already more than enough to reach it from (0,0) - a real hard bound,
 * not just a typical-case expectation) so pathological content can't turn
 * this into an unbounded loop; real content converges in a handful of
 * iterations, well under that cap.
 */
inline int motionSearch16x16Fast(const MbEncodeContext& ctx,
                                  const Frame& ref, int16_t* outMv,
                                  int range = 8) {
  static const int kLdspDx[8] = {0, 0, -1, 1, -1, 1, -2, 2};
  static const int kLdspDy[8] = {-2, 2, -1, -1, 1, 1, 0, 0};
  static const int kSdspDx[4] = {0, 0, -1, 1};
  static const int kSdspDy[4] = {-1, 1, 0, 0};

  int bestDx = 0, bestDy = 0;
  int bestSad = sad16x16At(ctx, ref, 0, 0);
  int bestCost = bestSad;  // + abs(0) + abs(0)

  int cx = 0, cy = 0;
  int maxIterations = 2 * range + 1;
  for (int iter = 0; iter < maxIterations; iter++) {
    int roundBestDx = cx, roundBestDy = cy;
    int roundBestSad = bestSad, roundBestCost = bestCost;
    bool improved = false;
    for (int i = 0; i < 8; i++) {
      int dx = cx + kLdspDx[i], dy = cy + kLdspDy[i];
      if (dx < -range || dx > range || dy < -range || dy > range) continue;
      int sad = sad16x16At(ctx, ref, dx, dy);
      int cost = sad + (abs(dx) + abs(dy));
      if (cost < roundBestCost) {
        roundBestCost = cost;
        roundBestSad = sad;
        roundBestDx = dx;
        roundBestDy = dy;
        improved = true;
      }
    }
    if (!improved) break;
    bestCost = roundBestCost;
    bestSad = roundBestSad;
    bestDx = roundBestDx;
    bestDy = roundBestDy;
    cx = bestDx;
    cy = bestDy;
  }

  for (int i = 0; i < 4; i++) {
    int dx = cx + kSdspDx[i], dy = cy + kSdspDy[i];
    if (dx < -range || dx > range || dy < -range || dy > range) continue;
    int sad = sad16x16At(ctx, ref, dx, dy);
    int cost = sad + (abs(dx) + abs(dy));
    if (cost < bestCost) {
      bestCost = cost;
      bestSad = sad;
      bestDx = dx;
      bestDy = dy;
    }
  }

  outMv[0] = (int16_t)(bestDx * 4);
  outMv[1] = (int16_t)(bestDy * 4);
  return bestSad;
}

/**
 * Intra-vs-Inter fallback decision for one non-skip macroblock in a
 * P-slice (clause 7.3.5's mb_type >= 5 case) - a fast, SAD-based
 * heuristic (not full RDO, same design center as
 * h264_macroblock_encode.h's shouldUseIntra4x4()), run only after
 * P_Skip has already been ruled out (a genuine zero-bit skip is never
 * worse than either alternative, so it's checked first and short-
 * circuits this decision entirely - see Encoder::encodePFrame()).
 * `intraSad` is I_16x16's real best-mode SAD (the caller already ran
 * chooseIntra16x16Mode() to get it - see the same reasoning in
 * shouldUseIntra4x4()'s own comment for why reusing it here is cheap);
 * `interSad` is P_16x16's real best-match SAD from motionSearch16x16().
 * `kInterFavorMargin` deliberately biases *toward* keeping Inter unless
 * Intra is clearly better: real video content usually favors Inter
 * (temporal redundancy is normally stronger than spatial), so an
 * unbiased "whichever SAD is lower" comparison would flip to Intra on
 * marginal differences that don't actually pay for Intra's typically-
 * larger signaling cost, and would needlessly break the `interCoded`
 * neighbor chain P_16x16 macroblocks' own MV prediction relies on for
 * their surrounding macroblocks. The margin exists to make this trigger
 * mainly on genuine motion-search failures (scene cuts, content
 * entering/leaving the search range, occlusion) - not tuned against a
 * rate-distortion curve, a documented starting point (see
 * test/native/test_encode_pslice_intra_fallback.cpp for the "does this
 * actually trigger on a real scene cut, and does skipping it hurt
 * quality" check this was verified against).
 */
inline bool shouldUseIntraInPSlice(int intraSad, int interSad) {
  const int kInterFavorMargin = 128;
  return intraSad + kInterFavorMargin < interSad;
}

/**
 * Motion-compensates a 16x16 macroblock (luma + both chroma planes) from
 * `ref` into `ctx.frame` at the current macroblock position - the
 * encoder-side counterpart of decoder/h264_macroblock_inter.h's
 * motionCompensatePartition(), specialized to a single whole-MB
 * partition (this milestone's only shape) and a fixed refIdx of 0
 * (single-reference-frame scope). Reuses h264_motion.h's
 * motionCompLuma()/motionCompChroma() directly - the same interpolation
 * code the decoder uses, so a chosen integer-pel MV (fracX=fracY=0 at
 * the luma level; chroma can still land on a sub-pel position for an
 * odd integer luma MV, per clause 8.4.1.4's unit conversion, so the
 * chroma bilinear path still runs for real) reconstructs bit-identically
 * to what decoding this bitstream back would give.
 */
inline void motionCompensate16x16(MbEncodeContext& ctx,
                                   const Frame& ref, int16_t mvX,
                                   int16_t mvY) {
  int px = ctx.mbX * 16, py = ctx.mbY * 16;
  motionCompLuma(ctx.frame->yRow(py) + px, ctx.frame->strideY, ref, px, py,
                 16, 16, mvX, mvY);
  int cpx = ctx.mbX * 8, cpy = ctx.mbY * 8;
  motionCompChroma(ctx.frame->uRow(cpy) + cpx, ctx.frame->strideC, ref.u(),
                    ref.strideC, ref.width / 2, ref.height / 2, cpx, cpy, 8,
                    8, mvX, mvY);
  motionCompChroma(ctx.frame->vRow(cpy) + cpx, ctx.frame->strideC, ref.v(),
                    ref.strideC, ref.width / 2, ref.height / 2, cpx, cpy, 8,
                    8, mvX, mvY);
}

/**
 * Chroma transform/quantize for an Inter macroblock - structurally the
 * same math as quantizeChromaIntra() (h264_macroblock_encode.h), minus
 * the chroma mode search/signaling (Inter macroblocks have no
 * intra_chroma_pred_mode at all - chroma prediction is whatever motion
 * compensation already wrote into ctx.frame, matching
 * decodeMacroblockInter()'s chroma handling, which is pure residual
 * decode with no separate prediction-mode syntax). Not shared with
 * quantizeChromaIntra() itself since threading a "skip the mode search"
 * flag through it would complicate that function's signature for a
 * caller (Inter) that's the minority case; writeChromaResidual() *is*
 * shared as-is, since it's already mode-independent (pure quantized-
 * coefficient bitstream writing + reconstruction).
 */
inline void quantizeChromaInter(MbEncodeContext& ctx,
                                 MacroblockInfo& mb, int targetQp,
                                 int32_t chromaBlocks[2][4][16],
                                 int32_t chromaDcGrid[2][4]) {
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
  mb.cbpChroma = chromaAcNonzero ? 2 : (chromaDcNonzero ? 1 : 0);
}

/**
 * Motion-compensates the current macroblock at (mvX,mvY) and computes its
 * full P_L0_16x16 residual (luma 4x4 transform/quantize for all 16
 * blocks, chroma via quantizeChromaInter()) into caller-provided storage,
 * without writing any bitstream - the "residual computation" half of what
 * used to be one monolithic encodeMacroblockInter16x16(). Fills `mb`
 * (type=kMbInter, mv/refIdx via fillPartitionMv(), cbpLuma/cbpChroma) and
 * `lumaBlocks`/`chromaBlocks`/`chromaDcGrid` with the real quantized
 * coefficients - not a throwaway trial - so a caller that already needs
 * this MV's residual for a skip check (see wouldHaveZeroResidual() below)
 * can hand the *same* computed result straight to finishInterMacroblock()
 * instead of paying for motion compensation and the full transform/
 * quantize pass a second time. Returns whether the residual is entirely
 * zero (both luma and chroma - the hadamard2x2()-mixed chroma DC block
 * means this can't be determined from any single 4x4 block in isolation).
 * Leaves the motion-compensated prediction sitting in ctx.frame as a side
 * effect (needed by whichever caller runs next either way: real residual
 * reconstruction if non-zero, or the skip's own final state if zero).
 */
inline bool computeInterResidual(MbEncodeContext& ctx,
                                  const Frame& ref, int16_t mvX,
                                  int16_t mvY, int targetQp, MacroblockInfo& mb,
                                  int32_t lumaBlocks[16][16],
                                  int32_t chromaBlocks[2][4][16],
                                  int32_t chromaDcGrid[2][4]) {
  mb = MacroblockInfo();
  mb.type = kMbInter;
  fillPartitionMv(mb, 0, 0, 4, 4, mvX, mvY, 0);

  motionCompensate16x16(ctx, ref, mvX, mvY);

  int px0 = ctx.mbX * 16, py0 = ctx.mbY * 16;
  int cbpLuma = 0;
  for (int blk = 0; blk < 16; blk++) {
    int bx = kBlk4x4X[blk], by = kBlk4x4Y[blk];
    int px = px0 + bx * 4, py = py0 + by * 4;
    subtractBlock4x4(ctx.srcY + (size_t)py * ctx.srcStrideY + px,
                      ctx.srcStrideY, ctx.frame->yRow(py) + px,
                      ctx.frame->strideY, lumaBlocks[blk]);
    forwardDct4x4(lumaBlocks[blk]);
    if (quantizeBlock4x4(lumaBlocks[blk], targetQp, false))
      cbpLuma |= (1 << (blk / 4));
  }
  mb.cbpLuma = (uint8_t)cbpLuma;

  quantizeChromaInter(ctx, mb, targetQp, chromaBlocks, chromaDcGrid);

  return mb.cbpLuma == 0 && mb.cbpChroma == 0;
}

/**
 * Checks whether motion-compensating the current macroblock with
 * (mvX,mvY) and quantizing the resulting residual at `qp` would produce
 * an all-zero residual - used by the encoder's P-slice loop to decide
 * whether a macroblock whose motion-estimated best MV happens to equal
 * the *inferred* P_Skip MV (see skipMv() above) can actually be skipped
 * (skip always means "zero residual", so this check is what makes that
 * valid rather than just plausible). A thin wrapper around
 * computeInterResidual() that discards the computed mb/blocks - kept for
 * callers that only need the yes/no answer (see
 * test/native/test_encode_pslice_intra_fallback.cpp); Encoder::
 * encodePFrame() itself calls computeInterResidual() directly instead, so
 * it can reuse the computed residual via finishInterMacroblock() rather
 * than recomputing it from scratch when the answer turns out to be "no".
 */
inline bool wouldHaveZeroResidual(MbEncodeContext& ctx,
                                   const Frame& ref, int16_t mvX,
                                   int16_t mvY, int qp) {
  MacroblockInfo throwawayMb;
  int32_t throwawayLuma[16][16];
  int32_t throwawayChroma[2][4][16];
  int32_t throwawayChromaDc[2][4];
  return computeInterResidual(ctx, ref, mvX, mvY, qp, throwawayMb,
                               throwawayLuma, throwawayChroma,
                               throwawayChromaDc);
}

/**
 * Writes one P_L0_16x16 macroblock's bitstream (mb_type, mvd_l0, cbp,
 * mb_qp_delta, luma/chroma residual) and reconstructs it into ctx.frame,
 * from a residual already computed by computeInterResidual() for the same
 * (mvX,mvY) - the "bitstream writing" half of what used to be one
 * monolithic encodeMacroblockInter16x16() (see that function, now just a
 * thin compute-then-finish wrapper, and computeInterResidual()'s own
 * comment for why this split exists). `mb` is copied into
 * ctx.mbInfo->at(ctx.mbX, ctx.mbY) as-is (its cbpLuma/cbpChroma/mv/refIdx
 * were already set by computeInterResidual()); only qpY/nnz are filled in
 * here, since those depend on whether this macroblock actually has
 * residual to code. Returns the running QP after this macroblock (see the
 * mb_qp_delta gating note below - same "only coded if there's residual"
 * rule as I_4x4, not I_16x16's unconditional one).
 */
inline int finishInterMacroblock(BitWriter& bw, MbEncodeContext& ctx,
                                  const MacroblockInfo& computedMb,
                                  int32_t lumaBlocks[16][16],
                                  int32_t chromaBlocks[2][4][16],
                                  int32_t chromaDcGrid[2][4], int16_t mvX,
                                  int16_t mvY, int qpPrev, int targetQp) {
  MacroblockInfo& mb = ctx.mbInfo->at(ctx.mbX, ctx.mbY);
  mb = computedMb;
  int px0 = ctx.mbX * 16, py0 = ctx.mbY * 16;

  // --- mb_type, ref_idx_l0, mvd_l0 ---------------------------------------
  bw.ue(kPL0_16x16);
  /*
   * ref_idx_l0: decodeRefIdx() reads zero bits when numActiveRefs <= 1
   * (this milestone's only case, single-reference) - nothing to write.
   */
  int16_t predMv[2];
  predictMvGeneral(ctx, 0, 0, 4, 4, /*curRefIdx=*/0, -1, predMv);
  bw.se(mvX - predMv[0]);
  bw.se(mvY - predMv[1]);

  // --- coded_block_pattern (Inter uses kCbpInter, not kCbpIntra4x4) -----
  uint32_t cbpCode = 0;
  {
    uint8_t want = (uint8_t)(mb.cbpLuma | (mb.cbpChroma << 4));
    for (uint32_t i = 0; i < 48; i++) {
      if (kCbpInter[i] == want) {
        cbpCode = i;
        break;
      }
    }
  }
  bw.ue(cbpCode);

  /*
   * --- mb_qp_delta: only if there's residual (same gate as I_4x4, not
   * I_16x16's unconditional one - decodeMacroblockInter() never has an
   * `is16x16`-style override).
   */
  bool hasResidual = mb.cbpLuma != 0 || mb.cbpChroma != 0;
  int qpUsed = hasResidual ? targetQp : qpPrev;
  if (hasResidual) bw.se(targetQp - qpPrev);
  mb.qpY = (int8_t)qpUsed;

  // --- Luma residual: per-block, gated on this block's quadrant's cbpLuma
  for (int blk = 0; blk < 16; blk++) {
    if (mb.cbpLuma & (1 << (blk / 4))) {
      int bx = kBlk4x4X[blk], by = kBlk4x4Y[blk];
      int px = px0 + bx * 4, py = py0 + by * 4;
      int32_t scan[16];
      for (int k = 0; k < 16; k++) scan[k] = lumaBlocks[blk][kZigZag4x4[k]];
      int nA = lumaNeighborNnz(ctx, bx, by, -1, 0);
      int nB = lumaNeighborNnz(ctx, bx, by, 0, -1);
      int nC = predictNc(nA, nB);
      encodeResidualBlockCavlc(bw, nC, 16, scan);
      uint32_t totalCoeff = 0;
      for (int k = 0; k < 16; k++) if (scan[k] != 0) totalCoeff++;
      mb.nnz[blk] = (uint8_t)totalCoeff;

      int32_t block[16];
      for (int i = 0; i < 16; i++) block[i] = lumaBlocks[blk][i];
      dequant4x4(block, targetQp, false);
      idct4x4(block);
      addResidual4x4(ctx.frame->yRow(py) + px, ctx.frame->strideY, block);
    } else {
      mb.nnz[blk] = 0;
    }
  }

  writeChromaResidual(bw, ctx, mb, targetQp, chromaBlocks, chromaDcGrid);

  return qpUsed;
}

/**
 * Encodes one P_L0_16x16 macroblock: motion compensation, residual
 * transform/quantize/CAVLC (the residual side mirrors
 * encodeMacroblockIntra4x4()'s luma loop closely - full per-4x4-block
 * transform, no Hadamard DC special-case, since that only applies to
 * Intra16x16 - and reuses quantizeChromaInter()/writeChromaResidual() for
 * chroma). A thin wrapper around computeInterResidual() +
 * finishInterMacroblock() (see each one's own comment) for callers that
 * don't already have a computed residual to reuse - Encoder::
 * encodePFrame() calls the two split halves directly instead, to avoid
 * recomputing a residual it may have already computed via
 * wouldHaveZeroResidual()'s skip check. Returns the running QP after this
 * macroblock.
 */
inline int encodeMacroblockInter16x16(BitWriter& bw,
                                       MbEncodeContext& ctx,
                                       const Frame& ref,
                                       int16_t mvX, int16_t mvY, int qpPrev,
                                       int targetQp) {
  MacroblockInfo mb;
  int32_t lumaBlocks[16][16];
  int32_t chromaBlocks[2][4][16];
  int32_t chromaDcGrid[2][4];
  computeInterResidual(ctx, ref, mvX, mvY, targetQp, mb, lumaBlocks,
                        chromaBlocks, chromaDcGrid);
  return finishInterMacroblock(bw, ctx, mb, lumaBlocks, chromaBlocks,
                                chromaDcGrid, mvX, mvY, qpPrev, targetQp);
}

/**
 * Determines the MV a decoder would infer for a P_Skip macroblock at
 * the current position (clause 8.4.1.1) - the encoder-side counterpart
 * of decoder/h264_macroblock_inter.h's decodePSkipMacroblock(), factored
 * out so the encoder can check "does my motion-estimated best MV match
 * what skip would give for free" without duplicating the zero-mv rule.
 */
inline void skipMv(const MbEncodeContext& ctx, int16_t* outMv) {
  MvNeighbor a = mvNeighborAt(ctx, -1, 0);
  MvNeighbor b = mvNeighborAt(ctx, 0, -1);
  bool zeroMv =
      (!a.available || !b.available)
          ? true
          : ((a.interCoded && a.refIdx == 0 && a.mv[0] == 0 && a.mv[1] == 0) ||
             (b.interCoded && b.refIdx == 0 && b.mv[0] == 0 && b.mv[1] == 0));
  if (zeroMv) {
    outMv[0] = outMv[1] = 0;
  } else {
    predictMvGeneral(ctx, 0, 0, 4, 4, 0, -1, outMv);
  }
}

/**
 * Encodes a P_Skip macroblock: no macroblock_layer() syntax at all
 * (clause 7.3.5 - a skip is signaled purely by mb_skip_run in the slice
 * data, handled by the caller, Encoder::encodePFrame()) - just motion-
 * compensates with the inferred skipMv() and marks the macroblock's
 * state for future neighbor lookups, mirroring decodePSkipMacroblock()
 * exactly (always refIdx 0, no residual).
 */
inline void encodeMacroblockPSkip(MbEncodeContext& ctx,
                                   const Frame& ref, int qp) {
  MacroblockInfo& mb = ctx.mbInfo->at(ctx.mbX, ctx.mbY);
  mb = MacroblockInfo();
  mb.type = kMbPSkip;
  mb.qpY = (int8_t)qp;

  int16_t mv[2];
  skipMv(ctx, mv);
  fillPartitionMv(mb, 0, 0, 4, 4, mv[0], mv[1], 0);
  motionCompensate16x16(ctx, ref, mv[0], mv[1]);
}

}  // namespace tinyh264
