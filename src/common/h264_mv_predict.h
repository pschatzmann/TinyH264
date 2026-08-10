#pragma once
#include <stdint.h>
#include "h264_mb_info.h"
#include "h264_motion.h"

/*
 * Header-only. Motion vector *prediction* (clause 8.4.1.3: the neighbor
 * lookup and median/directional-shortcut logic that turns A/B/C neighbor
 * MVs into a predicted MV) - shared by the decoder (decoder/
 * h264_macroblock_inter.h, which adds mvd_l0 to get the real MV) and the
 * encoder (encoder/h264_macroblock_encode_inter.h, which subtracts the
 * predicted MV from a motion-estimated real MV to get mvd_l0 to encode).
 * Templated on the context type (structural, not tied to
 * decoder::MbDecodeContext) the same way lumaNeighborNnz()/
 * predictIntra4x4Mode() (h264_mb_info.h) are - both decoder::
 * MbDecodeContext and encoder::MbEncodeContext expose the same
 * mbInfo/mbX/mbY/sliceId members this needs. Motion *compensation*
 * (turning a final MV into predicted pixels) is NOT here - that needs a
 * reference-picture list, which decoder::MbDecodeContext and encoder::
 * MbEncodeContext represent differently (the decoder's ctx.refList vs.
 * the encoder's own single-reference-frame storage, see
 * encoder/h264_encoder.h) - each side wraps h264_motion.h's
 * motionCompLuma()/motionCompChroma() (already context-independent) with
 * its own thin adapter instead.
 */

namespace tinyh264 {

/**
 * A neighbor 4x4-grid lookup result for MV prediction: available means the
 * neighbor exists (in-picture, same slice); interCoded additionally means
 * it carries a real (non-intra) MV - intra neighbors are "available" with
 * mv=(0,0) but never satisfy a refIdx-match shortcut. `refIdx` is -1 for
 * Intra/unavailable neighbors (clause 8.4.1.3.2's refIdxLXN == -1), never
 * equal to any real partition's refIdx (always >= 0). Produced by
 * mvNeighborAt()/mvNeighborC(), consumed by predictMvGeneral().
 */
struct MvNeighbor {
  bool available = false;
  bool interCoded = false;
  int8_t refIdx = -1;
  int16_t mv[2] = {0, 0};
};

/**
 * Looks up the 4x4-grid neighbor at (bx, by) relative to the current
 * macroblock's top-left, following the same local/cross-MB derivation as
 * lumaNeighborNnz/predictIntra4x4Mode (h264_mb_info.h): local positions
 * use the current MB's own (partially filled-in) mv[] array, positions
 * crossing bx<0/by<0 use the appropriate already-decoded neighbor MB.
 * Positions with bx>=4 (crossing right) or by>=4 (crossing down) are
 * never available (those macroblocks/partitions haven't been
 * decoded/encoded yet in raster order) - the sole exception is bx==4
 * with by<0 (the top-right neighbor MB), which legitimately can be
 * available.
 */
template <typename CtxT>
inline MvNeighbor mvNeighborAt(const CtxT& ctx, int bx, int by) {
  MvNeighbor n;
  /*
   * "Below" is never decoded yet regardless of column. "Right" (bx>=4) is
   * only ever unavailable when it's the same row or below (the plain right
   * MB, never decoded yet); bx==4 combined with by<0 is the top-right MB,
   * which legitimately can be available.
   */
  if (by >= 4) return n;
  if (bx >= 4 && by >= 0) return n;
  if (bx >= 0 && by >= 0) {
    const MacroblockInfo& cur = ctx.mbInfo->at(ctx.mbX, ctx.mbY);
    n.available = true;
    n.interCoded = true;  // within-MB: only reached while decoding/encoding
                           // an Inter MB
    int idx = kInvBlk4x4[by][bx];
    n.refIdx = cur.refIdx[idx];
    n.mv[0] = cur.mv[idx][0];
    n.mv[1] = cur.mv[idx][1];
    return n;
  }
  int nmbX = ctx.mbX + (bx < 0 ? -1 : (bx >= 4 ? 1 : 0));
  int nmbY = ctx.mbY + (by < 0 ? -1 : 0);
  if (!ctx.mbInfo->decoded(nmbX, nmbY, ctx.sliceId)) return n;
  const MacroblockInfo& nmb = ctx.mbInfo->at(nmbX, nmbY);
  n.available = true;
  if (nmb.isIntra()) {
    n.interCoded = false;
    n.mv[0] = n.mv[1] = 0;
    return n;
  }
  n.interCoded = true;
  int wbx = (bx < 0) ? 3 : (bx >= 4 ? bx - 4 : bx);
  int wby = (by < 0) ? 3 : by;
  int idx = kInvBlk4x4[wby][wbx];
  n.refIdx = nmb.refIdx[idx];
  n.mv[0] = nmb.mv[idx][0];
  n.mv[1] = nmb.mv[idx][1];
  return n;
}

/**
 * Neighbor "C" for MV prediction (clause 8.4.1.3.2): the partition
 * immediately above-right of (bx,by) sized pw x ph (4x4 units), i.e. at
 * grid position (bx+pw, by-1), falling back to "D" (above-left, bx-1,
 * by-1) when C is unavailable. Mirrors the intra4x4 top-right exception
 * (h264_intra_pred.h) generalized to arbitrary partition widths: unlike
 * the strictly-4x4 intra case, a partition can be wider than one 4x4
 * block, so the "always unavailable" set isn't a fixed table - this just
 * applies the same local-vs-cross-MB reasoning directly to the query
 * position via mvNeighborAt().
 */
template <typename CtxT>
inline MvNeighbor mvNeighborC(const CtxT& ctx, int bx, int by, int pw,
                               int ph) {
  MvNeighbor c = mvNeighborAt(ctx, bx + pw, by - 1);
  if (c.available) return c;
  return mvNeighborAt(ctx, bx - 1, by - 1);  // D fallback (top-left)
}

/**
 * Standard median MV predictor (clause 8.4.1.3) for a partition at 4x4-grid
 * position (bx,by) sized pw x ph (in 4x4 units) using reference index
 * `curRefIdx`, writing the predicted MV into `outMv`. `directionalSide` is
 * -1 for no shortcut, 0 to try the "use A directly if refIdx matches"
 * shortcut (8x16 left, 16x8 bottom), 1 for "use B directly" (16x8 top), 2
 * for "use C directly" (8x16 right).
 */
template <typename CtxT>
inline void predictMvGeneral(const CtxT& ctx, int bx, int by, int pw, int ph,
                              int curRefIdx, int directionalSide,
                              int16_t* outMv) {
  MvNeighbor a = mvNeighborAt(ctx, bx - 1, by);
  MvNeighbor b = mvNeighborAt(ctx, bx, by - 1);
  MvNeighbor c = mvNeighborC(ctx, bx, by, pw, ph);

  if (directionalSide >= 0) {
    /*
     * P_L0_L0_16x8 / P_L0_L0_8x16 additionally get a one-sided pre-check
     * (clause 8.4.1.3, Table 8-9): if the *one* relevant side has a
     * matching ref_idx, use it directly. This is tried *before*, not
     * instead of, the general process below - if it doesn't apply, the
     * general "exactly one of A/B/C matches" rule can still apply. (Easy
     * to get wrong: a "one-sided check replaces the whole rest of the
     * process" reading looks plausible and mostly works, but fails
     * whenever the directional side doesn't match yet exactly one of the
     * *other* two neighbors does - caught via pixel-diffing full-frame P-
     * slice decodes against ffmpeg's reference, not by inspection.)
     */
    const MvNeighbor& side = directionalSide == 0 ? a
                              : directionalSide == 1 ? b
                                                      : c;
    if (side.interCoded && side.refIdx == curRefIdx) {
      outMv[0] = side.mv[0];
      outMv[1] = side.mv[1];
      return;
    }
  }
  {
    /*
     * General process (clause 8.4.1.3.1): "matching ref_idx" means
     * interCoded *and* refIdx == curRefIdx - unavailable/Intra neighbors
     * have refIdx == -1 (clause 8.4.1.3.2), which can never equal a real
     * partition's refIdx (always >= 0), so they never match without a
     * separate availability check. If exactly one of A/B/C matches, the
     * predictor is that one neighbor's MV directly.
     */
    bool ma = a.interCoded && a.refIdx == curRefIdx;
    bool mb_ = b.interCoded && b.refIdx == curRefIdx;
    bool mc = c.interCoded && c.refIdx == curRefIdx;
    int matches = (ma ? 1 : 0) + (mb_ ? 1 : 0) + (mc ? 1 : 0);
    if (matches == 1) {
      const MvNeighbor& only = ma ? a : (mb_ ? b : c);
      outMv[0] = only.mv[0];
      outMv[1] = only.mv[1];
      return;
    }
    /*
     * Zero-matches special case (still clause 8.4.1.3.1, cross-checked
     * against FFmpeg's pred_motion() - NOT reducible to "exactly one
     * match" as this project's earlier draft comment here incorrectly
     * claimed): when B and C are both *truly* unavailable (picture/slice
     * edge - not merely "available but Intra or non-matching ref_idx",
     * which still count as available here) and A is available in any
     * form, the predictor is A's MV directly, regardless of whether A's
     * own ref_idx matches curRefIdx or A is even Inter-coded at all (an
     * Intra A falls through to mv (0,0) via its default MvNeighbor
     * fields, matching what the ordinary median would give anyway - the
     * observable difference only shows up when A is Inter-coded with a
     * non-matching ref_idx, where this bypasses the median entirely and
     * uses A's real MV. Missing this case was a real bug, caught by
     * pixel-diffing a real multi-reference-frame P-slice stream against
     * ffmpeg's decode - a single-reference-frame stream can never
     * exercise "A available with a different ref_idx", so this bug was
     * invisible until multi-reference support was added.)
     */
    if (matches == 0 && !b.available && !c.available && a.available) {
      outMv[0] = a.mv[0];
      outMv[1] = a.mv[1];
      return;
    }
  }

  MvPredictorInput in;
  in.haveA = a.available;
  in.mvA[0] = a.mv[0];
  in.mvA[1] = a.mv[1];
  in.haveB = b.available;
  in.mvB[0] = b.mv[0];
  in.mvB[1] = b.mv[1];
  in.haveC = c.available;
  in.mvC[0] = c.mv[0];
  in.mvC[1] = c.mv[1];
  predictMv(in, outMv);
}

/**
 * Fills mb.mv[]/mb.refIdx[] for the 4x4 cells covered by a partition at
 * (bx,by) sized pw x ph (4x4 units), so later neighbor lookups (this MB's
 * own remaining partitions, and subsequent macroblocks) see a uniform
 * per-4x4-block MV/refIdx regardless of the actual partition shape (16x16
 * down to 4x4).
 */
inline void fillPartitionMv(MacroblockInfo& mb, int bx, int by, int pw,
                             int ph, int16_t mvX, int16_t mvY,
                             int8_t refIdx) {
  for (int y = by; y < by + ph; y++) {
    for (int x = bx; x < bx + pw; x++) {
      int idx = kInvBlk4x4[y][x];
      mb.mv[idx][0] = mvX;
      mb.mv[idx][1] = mvY;
      mb.refIdx[idx] = refIdx;
    }
  }
}

}  // namespace tinyh264
