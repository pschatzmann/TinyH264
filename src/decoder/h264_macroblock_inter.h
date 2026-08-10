#pragma once
#include <stdint.h>
#include "h264_bitreader.h"
#include "h264_macroblock.h"  // shared helpers: lumaNeighborNnz, decodeLumaBlockFull, etc.
#include "../common/h264_motion.h"
#include "../common/h264_mv_predict.h"
#include "../common/h264_tables.h"

/*
 * Header-only. P-slice (Inter) macroblock_layer() decode: mb_type/
 * sub_mb_type parsing, ref_idx_l0 parsing, motion vector prediction
 * (clause 8.4.1.3) and reconstruction, motion compensation, and residual
 * decode (reusing the same CAVLC/dequant/IDCT machinery as intra blocks -
 * Inter differs only in prediction, not residual coding). P_Skip is
 * handled separately since it has no macroblock_layer() syntax at all.
 *
 * Up to H264_MAX_REF_FRAMES reference pictures are supported (see
 * h264_config.h and Decoder::setMaxRefFrames()). Intra neighbors, and
 * truly-unavailable neighbors, are assigned refIdx -1 for MV-prediction
 * purposes (clause 8.4.1.3.2) - never equal to a real partition's refIdx
 * (always >= 0), so "does the neighbor's ref_idx match" comparisons work
 * out correctly without a separate availability check.
 */

namespace tinyh264 {

/**
 * Motion-compensates a partition (in pixel units derived from its 4x4-grid
 * position/size) from reference picture `refIdx` (an index into
 * ctx.refList, clause 8.2.4's RefPicList0) into the current frame, luma
 * and both chroma planes (clause 8.4.2), writing the prediction directly
 * - the caller adds the residual on top afterward (see
 * decodeMacroblockInter()'s residual section).
 */
template <typename Allocator>
inline void motionCompensatePartition(MbDecodeContext<Allocator>& ctx, int bx, int by,
                                       int pw, int ph, int16_t mvX,
                                       int16_t mvY, int8_t refIdx) {
  const Frame<Allocator>& ref = *ctx.refList[refIdx];
  int px = ctx.mbX * 16 + bx * 4, py = ctx.mbY * 16 + by * 4;
  int w = pw * 4, h = ph * 4;
  motionCompLuma(ctx.frame->yRow(py) + px, ctx.frame->strideY, ref,
                 px, py, w, h, mvX, mvY);

  int cpx = ctx.mbX * 8 + bx * 2, cpy = ctx.mbY * 8 + by * 2;
  int cw = pw * 2, ch = ph * 2;
  motionCompChroma(ctx.frame->uRow(cpy) + cpx, ctx.frame->strideC,
                    ref.u(), ref.strideC,
                    ref.width / 2, ref.height / 2, cpx,
                    cpy, cw, ch, mvX, mvY);
  motionCompChroma(ctx.frame->vRow(cpy) + cpx, ctx.frame->strideC,
                    ref.v(), ref.strideC,
                    ref.width / 2, ref.height / 2, cpx,
                    cpy, cw, ch, mvX, mvY);
}

/**
 * Decodes a P_Skip macroblock (clause 8.4.1.1) - no macroblock_layer()
 * syntax is present in the bitstream at all for a skipped MB (the caller,
 * Decoder::decodeSlice(), determines skip runs from mb_skip_run and calls
 * this once per skipped macroblock). Always references refIdx 0 (clause
 * 8.4.1.1). Derives a single 16x16 motion vector (special-cased zero-MV
 * rule, not the general median predictor, when either neighbor is
 * unavailable or uses refIdx 0 with a zero MV) and performs full-MB
 * motion compensation with no residual.
 */
template <typename Allocator>
inline void decodePSkipMacroblock(MbDecodeContext<Allocator>& ctx, int qpY) {
  MacroblockInfo& mb = ctx.mbInfo->at(ctx.mbX, ctx.mbY);
  mb = MacroblockInfo();
  mb.type = kMbPSkip;
  mb.qpY = (int8_t)qpY;

  MvNeighbor a = mvNeighborAt(ctx, -1, 0);
  MvNeighbor b = mvNeighborAt(ctx, 0, -1);

  /*
   * Per spec, the explicit zero-mv rule is specifically refIdxN==0 &&
   * mvN==(0,0) for an *available*, inter-coded neighbor; an unavailable
   * neighbor (either side) also forces zero mv.
   */
  int16_t mv[2] = {0, 0};
  bool zeroMv =
      (!a.available || !b.available)
          ? true
          : ((a.interCoded && a.refIdx == 0 && a.mv[0] == 0 && a.mv[1] == 0) ||
             (b.interCoded && b.refIdx == 0 && b.mv[0] == 0 && b.mv[1] == 0));
  if (zeroMv) {
    mv[0] = mv[1] = 0;
  } else {
    predictMvGeneral(ctx, 0, 0, 4, 4, 0, -1, mv);
  }

  fillPartitionMv(mb, 0, 0, 4, 4, mv[0], mv[1], 0);
  motionCompensatePartition(ctx, 0, 0, 4, 4, mv[0], mv[1], 0);
}

/**
 * Decodes mvd_l0[][][0..1] (clause 7.3.5.1: two se(v) values, x then y)
 * for one motion vector partition.
 */
inline bool decodeMvd(BitReader& br, int16_t* mvd) {
  int32_t x = br.se();
  int32_t y = br.se();
  if (br.error()) return false;
  mvd[0] = (int16_t)x;
  mvd[1] = (int16_t)y;
  return true;
}

/**
 * Decodes one ref_idx_l0[] value, coded as te(v) (truncated Exp-Golomb,
 * clause 9.1.1) with cMax = numActiveRefs - 1. Cross-checked against
 * FFmpeg's h264_cavlc.c rather than re-derived from the spec's te(v)
 * definition alone: when numActiveRefs <= 1 there is only one legal value
 * and *no bits are read at all* (matches the syntax table's `if
 * (num_ref_idx_l0_active_minus1 > 0 ...)` guard around ref_idx_l0 - so
 * this function can be called unconditionally, whether or not the guard
 * would have been true, and naturally reproduces "not present" as "read
 * nothing, value 0"); when numActiveRefs == 2 it's a single inverted bit
 * (te(v)'s cMax==1 special case: value = !u(1), NOT plain u(1)); only for
 * numActiveRefs > 2 does it become a plain bounds-checked ue(v). Returns
 * false on a bitstream error or an out-of-range decoded value.
 */
inline bool decodeRefIdx(BitReader& br, int numActiveRefs, int8_t* outRefIdx) {
  if (numActiveRefs <= 1) {
    *outRefIdx = 0;
    return true;
  }
  if (numActiveRefs == 2) {
    *outRefIdx = (int8_t)(br.flag() ? 0 : 1);
    return !br.error();
  }
  uint32_t v = br.ue();
  if (br.error() || (int)v >= numActiveRefs) return false;
  *outRefIdx = (int8_t)v;
  return true;
}

/**
 * Decodes one Inter (P) macroblock's full macroblock_layer() (clause
 * 7.3.5): mb_type/sub_mb_type + partition layout, per-partition MV
 * prediction and reconstruction with immediate motion compensation, then
 * coded_block_pattern/mb_qp_delta and the luma/chroma residual (reusing
 * the same CAVLC/dequant/IDCT helpers as the Intra path in
 * h264_macroblock.h - Inter differs from Intra only in how the
 * prediction samples are produced, not in residual coding). Falls
 * through to decodeMacroblockIntraWithType() for the "Intra macroblock
 * inside a P slice" case (mb_type >= 5). `*qpY` is the running QP,
 * updated in place; `result` reports unsupported-feature/error status.
 */
template <typename Allocator>
inline bool decodeMacroblockInter(BitReader& br, MbDecodeContext<Allocator>& ctx,
                                   int* qpY, MacroblockDecodeResult* result) {
  *result = MacroblockDecodeResult();
  MacroblockInfo& mb = ctx.mbInfo->at(ctx.mbX, ctx.mbY);
  mb = MacroblockInfo();
  mb.type = kMbInter;

  uint32_t mbTypeRaw = br.ue();
  if (mbTypeRaw >= 5) {
    /*
     * Intra macroblock inside a P slice: same numbering as I-slice
     * mb_type, offset by 5 (clause 7.4.5, Table 7-13/7-14).
     */
    return decodeMacroblockIntraWithType(br, ctx, mbTypeRaw - 5, qpY, result);
  }

  /*
   * --- Partition layout + ref_idx_l0 + MV prediction/reconstruction ------
   * Syntax order (clause 7.3.5.1 mb_pred() / 7.3.5.2 sub_mb_pred(), cross-
   * checked against FFmpeg's h264_cavlc.c): ALL ref_idx_l0 values for a
   * macroblock are read first (one per top-level partition, or one per
   * 8x8 quadrant for P_8x8/P_8x8ref0 - shared by every sub-partition
   * within that quadrant, even if sub_mb_type further splits it), THEN
   * all mvd_l0 values are read afterward - the two are NOT interleaved
   * per-partition.
   */
  struct Part {
    int bx, by, pw, ph, dirSide;
    int8_t refIdx;
  };
  Part parts[4];
  int numParts = 0;

  if (mbTypeRaw == kPL0_16x16) {
    int8_t r;
    if (!decodeRefIdx(br, ctx.numActiveRefs, &r)) return false;
    parts[numParts++] = {0, 0, 4, 4, -1, r};
  } else if (mbTypeRaw == kPL0_L0_16x8) {
    int8_t r0, r1;
    if (!decodeRefIdx(br, ctx.numActiveRefs, &r0)) return false;
    if (!decodeRefIdx(br, ctx.numActiveRefs, &r1)) return false;
    parts[numParts++] = {0, 0, 4, 2, 1, r0};
    parts[numParts++] = {0, 2, 4, 2, 0, r1};
  } else if (mbTypeRaw == kPL0_L0_8x16) {
    int8_t r0, r1;
    if (!decodeRefIdx(br, ctx.numActiveRefs, &r0)) return false;
    if (!decodeRefIdx(br, ctx.numActiveRefs, &r1)) return false;
    parts[numParts++] = {0, 0, 2, 4, 0, r0};
    parts[numParts++] = {2, 0, 2, 4, 2, r1};
  } else {
    /*
     * P_8x8 / P_8x8ref0: four 8x8 (2x2 in 4x4 units) quadrants, each with
     * its own sub_mb_type. P_8x8ref0 never codes ref_idx_l0 (always 0,
     * regardless of numActiveRefs) - clause 7.3.5.2's
     * `mb_type != P_8x8ref0` guard.
     */
    uint32_t subType[4];
    for (int i = 0; i < 4; i++) subType[i] = br.ue();
    if (br.error()) return false;

    int8_t quadRefIdx[4];
    for (int q = 0; q < 4; q++) {
      if (mbTypeRaw == kP8x8ref0) {
        quadRefIdx[q] = 0;
      } else if (!decodeRefIdx(br, ctx.numActiveRefs, &quadRefIdx[q])) {
        return false;
      }
    }

    static const int quadX[4] = {0, 2, 0, 2};
    static const int quadY[4] = {0, 0, 2, 2};
    for (int q = 0; q < 4; q++) {
      int qx = quadX[q], qy = quadY[q];
      int8_t r = quadRefIdx[q];
      switch (subType[q]) {
        case kSub8x8:
          parts[numParts++] = {qx, qy, 2, 2, -1, r};
          break;
        case kSub8x4:
          parts[numParts++] = {qx, qy, 2, 1, -1, r};
          parts[numParts++] = {qx, qy + 1, 2, 1, -1, r};
          break;
        case kSub4x8:
          parts[numParts++] = {qx, qy, 1, 2, -1, r};
          parts[numParts++] = {qx + 1, qy, 1, 2, -1, r};
          break;
        case kSub4x4:
        default:
          parts[numParts++] = {qx, qy, 1, 1, -1, r};
          parts[numParts++] = {qx + 1, qy, 1, 1, -1, r};
          parts[numParts++] = {qx, qy + 1, 1, 1, -1, r};
          parts[numParts++] = {qx + 1, qy + 1, 1, 1, -1, r};
          break;
      }
    }
  }

#ifdef TINYH264_DEBUG_MB
  fprintf(stderr, "MB(%d,%d) INTER mbType=%u numParts=%d\n", ctx.mbX, ctx.mbY,
          mbTypeRaw, numParts);
#endif
  for (int i = 0; i < numParts; i++) {
    int16_t pred[2], mvd[2];
    predictMvGeneral(ctx, parts[i].bx, parts[i].by, parts[i].pw, parts[i].ph,
                      parts[i].refIdx, parts[i].dirSide, pred);
    if (!decodeMvd(br, mvd)) return false;
    int16_t mv[2] = {(int16_t)(pred[0] + mvd[0]), (int16_t)(pred[1] + mvd[1])};
#ifdef TINYH264_DEBUG_MB
    fprintf(stderr,
            "  part%d (%d,%d %dx%d) refIdx=%d pred=(%d,%d) mvd=(%d,%d) mv=(%d,%d)\n", i,
            parts[i].bx, parts[i].by, parts[i].pw, parts[i].ph, parts[i].refIdx,
            pred[0], pred[1], mvd[0], mvd[1], mv[0], mv[1]);
#endif
    fillPartitionMv(mb, parts[i].bx, parts[i].by, parts[i].pw, parts[i].ph,
                     mv[0], mv[1], parts[i].refIdx);
    motionCompensatePartition(ctx, parts[i].bx, parts[i].by, parts[i].pw,
                               parts[i].ph, mv[0], mv[1], parts[i].refIdx);
  }

  /*
   * --- CBP / QP / residual (mirrors the Intra path in h264_macroblock.h,
   *     minus the I_16x16-specific DC-block handling which doesn't apply
   *     to Inter macroblocks) -------------------------------------------
   */
  uint32_t cbpCode = br.ue();
  if (cbpCode > 47 || br.error()) return false;
  uint8_t cbpCombined = kCbpInter[cbpCode];
  mb.cbpLuma = cbpCombined & 0xF;
  mb.cbpChroma = cbpCombined >> 4;

  int dquant = 0;
  if (mb.cbpLuma != 0 || mb.cbpChroma != 0) {
    dquant = br.se();
  }
  int qp = *qpY + dquant;
  qp = ((qp + 52) % 52 + 52) % 52;
  *qpY = qp;
  mb.qpY = (int8_t)qp;
  if (br.error()) return false;

  for (int blk = 0; blk < 16; blk++) {
    int quadrant = blk / 4;
    if (mb.cbpLuma & (1 << quadrant)) {
      if (!decodeLumaBlockFull(br, ctx, blk, qp)) return false;
    } else {
      mb.nnz[blk] = 0;
    }
  }

  int cQp = chromaQp(qp, ctx.pps->chromaQpIndexOffset);
  int32_t cbDc[4] = {0, 0, 0, 0};
  int32_t crDc[4] = {0, 0, 0, 0};
  if (mb.cbpChroma >= 1) {
    for (int plane = 0; plane < 2; plane++) {
      int32_t coeff[4];
      uint32_t totalCoeff = 0;
      if (!residualBlockCavlc(br, -1, 4, coeff, &totalCoeff)) return false;
      int32_t* dst = plane == 0 ? cbDc : crDc;
      dst[0] = coeff[0];
      dst[1] = coeff[1];
      dst[2] = coeff[2];
      dst[3] = coeff[3];
      hadamard2x2(dst);
      dequantChromaDC2x2(dst, cQp);
    }
  }
  for (int plane = 0; plane < 2; plane++) {
    const int32_t* dc = plane == 0 ? cbDc : crDc;
    for (int cy = 0; cy < 2; cy++) {
      for (int cx = 0; cx < 2; cx++) {
        int32_t dcVal = dc[cy * 2 + cx];
        if (mb.cbpChroma >= 2) {
          if (!decodeChromaBlockAc(br, ctx, plane, cx, cy, cQp, dcVal))
            return false;
        } else {
          int32_t block[16] = {0};
          block[0] = dcVal;
          idct4x4(block);
          int px = ctx.mbX * 8 + cx * 4, py = ctx.mbY * 8 + cy * 4;
          uint8_t* planeBuf = plane == 0 ? ctx.frame->u() : ctx.frame->v();
          addResidual4x4(planeBuf + (size_t)py * ctx.frame->strideC + px,
                          ctx.frame->strideC, block);
          mb.nnz[16 + plane * 4 + cy * 2 + cx] = 0;
        }
      }
    }
  }

  if (br.error()) return false;
  result->ok = true;
  return true;
}

}  // namespace tinyh264
