#pragma once
#include <stdint.h>
#include <stdlib.h>  // abs()
#include "../common/h264_frame.h"
#include "h264_macroblock.h"  // kInvBlk4x4
#include "../common/h264_mb_info.h"
#include "h264_sps_pps.h"
#include "../common/h264_transform.h"

/*
 * Header-only. In-loop deblocking filter, ITU-T H.264 clause 8.7. Table
 * values (alpha/beta/tC0) cross-checked against FFmpeg's
 * libavcodec/h264_loopfilter.c rather than transcribed from memory (same
 * rationale as h264_cavlc_tables.h / h264_tables.h).
 *
 * Boundary-strength derivation (8.7.2.1) covers all of Baseline's cases:
 * bS 4 (macroblock-boundary edges) / 3 (internal edges) whenever either
 * side is Intra-coded, bS 2 when either side has coded residual, and bS 1
 * (motion vector difference >= 4 quarter-samples) / 0 (identical motion,
 * no filtering) for Inter/P_Skip macroblocks - see boundaryStrength().
 */

namespace tinyh264 {

/// alpha threshold (Table 8-16), indexed by indexA = Clip3(0,51,qPav+FilterOffsetA).
static const uint8_t kDeblockAlpha[52] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    4,  4,  5,  6,  7,  8,  9,  10, 12, 13, 15, 17, 20, 22, 25, 28,
    32, 36, 40, 45, 50, 56, 63, 71, 80, 90, 101, 113, 127, 144, 162, 182,
    203, 226, 255, 255,
};
/// beta threshold (Table 8-16), indexed by indexB = Clip3(0,51,qPav+FilterOffsetB).
static const uint8_t kDeblockBeta[52] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    2,  2,  2,  3,  3,  3,  3,  4,  4,  4,  6,  6,  7,  7,  8,  8,
    9,  9,  10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16,
    17, 17, 18, 18,
};
/**
 * tC0 (Table 8-17), indexed [indexA][bS], bS 0..3 (bS==0 never used - no
 * filtering at all; bS==4 uses the separate "strong" filter below, not
 * this table).
 */
static const int8_t kDeblockTc0[52][4] = {
    {-1, 0, 0, 0}, {-1, 0, 0, 0}, {-1, 0, 0, 0}, {-1, 0, 0, 0},
    {-1, 0, 0, 0}, {-1, 0, 0, 0}, {-1, 0, 0, 0}, {-1, 0, 0, 0},
    {-1, 0, 0, 0}, {-1, 0, 0, 0}, {-1, 0, 0, 0}, {-1, 0, 0, 0},
    {-1, 0, 0, 0}, {-1, 0, 0, 0}, {-1, 0, 0, 0}, {-1, 0, 0, 0},
    {-1, 0, 0, 0}, {-1, 0, 0, 1}, {-1, 0, 0, 1}, {-1, 0, 0, 1},
    {-1, 0, 0, 1}, {-1, 0, 1, 1}, {-1, 0, 1, 1}, {-1, 1, 1, 1},
    {-1, 1, 1, 1}, {-1, 1, 1, 1}, {-1, 1, 1, 1}, {-1, 1, 1, 2},
    {-1, 1, 1, 2}, {-1, 1, 1, 2}, {-1, 1, 1, 2}, {-1, 1, 2, 3},
    {-1, 1, 2, 3}, {-1, 2, 2, 3}, {-1, 2, 2, 4}, {-1, 2, 3, 4},
    {-1, 2, 3, 4}, {-1, 3, 3, 5}, {-1, 3, 4, 6}, {-1, 3, 4, 6},
    {-1, 4, 5, 7}, {-1, 4, 5, 8}, {-1, 4, 6, 9}, {-1, 5, 7, 10},
    {-1, 6, 8, 11}, {-1, 6, 8, 13}, {-1, 7, 10, 14}, {-1, 8, 11, 16},
    {-1, 9, 12, 18}, {-1, 10, 13, 20}, {-1, 11, 15, 23}, {-1, 13, 17, 25},
};

/**
 * Clip3(lo, hi, v) as defined by clause 5.8, used throughout the filter
 * for both index/threshold clamping and sample-delta clamping.
 */
inline int clip3(int lo, int hi, int v) { return v < lo ? lo : (v > hi ? hi : v); }

/**
 * Filters one line of luma samples at consecutive `step` offsets from
 * `pix` (pix[-1*step]=p0, pix[0]=q0), clause 8.7.2.3 (bS<4, "Filtering
 * process for edges with bS less than 4") and 8.7.2.4 (bS==4, the
 * stronger "Filtering process for edges with bS equal to 4"). Gets the
 * full p1/p2 adjustments that filterEdgeChroma() below does not, since
 * only luma evaluates the ap/aq conditions for extending the filter to a
 * second sample on each side.
 */
inline void filterEdgeLuma(uint8_t* pix, int step, int bS, int alpha,
                            int beta, int tc0) {
  int p0 = pix[-1 * step], p1 = pix[-2 * step], p2 = pix[-3 * step];
  int q0 = pix[0], q1 = pix[1 * step], q2 = pix[2 * step];
  if (abs(p0 - q0) >= alpha || abs(p1 - p0) >= beta || abs(q1 - q0) >= beta)
    return;

  if (bS < 4) {
    bool ap = abs(p2 - p0) < beta;
    bool aq = abs(q2 - q0) < beta;
    int tc = tc0 + (ap ? 1 : 0) + (aq ? 1 : 0);
    int delta = clip3(-tc, tc, (((q0 - p0) << 2) + (p1 - q1) + 4) >> 3);
    pix[-1 * step] = clip255(p0 + delta);
    pix[0] = clip255(q0 - delta);
    if (ap) {
      pix[-2 * step] = (uint8_t)(p1 + clip3(-tc0, tc0,
                                             (p2 + ((p0 + q0 + 1) >> 1) -
                                              2 * p1) >>
                                                 1));
    }
    if (aq) {
      pix[1 * step] = (uint8_t)(q1 + clip3(-tc0, tc0,
                                            (q2 + ((p0 + q0 + 1) >> 1) -
                                             2 * q1) >>
                                                1));
    }
  } else {
    bool ap = abs(p2 - p0) < beta;
    bool aq = abs(q2 - q0) < beta;
    bool strongCond = abs(p0 - q0) < ((alpha >> 2) + 2);
    if (ap && strongCond) {
      int p3 = pix[-4 * step];
      pix[-1 * step] = (uint8_t)((p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3);
      pix[-2 * step] = (uint8_t)((p2 + p1 + p0 + q0 + 2) >> 2);
      pix[-3 * step] = (uint8_t)((2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3);
    } else {
      pix[-1 * step] = (uint8_t)((2 * p1 + p0 + q1 + 2) >> 2);
    }
    if (aq && strongCond) {
      int q3 = pix[3 * step];
      pix[0] = (uint8_t)((q2 + 2 * q1 + 2 * q0 + 2 * p0 + p1 + 4) >> 3);
      pix[1 * step] = (uint8_t)((q2 + q1 + q0 + p0 + 2) >> 2);
      pix[2 * step] = (uint8_t)((2 * q3 + 3 * q2 + q1 + q0 + p0 + 4) >> 3);
    } else {
      pix[0] = (uint8_t)((2 * q1 + q0 + p1 + 2) >> 2);
    }
  }
}

/**
 * Filters one line of chroma samples (one plane, Cb or Cr) at consecutive
 * `step` offsets from `pix`, clause 8.7.2.3/8.7.2.4's chromaEdgeFlag==1
 * case: only p0/q0 are ever modified (chroma has no p1/p2 "ap/aq"
 * strong-filter extension), and the bS==4 case always uses the simple
 * two-tap average rather than luma's 3-sample "strong" filter.
 */
inline void filterEdgeChroma(uint8_t* pix, int step, int bS, int alpha,
                              int beta, int tc0) {
  int p0 = pix[-1 * step], p1 = pix[-2 * step];
  int q0 = pix[0], q1 = pix[1 * step];
  if (abs(p0 - q0) >= alpha || abs(p1 - p0) >= beta || abs(q1 - q0) >= beta)
    return;

  if (bS < 4) {
    int tc = tc0 + 1;
    int delta = clip3(-tc, tc, (((q0 - p0) << 2) + (p1 - q1) + 4) >> 3);
    pix[-1 * step] = clip255(p0 + delta);
    pix[0] = clip255(q0 - delta);
  } else {
    pix[-1 * step] = (uint8_t)((2 * p1 + p0 + q1 + 2) >> 2);
    pix[0] = (uint8_t)((2 * q1 + q0 + p1 + 2) >> 2);
  }
}

/**
 * Boundary strength (bS) between two 4x4 luma blocks p (before the edge)
 * and q (after it), clause 8.7.2.1: 4/3 (isMbEdge picks which) if either
 * side is Intra, else 2 if either side has coded residual (nnz > 0), else
 * 1 if the two sides use different reference pictures OR their motion
 * vectors differ by >= 4 quarter-samples in either component, else 0 (no
 * filtering). Comparing refIdx values directly (rather than the actual
 * referenced picture identity) is exact, not an approximation, given this
 * decoder's other simplifications: ref_pic_list_modification() reordering
 * is rejected as unsupported (h264_slice_header.h), so within one
 * picture refIdx N always maps to the same stored reference picture
 * across every slice of that picture - reference lists only change at
 * picture boundaries. Reused verbatim for the co-located chroma edge (see
 * deblockPicture()'s chroma section) since clause 8.7.2.1 derives bS from
 * luma properties even when the filter being applied is chroma's.
 */
inline int boundaryStrength(const MacroblockInfo& p, int pBlk,
                             const MacroblockInfo& q, int qBlk,
                             bool isMbEdge) {
  if (p.isIntra() || q.isIntra()) return isMbEdge ? 4 : 3;
  if (p.nnz[pBlk] > 0 || q.nnz[qBlk] > 0) return 2;
  if (p.refIdx[pBlk] != q.refIdx[qBlk]) return 1;
  int dx = p.mv[pBlk][0] - q.mv[qBlk][0];
  int dy = p.mv[pBlk][1] - q.mv[qBlk][1];
  if (dx < 0) dx = -dx;
  if (dy < 0) dy = -dy;
  return (dx >= 4 || dy >= 4) ? 1 : 0;
}

/**
 * Whether the macroblock-boundary edge between `mb` (at mbX,mbY) and its
 * neighbor (at neighborX,neighborY) should be filtered at all, per clause
 * 8.7's disable_deblocking_filter_idc handling: 0 = filter normally, 1 =
 * never filter this MB's edges (caller skips this whole macroblock before
 * even calling this function - see deblockPicture()), 2 = filter internal
 * edges but skip the MB boundary specifically when the neighbor is in a
 * different slice.
 */
inline bool mbEdgeFilterable(const MacroblockInfo& mb, uint8_t mbSliceId,
                              const MbInfoTable& mbInfo, int neighborX,
                              int neighborY) {
  if (mb.disableDeblockIdc == 1) return false;
  if (mb.disableDeblockIdc == 2) {
    return mbInfo.sliceIdAt(neighborX, neighborY) == (int)mbSliceId;
  }
  return true;
}

/**
 * Filters an entire picture in place (clause 8.7's outer loop: every
 * macroblock in raster order, each MB's left then top edges - vertical
 * edges before horizontal, per spec - luma before chroma). Must run
 * after all slices of the picture have been fully reconstructed
 * (deblocking reads/writes across macroblock boundaries, so
 * partially-decoded neighbors would corrupt the result); called once per
 * completed picture from Decoder::decodeSlice(). `mbInfo` supplies both
 * the per-macroblock coding state (mb_type, nnz, mv, qpY) boundaryStrength()
 * needs and the per-slice bookkeeping mbEdgeFilterable() needs for
 * disable_deblocking_filter_idc == 2.
 */
template <typename Allocator>
inline void deblockPicture(Frame<Allocator>& frame, MbInfoTable& mbInfo,
                            const Pps& pps) {
  int mbWidth = mbInfo.mbWidth(), mbHeight = mbInfo.mbHeight();

  for (int mbY = 0; mbY < mbHeight; mbY++) {
    for (int mbX = 0; mbX < mbWidth; mbX++) {
      MacroblockInfo& mb = mbInfo.at(mbX, mbY);
      if (mb.disableDeblockIdc == 1) continue;
      int mySliceId = mbInfo.sliceIdAt(mbX, mbY);

      int alphaOff = mb.alphaC0OffsetDiv2 * 2;
      int betaOff = mb.betaOffsetDiv2 * 2;
      int qp = mb.qpY;
      int cqp = chromaQp(qp, pps.chromaQpIndexOffset);

      bool leftEdgeOk =
          mbX > 0 &&
          mbEdgeFilterable(mb, (uint8_t)mySliceId, mbInfo, mbX - 1, mbY);
      bool topEdgeOk =
          mbY > 0 &&
          mbEdgeFilterable(mb, (uint8_t)mySliceId, mbInfo, mbX, mbY - 1);

      int px0 = mbX * 16, py0 = mbY * 16;

      /*
       * --- Luma vertical edges (step=1 across the row); bS is computed
       *     per 4x4 *row group* (not once for the whole 16px edge) since
       *     Inter macroblocks can have different coefficients/motion per
       *     4x4 block along the same edge. --------------------------------
       */
      for (int edge = 0; edge < 4; edge++) {
        bool isMbEdge = (edge == 0);
        if (isMbEdge && !leftEdgeOk) continue;
        int ex = px0 + edge * 4;
        const MacroblockInfo* pMb = isMbEdge ? &mbInfo.at(mbX - 1, mbY) : &mb;
        int qpNeighbor = isMbEdge ? pMb->qpY : qp;
        int qpAvg = (qp + qpNeighbor + 1) >> 1;
        int indexA = clip3(0, 51, qpAvg + alphaOff);
        int indexB = clip3(0, 51, qpAvg + betaOff);
        int alpha = kDeblockAlpha[indexA];
        int beta = kDeblockBeta[indexB];
        if (alpha == 0) continue;
        for (int g = 0; g < 4; g++) {
          int qBlk = kInvBlk4x4[g][edge];
          int pBlk = isMbEdge ? kInvBlk4x4[g][3] : kInvBlk4x4[g][edge - 1];
          int bS = boundaryStrength(*pMb, pBlk, mb, qBlk, isMbEdge);
          if (bS == 0) continue;
          int tc0 = kDeblockTc0[indexA][bS < 4 ? bS : 0];
          for (int y = g * 4; y < g * 4 + 4; y++) {
            filterEdgeLuma(frame.yRow(py0 + y) + ex, 1, bS, alpha, beta, tc0);
          }
        }
      }
      // --- Luma horizontal edges (step=strideY down the column) ----------
      for (int edge = 0; edge < 4; edge++) {
        bool isMbEdge = (edge == 0);
        if (isMbEdge && !topEdgeOk) continue;
        int ey = py0 + edge * 4;
        const MacroblockInfo* pMb = isMbEdge ? &mbInfo.at(mbX, mbY - 1) : &mb;
        int qpNeighbor = isMbEdge ? pMb->qpY : qp;
        int qpAvg = (qp + qpNeighbor + 1) >> 1;
        int indexA = clip3(0, 51, qpAvg + alphaOff);
        int indexB = clip3(0, 51, qpAvg + betaOff);
        int alpha = kDeblockAlpha[indexA];
        int beta = kDeblockBeta[indexB];
        if (alpha == 0) continue;
        for (int g = 0; g < 4; g++) {
          int qBlk = kInvBlk4x4[edge][g];
          int pBlk = isMbEdge ? kInvBlk4x4[3][g] : kInvBlk4x4[edge - 1][g];
          int bS = boundaryStrength(*pMb, pBlk, mb, qBlk, isMbEdge);
          if (bS == 0) continue;
          int tc0 = kDeblockTc0[indexA][bS < 4 ? bS : 0];
          for (int x = g * 4; x < g * 4 + 4; x++) {
            filterEdgeLuma(frame.yRow(ey) + px0 + x, frame.strideY, bS, alpha,
                            beta, tc0);
          }
        }
      }

      /*
       * --- Chroma: MB-boundary edge + one internal edge per direction
       *     (4:2:0, 4x4 chroma transform blocks in an 8x8 area). Reuses
       *     the bS computed from the co-located *luma* block pair (clause
       *     8.7.2.1 defines bS from luma properties even when filtering
       *     chroma). Chroma MB height/width (8) is exactly half luma's
       *     (16), so each of the 4 luma groups along an edge (bS[0..3],
       *     4 luma rows/cols each) maps 1:1 to a 2-sample chroma segment -
       *     *all four* bS values are used, not just every other one (see
       *     FFmpeg's h264_loop_filter_chroma: 4 groups x inner_iters=2).
       *     Uses the chroma QP (cqp), averaged with the neighbor's chroma
       *     QP.
       */
      int cpx0 = mbX * 8, cpy0 = mbY * 8;
      for (int edge = 0; edge < 2; edge++) {
        bool isMbEdge = (edge == 0);
        if (isMbEdge && !leftEdgeOk) continue;
        int ex = cpx0 + edge * 4;
        int lumaEdge = edge * 2;
        const MacroblockInfo* pMb = isMbEdge ? &mbInfo.at(mbX - 1, mbY) : &mb;
        int qpNeighbor =
            isMbEdge
                ? chromaQp(pMb->qpY, pps.chromaQpIndexOffset)
                : cqp;
        int qpAvg = (cqp + qpNeighbor + 1) >> 1;
        int indexA = clip3(0, 51, qpAvg + alphaOff);
        int indexB = clip3(0, 51, qpAvg + betaOff);
        int alpha = kDeblockAlpha[indexA];
        int beta = kDeblockBeta[indexB];
        if (alpha == 0) continue;
        for (int g = 0; g < 4; g++) {
          int qBlk = kInvBlk4x4[g][lumaEdge];
          int pBlk = isMbEdge ? kInvBlk4x4[g][3] : kInvBlk4x4[g][lumaEdge - 1];
          int bS = boundaryStrength(*pMb, pBlk, mb, qBlk, isMbEdge);
          if (bS == 0) continue;
          int tc0 = kDeblockTc0[indexA][bS < 4 ? bS : 0];
          for (int plane = 0; plane < 2; plane++) {
            uint8_t* base = plane == 0 ? frame.u() : frame.v();
            for (int y = g * 2; y < g * 2 + 2; y++) {
              filterEdgeChroma(base + (size_t)(cpy0 + y) * frame.strideC + ex,
                                1, bS, alpha, beta, tc0);
            }
          }
        }
      }
      for (int edge = 0; edge < 2; edge++) {
        bool isMbEdge = (edge == 0);
        if (isMbEdge && !topEdgeOk) continue;
        int ey = cpy0 + edge * 4;
        int lumaEdge = edge * 2;
        const MacroblockInfo* pMb = isMbEdge ? &mbInfo.at(mbX, mbY - 1) : &mb;
        int qpNeighbor =
            isMbEdge
                ? chromaQp(pMb->qpY, pps.chromaQpIndexOffset)
                : cqp;
        int qpAvg = (cqp + qpNeighbor + 1) >> 1;
        int indexA = clip3(0, 51, qpAvg + alphaOff);
        int indexB = clip3(0, 51, qpAvg + betaOff);
        int alpha = kDeblockAlpha[indexA];
        int beta = kDeblockBeta[indexB];
        if (alpha == 0) continue;
        for (int g = 0; g < 4; g++) {
          int qBlk = kInvBlk4x4[lumaEdge][g];
          int pBlk = isMbEdge ? kInvBlk4x4[3][g] : kInvBlk4x4[lumaEdge - 1][g];
          int bS = boundaryStrength(*pMb, pBlk, mb, qBlk, isMbEdge);
          if (bS == 0) continue;
          int tc0 = kDeblockTc0[indexA][bS < 4 ? bS : 0];
          for (int plane = 0; plane < 2; plane++) {
            uint8_t* base = plane == 0 ? frame.u() : frame.v();
            for (int x = g * 2; x < g * 2 + 2; x++) {
              filterEdgeChroma(base + (size_t)ey * frame.strideC + cpx0 + x,
                                frame.strideC, bS, alpha, beta, tc0);
            }
          }
        }
      }
    }
  }
}

}  // namespace tinyh264
