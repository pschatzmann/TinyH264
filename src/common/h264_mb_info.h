#pragma once
#include <stdint.h>
#include <vector>
#include "h264_config.h"
#include "h264_tables.h"

/*
 * Header-only. Per-picture macroblock metadata: everything neighbor
 * prediction (intra mode context, CAVLC nC, deblocking) needs to know
 * about macroblocks already decoded earlier in the same picture.
 *
 * MB and 4x4-block neighbor availability rules here are derived directly
 * from the raster decode order (clause 6.4.9/6.4.11 neighbor derivation,
 * specialized to the no-MBAFF, no-FMO case this decoder supports): a
 * neighbor is available iff it has already been decoded (raster MB order)
 * and belongs to the same slice. See h264_intra_pred.h for how the
 * per-4x4-block top-right exception set {3,7,11,13,15} falls out of this.
 */

namespace tinyh264 {

/**
 * Macroblock coding type, collapsed from the many mb_type codeNums used
 * on the wire (clause 7.4.5, Tables 7-11/7-13/7-14) down to the handful
 * of cases the rest of the decoder needs to branch on.
 */
enum MbType {
  kMbIntra4x4 = 0,   ///< I_NxN with Intra_4x4 prediction (I-slice or P-slice Intra)
  kMbIntra16x16 = 1, ///< I_16x16_* (prediction mode + cbp encoded in mb_type)
  kMbIntraPcm = 2,   ///< I_PCM: raw, unpredicted, unfiltered sample values
  kMbPSkip = 3,      ///< P_Skip: no mb_type/residual coded, MV from clause 8.4.1.1
  kMbInter = 4,       ///< P_16x16/P_16x8/P_8x16/P_8x8 (all Inter partition shapes)
};

/**
 * Everything about a decoded macroblock that a *later* macroblock in the
 * same picture needs for spatial prediction: intra mode/CAVLC-nC context
 * (clause 9.2.1, 8.3), and (once the whole picture is reconstructed) the
 * per-4x4-block state the deblocking filter's boundary-strength
 * derivation reads (clause 8.7.2.1). One instance per macroblock,
 * persisted for the lifetime of the picture in MbInfoTable.
 */
struct MacroblockInfo {
  uint8_t type = kMbIntra4x4;
  uint8_t intra4x4PredMode[16] = {0};  ///< valid only if type == kMbIntra4x4
  uint8_t intra16x16PredMode = 0;      ///< valid only if type == kMbIntra16x16
  uint8_t chromaPredMode = 0;          ///< intra_chroma_pred_mode, clause 8.3.4
  uint8_t cbpLuma = 0;    ///< coded_block_pattern, 4 bits, one per 8x8 luma quadrant
  uint8_t cbpChroma = 0;  ///< coded_block_pattern chroma part, 0..2
  int8_t qpY = 0;         ///< QP_Y in effect for this macroblock (clause 8.6.1)
  /*
   * Deblocking filter parameters in effect for this MB (from its slice
   * header), clause 8.7: disableDeblockingFilterIdc 0=filter normally,
   * 1=disable entirely, 2=disable only at slice boundaries.
   */
  uint8_t disableDeblockIdc = 0; ///< disable_deblocking_filter_idc, clause 7.4.3
  int8_t alphaC0OffsetDiv2 = 0;  ///< slice_alpha_c0_offset_div2, clause 7.4.3
  int8_t betaOffsetDiv2 = 0;     ///< slice_beta_offset_div2, clause 7.4.3
  /*
   * Non-zero coefficient counts, for CAVLC nC prediction (clause 9.2.1):
   * indices 0..15 = luma 4x4 blocks (kBlk4x4X/Y order), 16..19 = Cb 4x4
   * blocks, 20..23 = Cr 4x4 blocks. I_PCM macroblocks set all of these to
   * 16 per spec (their coefficients are, in effect, maximally "coded").
   */
  uint8_t nnz[24] = {0};
  /*
   * Motion vectors in quarter-luma-sample units, one per 4x4 block
   * (kBlk4x4X/Y order), valid only for kMbPSkip/kMbInter. All partition
   * shapes (16x16 down to 4x4) get expanded to fill all 16 entries with
   * their partition's MV, so neighbor lookups never need to know the
   * partition shape.
   */
  int16_t mv[16][2] = {{0, 0}};
  /*
   * ref_idx_l0 into the current slice's reference picture list (index into
   * Decoder::refFrames_, 0 = most recently decoded reference), one per
   * 4x4 block, same expand-to-fill-16 convention as mv[]. Valid only for
   * kMbPSkip/kMbInter (always 0 for P_Skip, clause 8.4.1.1).
   */
  int8_t refIdx[16] = {0};

  /**
   * True for any of the three Intra macroblock types (as opposed to
   * P_Skip/Inter), matching the spec's frequent "if( mb_type is Intra )"
   * branches (e.g. clause 8.7.2.1 boundary strength derivation).
   */
  bool isIntra() const {
    return type == kMbIntra4x4 || type == kMbIntra16x16 || type == kMbIntraPcm;
  }
};

/**
 * Per-picture array of MacroblockInfo plus the "which slice decoded this
 * macroblock" bookkeeping (sliceId_) that clause 6.4.9/6.4.11's neighbor
 * derivation needs: a neighbor is available for prediction only if it
 * has already been decoded *and* belongs to the same slice (so
 * prediction never reaches across an as-yet-undecoded or
 * differently-sliced macroblock). Backing storage is heap-allocated up
 * to maxMbs_ entries (default H264_MAX_MBS, see setMaxDimension()),
 * lazily on first reset() call, and never resized again after that
 * (unless setMaxDimension() changes the ceiling) - the same
 * allocate-once idiom Frame::ensureAllocated() uses, for the same
 * reason: at larger resolutions (e.g. QVGA's 300 macroblocks, ~40KB of
 * MacroblockInfo alone) a *static* array member here overflows a plain
 * ESP32's DRAM .bss segment before the sketch even runs - confirmed by
 * an actual arduino-cli link failure, not just budget math, the same
 * way the original Frame overflow was found.
 */
class MbInfoTable {
 public:
  /**
   * Overrides the allocation ceiling (in pixels, not macroblocks -
   * converted internally the same way H264_MAX_MBS itself is derived
   * from H264_MAX_WIDTH/H264_MAX_HEIGHT in h264_config.h) for this
   * table's backing storage - defaults to H264_MAX_MBS. Decoder::
   * setMaxDimension()/Encoder::setMaxDimension() call this with the same
   * width/height passed to Frame::setMaxDimension(), so both stay in
   * sync. If storage is already allocated, releases it so the next
   * reset() call reallocates at the new size.
   */
  void setMaxDimension(int maxWidth, int maxHeight) {
    maxMbs_ = ((maxWidth + 15) / 16) * ((maxHeight + 15) / 16);
    if (!sliceId_.empty()) {
      sliceId_.clear();
      sliceId_.shrink_to_fit();
      mb_.clear();
      mb_.shrink_to_fit();
    }
  }

  /**
   * Clears all per-macroblock state and marks every macroblock as
   * "not yet decoded" (sliceId -1), ready to start a new picture of
   * `mbWidth` x `mbHeight` macroblocks. Allocates the backing storage
   * (at maxMbs_ capacity) on the first call; a no-op allocation-wise on
   * subsequent calls, so this doesn't reintroduce per-picture heap
   * allocation into the decode/encode hot path.
   */
  void reset(int mbWidth, int mbHeight) {
    mbWidth_ = mbWidth;
    mbHeight_ = mbHeight;
    if (sliceId_.empty()) {
      sliceId_.resize(maxMbs_);
      mb_.resize(maxMbs_);
    }
    int n = mbWidth * mbHeight;
    for (int i = 0; i < n; i++) {
      sliceId_[i] = -1;
      mb_[i] = MacroblockInfo();
    }
  }

  int mbWidth() const { return mbWidth_; }   ///< picture width in macroblocks
  int mbHeight() const { return mbHeight_; } ///< picture height in macroblocks

  /**
   * Raster-scan linear index of macroblock (mbX, mbY), clause 6.4.1's
   * CurrMbAddr numbering (no MBAFF/FMO, so this is just row-major).
   */
  int addr(int mbX, int mbY) const { return mbY * mbWidth_ + mbX; }

  /**
   * Marks macroblock (mbX, mbY) as belonging to slice `sliceId`, making
   * it visible to later decoded() / *Available() queries. Must be called
   * before decoding the macroblock's syntax (some of that syntax, e.g.
   * Intra4x4 mode prediction, queries this MB's own neighbors while
   * still parsing this MB).
   */
  void beginMb(int mbX, int mbY, int sliceId) {
    sliceId_[addr(mbX, mbY)] = sliceId;
  }

  /**
   * Mutable access to a macroblock's decoded info, for filling in as it
   * is decoded.
   */
  MacroblockInfo& at(int mbX, int mbY) { return mb_[addr(mbX, mbY)]; }
  /// const overload of at(), for neighbor lookups from later macroblocks.
  const MacroblockInfo& at(int mbX, int mbY) const {
    return mb_[addr(mbX, mbY)];
  }

  /**
   * Which slice decoded macroblock (mbX, mbY), or -1 if that macroblock
   * hasn't been decoded yet (or is out of the picture bounds).
   */
  int sliceIdAt(int mbX, int mbY) const {
    if (mbX < 0 || mbY < 0 || mbX >= mbWidth_ || mbY >= mbHeight_) return -1;
    return sliceId_[addr(mbX, mbY)];
  }

  /**
   * True if (mbX, mbY) is within the picture, has already been decoded,
   * and belongs to the same slice as `sliceId` - the availability test
   * clause 6.4.9/6.4.11 boils down to for this decoder's no-MBAFF,
   * no-FMO case.
   */
  bool decoded(int mbX, int mbY, int sliceId) const {
    if (mbX < 0 || mbY < 0 || mbX >= mbWidth_ || mbY >= mbHeight_)
      return false;
    return sliceId_[addr(mbX, mbY)] == sliceId;
  }

  /**
   * Whether the macroblock immediately to the left is available for
   * prediction (clause 6.4.9, mbAddrA).
   */
  bool leftAvailable(int mbX, int mbY, int sliceId) const {
    return decoded(mbX - 1, mbY, sliceId);
  }
  /// Whether the macroblock immediately above is available (mbAddrB).
  bool topAvailable(int mbX, int mbY, int sliceId) const {
    return decoded(mbX, mbY - 1, sliceId);
  }
  /**
   * Whether the macroblock above-left is available (mbAddrC... strictly
   * mbAddrD in clause 6.4.9's naming - "top-left").
   */
  bool topLeftAvailable(int mbX, int mbY, int sliceId) const {
    return decoded(mbX - 1, mbY - 1, sliceId);
  }
  /**
   * Whether the macroblock above-right is available (mbAddrC). See
   * h264_intra_pred.h for the per-4x4-block exception set {3,7,11,13,15}
   * where this is *not* the right test at the sub-block level.
   */
  bool topRightAvailable(int mbX, int mbY, int sliceId) const {
    return decoded(mbX + 1, mbY - 1, sliceId);
  }

 private:
  int mbWidth_ = 0, mbHeight_ = 0;
  int maxMbs_ = H264_MAX_MBS;
  std::vector<int16_t> sliceId_;
  std::vector<MacroblockInfo> mb_;
};

/**
 * nC prediction for CAVLC coeff_token (clause 9.2.1). `nnzLeft`/`nnzTop`
 * are the neighbor 4x4 block's non-zero count, or -1 if that neighbor is
 * unavailable (picture/slice boundary): per spec, if both neighbors are
 * unavailable nC=0; if only one is available, nC = that one's count
 * (spec: "the value of nC is set equal to nA" or "nB" alone in that case).
 */
inline int predictNc(int nnzLeft, int nnzTop) {
  bool haveLeft = nnzLeft >= 0;
  bool haveTop = nnzTop >= 0;
  if (haveLeft && haveTop) return (nnzLeft + nnzTop + 1) >> 1;
  if (haveLeft) return nnzLeft;
  if (haveTop) return nnzTop;
  return 0;
}

/**
 * (row, col) within a macroblock's 4x4-block grid -> luma4x4BlkIdx.
 * Inverse of kBlk4x4X/kBlk4x4Y in h264_tables.h. Shared by the decoder's
 * macroblock layer (decoder/h264_macroblock.h) and the encoder's
 * (encoder/h264_macroblock_encode.h) - both need the same 4x4-grid
 * addressing for nnz-context lookups.
 */
static const uint8_t kInvBlk4x4[4][4] = {
    {0, 1, 4, 5},
    {2, 3, 6, 7},
    {8, 9, 12, 13},
    {10, 11, 14, 15},
};

/**
 * Looks up the neighboring 4x4 luma block's nnz for CAVLC nC prediction
 * (clause 9.2.1). `bx`, `by` are the *querying* block's own grid position
 * (0..3); `dx`,`dy` are -1/0 offsets (only one of them nonzero)
 * indicating which neighbor (left or top). Returns -1 if that neighbor is
 * unavailable (picture/slice edge). Templated on the context type (not
 * fixed to MbDecodeContext) purely structurally - both
 * decoder::MbDecodeContext and encoder::MbEncodeContext expose the same
 * `mbInfo`/`mbX`/`mbY`/`sliceId` members this needs, so one definition
 * serves both directions without either depending on the other's header.
 */
template <typename CtxT>
inline int lumaNeighborNnz(const CtxT& ctx, int bx, int by, int dx, int dy) {
  int nbx = bx + dx, nby = by + dy;
  if (nbx >= 0 && nbx < 4 && nby >= 0 && nby < 4) {
    return ctx.mbInfo->at(ctx.mbX, ctx.mbY).nnz[kInvBlk4x4[nby][nbx]];
  }
  int nmbX = ctx.mbX + (nbx < 0 ? -1 : 0);
  int nmbY = ctx.mbY + (nby < 0 ? -1 : 0);
  if (!ctx.mbInfo->decoded(nmbX, nmbY, ctx.sliceId)) return -1;
  int wbx = (nbx < 0) ? 3 : nbx;
  int wby = (nby < 0) ? 3 : nby;
  return ctx.mbInfo->at(nmbX, nmbY).nnz[kInvBlk4x4[wby][wbx]];
}

/**
 * Chroma counterpart of lumaNeighborNnz(): looks up the neighboring 2x2
 * (per-plane) chroma 4x4 block's nnz. `cx`,`cy` are the querying block's
 * own grid position (0..1) within the 8x8 chroma plane. Templated for
 * the same reason as lumaNeighborNnz() above.
 */
template <typename CtxT>
inline int chromaNeighborNnz(const CtxT& ctx, int plane, int cx, int cy,
                              int dx, int dy) {
  int ncx = cx + dx, ncy = cy + dy;
  int base = 16 + plane * 4;
  if (ncx >= 0 && ncx < 2 && ncy >= 0 && ncy < 2) {
    return ctx.mbInfo->at(ctx.mbX, ctx.mbY).nnz[base + ncy * 2 + ncx];
  }
  int nmbX = ctx.mbX + (ncx < 0 ? -1 : 0);
  int nmbY = ctx.mbY + (ncy < 0 ? -1 : 0);
  if (!ctx.mbInfo->decoded(nmbX, nmbY, ctx.sliceId)) return -1;
  int wcx = (ncx < 0) ? 1 : ncx;
  int wcy = (ncy < 0) ? 1 : ncy;
  return ctx.mbInfo->at(nmbX, nmbY).nnz[base + wcy * 2 + wcx];
}

/**
 * Predicts Intra_4x4 pred mode for luma block (bx,by), clause 8.3.1.1 -
 * used both to *decode* prev_intra4x4_pred_mode_flag/rem_intra4x4_pred_mode
 * (decoder/h264_macroblock.h) and, identically, by the encoder to know
 * what a decoder will predict so it can encode the matching flag/rem
 * pair (encoder/h264_macroblock_encode.h) - genericized the same way as
 * lumaNeighborNnz()/chromaNeighborNnz() above. Two distinct rules stack
 * here, and conflating them is a natural (and silent) mistake:
 *  1. dcPredModePredictedFlag is a *joint* condition covering true
 *     unavailability (mbAddrA and/or mbAddrB doesn't exist - picture/
 *     slice edge - or, under constrained_intra_pred, is Inter-coded): if
 *     triggered by *either* side, BOTH intraMxMPredModeA and
 *     intraMxMPredModeB are forced to DC(2), not just the missing side.
 *  2. Only when that flag is 0 (both neighbors genuinely available) does
 *     each side *independently* fall back to DC(2) for the narrower
 *     reason of simply not being Intra_4x4/8x8-coded (e.g. an available
 *     I_16x16 neighbor) - that check does NOT feed back into the joint
 *     flag, and does NOT force the other side to DC too.
 * Getting this wrong (checking availability independently, or making the
 * "wrong type" check joint too) silently produces a locally-plausible but
 * spec-incorrect predicted mode - found (decode-side) by pixel-diffing
 * whole decoded frames against ffmpeg's reference decode; unit-level
 * reasoning alone couldn't distinguish these variants from the spec text.
 */
template <typename CtxT>
inline int predictIntra4x4Mode(CtxT& ctx, int bx, int by, bool mbLeftAvail,
                                bool mbTopAvail) {
  bool leftIsMbBoundary = (bx == 0);
  bool topIsMbBoundary = (by == 0);

  bool leftAvailable = leftIsMbBoundary ? mbLeftAvail : true;
  bool topAvailable = topIsMbBoundary ? mbTopAvail : true;

  /*
   * True picture/slice-edge unavailability is checked here (the joint
   * dcPredModePredictedFlag's first two conditions). The joint flag's
   * other two conditions - an available neighbor that's Inter-coded, but
   * only if constrained_intra_pred_flag == 1 - are not evaluated at all:
   * pps.constrainedIntraPredFlag is parsed (h264_sps_pps.h) but never read
   * here, so this function always behaves as if it were 0. For
   * constrained_intra_pred_flag == 0 (by far the common case, and the
   * only one exercised by this decoder's test streams) that's exactly
   * correct: an Inter neighbor still ends up forced to DC(2) below via the
   * plain independent "not Intra_4x4-coded" per-side rule, which is the
   * spec-mandated behavior for flag == 0. It would be spec-incorrect for
   * an encoder that sets flag == 1 with a mixed Intra/Inter neighbor pair
   * (the *other*, genuinely-Intra_4x4 side should also be forced to DC in
   * that case, and currently isn't).
   */
  if (!leftAvailable || !topAvailable) return 2;

  const MacroblockInfo* leftMb =
      leftIsMbBoundary ? &ctx.mbInfo->at(ctx.mbX - 1, ctx.mbY) : nullptr;
  const MacroblockInfo* topMb =
      topIsMbBoundary ? &ctx.mbInfo->at(ctx.mbX, ctx.mbY - 1) : nullptr;

  int modeA;
  if (leftIsMbBoundary) {
    /*
     * "not Intra_4x4-coded" also covers Inter/P_Skip neighbors (any
     * MbType other than kMbIntra4x4), so this line does double duty as
     * both the plain-Intra_16x16-neighbor case and the P-slice
     * Inter-neighbor case described above.
     */
    modeA = (leftMb->type != kMbIntra4x4)
                ? 2
                : leftMb->intra4x4PredMode[kInvBlk4x4[by][3]];
  } else {
    modeA = ctx.mbInfo->at(ctx.mbX, ctx.mbY)
                .intra4x4PredMode[kInvBlk4x4[by][bx - 1]];
  }
  int modeB;
  if (topIsMbBoundary) {
    modeB = (topMb->type != kMbIntra4x4)
                ? 2
                : topMb->intra4x4PredMode[kInvBlk4x4[3][bx]];
  } else {
    modeB = ctx.mbInfo->at(ctx.mbX, ctx.mbY)
                .intra4x4PredMode[kInvBlk4x4[by - 1][bx]];
  }
  return modeA < modeB ? modeA : modeB;
}

}  // namespace tinyh264
