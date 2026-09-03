#pragma once
#include <stdint.h>
#ifdef TINYH264_DEBUG_MB
#include <stdio.h>
#endif
#include "../common/Logger.h"
#include "h264_bitreader.h"
#include "h264_cavlc.h"
#include "../common/h264_frame.h"
#include "../common/h264_intra_pred.h"
#include "../common/h264_mb_info.h"
#include "h264_slice_header.h"
#include "h264_sps_pps.h"
#include "../common/h264_tables.h"
#include "../common/h264_transform.h"

/*
 * Header-only. macroblock_layer() for I slices (I_4x4, I_16x16, I_PCM),
 * ITU-T H.264 clause 7.3.5, tying together CAVLC residual decoding,
 * dequant/inverse transform (h264_transform.h) and intra prediction
 * (h264_intra_pred.h) into full pixel reconstruction. P-slice macroblock
 * types (P_16x16/16x8/8x16/8x8, P_Skip) are handled by
 * h264_macroblock_inter.h, which shares the Intra decode path here for
 * the Intra-macroblock-inside-a-P-slice case.
 *
 * Nothing here is templated - MbDecodeContext holds plain Frame/
 * MbInfoTable pointers, both non-templated types backed by a
 * MemoryResource chosen at construction (see ../MemoryResource.h) rather
 * than a compile-time Allocator parameter.
 */

namespace tinyh264 {

/**
 * Outcome of decoding one macroblock's syntax: `ok` for a clean decode,
 * `unsupported` for a recognized-but-unimplemented feature (result of
 * `false` from the decode function itself, rather than this struct,
 * signals an actual bitstream error).
 */
struct MacroblockDecodeResult {
  bool ok = false;
  bool unsupported = false;
};

/// Everything a macroblock decode needs that stays constant across the
/// whole slice (or picture): the destination frame, the slice's reference
/// picture list (P-slices), per-picture macroblock metadata, and the
/// active SPS/PPS. Passed by reference to every decode*/predict* function
/// in this file and h264_macroblock_inter.h so they don't each need a long,
/// repeated parameter list. `mbX`/`mbY` are updated by the caller (Decoder::
/// decodeSlice()) before each macroblock.
struct MbDecodeContext {
  Frame* frame;
  /*
   * RefPicList0 for the current slice (clause 8.2.4): refList[0] is the
   * most recently decoded reference picture, refList[numActiveRefs-1] the
   * oldest - this decoder only implements the *default* list construction
   * (no ref_pic_list_modification() reordering, which is flagged
   * unsupported in h264_slice_header.h if present), so this is simply a
   * window into Decoder::refFrames_. Unused (numActiveRefs == 0) for
   * I-slices.
   */
  const Frame* refList[H264_MAX_REF_FRAMES] = {nullptr};
  int numActiveRefs = 0;
  MbInfoTable* mbInfo;
  const Sps* sps;
  const Pps* pps;
  int sliceId;
  int mbX, mbY;
};

/**
 * Decodes one 4x4 luma block's residual (full 16 coeffs incl. DC, I_4x4/
 * Inter case - clause 8.5.9 dequant then 8.5.12.2 inverse transform) and
 * adds it to the already-predicted pixels at the block's position.
 */
inline bool decodeLumaBlockFull(BitReader& br, MbDecodeContext& ctx,
                                 int blkIdx, int qp) {
  int bx = kBlk4x4X[blkIdx], by = kBlk4x4Y[blkIdx];
  int nA = lumaNeighborNnz(ctx, bx, by, -1, 0);
  int nB = lumaNeighborNnz(ctx, bx, by, 0, -1);
  int nC = predictNc(nA, nB);

  int32_t coeff[16];
  uint32_t totalCoeff = 0;
  if (!residualBlockCavlc(br, nC, 16, coeff, &totalCoeff)) {
    H264LOG.error("decodeLumaBlockFull: CAVLC residual decode failed at mb(%d,%d) blk=%d",
                   ctx.mbX, ctx.mbY, blkIdx);
    return false;
  }
  ctx.mbInfo->at(ctx.mbX, ctx.mbY).nnz[blkIdx] = (uint8_t)totalCoeff;
#ifdef TINYH264_DEBUG_MB
  if (ctx.mbY >= 6) {
    fprintf(stderr,
            "    lumaFull mb(%d,%d) blk%d nC=%d totalCoeff=%u bits=%zu "
            "err=%d\n",
            ctx.mbX, ctx.mbY, blkIdx, nC, totalCoeff, br.bitsConsumed(),
            br.error());
  }
#endif

  int32_t block[16] = {0};
  for (int k = 0; k < 16; k++) block[kZigZag4x4[k]] = coeff[k];
  dequant4x4(block, qp, false);
  idct4x4(block);

  int px = ctx.mbX * 16 + bx * 4, py = ctx.mbY * 16 + by * 4;
  addResidual4x4(ctx.frame->yRow(py) + px, ctx.frame->strideY, block);
  return true;
}

/**
 * Decodes one 4x4 luma AC-only block (I_16x16 case, scan positions
 * 1..15 - position 0 comes from the macroblock-wide DC block instead,
 * clause 8.5.6) and adds it, using the given already Hadamard-transformed
 * and dequantized DC value at raster position 0.
 */
inline bool decodeLumaBlockAc(BitReader& br, MbDecodeContext& ctx,
                               int blkIdx, int qp, int32_t dcValue) {
  int bx = kBlk4x4X[blkIdx], by = kBlk4x4Y[blkIdx];
  int nA = lumaNeighborNnz(ctx, bx, by, -1, 0);
  int nB = lumaNeighborNnz(ctx, bx, by, 0, -1);
  int nC = predictNc(nA, nB);

  int32_t coeff[15];
  uint32_t totalCoeff = 0;
  if (!residualBlockCavlc(br, nC, 15, coeff, &totalCoeff)) {
    H264LOG.error("decodeLumaBlockAc: CAVLC residual decode failed at mb(%d,%d) blk=%d",
                   ctx.mbX, ctx.mbY, blkIdx);
    return false;
  }
  ctx.mbInfo->at(ctx.mbX, ctx.mbY).nnz[blkIdx] = (uint8_t)totalCoeff;

  int32_t block[16] = {0};
  for (int k = 0; k < 15; k++) block[kZigZag4x4[k + 1]] = coeff[k];
  dequant4x4(block, qp, true);
  block[0] = dcValue;
  idct4x4(block);

  int px = ctx.mbX * 16 + bx * 4, py = ctx.mbY * 16 + by * 4;
  addResidual4x4(ctx.frame->yRow(py) + px, ctx.frame->strideY, block);
  return true;
}

/**
 * Decodes one chroma 4x4 AC-only block for the given plane (0=Cb,1=Cr) and
 * sub-block position (cx,cy in 0..1), using the already Hadamard-
 * transformed and dequantized chroma DC value at raster position 0
 * (clause 8.5.11), and adds it on top of the already-predicted samples.
 */
inline bool decodeChromaBlockAc(BitReader& br, MbDecodeContext& ctx,
                                 int plane, int cx, int cy, int chromaQp,
                                 int32_t dcValue) {
  int nA = chromaNeighborNnz(ctx, plane, cx, cy, -1, 0);
  int nB = chromaNeighborNnz(ctx, plane, cx, cy, 0, -1);
  int nC = predictNc(nA, nB);

  int32_t coeff[15];
  uint32_t totalCoeff = 0;
  if (!residualBlockCavlc(br, nC, 15, coeff, &totalCoeff)) {
    H264LOG.error("decodeChromaBlockAc: CAVLC residual decode failed at mb(%d,%d) plane=%d cx=%d cy=%d",
                   ctx.mbX, ctx.mbY, plane, cx, cy);
    return false;
  }
  ctx.mbInfo->at(ctx.mbX, ctx.mbY).nnz[16 + plane * 4 + cy * 2 + cx] =
      (uint8_t)totalCoeff;

  int32_t block[16] = {0};
  for (int k = 0; k < 15; k++) block[kZigZag4x4[k + 1]] = coeff[k];
  dequant4x4(block, chromaQp, true);
  block[0] = dcValue;
  idct4x4(block);

  int px = ctx.mbX * 8 + cx * 4, py = ctx.mbY * 8 + cy * 4;
  uint8_t* planeBuf = plane == 0 ? ctx.frame->u() : ctx.frame->v();
  addResidual4x4(planeBuf + (size_t)py * ctx.frame->strideC + px,
                 ctx.frame->strideC, block);
  return true;
}

/**
 * Derives the predicted Intra_4x4 mode for luma block (bx,by) within the
 * current macroblock, clause 8.3.1.1. Two distinct rules stack here, and
 * conflating them is a natural (and silent) mistake:
 *  1. dcPredModePredictedFlag is a *joint* condition covering true
 *     unavailability (mbAddrA and/or mbAddrB doesn't exist - picture/slice
 *     edge - or, under constrained_intra_pred, is Inter-coded): if
 *     triggered by *either* side, BOTH intraMxMPredModeA and
 *     intraMxMPredModeB are forced to DC(2), not just the missing side.
 *  2. Only when that flag is 0 (both neighbors genuinely available) does
 *     each side *independently* fall back to DC(2) for the narrower
 *     reason of simply not being Intra_4x4/8x8-coded (e.g. a available
 *     I_16x16 neighbor) - that check does NOT feed back into the joint
 *     flag, and does NOT force the other side to DC too.
 * Getting this wrong (checking availability independently, or making the
 * "wrong type" check joint too) silently produces a locally-plausible but
 * spec-incorrect predicted mode - found by pixel-diffing whole decoded
 * frames against ffmpeg's reference decode; unit-level reasoning alone
 * couldn't distinguish these variants from the spec text.
 * Decodes the remainder of macroblock_layer() (clause 7.3.5) given an
 * already-known I-slice-numbered mb_type (0=I_NxN, 1..24=I_16x16
 * variants, 25=I_PCM): prediction mode syntax, coded_block_pattern,
 * mb_qp_delta, then full intra prediction + residual reconstruction for
 * luma and chroma. Shared by decodeMacroblockIntra() (I-slice: reads
 * mb_type itself) and the P-slice Inter path (h264_macroblock_inter.h),
 * where mb_type >= 5 signals an Intra macroblock using this same
 * numbering minus 5. `*qpY` is the running QP, read and updated in
 * place; `result` reports unsupported-feature status (I_PCM with
 * PPS transform_8x8_mode, an out-of-range mb_type, etc.) - an actual
 * bitstream error is instead signaled by a `false` return.
 */
inline bool decodeMacroblockIntraWithType(BitReader& br, MbDecodeContext& ctx,
                                           uint32_t mbTypeRaw, int* qpY,
                                           MacroblockDecodeResult* result) {
  *result = MacroblockDecodeResult();
  MacroblockInfo& mb = ctx.mbInfo->at(ctx.mbX, ctx.mbY);
  mb = MacroblockInfo();

  bool leftAvail = ctx.mbInfo->leftAvailable(ctx.mbX, ctx.mbY, ctx.sliceId);
  bool topAvail = ctx.mbInfo->topAvailable(ctx.mbX, ctx.mbY, ctx.sliceId);
  bool topLeftAvail =
      ctx.mbInfo->topLeftAvailable(ctx.mbX, ctx.mbY, ctx.sliceId);
  bool topRightAvail =
      ctx.mbInfo->topRightAvailable(ctx.mbX, ctx.mbY, ctx.sliceId);

  // --- I_PCM ---------------------------------------------------------
  if (mbTypeRaw == 25) {
    mb.type = kMbIntraPcm;
    br.byteAlign();
    int px = ctx.mbX * 16, py = ctx.mbY * 16;
    for (int y = 0; y < 16; y++) {
      uint8_t* row = ctx.frame->yRow(py + y) + px;
      for (int x = 0; x < 16; x++) row[x] = (uint8_t)br.u(8);
    }
    int cpx = ctx.mbX * 8, cpy = ctx.mbY * 8;
    for (int y = 0; y < 8; y++) {
      uint8_t* row = ctx.frame->uRow(cpy + y) + cpx;
      for (int x = 0; x < 8; x++) row[x] = (uint8_t)br.u(8);
    }
    for (int y = 0; y < 8; y++) {
      uint8_t* row = ctx.frame->vRow(cpy + y) + cpx;
      for (int x = 0; x < 8; x++) row[x] = (uint8_t)br.u(8);
    }
    for (int i = 0; i < 24; i++) mb.nnz[i] = 16;
    mb.qpY = 0;
    *qpY = 0;
    if (br.error()) {
      H264LOG.error("decodeMacroblockIntraWithType: I_PCM raw-sample read truncated at mb(%d,%d)",
                     ctx.mbX, ctx.mbY);
      return false;
    }
    result->ok = true;
    return true;
  }

  bool is16x16 = mbTypeRaw >= 1 && mbTypeRaw <= 24;
  if (mbTypeRaw > 25) {
    H264LOG.error("decodeMacroblockIntraWithType: mb_type=%u out of range at mb(%d,%d)",
                   mbTypeRaw, ctx.mbX, ctx.mbY);
    result->unsupported = true;  // not a valid I-slice mb_type
    return true;
  }

  if (!is16x16) {
    /*
     * I_NxN. transform_size_8x8_flag would appear here if
     * pps.transform8x8ModeFlag were set; a Baseline-profile encoder never
     * sets it (see h264_sps_pps.h), so treat that combination as
     * unsupported rather than silently mis-decoding 8x8-transform data.
     */
    if (ctx.pps->transform8x8ModeFlag) {
      H264LOG.warn("decodeMacroblockIntraWithType: I_NxN + 8x8 transform not supported at mb(%d,%d)",
                    ctx.mbX, ctx.mbY);
      result->unsupported = true;
      return true;
    }
    mb.type = kMbIntra4x4;
    for (int blk = 0; blk < 16; blk++) {
      int bx = kBlk4x4X[blk], by = kBlk4x4Y[blk];
      int predicted = predictIntra4x4Mode(ctx, bx, by, leftAvail, topAvail);
      bool usePredicted = br.flag();
#ifdef TINYH264_DEBUG_MB
      fprintf(stderr, "  blk%d (%d,%d) predicted=%d usePredicted=%d", blk, bx,
              by, predicted, usePredicted);
#endif
      int mode;
      if (usePredicted) {
        mode = predicted;
      } else {
        int rem = (int)br.u(3);
        mode = (rem < predicted) ? rem : rem + 1;
#ifdef TINYH264_DEBUG_MB
        fprintf(stderr, " rem=%d", rem);
#endif
      }
      mb.intra4x4PredMode[blk] = (uint8_t)mode;
#ifdef TINYH264_DEBUG_MB
      fprintf(stderr, " -> mode=%d\n", mode);
#endif
    }
    mb.chromaPredMode = (uint8_t)br.ue();
  } else {
    mb.type = kMbIntra16x16;
    uint32_t idx = mbTypeRaw - 1;
    mb.intra16x16PredMode = (uint8_t)(idx % 4);
    mb.cbpChroma = (uint8_t)((idx / 4) % 3);
    mb.cbpLuma = (idx >= 12) ? 15 : 0;
    mb.chromaPredMode = (uint8_t)br.ue();
  }

  if (!is16x16) {
    uint32_t cbpCode = br.ue();
    if (cbpCode > 47) {
      H264LOG.error("decodeMacroblockIntraWithType: invalid coded_block_pattern %u at mb(%d,%d)",
                     cbpCode, ctx.mbX, ctx.mbY);
      return false;
    }
    uint8_t cbpCombined = kCbpIntra4x4[cbpCode];
    mb.cbpLuma = cbpCombined & 0xF;
    mb.cbpChroma = cbpCombined >> 4;
  }

  int dquant = 0;
  if (mb.cbpLuma != 0 || mb.cbpChroma != 0 || is16x16) {
    dquant = br.se();
  }
  int qp = *qpY + dquant;
  qp = ((qp + 52) % 52 + 52) % 52;  // wrap into 0..51 (8-bit, QpBdOffset=0)
  *qpY = qp;
  mb.qpY = (int8_t)qp;

#ifdef TINYH264_DEBUG_MB
  fprintf(stderr,
          "MB(%d,%d) type=%s cbpL=%d cbpC=%d qp=%d chromaPred=%d",
          ctx.mbX, ctx.mbY, is16x16 ? "I16x16" : "I4x4", mb.cbpLuma,
          mb.cbpChroma, qp, mb.chromaPredMode);
  if (is16x16) {
    fprintf(stderr, " lumaPred=%d\n", mb.intra16x16PredMode);
  } else {
    fprintf(stderr, " lumaPred=[");
    for (int i = 0; i < 16; i++) fprintf(stderr, "%d ", mb.intra4x4PredMode[i]);
    fprintf(stderr, "]\n");
  }
#endif

  if (br.error()) {
    H264LOG.error("decodeMacroblockIntraWithType: bitstream error reading mb_qp_delta at mb(%d,%d)",
                   ctx.mbX, ctx.mbY);
    return false;
  }

  // --- Luma residual ---------------------------------------------------
  if (is16x16) {
    predictIntra16x16(*ctx.frame, ctx.mbX, ctx.mbY, mb.intra16x16PredMode,
                       leftAvail, topAvail, topLeftAvail);

    int bxA = lumaNeighborNnz(ctx, 0, 0, -1, 0);
    int bxB = lumaNeighborNnz(ctx, 0, 0, 0, -1);
    int nCdc = predictNc(bxA, bxB);
    int32_t dcCoeff[16];
    uint32_t dcTotalCoeff = 0;
    if (!residualBlockCavlc(br, nCdc, 16, dcCoeff, &dcTotalCoeff)) {
      H264LOG.error("decodeMacroblockIntraWithType: CAVLC I_16x16 luma DC decode failed at mb(%d,%d)",
                     ctx.mbX, ctx.mbY);
      return false;
    }
    int32_t dcBlock[16] = {0};
    for (int k = 0; k < 16; k++) dcBlock[kZigZag4x4[k]] = dcCoeff[k];
    hadamard4x4(dcBlock);
    dequantLumaDC4x4(dcBlock, qp);

    for (int blk = 0; blk < 16; blk++) {
      int bx = kBlk4x4X[blk], by = kBlk4x4Y[blk];
      int32_t dc = dcBlock[by * 4 + bx];
      if (mb.cbpLuma != 0) {
        if (!decodeLumaBlockAc(br, ctx, blk, qp, dc)) return false;
      } else {
        /*
         * No AC data coded: still need to add the (possibly nonzero) DC
         * term to the prediction.
         */
        int32_t block[16] = {0};
        block[0] = dc;
        idct4x4(block);
        int px = ctx.mbX * 16 + bx * 4, py = ctx.mbY * 16 + by * 4;
        addResidual4x4(ctx.frame->yRow(py) + px, ctx.frame->strideY, block);
        mb.nnz[blk] = 0;
      }
    }
  } else {
    for (int blk = 0; blk < 16; blk++) {
      int bx = kBlk4x4X[blk], by = kBlk4x4Y[blk];
      Neighbors4x4 nb = gatherNeighbors4x4(
          *ctx.frame, ctx.mbX * 16 + bx * 4, ctx.mbY * 16 + by * 4, blk,
          leftAvail, topAvail, topLeftAvail, topRightAvail);
      predictIntra4x4(*ctx.frame, ctx.mbX * 16 + bx * 4,
                       ctx.mbY * 16 + by * 4, mb.intra4x4PredMode[blk], nb);
      int quadrant = blk / 4;
      if (mb.cbpLuma & (1 << quadrant)) {
        if (!decodeLumaBlockFull(br, ctx, blk, qp)) return false;
      } else {
        mb.nnz[blk] = 0;
      }
    }
  }

  // --- Chroma ------------------------------------------------------------
  int cQp = chromaQp(qp, ctx.pps->chromaQpIndexOffset);
  int32_t cbDc[4] = {0, 0, 0, 0};
  int32_t crDc[4] = {0, 0, 0, 0};
  if (mb.cbpChroma >= 1) {
    for (int plane = 0; plane < 2; plane++) {
      int32_t coeff[4];
      uint32_t totalCoeff = 0;
      if (!residualBlockCavlc(br, -1, 4, coeff, &totalCoeff)) {
        H264LOG.error("decodeMacroblockIntraWithType: CAVLC chroma DC decode failed at mb(%d,%d) plane=%d",
                       ctx.mbX, ctx.mbY, plane);
        return false;
      }
      int32_t* dst = plane == 0 ? cbDc : crDc;
      dst[0] = coeff[0];
      dst[1] = coeff[1];
      dst[2] = coeff[2];
      dst[3] = coeff[3];
      hadamard2x2(dst);
      dequantChromaDC2x2(dst, cQp);
    }
  }

  predictIntraChromaPlane(ctx.frame->u(), ctx.frame->strideC, ctx.mbX * 8,
                           ctx.mbY * 8, mb.chromaPredMode, leftAvail,
                           topAvail, topLeftAvail);
  predictIntraChromaPlane(ctx.frame->v(), ctx.frame->strideC, ctx.mbX * 8,
                           ctx.mbY * 8, mb.chromaPredMode, leftAvail,
                           topAvail, topLeftAvail);

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

  if (br.error()) {
    H264LOG.error("decodeMacroblockIntraWithType: bitstream error at end of macroblock at mb(%d,%d)",
                   ctx.mbX, ctx.mbY);
    return false;
  }
  result->ok = true;
  return true;
}

/**
 * I-slice entry point: reads mb_type itself (clause 7.3.5, the
 * unconditional `mb_type` read that only happens directly like this in
 * an I-slice - P-slices read it via the Inter path's own mb_type >= 5
 * branch instead), then defers to decodeMacroblockIntraWithType() for
 * everything else.
 */
inline bool decodeMacroblockIntra(BitReader& br, MbDecodeContext& ctx,
                                   int* qpY, MacroblockDecodeResult* result) {
  uint32_t mbTypeRaw = br.ue();
  if (mbTypeRaw > 25 || br.error()) {
    H264LOG.error("decodeMacroblockIntra: invalid/truncated mb_type at mb(%d,%d)",
                   ctx.mbX, ctx.mbY);
    *result = MacroblockDecodeResult();
    result->unsupported = true;
    return true;
  }
  return decodeMacroblockIntraWithType(br, ctx, mbTypeRaw, qpY, result);
}

}  // namespace tinyh264
