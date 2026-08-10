#pragma once
#include <stdint.h>
#include "h264_bitreader.h"
#include "h264_config.h"
#include "h264_nal.h"
#include "h264_sps_pps.h"

/*
 * Header-only: slice_header() parsing, spec clause 7.3.3. Only I and P
 * slices are supported (B/SP/SI are flagged unsupported). Up to
 * H264_MAX_REF_FRAMES reference pictures are supported (see
 * h264_config.h/Decoder::setMaxRefFrames()), but only the *default*
 * reference picture list construction (clause 8.2.4.2) and *sliding
 * window* reference marking (clause 8.2.5.3) - ref_pic_list_modification()
 * (explicit reordering) and adaptive_ref_pic_marking_mode (MMCO-based
 * marking) are both fully parsed (to keep the bitstream in sync for the
 * fields that follow) but flagged unsupported if actually present, rather
 * than implementing their full semantics.
 */

namespace tinyh264 {

/**
 * slice_type % 5 (clause 7.4.3, Table 7-6): the wire value can also be
 * offset by +5 to additionally signal "all other slices in this picture
 * have the same type" (handled by taking `% 5` when parsing, see
 * SliceHeader::sliceType). Only kSliceI and kSliceP are actually
 * supported - B/SP/SI slices are parsed far enough to flag
 * SliceHeader::unsupported but never decoded.
 */
enum SliceTypeBase {
  kSliceP = 0,
  kSliceB = 1,
  kSliceI = 2,
  kSliceSP = 3,
  kSliceSI = 4,
};

/**
 * Parsed slice_header() (clause 7.3.3) plus the fields it computes with
 * help from the active SPS/PPS (e.g. sliceQp). Fully re-parsed for every
 * slice NAL - nothing here is cached across slices/pictures.
 */
struct SliceHeader {
  bool valid = false;
  bool unsupported = false;

  uint32_t firstMbInSlice = 0;
  uint32_t sliceTypeRaw = 0;
  uint8_t sliceType = 0;  // sliceTypeRaw % 5, one of SliceTypeBase
  uint32_t ppsId = 0;
  uint32_t frameNum = 0;
  bool isIdr = false;
  uint32_t idrPicId = 0;

  uint32_t picOrderCntLsb = 0;
  int32_t deltaPicOrderCntBottom = 0;
  int32_t deltaPicOrderCnt[2] = {0, 0};

  uint32_t redundantPicCnt = 0;

  bool numRefIdxActiveOverrideFlag = false;
  uint32_t numRefIdxL0ActiveMinus1 = 0;  // effective value filled below

  bool refPicListModificationFlagL0 = false;

  bool noOutputOfPriorPicsFlag = false;
  bool longTermReferenceFlag = false;
  bool adaptiveRefPicMarkingModeFlag = false;
  bool memoryManagementControlOp5 = false;  // MMCO 5 seen (resets POC/frame_num)

  int32_t sliceQpDelta = 0;
  int32_t sliceQp = 0;  // derived: pps.picInitQpMinus26 + 26 + sliceQpDelta

  uint32_t disableDeblockingFilterIdc = 0;
  int32_t sliceAlphaC0OffsetDiv2 = 0;
  int32_t sliceBetaOffsetDiv2 = 0;
};

/**
 * Consumes ref_pic_list_modification() (clause 7.3.3.1) for list 0 only
 * (no B-slice list 1, since B-slices are unsupported). The parsed
 * modification operations themselves are discarded - explicit reference
 * list reordering isn't implemented (see parseSliceHeader(), which flags
 * `refPicListModificationFlagL0 == true` as unsupported) - but the syntax
 * must still be consumed to keep the bit reader in sync with what
 * follows even in that unsupported case. Returns false on a bitstream
 * error.
 */
inline bool parseRefPicListModificationL0(BitReader& br, SliceHeader* sh) {
  sh->refPicListModificationFlagL0 = br.flag();
  if (sh->refPicListModificationFlagL0) {
    uint32_t idc;
    int guard = 0;
    do {
      idc = br.ue();
      if (idc == 0 || idc == 1) {
        br.ue();  // abs_diff_pic_num_minus1
      } else if (idc == 2) {
        br.ue();  // long_term_pic_num
      }
      if (br.error() || ++guard > 64) return false;
    } while (idc != 3);
  }
  return true;
}

/**
 * Consumes dec_ref_pic_marking() (clause 7.3.3.3). `sh->isIdr` selects the
 * IDR vs. non-IDR syntax. Adaptive (MMCO-based) reference marking isn't
 * implemented - only the *default sliding window* process (clause
 * 8.2.5.3) is (see Decoder::decodeSlice()) - so the individual MMCO
 * operations are parsed (to stay in sync with the bitstream) but
 * discarded; parseSliceHeader() flags `adaptiveRefPicMarkingModeFlag ==
 * true` as unsupported. The one exception recorded here regardless is
 * whether MMCO op 5 ("reset frame numbering") appeared
 * (SliceHeader::memoryManagementControlOp5); currently unused by the
 * decoder (picture order count isn't tracked), kept parsed/flagged for
 * forward compatibility. Returns false on a bitstream error.
 */
inline bool parseDecRefPicMarking(BitReader& br, SliceHeader* sh) {
  if (sh->isIdr) {
    sh->noOutputOfPriorPicsFlag = br.flag();
    sh->longTermReferenceFlag = br.flag();
    return !br.error();
  }
  sh->adaptiveRefPicMarkingModeFlag = br.flag();
  if (!sh->adaptiveRefPicMarkingModeFlag) return !br.error();

  uint32_t op;
  int guard = 0;
  do {
    op = br.ue();
    switch (op) {
      case 1: br.ue(); break;                 // difference_of_pic_nums_minus1
      case 2: br.ue(); break;                 // long_term_pic_num
      case 3: br.ue(); br.ue(); break;        // diff_of_pic_nums_minus1, long_term_frame_idx
      case 4: br.ue(); break;                 // max_long_term_frame_idx_plus1
      case 5: sh->memoryManagementControlOp5 = true; break;
      case 6: br.ue(); break;                 // long_term_frame_idx
      default: break;                         // 0 = end of loop
    }
    if (br.error() || ++guard > 64) return false;
  } while (op != 0);
  return true;
}

/**
 * Parses slice_header() (clause 7.3.3) from `br` into `sh`, given the
 * already-decoded `nal` header and the SPS/PPS the slice's pic_parameter_
 * set_id refers to (looked up by the caller before calling this - the
 * slice's own ppsId is re-read here for completeness but not used to
 * look anything up). Sets `sh->unsupported` (returning true, not false)
 * for any recognized-but-unimplemented feature (B/SP/SI slice types,
 * weighted prediction, explicit reference list reordering, adaptive
 * reference marking, etc.) so the caller can skip the picture cleanly;
 * returns false only on an actual bitstream/syntax error (ran out of
 * data mid-parse). The active reference count is validated by the caller
 * against the runtime-configurable maxRefFrames (Decoder::decodeSlice()),
 * not here - this function only parses the bitstream.
 */
inline bool parseSliceHeader(BitReader& br, const NalUnit& nal, const Sps& sps,
                              const Pps& pps, SliceHeader* sh) {
  *sh = SliceHeader();
  sh->isIdr = (nal.type == kNalSliceIdr);

  sh->firstMbInSlice = br.ue();
  sh->sliceTypeRaw = br.ue();
  sh->sliceType = (uint8_t)(sh->sliceTypeRaw % 5);
  sh->ppsId = br.ue();

  uint32_t frameNumBits = sps.log2MaxFrameNumMinus4 + 4;
  sh->frameNum = br.u((int)frameNumBits);

  /*
   * field_pic_flag / bottom_field_flag are never present: parseSps() already
   * flags frame_mbs_only_flag == 0 streams as unsupported before we get here.
   */

  if (sh->isIdr) {
    sh->idrPicId = br.ue();
  }

  if (sps.picOrderCntType == 0) {
    uint32_t pocLsbBits = sps.log2MaxPicOrderCntLsbMinus4 + 4;
    sh->picOrderCntLsb = br.u((int)pocLsbBits);
    if (pps.bottomFieldPicOrderInFramePresentFlag) {
      sh->deltaPicOrderCntBottom = br.se();
    }
  } else if (sps.picOrderCntType == 1 && !sps.deltaPicOrderAlwaysZeroFlag) {
    sh->deltaPicOrderCnt[0] = br.se();
    if (pps.bottomFieldPicOrderInFramePresentFlag) {
      sh->deltaPicOrderCnt[1] = br.se();
    }
  }

  if (pps.redundantPicCntPresentFlag) {
    sh->redundantPicCnt = br.ue();
  }

  if (sh->sliceType != kSliceI && sh->sliceType != kSliceSI) {
    sh->numRefIdxActiveOverrideFlag = br.flag();
    if (sh->numRefIdxActiveOverrideFlag) {
      sh->numRefIdxL0ActiveMinus1 = br.ue();
    } else {
      sh->numRefIdxL0ActiveMinus1 = pps.numRefIdxL0DefaultActiveMinus1;
    }
  }

  if (sh->sliceType != kSliceI && sh->sliceType != kSliceSI) {
    if (!parseRefPicListModificationL0(br, sh)) {
      sh->unsupported = true;
      return true;
    }
    if (sh->refPicListModificationFlagL0) {
      /*
       * Explicit reference list reordering: parsed above (to stay in sync
       * with the bitstream) but not implemented - see
       * parseRefPicListModificationL0()'s comment.
       */
      sh->unsupported = true;
      return true;
    }
  }

  if (pps.weightedPredFlag &&
      (sh->sliceType == kSliceP || sh->sliceType == kSliceSP)) {
    /*
     * Explicit weighted prediction is a Main/High-profile feature; a
     * Baseline-conformant encoder never sets weighted_pred_flag. Rather
     * than implement pred_weight_table() parsing for a syntax element we
     * can't apply anyway, flag the slice unsupported now.
     */
    sh->unsupported = true;
    return true;
  }

  if (nal.refIdc != 0) {
    if (!parseDecRefPicMarking(br, sh)) {
      sh->unsupported = true;
      return true;
    }
    if (sh->adaptiveRefPicMarkingModeFlag) {
      /*
       * Adaptive (MMCO-based) reference marking: parsed above (to stay in
       * sync with the bitstream) but not implemented - only the default
       * sliding window process is (see parseDecRefPicMarking()'s comment
       * and Decoder::decodeSlice()).
       */
      sh->unsupported = true;
      return true;
    }
  }

  // entropy_coding_mode_flag is guaranteed false (CAVLC) by parsePps().
  sh->sliceQpDelta = br.se();
  sh->sliceQp = pps.picInitQpMinus26 + 26 + sh->sliceQpDelta;

  if (pps.deblockingFilterControlPresentFlag) {
    sh->disableDeblockingFilterIdc = br.ue();
    if (sh->disableDeblockingFilterIdc != 1) {
      sh->sliceAlphaC0OffsetDiv2 = br.se();
      sh->sliceBetaOffsetDiv2 = br.se();
    }
  }

  // pps.numSliceGroupsMinus1 > 0 (FMO) is already rejected in parsePps().

  if (br.error()) {
    sh->unsupported = true;
    return true;
  }

  if (sh->sliceType != kSliceI && sh->sliceType != kSliceP) {
    sh->unsupported = true;  // B / SP / SI not supported
    return true;
  }

  sh->valid = true;
  return true;
}

}  // namespace tinyh264
