#pragma once
#include <stdint.h>
#include "h264_bitreader.h"
#include "h264_config.h"

// Header-only: SPS/PPS structs plus parse functions, per ITU-T H.264
// clauses 7.3.2.1 (SPS) and 7.3.2.2 (PPS). Only the subset needed to decode
// Baseline-profile (I/P slices, CAVLC) streams is fully supported; streams
// that require features we don't decode (High-profile chroma/scaling
// extensions, FMO slice groups, PPS scaling lists) are detected and flagged
// via `unsupported` rather than mis-parsed or crashed on.

#ifndef H264_MAX_POC_CYCLE
#define H264_MAX_POC_CYCLE 8  // num_ref_frames_in_pic_order_cnt_cycle cap
#endif

namespace tinyh264 {

/// Parsed seq_parameter_set_rbsp() (clause 7.3.2.1). One SPS is cached per
/// distinct seq_parameter_set_id seen in the stream (Decoder::spsTable_);
/// a slice's PPS points back at the SPS it needs via Pps::spsId.
struct Sps {
  bool valid = false;
  bool unsupported = false;  // parsed, but describes a stream we can't decode

  uint8_t profileIdc = 0;
  uint8_t levelIdc = 0;
  uint32_t id = 0;

  uint32_t log2MaxFrameNumMinus4 = 0;
  uint32_t picOrderCntType = 0;
  uint32_t log2MaxPicOrderCntLsbMinus4 = 0;
  bool deltaPicOrderAlwaysZeroFlag = false;
  int32_t offsetForNonRefPic = 0;
  int32_t offsetForTopToBottomField = 0;
  uint32_t numRefFramesInPicOrderCntCycle = 0;
  int32_t offsetForRefFrame[H264_MAX_POC_CYCLE] = {0};

  uint32_t maxNumRefFrames = 0;
  bool gapsInFrameNumValueAllowed = false;

  uint32_t picWidthInMbsMinus1 = 0;
  uint32_t picHeightInMapUnitsMinus1 = 0;
  bool frameMbsOnlyFlag = false;
  bool mbAdaptiveFrameFieldFlag = false;
  bool direct8x8InferenceFlag = false;

  bool frameCroppingFlag = false;
  uint32_t cropLeft = 0, cropRight = 0, cropTop = 0, cropBottom = 0;

  // Derived (filled in by finalize(), called at the end of parseSps()):
  uint32_t picWidthInMbs = 0;
  uint32_t picHeightInMbs = 0;
  uint32_t codedWidth = 0, codedHeight = 0;      // MB-aligned
  uint32_t displayWidth = 0, displayHeight = 0;  // after cropping

  /// Derives the picture-size fields (clause 7.4.2.1.1's PicWidthInMbs /
  /// FrameHeightInMbs / cropping formulas) from the raw parsed
  /// *Minus1 syntax elements. Called once at the end of parseSps() after
  /// all the raw fields are in place.
  void finalize() {
    picWidthInMbs = picWidthInMbsMinus1 + 1;
    // frameMbsOnlyFlag must be true for anything we support (see parseSps);
    // FrameHeightInMbs = (2 - frame_mbs_only_flag) * (map_units + 1).
    picHeightInMbs = picHeightInMapUnitsMinus1 + 1;
    codedWidth = picWidthInMbs * H264_MB_SIZE;
    codedHeight = picHeightInMbs * H264_MB_SIZE;

    // 4:2:0 only (chroma_format_idc == 1, the only case we accept):
    // CropUnitX = 2, CropUnitY = 2 * (2 - frame_mbs_only_flag) = 2.
    uint32_t cropUnitX = 2, cropUnitY = 2;
    displayWidth = codedWidth;
    displayHeight = codedHeight;
    if (frameCroppingFlag) {
      uint32_t cw = cropUnitX * (cropLeft + cropRight);
      uint32_t ch = cropUnitY * (cropTop + cropBottom);
      displayWidth = (cw < codedWidth) ? codedWidth - cw : 0;
      displayHeight = (ch < codedHeight) ? codedHeight - ch : 0;
    }
  }
};

/// Parsed pic_parameter_set_rbsp() (clause 7.3.2.2). One PPS is cached per
/// distinct pic_parameter_set_id seen in the stream (Decoder::ppsTable_);
/// each slice header names the PPS (and, transitively via Pps::spsId, the
/// SPS) it uses.
struct Pps {
  bool valid = false;
  bool unsupported = false;

  uint32_t id = 0;
  uint32_t spsId = 0;
  bool entropyCodingModeFlag = false;  // false = CAVLC (only mode we decode)
  bool bottomFieldPicOrderInFramePresentFlag = false;
  uint32_t numSliceGroupsMinus1 = 0;  // >0 = FMO, not supported
  uint32_t numRefIdxL0DefaultActiveMinus1 = 0;
  uint32_t numRefIdxL1DefaultActiveMinus1 = 0;
  bool weightedPredFlag = false;
  uint8_t weightedBipredIdc = 0;
  int32_t picInitQpMinus26 = 0;
  int32_t picInitQsMinus26 = 0;
  int32_t chromaQpIndexOffset = 0;
  bool deblockingFilterControlPresentFlag = false;
  bool constrainedIntraPredFlag = false;
  bool redundantPicCntPresentFlag = false;
  bool transform8x8ModeFlag = false;
  int32_t secondChromaQpIndexOffset = 0;
};

/// profile_idc values whose SPS carries the High-profile chroma/bit-depth/
/// scaling-list extension fields (spec 7.3.2.1.1, the `if (profile_idc ==
/// ...)` list). None of these are Baseline, so we only need to parse far
/// enough to recognize them and bail.
inline bool spsHasChromaExtension(uint8_t profileIdc) {
  switch (profileIdc) {
    case 100: case 110: case 122: case 244: case 44:
    case 83: case 86: case 118: case 128: case 138:
    case 139: case 134: case 135:
      return true;
    default:
      return false;
  }
}

/// Parses seq_parameter_set_rbsp() (clause 7.3.2.1) from `br` into `sps`.
/// Always returns true (the bool return exists only to mirror
/// parseSliceHeader()'s/parsePps()'s signature) - callers must check
/// `sps->unsupported` for either a raw bitstream error or a structurally
/// valid SPS that describes a stream this decoder can't handle
/// (interlaced content, non-4:2:0 chroma, High-profile scaling lists, an
/// oversized POC reorder cycle), and `sps->valid` for outright success.
inline bool parseSps(BitReader& br, Sps* sps) {
  *sps = Sps();

  sps->profileIdc = (uint8_t)br.u(8);
  br.u(8);  // constraint_set0..5_flag (6 bits) + reserved_zero_2bits
  sps->levelIdc = (uint8_t)br.u(8);
  sps->id = br.ue();

  if (spsHasChromaExtension(sps->profileIdc)) {
    uint32_t chromaFormatIdc = br.ue();
    if (chromaFormatIdc == 3) br.u(1);  // separate_colour_plane_flag
    br.ue();  // bit_depth_luma_minus8
    br.ue();  // bit_depth_chroma_minus8
    br.u(1);  // qpprime_y_zero_transform_bypass_flag
    bool scalingMatrixPresent = br.flag();
    if (scalingMatrixPresent || chromaFormatIdc != 1) {
      // High-profile scaling lists / non-4:2:0 chroma: not something a
      // Baseline-only decoder needs to handle. Flag and stop.
      sps->unsupported = true;
      return true;
    }
  }

  sps->log2MaxFrameNumMinus4 = br.ue();
  sps->picOrderCntType = br.ue();
  if (sps->picOrderCntType == 0) {
    sps->log2MaxPicOrderCntLsbMinus4 = br.ue();
  } else if (sps->picOrderCntType == 1) {
    sps->deltaPicOrderAlwaysZeroFlag = br.flag();
    sps->offsetForNonRefPic = br.se();
    sps->offsetForTopToBottomField = br.se();
    sps->numRefFramesInPicOrderCntCycle = br.ue();
    if (sps->numRefFramesInPicOrderCntCycle > H264_MAX_POC_CYCLE) {
      sps->unsupported = true;
      return true;
    }
    for (uint32_t i = 0; i < sps->numRefFramesInPicOrderCntCycle; i++) {
      sps->offsetForRefFrame[i] = br.se();
    }
  }
  // picOrderCntType == 2: nothing further to read here.

  sps->maxNumRefFrames = br.ue();
  sps->gapsInFrameNumValueAllowed = br.flag();
  sps->picWidthInMbsMinus1 = br.ue();
  sps->picHeightInMapUnitsMinus1 = br.ue();
  sps->frameMbsOnlyFlag = br.flag();
  if (!sps->frameMbsOnlyFlag) {
    sps->mbAdaptiveFrameFieldFlag = br.flag();
    // Interlaced (PAFF/MBAFF) content: out of scope for a Baseline-only
    // decoder (Baseline requires frame_mbs_only_flag == 1 anyway, but be
    // defensive against non-conformant/mislabeled streams).
    sps->unsupported = true;
    return true;
  }
  sps->direct8x8InferenceFlag = br.flag();
  sps->frameCroppingFlag = br.flag();
  if (sps->frameCroppingFlag) {
    sps->cropLeft = br.ue();
    sps->cropRight = br.ue();
    sps->cropTop = br.ue();
    sps->cropBottom = br.ue();
  }
  // vui_parameters_present_flag / vui_parameters(): not needed for decode,
  // and nothing meaningful follows it in the RBSP, so we stop reading here.

  if (br.error()) {
    sps->unsupported = true;
    return true;
  }

  sps->finalize();
  sps->valid = true;
  return true;
}

/// Parses pic_parameter_set_rbsp() (clause 7.3.2.2) from `br` into `pps`.
/// Like parseSps(), always returns true - check `pps->unsupported`
/// (FMO slice groups, CABAC, High-profile PPS scaling lists, or a raw
/// bitstream error) and `pps->valid` for the actual outcome.
inline bool parsePps(BitReader& br, Pps* pps) {
  *pps = Pps();

  pps->id = br.ue();
  pps->spsId = br.ue();
  pps->entropyCodingModeFlag = br.flag();
  pps->bottomFieldPicOrderInFramePresentFlag = br.flag();
  pps->numSliceGroupsMinus1 = br.ue();
  if (pps->numSliceGroupsMinus1 > 0) {
    // Flexible Macroblock Ordering: technically legal in Baseline profile,
    // but essentially never emitted by real camera/RTSP H.264 encoders.
    // Implementing the slice-group map machinery adds real complexity for
    // a feature this decoder's target sources don't use, so we flag it
    // as unsupported rather than mis-decode it.
    pps->unsupported = true;
    return true;
  }
  pps->numRefIdxL0DefaultActiveMinus1 = br.ue();
  pps->numRefIdxL1DefaultActiveMinus1 = br.ue();
  pps->weightedPredFlag = br.flag();
  pps->weightedBipredIdc = (uint8_t)br.u(2);
  pps->picInitQpMinus26 = br.se();
  pps->picInitQsMinus26 = br.se();
  pps->chromaQpIndexOffset = br.se();
  pps->deblockingFilterControlPresentFlag = br.flag();
  pps->constrainedIntraPredFlag = br.flag();
  pps->redundantPicCntPresentFlag = br.flag();

  pps->secondChromaQpIndexOffset = pps->chromaQpIndexOffset;  // spec default
  if (br.moreRbspData()) {
    pps->transform8x8ModeFlag = br.flag();
    bool scalingMatrixPresent = br.flag();
    if (scalingMatrixPresent) {
      // High-profile PPS scaling lists: same rationale as SPS above.
      pps->unsupported = true;
      return true;
    }
    pps->secondChromaQpIndexOffset = br.se();
  }

  if (br.error()) {
    pps->unsupported = true;
    return true;
  }

  if (pps->entropyCodingModeFlag) {
    // CABAC: this decoder only implements CAVLC.
    pps->unsupported = true;
    return true;
  }

  pps->valid = true;
  return true;
}

}  // namespace tinyh264
