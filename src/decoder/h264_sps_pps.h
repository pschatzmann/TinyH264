#pragma once
#include <stdint.h>
#include "../common/Logger.h"
#include "h264_bitreader.h"
#include "h264_config.h"

/*
 * Header-only: SPS/PPS structs plus parse functions, per ITU-T H.264
 * clauses 7.3.2.1 (SPS) and 7.3.2.2 (PPS). Only the subset needed to decode
 * Baseline-profile (I/P slices, CAVLC) streams is fully supported; streams
 * that require features we don't decode (High-profile chroma/scaling
 * extensions, FMO slice groups, PPS scaling lists) are detected and flagged
 * via `unsupported` rather than mis-parsed or crashed on.
 */

#ifndef H264_MAX_POC_CYCLE
#define H264_MAX_POC_CYCLE 8  // num_ref_frames_in_pic_order_cnt_cycle cap
#endif

namespace tinyh264 {

/**
 * Parsed seq_parameter_set_rbsp() (clause 7.3.2.1). One SPS is cached per
 * distinct seq_parameter_set_id seen in the stream (Decoder::spsTable_);
 * a slice's PPS points back at the SPS it needs via Pps::spsId.
 */
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

  /*
   * vui_parameters()'s timing_info (clause E.1.1, E.2.1) - the encoder's
   * *declared* source frame rate, not anything measured by this decoder.
   * Optional: many encoders (including this library's own
   * TinyH264Encoder) never emit vui_parameters() at all, in which case
   * timingInfoPresentFlag stays false and frameRate() below returns 0.
   */
  bool timingInfoPresentFlag = false;
  uint32_t numUnitsInTick = 0;
  uint32_t timeScale = 0;

  // Derived (filled in by finalize(), called at the end of parseSps()):
  uint32_t picWidthInMbs = 0;
  uint32_t picHeightInMbs = 0;
  uint32_t codedWidth = 0, codedHeight = 0;      // MB-aligned
  uint32_t displayWidth = 0, displayHeight = 0;  // after cropping

  /**
   * Derives the picture-size fields (clause 7.4.2.1.1's PicWidthInMbs /
   * FrameHeightInMbs / cropping formulas) from the raw parsed
   * *Minus1 syntax elements. Called once at the end of parseSps() after
   * all the raw fields are in place.
   */
  void finalize() {
    picWidthInMbs = picWidthInMbsMinus1 + 1;
    /*
     * frameMbsOnlyFlag must be true for anything we support (see parseSps);
     * FrameHeightInMbs = (2 - frame_mbs_only_flag) * (map_units + 1).
     */
    picHeightInMbs = picHeightInMapUnitsMinus1 + 1;
    codedWidth = picWidthInMbs * H264_MB_SIZE;
    codedHeight = picHeightInMbs * H264_MB_SIZE;

    /*
     * 4:2:0 only (chroma_format_idc == 1, the only case we accept):
     * CropUnitX = 2, CropUnitY = 2 * (2 - frame_mbs_only_flag) = 2.
     */
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

  /**
   * The encoder-declared frame rate from vui_parameters()'s timing_info,
   * in frames/second - clause E.2.1's num_units_in_tick/time_scale, per
   * the standard `time_scale / (2 * num_units_in_tick)` relationship
   * (the factor of 2 is the spec's clock-tick convention, which counts
   * ticks per *field* even for progressive streams - cross-checked
   * against FFmpeg's own SPS-to-framerate conversion in h264_slice.c).
   * Returns 0.0 if the stream's SPS never set timingInfoPresentFlag - a
   * common, spec-legal case, not an error (see timingInfoPresentFlag's
   * own comment above).
   */
  double frameRate() const {
    return (timingInfoPresentFlag && numUnitsInTick != 0)
               ? (double)timeScale / (2.0 * (double)numUnitsInTick)
               : 0.0;
  }
};

/**
 * Parsed pic_parameter_set_rbsp() (clause 7.3.2.2). One PPS is cached per
 * distinct pic_parameter_set_id seen in the stream (Decoder::ppsTable_);
 * each slice header names the PPS (and, transitively via Pps::spsId, the
 * SPS) it uses.
 */
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

/**
 * profile_idc values whose SPS carries the High-profile chroma/bit-depth/
 * scaling-list extension fields (spec 7.3.2.1.1, the `if (profile_idc ==
 * ...)` list). None of these are Baseline, so we only need to parse far
 * enough to recognize them and bail.
 */
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

/**
 * Parses the leading part of vui_parameters() (clause E.1.1) just far
 * enough to reach timing_info - aspect_ratio_info, overscan_info,
 * video_signal_type, and chroma_loc_info all precede it in the syntax
 * and must be walked (not skipped) to stay byte/bit-aligned, even though
 * none of their *values* are used here. Stops right after timing_info;
 * nal_hrd_parameters()/vcl_hrd_parameters()/pic_struct_present_flag/
 * bitstream_restriction (everything else in vui_parameters()) are never
 * read, since nothing after timing_info is needed and nothing meaningful
 * follows vui_parameters() in the RBSP anyway. Field order cross-checked
 * against FFmpeg's ff_h2645_decode_common_vui_params()/
 * decode_vui_parameters() (h2645_vui.c/h264_ps.c).
 */
inline void parseVuiTimingInfo(BitReader& br, Sps* sps) {
  if (br.flag()) {  // aspect_ratio_info_present_flag
    uint32_t aspectRatioIdc = br.u(8);
    if (aspectRatioIdc == 255) {  // Extended_SAR
      br.u(16);  // sar_width
      br.u(16);  // sar_height
    }
  }
  if (br.flag()) {  // overscan_info_present_flag
    br.flag();       // overscan_appropriate_flag
  }
  if (br.flag()) {  // video_signal_type_present_flag
    br.u(3);          // video_format
    br.flag();        // video_full_range_flag
    if (br.flag()) {  // colour_description_present_flag
      br.u(8);  // colour_primaries
      br.u(8);  // transfer_characteristics
      br.u(8);  // matrix_coefficients
    }
  }
  if (br.flag()) {  // chroma_loc_info_present_flag
    br.ue();          // chroma_sample_loc_type_top_field
    br.ue();          // chroma_sample_loc_type_bottom_field
  }

  sps->timingInfoPresentFlag = br.flag();
  if (sps->timingInfoPresentFlag) {
    sps->numUnitsInTick = br.u(32);
    sps->timeScale = br.u(32);
    br.flag();  // fixed_frame_rate_flag - not currently exposed
    if (sps->numUnitsInTick == 0 || sps->timeScale == 0) {
      // Spec-illegal (would divide by zero); treat as "not present" per
      // the same tolerance FFmpeg's own decoder applies.
      sps->timingInfoPresentFlag = false;
    }
  }
}

/**
 * Parses seq_parameter_set_rbsp() (clause 7.3.2.1) from `br` into `sps`.
 * Always returns true (the bool return exists only to mirror
 * parseSliceHeader()'s/parsePps()'s signature) - callers must check
 * `sps->unsupported` for either a raw bitstream error or a structurally
 * valid SPS that describes a stream this decoder can't handle
 * (interlaced content, non-4:2:0 chroma, High-profile scaling lists, an
 * oversized POC reorder cycle), and `sps->valid` for outright success.
 */
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
      /*
       * High-profile scaling lists / non-4:2:0 chroma: not something a
       * Baseline-only decoder needs to handle. Flag and stop.
       */
      H264LOG.warn("parseSps: scaling lists / non-4:2:0 chroma not supported");
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
      H264LOG.warn("parseSps: numRefFramesInPicOrderCntCycle=%u exceeds H264_MAX_POC_CYCLE=%d",
                    sps->numRefFramesInPicOrderCntCycle, H264_MAX_POC_CYCLE);
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
    /*
     * Interlaced (PAFF/MBAFF) content: out of scope for a Baseline-only
     * decoder (Baseline requires frame_mbs_only_flag == 1 anyway, but be
     * defensive against non-conformant/mislabeled streams).
     */
    H264LOG.warn("parseSps: interlaced (PAFF/MBAFF) content not supported");
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
  if (br.flag()) {  // vui_parameters_present_flag
    parseVuiTimingInfo(br, sps);
  }

  if (br.error()) {
    H264LOG.error("parseSps: bitstream error/truncation");
    sps->unsupported = true;
    return true;
  }

  sps->finalize();
  sps->valid = true;
  return true;
}

/**
 * Parses pic_parameter_set_rbsp() (clause 7.3.2.2) from `br` into `pps`.
 * Like parseSps(), always returns true - check `pps->unsupported`
 * (FMO slice groups, CABAC, High-profile PPS scaling lists, or a raw
 * bitstream error) and `pps->valid` for the actual outcome.
 */
inline bool parsePps(BitReader& br, Pps* pps) {
  *pps = Pps();

  pps->id = br.ue();
  pps->spsId = br.ue();
  pps->entropyCodingModeFlag = br.flag();
  pps->bottomFieldPicOrderInFramePresentFlag = br.flag();
  pps->numSliceGroupsMinus1 = br.ue();
  if (pps->numSliceGroupsMinus1 > 0) {
    /*
     * Flexible Macroblock Ordering: technically legal in Baseline profile,
     * but essentially never emitted by real camera/RTSP H.264 encoders.
     * Implementing the slice-group map machinery adds real complexity for
     * a feature this decoder's target sources don't use, so we flag it
     * as unsupported rather than mis-decode it.
     */
    H264LOG.warn("parsePps: FMO (numSliceGroupsMinus1=%u) not supported",
                  pps->numSliceGroupsMinus1);
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
      H264LOG.warn("parsePps: scaling lists not supported");
      pps->unsupported = true;
      return true;
    }
    pps->secondChromaQpIndexOffset = br.se();
  }

  if (br.error()) {
    H264LOG.error("parsePps: bitstream error/truncation");
    pps->unsupported = true;
    return true;
  }

  if (pps->entropyCodingModeFlag) {
    // CABAC: this decoder only implements CAVLC.
    H264LOG.warn("parsePps: CABAC entropy coding not supported (CAVLC only)");
    pps->unsupported = true;
    return true;
  }

  pps->valid = true;
  return true;
}

}  // namespace tinyh264
