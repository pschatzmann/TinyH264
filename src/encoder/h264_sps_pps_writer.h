#pragma once
#include <stdint.h>
#include "h264_bitwriter.h"

/*
 * Header-only. SPS/PPS/slice-header emission - the encoder-side
 * counterpart to decoder/h264_sps_pps.h and decoder/h264_slice_header.h.
 * Deliberately writes a small, fixed field set rather than modeling the
 * full Sps/Pps/SliceHeader structs those decode-side files use: this
 * encoder only ever needs to *produce* one specific, simple Baseline
 * configuration (CAVLC, frame_mbs_only, pic_order_cnt_type 2 - no POC
 * fields needed at all, one slice per picture), not represent every
 * field a real-world stream's SPS/PPS could carry. Every field written
 * here was checked against decoder/h264_sps_pps.h's/h264_slice_header.h's
 * parse order directly (this file's own comments cite the exact syntax
 * each block matches) so a round-trip through this project's own decoder
 * - and, since these are exactly the fields any Baseline decoder expects,
 * through real ffmpeg too - parses cleanly. See
 * test/native/test_encode_iframe.cpp for that round-trip verification.
 */

namespace tinyh264 {

/**
 * Writes seq_parameter_set_rbsp() (clause 7.3.2.1) for a Baseline-profile
 * CAVLC stream of `width`x`height` (must each be a multiple of 16 -
 * frame_cropping is not implemented, matching this encoder's I_16x16-
 * only first milestone's scope). `levelIdc` is the raw level_idc byte
 * (30 = level 3.0, this library's README convention). Fixed choices not
 * exposed as parameters: pic_order_cnt_type = 2 (no POC fields needed at
 * all - the simplest valid choice, and what parseSps() already handles
 * with nothing further to read), log2_max_frame_num_minus4 = 4
 * (max_frame_num = 256, headroom for a future multi-frame encoder
 * without revisiting this function), frame_mbs_only_flag = 1 (progressive
 * only, the only case parseSps() accepts anyway).
 */
inline void writeSpsRbsp(BitWriter& bw, int width, int height,
                          uint8_t levelIdc, int maxNumRefFrames) {
  bw.u(66, 8);   // profile_idc: Baseline
  bw.u(0, 8);    // constraint_set0..5_flag (6) + reserved_zero_2bits
  bw.u(levelIdc, 8);
  bw.ue(0);      // seq_parameter_set_id

  /*
   * profile_idc 66 has no chroma-extension fields (spsHasChromaExtension()
   * only applies to High-profile-family profile_idc values) - nothing to
   * write here, matching parseSps()'s own `if` gate.
   */

  bw.ue(4);      // log2_max_frame_num_minus4
  bw.ue(2);      // pic_order_cnt_type = 2: no further POC fields at all

  bw.ue((uint32_t)(maxNumRefFrames > 0 ? maxNumRefFrames : 1));  // max_num_ref_frames
  bw.flag(false);  // gaps_in_frame_num_value_allowed_flag

  bw.ue((uint32_t)(width / 16 - 1));   // pic_width_in_mbs_minus1
  bw.ue((uint32_t)(height / 16 - 1));  // pic_height_in_map_units_minus1
  bw.flag(true);   // frame_mbs_only_flag
  bw.flag(true);   // direct_8x8_inference_flag (irrelevant without B-slices,
                    // true is the conventional/simplest choice)
  bw.flag(false);  // frame_cropping_flag (width/height already MB-aligned)
  bw.flag(false);  // vui_parameters_present_flag

  bw.rbspTrailingBits();
}

/**
 * Writes pic_parameter_set_rbsp() (clause 7.3.2.2) - CAVLC
 * (entropy_coding_mode_flag = 0), one slice group, no weighted
 * prediction, no PPS-level scaling lists/8x8 transform (all Baseline-
 * profile requirements anyway). `qp` (0-51) becomes
 * pic_init_qp_minus26 directly, with every slice using slice_qp_delta =
 * 0 (see writeSliceHeaderIdr()) - i.e. this PPS bakes in a fixed QP for
 * the whole stream rather than exposing per-slice QP adjustment, matching
 * this encoder's current fixed-QP design (no rate control yet).
 *
 * `dbEna` (default false, matching this function's original fixed
 * behavior): when true, sets deblocking_filter_control_present_flag
 * instead, so a per-slice disable_deblocking_filter_idc/offset triplet
 * can be written - see writeSliceHeaderIdr()/writeSliceHeaderP()'s own
 * `dbEna` parameter, which must be passed the same value this was
 * called with (the PPS flag and the slice header fields it gates must
 * agree, or a spec-compliant decoder will misparse the slice header).
 */
inline void writePpsRbsp(BitWriter& bw, int qp, bool dbEna = false) {
  bw.ue(0);  // pic_parameter_set_id
  bw.ue(0);  // seq_parameter_set_id
  bw.flag(false);  // entropy_coding_mode_flag: CAVLC
  bw.flag(false);  // bottom_field_pic_order_in_frame_present_flag
  bw.ue(0);        // num_slice_groups_minus1
  bw.ue(0);        // num_ref_idx_l0_default_active_minus1
  bw.ue(0);        // num_ref_idx_l1_default_active_minus1
  bw.flag(false);  // weighted_pred_flag
  bw.u(0, 2);       // weighted_bipred_idc
  bw.se(qp - 26);  // pic_init_qp_minus26
  bw.se(0);        // pic_init_qs_minus26
  bw.se(0);        // chroma_qp_index_offset
  bw.flag(dbEna);  // deblocking_filter_control_present_flag
  bw.flag(false);  // constrained_intra_pred_flag
  bw.flag(false);  // redundant_pic_cnt_present_flag

  bw.rbspTrailingBits();
}

/**
 * Writes slice_header() (clause 7.3.3) for the (only) slice of an IDR
 * I-frame, up to but not including slice_data() - the caller continues
 * writing macroblock layer data into the same BitWriter immediately
 * after this returns, then calls bw.rbspTrailingBits() itself once the
 * whole slice_data() is done (slice_header() and slice_data() share one
 * NAL unit's RBSP - no trailing-bits boundary between them). Matches
 * parseSliceHeader()'s field order exactly for slice_type == I,
 * pic_order_cnt_type == 2, entropy_coding_mode_flag == 0,
 * redundant_pic_cnt_present_flag == 0, weighted_pred_flag == 0,
 * deblocking_filter_control_present_flag == 0 (i.e. the specific
 * SPS/PPS this file's writeSpsRbsp()/writePpsRbsp() produce). No `qp`
 * parameter: slice_qp_delta is always written as 0, since
 * writePpsRbsp()'s pic_init_qp_minus26 already carries the whole
 * stream's (fixed, no-rate-control-yet) QP.
 *
 * `dbEna` must match the deblocking_filter_control_present_flag the
 * PPS this slice references was written with (writePpsRbsp()'s own
 * `dbEna`) - default false, matching this function's original fixed
 * behavior.
 */
inline void writeSliceHeaderIdr(BitWriter& bw, bool dbEna = false) {
  bw.ue(0);  // first_mb_in_slice
  bw.ue(7);  // slice_type = 7 (I, "all slices in picture are I" form -
             /*
              * conventional encoder choice, decodes identically to 2 since
              * the decoder takes slice_type % 5)
              */
  bw.ue(0);  // pic_parameter_set_id
  bw.u(0, 8);  // frame_num (log2_max_frame_num_minus4 == 4 -> 8 bits)
  bw.ue(0);    // idr_pic_id

  /*
   * pic_order_cnt_type == 2: no POC fields.
   * redundant_pic_cnt_present_flag == 0: no redundant_pic_cnt.
   * sliceType == I: no num_ref_idx_active_override_flag, no
   * ref_pic_list_modification().
   * weighted_pred_flag == 0 and/or sliceType == I: no pred_weight_table().
   */

  /*
   * nal_ref_idc != 0 (IDR is always a reference) + isIdr:
   * dec_ref_pic_marking()'s IDR form.
   */
  bw.flag(false);  // no_output_of_prior_pics_flag
  bw.flag(false);  // long_term_reference_flag

  bw.se(0);  // slice_qp_delta: 0 (pic_init_qp_minus26 already == qp - 26)

  if (dbEna) {
    // Matches Espressif's own esp_h264_enc_hw_set_slice(): all three
    // values fixed at 0 (deblocking_filter_idc=0 => filter applied
    // normally, no alpha/beta offset) - same *filtering behavior* as
    // the deblocking_filter_control_present_flag==0 default, just
    // explicitly signaled instead of implied.
    bw.se(0);  // disable_deblocking_filter_idc
    bw.se(0);  // slice_alpha_c0_offset_div2
    bw.se(0);  // slice_beta_offset_div2
  } else {
    /*
     * deblocking_filter_control_present_flag == 0: no disable_deblocking_
     * filter_idc / offsets.
     */
  }
  // num_slice_groups_minus1 == 0: no slice_group_change_cycle.
}

/**
 * Writes slice_header() (clause 7.3.3) for the (only) slice of a non-IDR
 * P-frame, up to but not including slice_data() - same "caller keeps
 * writing macroblock/mb_skip_run data into the same BitWriter" contract
 * as writeSliceHeaderIdr() above. `frameNum` must fit in 8 bits (this
 * file's fixed log2_max_frame_num_minus4 == 4) - the caller
 * (Encoder::encodePFrame()) is responsible for wrapping it modulo 256.
 * Unlike writeSliceHeaderIdr(), this *does* take a `qp`: a P-frame can
 * use a different QP than the I-frame that established the PPS's
 * pic_init_qp_minus26 without needing a whole new PPS - slice_qp_delta
 * (clause 7.3.3) exists exactly for this, computed here as `qp -
 * ppsBaseQp` (the QP writePpsRbsp() was originally called with). This
 * is also the mechanism rate control (adjusting QP frame-to-frame)
 * hooks into - see Encoder's qp-tracking members.
 *
 * `dbEna`: see writeSliceHeaderIdr()'s own parameter of the same name -
 * must match the PPS's deblocking_filter_control_present_flag.
 */
inline void writeSliceHeaderP(BitWriter& bw, int frameNum, int qp,
                               int ppsBaseQp, bool dbEna = false) {
  bw.ue(0);  // first_mb_in_slice
  bw.ue(5);  // slice_type = 5 (P, "all slices in picture are P" form -
             // same convention as writeSliceHeaderIdr()'s slice_type=7)
  bw.ue(0);  // pic_parameter_set_id
  bw.u((uint32_t)frameNum, 8);  // frame_num (log2_max_frame_num_minus4==4)

  /*
   * Not IDR: no idr_pic_id.
   * pic_order_cnt_type == 2: no POC fields.
   * redundant_pic_cnt_present_flag == 0: no redundant_pic_cnt.
   */

  bw.flag(false);  // num_ref_idx_active_override_flag: use PPS default
                    /*
                     * (num_ref_idx_l0_default_active_minus1 == 0, i.e. 1
                     * active reference - this encoder's single-reference-
                     * frame scope, see h264_macroblock_encode_inter.h)
                     */
  bw.flag(false);  // ref_pic_list_modification_flag_l0: no reordering

  // weighted_pred_flag == 0 (PPS): no pred_weight_table().

  /*
   * nal_ref_idc != 0 (every P-frame here is itself a future reference) +
   * !isIdr: dec_ref_pic_marking()'s non-IDR form.
   */
  bw.flag(false);  // adaptive_ref_pic_marking_mode_flag: default sliding
                    /*
                     * window (this encoder's only supported reference
                     * marking, matching the decoder's own scope)
                     */

  bw.se(qp - ppsBaseQp);  // slice_qp_delta

  if (dbEna) {
    bw.se(0);  // disable_deblocking_filter_idc
    bw.se(0);  // slice_alpha_c0_offset_div2
    bw.se(0);  // slice_beta_offset_div2
  } else {
    /*
     * deblocking_filter_control_present_flag == 0: no disable_deblocking_
     * filter_idc / offsets.
     */
  }
  // num_slice_groups_minus1 == 0: no slice_group_change_cycle.
}

}  // namespace tinyh264
