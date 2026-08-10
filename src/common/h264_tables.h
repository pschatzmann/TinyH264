#pragma once
#include <stdint.h>

// Header-only. Small numeric tables that aren't CAVLC-specific (those live
// in h264_cavlc_tables.h). Values cross-checked against FFmpeg's
// libavcodec/h264data.c (ff_zigzag_scan, ff_h264_golomb_to_intra4x4_cbp,
// ff_h264_golomb_to_inter_cbp, ff_h264_dequant4_coeff_init) rather than
// transcribed from memory, for the same reason as the CAVLC tables.

namespace tinyh264 {

// 4x4 zig-zag scan: scanIndex -> raster position (row*4+col), frame
// (non-field) macroblocks. ITU-T H.264 Fig. 8-8 / Table 8-13.
static const uint8_t kZigZag4x4[16] = {
    0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15,
};

// luma4x4BlkIdx (0..15) -> (x,y) position in 4x4-block units within the
// macroblock, per clause 6.4.3 (the "raster of 8x8 quadrants, each a
// raster of 4x4 blocks" ordering used throughout the spec for scanning
// and neighbor derivation).
static const uint8_t kBlk4x4X[16] = {0, 1, 0, 1, 2, 3, 2, 3,
                                      0, 1, 0, 1, 2, 3, 2, 3};
static const uint8_t kBlk4x4Y[16] = {0, 0, 1, 1, 0, 0, 1, 1,
                                      2, 2, 3, 3, 2, 2, 3, 3};

// coded_block_pattern: me(v) codeNum (0..47) -> (CodedBlockPatternChroma<<4)
// | CodedBlockPatternLuma, for 4:2:0 (chroma_format_idc == 1). Separate
// tables for Intra_4x4/Intra_8x8 vs. all other (Inter, and by extension
// I_16x16 doesn't use this table at all - its cbp is derived from mb_type).
// ITU-T H.264 Table 9-4.
static const uint8_t kCbpIntra4x4[48] = {
    47, 31, 15, 0,  23, 27, 29, 30, 7,  11, 13, 14, 39, 43, 45, 46,
    16, 3,  5,  10, 12, 19, 21, 26, 28, 35, 37, 42, 44, 1,  2,  4,
    8,  17, 18, 20, 24, 6,  9,  22, 25, 32, 33, 34, 36, 40, 38, 41,
};
static const uint8_t kCbpInter[48] = {
    0,  16, 1,  2,  4,  8,  32, 3,  5,  10, 12, 15, 47, 7,  11, 13,
    14, 6,  9,  31, 35, 37, 42, 44, 33, 34, 36, 40, 39, 43, 45, 46,
    17, 18, 20, 24, 19, 21, 26, 28, 23, 27, 29, 30, 22, 25, 38, 41,
};

/// P-slice mb_type values 0..4 (clause 7.4.5, Table 7-13) - the partition
/// shape for an Inter macroblock. Values >= 5 instead mean "Intra
/// macroblock using I-slice mb_type numbering minus 5" and are not part of
/// this enum. A wire-format constant, needed identically by the decoder
/// (decoder/h264_macroblock_inter.h) and the encoder
/// (encoder/h264_macroblock_encode_inter.h), so it lives here rather than
/// being duplicated or owned by one side - same reasoning as
/// common/h264_nal_types.h's NalUnitType.
enum PMbType {
  kPL0_16x16 = 0,   ///< one 16x16 partition
  kPL0_L0_16x8 = 1, ///< two 16x8 partitions (stacked)
  kPL0_L0_8x16 = 2, ///< two 8x16 partitions (side by side)
  kP8x8 = 3,        ///< four 8x8 partitions, each with its own sub_mb_type
  kP8x8ref0 = 4,    ///< same as kP8x8, but ref_idx_l0 is never coded (always 0)
};
/// sub_mb_type values for a P_8x8/P_8x8ref0 quadrant (clause 7.4.5.2,
/// Table 7-15) - how one 8x8 region further splits.
enum SubMbType { kSub8x8 = 0, kSub8x4 = 1, kSub4x8 = 2, kSub4x4 = 3 };

// normAdjust4x4(qP%6, class), class = (row%2)+(col%2) in {0=both-even,
// 1=mixed, 2=both-odd}. ITU-T H.264 clause 8.5.9 / Table (via
// LevelScale4x4 with a flat weightScale of 16, the only case a
// Baseline-profile stream can signal since it never carries scaling
// matrices - see parseSps()/parsePps() in h264_sps_pps.h).
static const uint8_t kNormAdjust4x4[6][3] = {
    {10, 13, 16}, {11, 14, 18}, {13, 16, 20},
    {14, 18, 23}, {16, 20, 25}, {18, 23, 29},
};

}  // namespace tinyh264
