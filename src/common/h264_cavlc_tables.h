#pragma once
#include <stdint.h>

// Header-only. Raw CAVLC VLC table constants (ITU-T H.264 Table 9-5
// coeff_token, Table 9-7/9-8 total_zeros, Table 9-9 chroma DC total_zeros,
// Table 9-10 run_before).
//
// Transcribed from FFmpeg's libavcodec/h264_cavlc.c (LGPL, Michael
// Niedermayer et al.) rather than from memory, to avoid silent transcription
// errors in these exact bit-pattern constants; cross-checked against the
// spec's combinatorial structure (see h264_cavlc.h). Layout kept as
// {length, bits} pairs so a small generic prefix-code matcher (see
// decodeVlc() in h264_cavlc.h) can decode any of them without per-table
// special-casing.

namespace tinyh264 {

// coeff_token, indexed [table][TotalCoeff*4 + TrailingOnes], table in
// {0,1,2,3} selected by nC range (see nCToCoeffTokenTable() in h264_cavlc.h).
// Table 3 (nC>=8) is a closed-form FLC, not actually looked up via this
// array, but is included for completeness/self-checking.
static const uint8_t kCoeffTokenLen[4][4 * 17] = {
    {
        1, 0, 0, 0,
        6, 2, 0, 0,     8, 6, 3, 0,     9, 8, 7, 5,    10, 9, 8, 6,
       11,10, 9, 7,    13,11,10, 8,    13,13,11, 9,    13,13,13,10,
       14,14,13,11,    14,14,14,13,    15,15,14,14,    15,15,15,14,
       16,15,15,15,    16,16,16,15,    16,16,16,16,    16,16,16,16,
    },
    {
        2, 0, 0, 0,
        6, 2, 0, 0,     6, 5, 3, 0,     7, 6, 6, 4,     8, 6, 6, 4,
        8, 7, 7, 5,     9, 8, 8, 6,    11, 9, 9, 6,    11,11,11, 7,
       12,11,11, 9,    12,12,12,11,    12,12,12,11,    13,13,13,12,
       13,13,13,13,    13,14,13,13,    14,14,14,13,    14,14,14,14,
    },
    {
        4, 0, 0, 0,
        6, 4, 0, 0,     6, 5, 4, 0,     6, 5, 5, 4,     7, 5, 5, 4,
        7, 5, 5, 4,     7, 6, 6, 4,     7, 6, 6, 4,     8, 7, 7, 5,
        8, 8, 7, 6,     9, 8, 8, 7,     9, 9, 8, 8,     9, 9, 9, 8,
       10, 9, 9, 9,    10,10,10,10,    10,10,10,10,    10,10,10,10,
    },
    {
        6, 0, 0, 0,
        6, 6, 0, 0,     6, 6, 6, 0,     6, 6, 6, 6,     6, 6, 6, 6,
        6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,
        6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,
        6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,     6, 6, 6, 6,
    }
};

static const uint8_t kCoeffTokenBits[4][4 * 17] = {
    {
        1, 0, 0, 0,
        5, 1, 0, 0,     7, 4, 1, 0,     7, 6, 5, 3,     7, 6, 5, 3,
        7, 6, 5, 4,    15, 6, 5, 4,    11,14, 5, 4,     8,10,13, 4,
       15,14, 9, 4,    11,10,13,12,    15,14, 9,12,    11,10,13, 8,
       15, 1, 9,12,    11,14,13, 8,     7,10, 9,12,     4, 6, 5, 8,
    },
    {
        3, 0, 0, 0,
       11, 2, 0, 0,     7, 7, 3, 0,     7,10, 9, 5,     7, 6, 5, 4,
        4, 6, 5, 6,     7, 6, 5, 8,    15, 6, 5, 4,    11,14,13, 4,
       15,10, 9, 4,    11,14,13,12,     8,10, 9, 8,    15,14,13,12,
       11,10, 9,12,     7,11, 6, 8,     9, 8,10, 1,     7, 6, 5, 4,
    },
    {
       15, 0, 0, 0,
       15,14, 0, 0,    11,15,13, 0,     8,12,14,12,    15,10,11,11,
       11, 8, 9,10,     9,14,13, 9,     8,10, 9, 8,    15,14,13,13,
       11,14,10,12,    15,10,13,12,    11,14, 9,12,     8,10,13, 8,
       13, 7, 9,12,     9,12,11,10,     5, 8, 7, 6,     1, 4, 3, 2,
    },
    {
        3, 0, 0, 0,
        0, 1, 0, 0,     4, 5, 6, 0,     8, 9,10,11,    12,13,14,15,
       16,17,18,19,    20,21,22,23,    24,25,26,27,    28,29,30,31,
       32,33,34,35,    36,37,38,39,    40,41,42,43,    44,45,46,47,
       48,49,50,51,    52,53,54,55,    56,57,58,59,    60,61,62,63,
    }
};

// Chroma DC (2x2, 4:2:0) coeff_token, indexed [TotalCoeff*4 + TrailingOnes],
// TotalCoeff 0..4.
static const uint8_t kChromaDcCoeffTokenLen[4 * 5] = {
    2, 0, 0, 0,
    6, 1, 0, 0,
    6, 6, 3, 0,
    6, 7, 7, 6,
    6, 8, 8, 7,
};
static const uint8_t kChromaDcCoeffTokenBits[4 * 5] = {
    1, 0, 0, 0,
    7, 1, 0, 0,
    4, 6, 1, 0,
    3, 3, 2, 5,
    2, 3, 2, 0,
};

// total_zeros for luma/chroma-AC 4x4 blocks (Table 9-7/9-8). Row index =
// TotalCoeff - 1 (rows for TotalCoeff 1..15; TotalCoeff==16 implies
// zeros_left == 0 with nothing coded, handled by the caller). Row i has
// (16 - i) valid entries for total_zeros = 0 .. (15 - i).
static const uint8_t kTotalZerosLen[15][16] = {
    {1,3,3,4,4,5,5,6,6,7,7,8,8,9,9,9},
    {3,3,3,3,3,4,4,4,4,5,5,6,6,6,6},
    {4,3,3,3,4,4,3,3,4,5,5,6,5,6},
    {5,3,4,4,3,3,3,4,3,4,5,5,5},
    {4,4,4,3,3,3,3,3,4,5,4,5},
    {6,5,3,3,3,3,3,3,4,3,6},
    {6,5,3,3,3,2,3,4,3,6},
    {6,4,5,3,2,2,3,3,6},
    {6,6,4,2,2,3,2,5},
    {5,5,3,2,2,2,4},
    {4,4,3,3,1,3},
    {4,4,2,1,3},
    {3,3,1,2},
    {2,2,1},
    {1,1},
};
static const uint8_t kTotalZerosBits[15][16] = {
    {1,3,2,3,2,3,2,3,2,3,2,3,2,3,2,1},
    {7,6,5,4,3,5,4,3,2,3,2,3,2,1,0},
    {5,7,6,5,4,3,4,3,2,3,2,1,1,0},
    {3,7,5,4,6,5,4,3,3,2,2,1,0},
    {5,4,3,7,6,5,4,3,2,1,1,0},
    {1,1,7,6,5,4,3,2,1,1,0},
    {1,1,5,4,3,3,2,1,1,0},
    {1,1,1,3,3,2,2,1,0},
    {1,0,1,3,2,1,1,1},
    {1,0,1,3,2,1,1},
    {0,1,1,2,1,3},
    {0,1,1,1,1},
    {0,1,1,1},
    {0,1,1},
    {0,1},
};

// total_zeros for chroma DC (2x2, 4:2:0) blocks (Table 9-9a). Row index =
// TotalCoeff - 1 (TotalCoeff 1..3). Row i has (4 - i) valid entries.
static const uint8_t kChromaDcTotalZerosLen[3][4] = {
    {1, 2, 3, 3},
    {1, 2, 2, 0},
    {1, 1, 0, 0},
};
static const uint8_t kChromaDcTotalZerosBits[3][4] = {
    {1, 1, 1, 0},
    {1, 1, 0, 0},
    {1, 0, 0, 0},
};

// run_before (Table 9-10). Rows 0..5 = zerosLeft 1..6 (row i has (i+2)
// valid entries for run_before 0..(i+1)). Row 6 = the "more than 6" table,
// 15 valid entries for run_before 0..14.
static const uint8_t kRunBeforeLen[7][16] = {
    {1,1},
    {1,2,2},
    {2,2,2,2},
    {2,2,2,3,3},
    {2,2,3,3,3,3},
    {2,3,3,3,3,3,3},
    {3,3,3,3,3,3,3,4,5,6,7,8,9,10,11},
};
static const uint8_t kRunBeforeBits[7][16] = {
    {1,0},
    {1,1,0},
    {3,2,1,0},
    {3,2,1,1,0},
    {3,2,3,2,1,0},
    {3,0,1,3,2,5,4},
    {7,6,5,4,3,2,1,1,1,1,1,1,1,1,1},
};

/// Selects which of the 4 coeff_token VLC tables (Table 9-5) applies for a
/// given nC context, clause 9.2.1: table 0 for nC in [0,2), 1 for [2,4),
/// 2 for [4,8); nC >= 8 uses a fixed-length code handled separately by the
/// caller rather than through this table selection. Shared by both the
/// decoder's coeff_token matcher (decoder/h264_cavlc.h) and the encoder's
/// coeff_token writer (encoder/h264_cavlc_encode.h) - both directions need
/// the exact same nC-to-table mapping.
inline int nCToCoeffTokenTable(int nC) {
  if (nC < 2) return 0;
  if (nC < 4) return 1;
  if (nC < 8) return 2;
  return 3;
}

}  // namespace tinyh264
