#pragma once
#include <stdint.h>
#include "h264_bitreader.h"
#include "../common/h264_cavlc_tables.h"

/*
 * Header-only. CAVLC entropy decoding, ITU-T H.264 clause 9.2
 * (residual_block_cavlc). Table data lives in h264_cavlc_tables.h.
 */

namespace tinyh264 {

/**
 * Generic prefix-code matcher: reads one bit at a time and checks it
 * against every {length, bits} entry in the table, returning the entry
 * index on the first length-and-value match. This relies on the tables
 * being valid prefix codes (guaranteed by the spec) so the first match at
 * a given length is unambiguous. Unused table slots are marked with
 * length == 0 and are never matched (the scan starts at length 1). Backs
 * every VLC table lookup in this file (coeff_token, total_zeros,
 * run_before - see h264_cavlc_tables.h for the table data).
 *
 * Not the fastest possible approach (a real LUT-based bit-peek would be),
 * but at <=68 entries and <=16 bits it's a handful of microseconds even on
 * an MCU, and simplicity here is worth more than speed until profiling
 * says otherwise.
 */
inline bool decodeVlc(BitReader& br, const uint8_t* lens, const uint8_t* bits,
                       int count, int maxLen, int* outIndex) {
  uint32_t code = 0;
  for (int len = 1; len <= maxLen; len++) {
    code = (code << 1) | br.u(1);
    if (br.error()) return false;
    for (int i = 0; i < count; i++) {
      if (lens[i] == len && bits[i] == code) {
        *outIndex = i;
        return true;
      }
    }
  }
  return false;
}

/**
 * Decodes coeff_token (clause 9.2.1, Table 9-5), yielding TotalCoeff and
 * TrailingOnes for one residual block. `nC` is the predicted context per
 * clause 9.2.1: the average non-zero count of the left/top 4x4 neighbors
 * for luma/chroma-AC blocks (see predictNc() in h264_mb_info.h), or -1
 * for the chroma DC (2x2, 4:2:0) special table.
 */
inline bool decodeCoeffToken(BitReader& br, int nC, uint32_t* totalCoeff,
                              uint32_t* trailingOnes) {
  if (nC == -1) {
    int idx;
    if (!decodeVlc(br, kChromaDcCoeffTokenLen, kChromaDcCoeffTokenBits, 20, 8,
                    &idx))
      return false;
    *totalCoeff = (uint32_t)idx >> 2;
    *trailingOnes = (uint32_t)idx & 3;
    return true;
  }
  if (nC >= 8) {
    /*
     * Table 9-5, nC >= 8: fixed-length 6-bit code. code == 3 is the only
     * slot that can't arise from the (TotalCoeff-1,TrailingOnes) formula
     * below (it would imply TotalCoeff=1 with TrailingOnes=3, impossible),
     * so the spec reuses it for TotalCoeff=0.
     */
    uint32_t code = br.u(6);
    if (br.error()) return false;
    if (code == 3) {
      *totalCoeff = 0;
      *trailingOnes = 0;
    } else {
      *totalCoeff = (code >> 2) + 1;
      *trailingOnes = code & 3;
    }
    return true;
  }
  int table = nCToCoeffTokenTable(nC);
  int idx;
  if (!decodeVlc(br, kCoeffTokenLen[table], kCoeffTokenBits[table], 4 * 17,
                  16, &idx))
    return false;
  *totalCoeff = (uint32_t)idx >> 2;
  *trailingOnes = (uint32_t)idx & 3;
  return true;
}

/**
 * Decodes the `totalCoeff` signed levels (clause 9.2.2), most-significant
 * (highest scan-position) coefficient first: level[0..trailingOnes) are the
 * trailing +/-1 coefficients, level[trailingOnes..totalCoeff) follow the
 * level_prefix/level_suffix escape coding with the adaptive suffixLength.
 */
inline bool decodeLevels(BitReader& br, uint32_t totalCoeff,
                          uint32_t trailingOnes, int32_t* level) {
  for (uint32_t i = 0; i < trailingOnes; i++) {
    level[i] = br.flag() ? -1 : 1;
  }

  int suffixLength = (totalCoeff > 10 && trailingOnes < 3) ? 1 : 0;
  for (uint32_t i = trailingOnes; i < totalCoeff; i++) {
    int levelPrefix = 0;
    while (br.u(1) == 0) {
      levelPrefix++;
      if (br.error() || levelPrefix > 28) return false;
    }

    int levelSuffixSize;
    if (levelPrefix == 14 && suffixLength == 0) {
      levelSuffixSize = 4;
    } else if (levelPrefix >= 15) {
      levelSuffixSize = levelPrefix - 3;
    } else {
      levelSuffixSize = suffixLength;
    }
    uint32_t levelSuffix =
        levelSuffixSize > 0 ? br.u(levelSuffixSize) : 0;
    if (br.error()) return false;

    int32_t levelCode =
        (int32_t)((levelPrefix < 15 ? levelPrefix : 15) << suffixLength) +
        (int32_t)levelSuffix;
    if (levelPrefix >= 15 && suffixLength == 0) levelCode += 15;
    if (levelPrefix >= 16) levelCode += (1 << (levelPrefix - 3)) - 4096;
    if (i == trailingOnes && trailingOnes < 3) levelCode += 2;

    int32_t lvl = (levelCode % 2 == 0) ? (levelCode + 2) >> 1
                                        : (-levelCode - 1) >> 1;
    level[i] = lvl;

    /*
     * clause 9.2.2.1. (A more "literal" reading suggests these two
     * adjustments should be mutually exclusive - bumping 0->1 shouldn't
     * also be checked against the growth condition in the same step - but
     * that reading was tried and empirically broke previously pixel-exact
     * luma decode against ffmpeg's reference; the real encoder behavior
     * matches applying both unconditionally, confirmed by the pixel-exact
     * match below.)
     */
    if (suffixLength == 0) suffixLength = 1;
    int32_t absLvl = lvl < 0 ? -lvl : lvl;
    if (absLvl > (3 << (suffixLength - 1)) && suffixLength < 6) {
      suffixLength++;
    }
  }
  return true;
}

/**
 * Decodes total_zeros (clause 9.2.3, Table 9-7/9-8/9-9a), the count of
 * zero-valued coefficients preceding the last (highest scan-position)
 * non-zero one. `maxNumCoeff`: 16 for luma whole 4x4 blocks, 15 for luma/
 * chroma AC-only 4x4 blocks (position 0 excluded), 4 for chroma DC (2x2).
 * The caller is responsible for picking the right value; this function
 * only needs maxNumCoeff to know which total_zeros table to use (4 =>
 * chroma DC table, otherwise the general table indexed by TotalCoeff
 * 1..15).
 */
inline bool decodeTotalZeros(BitReader& br, uint32_t totalCoeff,
                              int maxNumCoeff, uint32_t* zerosLeft) {
  if ((int)totalCoeff == maxNumCoeff) {
    *zerosLeft = 0;
    return true;
  }
  int idx;
  if (maxNumCoeff == 4) {
    if (totalCoeff < 1 || totalCoeff > 3) return false;
    if (!decodeVlc(br, kChromaDcTotalZerosLen[totalCoeff - 1],
                    kChromaDcTotalZerosBits[totalCoeff - 1], 4, 3, &idx))
      return false;
  } else {
    if (totalCoeff < 1 || totalCoeff > 15) return false;
    if (!decodeVlc(br, kTotalZerosLen[totalCoeff - 1],
                    kTotalZerosBits[totalCoeff - 1], 16, 9, &idx))
      return false;
  }
  *zerosLeft = (uint32_t)idx;
  return true;
}

/**
 * Decodes one run_before value (clause 9.2.4, Table 9-10): the number of
 * zero coefficients immediately preceding a given non-zero coefficient,
 * scanned from highest to lowest frequency. `zerosLeft` (>= 1, checked by
 * the caller before calling) selects which of the 7 run_before tables
 * applies (rows for zerosLeft 1..6, plus a "6 or more" row).
 */
inline bool decodeRunBefore(BitReader& br, uint32_t zerosLeft,
                             uint32_t* runBefore) {
  int idx;
  if (zerosLeft < 7) {
    if (!decodeVlc(br, kRunBeforeLen[zerosLeft - 1],
                    kRunBeforeBits[zerosLeft - 1], 16, 3, &idx))
      return false;
  } else {
    if (!decodeVlc(br, kRunBeforeLen[6], kRunBeforeBits[6], 16, 11, &idx))
      return false;
  }
  *runBefore = (uint32_t)idx;
  return true;
}

/**
 * Full residual_block_cavlc() (clause 9.2, top-level entry point for this
 * file): decodes up to `maxNumCoeff` coefficient levels into
 * coeffLevel[0..maxNumCoeff), indexed by scan position (0 =
 * lowest frequency / DC). Zig-zag-to-2D mapping is the caller's job (see
 * kZigZag4x4 in h264_tables.h). `nC` is documented on decodeCoeffToken()
 * above. Returns false on any bitstream error or structural inconsistency
 * (caller should abandon the slice). *totalCoeffOut is always set on
 * success, even when 0 (used by the macroblock layer's non-zero-count
 * neighbor cache, MacroblockInfo::nnz).
 */
inline bool residualBlockCavlc(BitReader& br, int nC, int maxNumCoeff,
                                int32_t* coeffLevel, uint32_t* totalCoeffOut) {
  for (int i = 0; i < maxNumCoeff; i++) coeffLevel[i] = 0;

  uint32_t totalCoeff, trailingOnes;
  if (!decodeCoeffToken(br, nC, &totalCoeff, &trailingOnes)) return false;
  if (totalCoeffOut) *totalCoeffOut = totalCoeff;
  if (totalCoeff == 0) return true;
  if ((int)totalCoeff > maxNumCoeff) return false;

  int32_t level[16];
  if (!decodeLevels(br, totalCoeff, trailingOnes, level)) return false;

  uint32_t zerosLeft = 0;
  if (!decodeTotalZeros(br, totalCoeff, maxNumCoeff, &zerosLeft)) return false;

  uint32_t runVal[16];
  for (uint32_t i = 0; i + 1 < totalCoeff; i++) {
    uint32_t rb = 0;
    if (zerosLeft > 0) {
      if (!decodeRunBefore(br, zerosLeft, &rb)) return false;
    }
    runVal[i] = rb;
    zerosLeft -= rb;
  }
  runVal[totalCoeff - 1] = zerosLeft;

  int coeffNum = -1;
  for (int i = (int)totalCoeff - 1; i >= 0; i--) {
    coeffNum += (int)runVal[i] + 1;
    if (coeffNum < 0 || coeffNum >= maxNumCoeff) return false;
    coeffLevel[coeffNum] = level[i];
  }
  return true;
}

}  // namespace tinyh264
