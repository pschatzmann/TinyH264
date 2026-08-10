#pragma once
#include <stdint.h>
#include "h264_bitwriter.h"
#include "../common/h264_cavlc_tables.h"

// Header-only. CAVLC entropy encoding, ITU-T H.264 clause 9.2
// (residual_block_cavlc), the write-side counterpart to
// decoder/h264_cavlc.h. Every function here is derived as the mechanical,
// step-by-step inverse of the corresponding *already-verified* decode
// function (pixel-exact against real ffmpeg decodes, this whole project's
// methodology) - see each function's comment for exactly which decode
// function it inverts. Table data (coeff_token/total_zeros/run_before) is
// shared verbatim with the decoder via h264_cavlc_tables.h: those tables
// are already stored as {length, bits} pairs indexed by symbol value (not
// as a decode-only VLC tree), which is exactly what an encoder needs to
// look a symbol's code up directly - no separate encode-side tables
// needed, and no separate transcription-error risk.

namespace tinyh264 {

/// Encodes coeff_token (clause 9.2.1, Table 9-5) for `totalCoeff`/
/// `trailingOnes` - direct table lookup, the inverse of
/// decodeCoeffToken()'s decodeVlc() match. `nC` context selection uses
/// the same nCToCoeffTokenTable() (h264_cavlc_tables.h) the decoder uses.
inline void encodeCoeffToken(BitWriter& bw, int nC, uint32_t totalCoeff,
                              uint32_t trailingOnes) {
  if (nC == -1) {
    int idx = (int)(totalCoeff * 4 + trailingOnes);
    bw.u(kChromaDcCoeffTokenBits[idx], kChromaDcCoeffTokenLen[idx]);
    return;
  }
  int table = nCToCoeffTokenTable(nC);
  int idx = (int)(totalCoeff * 4 + trailingOnes);
  bw.u(kCoeffTokenBits[table][idx], kCoeffTokenLen[table][idx]);
}

/// Encodes one level_prefix/level_suffix escape code (clause 9.2.2.1) for
/// an already-adjusted `levelCode` (i.e. after the trailing-ones-adjacent
/// +2 has already been applied by the caller, matching decodeLevels()'s
/// order of operations) - the inverse of decodeLevels()'s per-coefficient
/// prefix/suffix read. Mechanically derived by inverting each of
/// decodeLevels()'s three branches (levelPrefix < 14; levelPrefix == 14
/// with suffixLength == 0; levelPrefix >= 15 escape) in turn - see the
/// three `if` cases below, each commented with the decode-side formula it
/// inverts.
inline void encodeLevelCode(BitWriter& bw, int32_t levelCode,
                             int suffixLength) {
  int prefix, suffixBits;
  int32_t suffixVal;
  if (suffixLength == 0 && levelCode < 14) {
    // Inverts: levelCode = levelPrefix (levelSuffixSize == suffixLength == 0).
    prefix = levelCode;
    suffixBits = 0;
    suffixVal = 0;
  } else if (suffixLength == 0 && levelCode < 30) {
    // Inverts: levelPrefix == 14 && suffixLength == 0 -> levelSuffixSize
    // == 4, levelCode = 14 + levelSuffix.
    prefix = 14;
    suffixBits = 4;
    suffixVal = levelCode - 14;
  } else if (suffixLength > 0 && (levelCode >> suffixLength) < 15) {
    // Inverts: levelPrefix < 15 (general case), levelCode =
    // (levelPrefix << suffixLength) + levelSuffix.
    prefix = levelCode >> suffixLength;
    suffixBits = suffixLength;
    suffixVal = levelCode & ((1 << suffixLength) - 1);
  } else {
    // Inverts the levelPrefix >= 15 escape: levelCode = (15 <<
    // suffixLength) + levelSuffix [+15 more if suffixLength == 0], then
    // += (1 << (levelPrefix-3)) - 4096 for each further escalation to
    // levelPrefix 16, 17, ... - walk the same escalation forward here.
    int32_t rem = levelCode - (15 << suffixLength);
    if (suffixLength == 0) rem -= 15;
    prefix = 15;
    while (rem >= (1 << (prefix - 3))) {
      rem -= (1 << (prefix - 3));
      prefix++;
    }
    suffixBits = prefix - 3;
    suffixVal = rem;
  }
  bw.u(1, prefix + 1);  // `prefix` zero bits then a terminating 1
  if (suffixBits > 0) bw.u((uint32_t)suffixVal, suffixBits);
}

/// Encodes the `totalCoeff` signed levels (clause 9.2.2), most-significant
/// (highest scan-position) coefficient first - the exact inverse of
/// decodeLevels(): level[0..trailingOnes) are trailing +/-1 coefficients
/// (one sign bit each), level[trailingOnes..totalCoeff) use the
/// level_prefix/level_suffix escape coding with the same adaptive
/// suffixLength state machine decodeLevels() drives (mirrored here
/// verbatim, including the "apply both adjustments unconditionally" note
/// decodeLevels() itself carries - see that function's comment).
inline void encodeLevels(BitWriter& bw, uint32_t totalCoeff,
                          uint32_t trailingOnes, const int32_t* level) {
  for (uint32_t i = 0; i < trailingOnes; i++) {
    bw.flag(level[i] < 0);  // decode: level[i] = flag() ? -1 : 1
  }

  int suffixLength = (totalCoeff > 10 && trailingOnes < 3) ? 1 : 0;
  for (uint32_t i = trailingOnes; i < totalCoeff; i++) {
    int32_t lvl = level[i];
    // Inverts: lvl = (levelCode%2==0) ? (levelCode+2)>>1 : (-levelCode-1)>>1.
    int32_t levelCode = (lvl > 0) ? (2 * lvl - 2) : (-2 * lvl - 1);
    if (i == trailingOnes && trailingOnes < 3) levelCode -= 2;
    encodeLevelCode(bw, levelCode, suffixLength);

    if (suffixLength == 0) suffixLength = 1;
    int32_t absLvl = lvl < 0 ? -lvl : lvl;
    if (absLvl > (3 << (suffixLength - 1)) && suffixLength < 6) {
      suffixLength++;
    }
  }
}

/// Encodes total_zeros (clause 9.2.3, Table 9-7/9-8/9-9a) - direct table
/// lookup, the inverse of decodeTotalZeros()'s decodeVlc() match. Same
/// maxNumCoeff convention as the decoder (16 luma whole 4x4, 15 luma/
/// chroma AC-only, 4 chroma DC). Not called at all when totalCoeff ==
/// maxNumCoeff (implicit zero, matching decodeTotalZeros()'s early exit).
inline void encodeTotalZeros(BitWriter& bw, uint32_t totalCoeff,
                              int maxNumCoeff, uint32_t totalZeros) {
  if (maxNumCoeff == 4) {
    bw.u(kChromaDcTotalZerosBits[totalCoeff - 1][totalZeros],
         kChromaDcTotalZerosLen[totalCoeff - 1][totalZeros]);
  } else {
    bw.u(kTotalZerosBits[totalCoeff - 1][totalZeros],
         kTotalZerosLen[totalCoeff - 1][totalZeros]);
  }
}

/// Encodes one run_before value (clause 9.2.4, Table 9-10) - direct table
/// lookup, the inverse of decodeRunBefore()'s decodeVlc() match.
inline void encodeRunBefore(BitWriter& bw, uint32_t zerosLeft,
                             uint32_t runBefore) {
  if (zerosLeft < 7) {
    bw.u(kRunBeforeBits[zerosLeft - 1][runBefore],
         kRunBeforeLen[zerosLeft - 1][runBefore]);
  } else {
    bw.u(kRunBeforeBits[6][runBefore], kRunBeforeLen[6][runBefore]);
  }
}

/// Full residual_block_cavlc() encode (clause 9.2, counterpart to
/// residualBlockCavlc() in decoder/h264_cavlc.h): encodes
/// coeffLevel[0..maxNumCoeff), indexed by scan position (0 = lowest
/// frequency/DC), zig-zag-to-scan-order mapping already applied by the
/// caller (see kZigZag4x4, h264_tables.h). `nC` is documented on
/// encodeCoeffToken() above.
///
/// Walks scan positions from the highest nonzero down to 0 - the same
/// order decodeLevels()/decodeRunBefore() consume, so `level[]`/
/// `runBefore[]` come out pre-arranged exactly as residualBlockCavlc()'s
/// decode-side reconstruction loop expects them, without needing to
/// separately re-derive the coeffNum/runVal index algebra that loop uses.
inline void encodeResidualBlockCavlc(BitWriter& bw, int nC, int maxNumCoeff,
                                      const int32_t* coeffLevel) {
  int hi = -1;
  for (int i = maxNumCoeff - 1; i >= 0; i--) {
    if (coeffLevel[i] != 0) {
      hi = i;
      break;
    }
  }
  if (hi < 0) {
    encodeCoeffToken(bw, nC, 0, 0);
    return;
  }

  int32_t level[16];
  uint32_t runBefore[16];
  uint32_t totalCoeff = 0;
  int zerosRun = 0;
  for (int i = hi; i >= 0; i--) {
    if (coeffLevel[i] != 0) {
      if (totalCoeff > 0) runBefore[totalCoeff - 1] = (uint32_t)zerosRun;
      level[totalCoeff] = coeffLevel[i];
      totalCoeff++;
      zerosRun = 0;
    } else {
      zerosRun++;
    }
  }
  runBefore[totalCoeff - 1] = (uint32_t)zerosRun;  // zerosLeft (remainder)

  uint32_t trailingOnes = 0;
  for (uint32_t i = 0; i < totalCoeff && i < 3; i++) {
    if (level[i] == 1 || level[i] == -1) {
      trailingOnes++;
    } else {
      break;
    }
  }

  encodeCoeffToken(bw, nC, totalCoeff, trailingOnes);
  encodeLevels(bw, totalCoeff, trailingOnes, level);

  if ((int)totalCoeff < maxNumCoeff) {
    uint32_t totalZeros = 0;
    for (uint32_t i = 0; i < totalCoeff; i++) totalZeros += runBefore[i];
    encodeTotalZeros(bw, totalCoeff, maxNumCoeff, totalZeros);
  }

  uint32_t zerosLeft = 0;
  for (uint32_t i = 0; i < totalCoeff; i++) zerosLeft += runBefore[i];
  for (uint32_t i = 0; i + 1 < totalCoeff; i++) {
    if (zerosLeft > 0) {
      encodeRunBefore(bw, zerosLeft, runBefore[i]);
      zerosLeft -= runBefore[i];
    }
  }
}

}  // namespace tinyh264
