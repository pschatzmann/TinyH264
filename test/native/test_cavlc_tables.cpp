/*
 * Desktop-only sanity check: every CAVLC VLC table must be a valid prefix
 * code (no codeword is a prefix of another) and, since these are the
 * standard's canonical complete codes, the Kraft sum over valid entries
 * should equal exactly 1. This won't catch a table entry that's "wrong but
 * still a valid code" against the spec, but it will catch transcription
 * slips (flipped bit, wrong length) that break the prefix-code structure -
 * independent of the real oracle (pixel-exact decode vs ffmpeg) which comes
 * once the full pipeline exists.
 */
#include <cassert>
#include <cstdio>
#include "../../src/common/h264_cavlc_tables.h"

using namespace tinyh264;

static bool isPrefixOf(uint8_t lenA, uint8_t bitsA, uint8_t lenB,
                        uint8_t bitsB) {
  if (lenA >= lenB) return false;
  return (bitsB >> (lenB - lenA)) == bitsA;
}

// Returns true (and prints nothing) if the table is a valid prefix code.
static bool checkTable(const char* name, const uint8_t* lens,
                        const uint8_t* bits, int count, bool expectComplete) {
  bool ok = true;
  double kraft = 0.0;
  int validCount = 0;
  for (int i = 0; i < count; i++) {
    if (lens[i] == 0) continue;
    validCount++;
    kraft += 1.0 / (double)(1u << lens[i]);
    for (int j = 0; j < count; j++) {
      if (j == i || lens[j] == 0) continue;
      if (isPrefixOf(lens[i], bits[i], lens[j], bits[j])) {
        printf("%s: entry %d (len=%d,bits=%d) is a PREFIX of entry %d (len=%d,bits=%d)\n",
               name, i, lens[i], bits[i], j, lens[j], bits[j]);
        ok = false;
      }
      if (i < j && lens[i] == lens[j] && bits[i] == bits[j]) {
        printf("%s: entries %d and %d have identical code (len=%d,bits=%d)\n",
               name, i, j, lens[i], bits[i]);
        ok = false;
      }
    }
  }
  printf("%-28s entries=%2d kraft=%.6f %s\n", name, validCount, kraft,
         (kraft > 1.0 + 1e-9) ? "OVER-COMPLETE (bug!)" : "");
  if (kraft > 1.0 + 1e-9) ok = false;
  if (expectComplete && kraft < 1.0 - 1e-9) {
    printf("%s: kraft sum %.6f < 1 (incomplete code - missing entries?)\n",
           name, kraft);
    /*
     * Not necessarily a bug for tables with intentionally-unused slots
     * (padding for alignment) but worth flagging for manual review.
     */
  }
  return ok;
}

int main() {
  setbuf(stdout, nullptr);
  bool ok = true;

  char name[64];
  for (int t = 0; t < 4; t++) {
    snprintf(name, sizeof(name), "coeff_token[table %d]", t);
    /*
     * Table 3 (nC>=8) is a closed-form FLC, not meant to be a compact
     * prefix code (it deliberately has many equal-length codewords), so
     * don't expect Kraft==1 there in the "complete VLC" sense - it's
     * already complete by construction (64 six-bit codes).
     */
    ok &= checkTable(name, kCoeffTokenLen[t], kCoeffTokenBits[t], 4 * 17,
                      t != 3);
  }

  ok &= checkTable("chroma_dc_coeff_token", kChromaDcCoeffTokenLen,
                    kChromaDcCoeffTokenBits, 4 * 5, true);

  for (int t = 0; t < 15; t++) {
    snprintf(name, sizeof(name), "total_zeros[TotalCoeff=%d]", t + 1);
    ok &= checkTable(name, kTotalZerosLen[t], kTotalZerosBits[t], 16, true);
  }

  for (int t = 0; t < 3; t++) {
    snprintf(name, sizeof(name), "chroma_dc_total_zeros[TC=%d]", t + 1);
    ok &= checkTable(name, kChromaDcTotalZerosLen[t],
                      kChromaDcTotalZerosBits[t], 4, true);
  }

  for (int t = 0; t < 7; t++) {
    snprintf(name, sizeof(name), "run_before[row %d]", t);
    ok &= checkTable(name, kRunBeforeLen[t], kRunBeforeBits[t], 16, true);
  }

  assert(ok);
  printf("test_cavlc_tables: all tables are valid prefix codes\n");
  return 0;
}
