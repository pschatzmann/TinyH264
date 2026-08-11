/*
 * Desktop-only test: verifies Encoder::setMotionSearchRange()/
 * TinyH264Encoder::setMotionSearchRange() (see
 * h264_macroblock_encode_inter.h's motionSearch16x16() for the
 * mechanism) - the runtime-configurable +/-pixel motion search window,
 * added per user request as the "shrink the search range" option
 * documented in docs/optimizations.md's "Encoding" chapter.
 *
 * Checks:
 * 1. Default is 8 (this project's original, unconfigurable-until-now
 *    value); setMotionSearchRange()/motionSearchRange() round-trip; 0
 *    and negative values are clamped to 1 rather than accepted as-is
 *    (a 0-or-negative range would make the search loop degenerate or
 *    silently skip all candidates).
 * 2. The range actually changes search *behavior*, not just the stored
 *    value: two identical synthetic sequences with a controlled, exact
 *    12-pixel horizontal shift between frame 0 and frame 1 (so a search
 *    window that reaches 12 pixels can find a near-perfect motion
 *    match, and one that can't is mathematically unable to) are encoded
 *    once with a range that can reach the shift (16) and once with a
 *    range that can't (4). Both must still produce a valid,
 *    self-decodable P-frame (correctness is never range-dependent - a
 *    motion vector search failure just means a bigger residual, not a
 *    broken bitstream) - but the wide-range encode's P-frame must be
 *    meaningfully smaller (a better motion match compresses better),
 *    which only happens if `range` is actually reaching the low-level
 *    search loop instead of being silently ignored.
 */
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../../src/TinyH264Decoder.h"
#include "../../src/TinyH264Encoder.h"

using namespace tinyh264;

int failures = 0;

#define CHECK(cond, msg)                     \
  do {                                       \
    if (!(cond)) {                           \
      printf("FAIL: %s\n", msg);             \
      failures++;                            \
    }                                        \
  } while (0)

static const int W = 176, H = 144;

// A diagonal gradient (same style as examples/EncodeSyntheticFrame) -
// shifting it horizontally by `shiftX` is an exact pixel-for-pixel
// translation, so the true motion is unambiguous and exactly
// representable by an integer-pel motion vector.
static void makePattern(std::vector<uint8_t>& y, std::vector<uint8_t>& u,
                        std::vector<uint8_t>& v, int shiftX) {
  y.resize((size_t)W * H);
  u.resize((size_t)(W / 2) * (H / 2));
  v.resize((size_t)(W / 2) * (H / 2));
  for (int row = 0; row < H; row++) {
    for (int col = 0; col < W; col++) {
      int val = ((col + shiftX + row) * 255) / (W + H);
      // Wrap into [0,255] via modulo instead of letting shiftX push the
      // index negative/out of the gradient's natural range - keeps the
      // pattern a pure horizontal translation for any shiftX.
      val = ((val % 256) + 256) % 256;
      y[(size_t)row * W + col] = (uint8_t)val;
    }
  }
  for (auto& s : u) s = 128;
  for (auto& s : v) s = 128;
}

// Encodes frame0 (I) + frame1 (P) at the given search range, returning
// the P-frame's encoded byte count. Also verifies the P-frame decodes
// back correctly (both against this project's own decoder), so a range
// that "wins" on size can't be cheating via a broken/truncated stream.
static size_t encodePFrameAt(int range, const std::vector<uint8_t>& y0,
                             const std::vector<uint8_t>& u0,
                             const std::vector<uint8_t>& v0,
                             const std::vector<uint8_t>& y1,
                             const std::vector<uint8_t>& u1,
                             const std::vector<uint8_t>& v1) {
  TinyH264Encoder<> enc(W, H);
  enc.setQp(24);
  enc.setMotionSearchRange(range);
  std::vector<uint8_t> tmp(65536);
  std::vector<uint8_t> combined;

  size_t n0 = enc.encodeFrame(y0.data(), u0.data(), v0.data(), tmp.data(), tmp.size());
  CHECK(n0 > 0, "I-frame encode failed");
  combined.insert(combined.end(), tmp.begin(), tmp.begin() + n0);

  size_t n1 = enc.encodeFrame(y1.data(), u1.data(), v1.data(), tmp.data(), tmp.size());
  CHECK(n1 > 0, "P-frame encode failed");
  combined.insert(combined.end(), tmp.begin(), tmp.begin() + n1);

  int decodedFrames = 0;
  TinyH264Decoder<> dec;
  dec.setCallback([](TinyH264Decoder<>&, void* userData) {
    (*(int*)userData)++;
  }, &decodedFrames);
  dec.write(combined.data(), combined.size());
  CHECK(!dec.hasError(), "self-decode reported an error");
  CHECK(decodedFrames == 2, "self-decode did not produce 2 frames");

  return n1;
}

int main() {
  // --- 1. Default value, round-trip, clamping ---
  {
    TinyH264Encoder<> enc;
    CHECK(enc.motionSearchRange() == 8, "default motionSearchRange() != 8");
    enc.setMotionSearchRange(3);
    CHECK(enc.motionSearchRange() == 3, "setMotionSearchRange(3) not reflected");
    enc.setMotionSearchRange(0);
    CHECK(enc.motionSearchRange() == 1, "setMotionSearchRange(0) not clamped to 1");
    enc.setMotionSearchRange(-5);
    CHECK(enc.motionSearchRange() == 1, "setMotionSearchRange(-5) not clamped to 1");
  }

  // --- 2. The range actually changes search behavior ---
  {
    std::vector<uint8_t> y0, u0, v0, y1, u1, v1;
    makePattern(y0, u0, v0, 0);
    makePattern(y1, u1, v1, 12);  // exact 12px horizontal shift

    size_t narrowBytes = encodePFrameAt(4, y0, u0, v0, y1, u1, v1);   // can't reach 12px
    size_t wideBytes = encodePFrameAt(16, y0, u0, v0, y1, u1, v1);    // can reach 12px

    printf("range=4 P-frame: %zu bytes, range=16 P-frame: %zu bytes\n",
           narrowBytes, wideBytes);
    CHECK(wideBytes < narrowBytes,
          "wider search range did not produce a smaller P-frame - "
          "range parameter may not be reaching the search loop");
  }

  printf("test_motion_search_range: failures=%d\n", failures);
  return failures == 0 ? 0 : 1;
}
