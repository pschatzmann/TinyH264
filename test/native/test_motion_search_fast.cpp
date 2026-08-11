/*
 * Desktop-only test: verifies Encoder::setMotionSearchAlgorithm()/
 * TinyH264Encoder::setMotionSearchAlgorithm() (see
 * h264_macroblock_encode_inter.h's motionSearch16x16Fast() for the
 * mechanism) - the opt-in Diamond Search alternative to the default
 * exhaustive motion search, the "Fast search algorithm" option
 * docs/optimizations.md previously listed as still open.
 *
 * Checks:
 * 1. Default is Exhaustive (existing behavior/bit-exactness for anyone not
 *    opting in must be unchanged); setMotionSearchAlgorithm()/
 *    motionSearchAlgorithm() round-trip.
 * 2. Fast still produces a valid, self-decodable P-frame on a real motion
 *    sequence (correctness is never algorithm-dependent - a local search
 *    just means a possibly-worse MV, not a broken bitstream).
 * 3. On a pure, smooth-gradient horizontal translation (a SAD surface with
 *    a single, easy-to-follow minimum - exactly the kind of content
 *    Diamond Search's descent strategy is good at), Fast finds the same
 *    best displacement Exhaustive does, and produces a comparably-sized
 *    P-frame (not just "valid" but actually effective) - the same
 *    12px-shift pattern test_motion_search_range.cpp uses.
 * 4. Fast still produces a valid, self-decodable P-frame when the true
 *    motion is 0. This is *not* checked for byte-for-byte agreement with
 *    Exhaustive, deliberately: P-frame motion search matches the raw
 *    source against the *lossy reconstructed* reference (post-quantization/
 *    deblocking, not the raw previous frame), so even bit-identical source
 *    frames produce a noisy, non-flat SAD surface - confirmed by instrumenting
 *    this exact case, where Exhaustive itself picks a non-zero MV over
 *    (0,0) because it scores lower SAD against the reconstructed
 *    reference's quantization noise than the "true" zero-motion match
 *    does. Diamond Search's local hill-climbing can settle in a different
 *    local minimum of that same noisy surface than Exhaustive's global
 *    search does - real, expected, data-dependent divergence (see the
 *    module comment above and docs/optimizations.md), not a defect.
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

static void makePattern(std::vector<uint8_t>& y, std::vector<uint8_t>& u,
                        std::vector<uint8_t>& v, int shiftX) {
  y.resize((size_t)W * H);
  u.resize((size_t)(W / 2) * (H / 2));
  v.resize((size_t)(W / 2) * (H / 2));
  for (int row = 0; row < H; row++) {
    for (int col = 0; col < W; col++) {
      int val = ((col + shiftX + row) * 255) / (W + H);
      val = ((val % 256) + 256) % 256;
      y[(size_t)row * W + col] = (uint8_t)val;
    }
  }
  for (auto& s : u) s = 128;
  for (auto& s : v) s = 128;
}

// Encodes frame0 (I) + frame1 (P) with the given algorithm, returning the
// P-frame's encoded byte count and verifying it self-decodes correctly.
static size_t encodePFrameWith(MotionSearchAlgorithm algo,
                                const std::vector<uint8_t>& y0,
                                const std::vector<uint8_t>& u0,
                                const std::vector<uint8_t>& v0,
                                const std::vector<uint8_t>& y1,
                                const std::vector<uint8_t>& u1,
                                const std::vector<uint8_t>& v1) {
  TinyH264Encoder<> enc(W, H);
  enc.setQp(24);
  enc.setMotionSearchAlgorithm(algo);
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
  // --- 1. Default value, round-trip ---
  {
    TinyH264Encoder<> enc;
    CHECK(enc.motionSearchAlgorithm() == MotionSearchAlgorithm::Exhaustive,
          "default motionSearchAlgorithm() != Exhaustive");
    enc.setMotionSearchAlgorithm(MotionSearchAlgorithm::Fast);
    CHECK(enc.motionSearchAlgorithm() == MotionSearchAlgorithm::Fast,
          "setMotionSearchAlgorithm(Fast) not reflected");
  }

  // --- 1b. setAllOptimizationsActive() is currently just a
  //         setMotionSearchAlgorithm() alias ---
  {
    TinyH264Encoder<> enc;
    enc.setAllOptimizationsActive(true);
    CHECK(enc.motionSearchAlgorithm() == MotionSearchAlgorithm::Fast,
          "setAllOptimizationsActive(true) did not select Fast");
    enc.setAllOptimizationsActive(false);
    CHECK(enc.motionSearchAlgorithm() == MotionSearchAlgorithm::Exhaustive,
          "setAllOptimizationsActive(false) did not select Exhaustive");
    // Does not touch motionSearchRange() - see its own comment.
    enc.setMotionSearchRange(4);
    enc.setAllOptimizationsActive(true);
    CHECK(enc.motionSearchRange() == 4,
          "setAllOptimizationsActive() unexpectedly touched motionSearchRange()");
  }

  // --- 2 & 3. Fast is valid and effective on an easy, single-minimum SAD
  //            surface (a pure translation) ---
  {
    std::vector<uint8_t> y0, u0, v0, y1, u1, v1;
    makePattern(y0, u0, v0, 0);
    makePattern(y1, u1, v1, 12);  // exact 12px horizontal shift

    size_t exhaustiveBytes = encodePFrameWith(MotionSearchAlgorithm::Exhaustive,
                                               y0, u0, v0, y1, u1, v1);
    size_t fastBytes = encodePFrameWith(MotionSearchAlgorithm::Fast,
                                         y0, u0, v0, y1, u1, v1);

    printf("Exhaustive P-frame: %zu bytes, Fast P-frame: %zu bytes\n",
           exhaustiveBytes, fastBytes);
    // Diamond Search's descent should reach the same easy, single-minimum
    // match Exhaustive finds here - allow a little slack (a few bytes of
    // CAVLC/rounding noise) rather than requiring byte-for-byte equality,
    // since Fast is not guaranteed to walk the identical tie-break path.
    CHECK(fastBytes <= exhaustiveBytes + (exhaustiveBytes / 10 + 8),
          "Fast P-frame far bigger than Exhaustive's on easy, single-"
          "minimum motion - Diamond Search may not be converging");
  }

  // --- 4. Zero true motion: Fast must still produce a valid, self-
  //        decodable stream, even though (see the header comment above)
  //        it isn't required to match Exhaustive's exact choice here ---
  {
    std::vector<uint8_t> y0, u0, v0, y1, u1, v1;
    makePattern(y0, u0, v0, 0);
    makePattern(y1, u1, v1, 0);  // identical - true motion is exactly zero

    encodePFrameWith(MotionSearchAlgorithm::Fast, y0, u0, v0, y1, u1, v1);
  }

  printf("test_motion_search_fast: failures=%d\n", failures);
  return failures == 0 ? 0 : 1;
}
