/*
 * Desktop-only test: verifies TinyH264Encoder's rate control
 * (setTargetBitrate() + qp = -1, encoder/h264_encoder.h's
 * resolveQp()/updateRateControl()) against a real 10-frame QCIF sequence
 * (assets/all_frames_ref.yuv, the same real motion-content oracle
 * test_encode_pframe.cpp uses). Encodes the same sequence at three very
 * different target bitrates (well below, close to, and well above what
 * a fixed qp=26 encode of this content naturally produces - ~293kbps,
 * see test_encode_pframe.cpp's 14677-byte/10-frame result at 25fps) and
 * checks, at each target:
 * 1. QP actually moves in the correct direction (up/coarser for a lower
 *    target, down/finer for a higher one) - not just "some number came
 *    out", the core claim rate control makes.
 * 2. The resulting stream still decodes cleanly via this project's own
 *    TinyH264Decoder (varying per-frame QP, via slice_qp_delta, is new
 *    surface a fixed-QP test never exercises).
 * 3. Actual average bitrate lands within a generous factor of the
 *    target - this is a simple, real-time proportional controller (not
 *    a tuned two-pass encoder), and 10 frames is a short convergence
 *    window, so the bound here is deliberately loose (0.4x-2x) rather
 *    than tight; the *direction* check above is the precise assertion.
 *
 * Development-time cross-check (not re-run here): the same three
 * streams were also decoded with real `ffmpeg` (0 decode errors on all
 * three, including the varying-QP case) - see
 * encoder/h264_encoder.h's updateRateControl() comment for the
 * controller's exact behavior.
 */
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../../src/TinyH264Decoder.h"
#include "../../src/TinyH264Encoder.h"

using namespace tinyh264;

static std::vector<uint8_t> readFile(const char* path) {
  FILE* f = fopen(path, "rb");
  if (!f) {
    fprintf(stderr, "cannot open %s\n", path);
    exit(1);
  }
  fseek(f, 0, SEEK_END);
  long size = ftell(f);
  fseek(f, 0, SEEK_SET);
  std::vector<uint8_t> buf((size_t)size);
  if (fread(buf.data(), 1, (size_t)size, f) != (size_t)size) exit(1);
  fclose(f);
  return buf;
}

static const int W = 176, H = 144;
static const size_t FRAME_SIZE = (size_t)W * H * 3 / 2;
static const double kFps = 25.0;
int failures = 0;

void decodeCheckCallback(TinyH264Decoder<>& /*d*/, void* userData) {
  (*(int*)userData)++;
}

/**
 * Encodes the sequence at `targetBps` and returns (lastQp, actual average
 * bps) - also asserts the stream decodes cleanly via this project's own
 * decoder before returning (a FAIL here aborts via failures++/return).
 */
struct RcResult { int firstPQp; int lastQp; double actualBps; bool ok; };

RcResult runAt(const std::vector<uint8_t>& allFrames, int numFrames,
               int targetBps) {
  RcResult r{};
  TinyH264Encoder<> enc;
  enc.setSize(W, H);
  enc.setQp(-1);  // rate control, per setTargetBitrate() below
  enc.setTargetBitrate(targetBps, kFps);
  std::vector<uint8_t> bitstream(2000000);
  size_t total = 0;

  for (int i = 0; i < numFrames; i++) {
    const uint8_t* frame = allFrames.data() + (size_t)i * FRAME_SIZE;
    const uint8_t* srcY = frame;
    const uint8_t* srcU = frame + (size_t)W * H;
    const uint8_t* srcV = srcU + (size_t)(W / 2) * (H / 2);
    size_t n = enc.encodeFrame(srcY, srcU, srcV, bitstream.data() + total,
                                bitstream.size() - total);
    if (n == 0) {
      printf("FAIL target=%d: frame %d encode returned 0\n", targetBps, i);
      return r;
    }
    if (i == 1) r.firstPQp = enc.lastQp();  // first *adapted* QP (frame 0
                                              /*
                                               * always starts at the
                                               * controller's initial guess)
                                               */
    total += n;
  }
  r.lastQp = enc.lastQp();
  r.actualBps = (double)total * 8 * kFps / numFrames;

  TinyH264Decoder<> dec;
  int frameCount = 0;
  dec.setCallback(decodeCheckCallback, &frameCount);
  dec.write(bitstream.data(), total);
  if (dec.hasError() || frameCount != numFrames) {
    printf("FAIL target=%d: self-decode failed (hasError=%d frames=%d/%d)\n",
           targetBps, dec.hasError(), frameCount, numFrames);
    return r;
  }
  r.ok = true;
  return r;
}

int main() {
  auto allFrames = readFile("assets/all_frames_ref.yuv");
  int numFrames = (int)(allFrames.size() / FRAME_SIZE);
  if (numFrames < 2 || allFrames.size() % FRAME_SIZE != 0) {
    printf("FAIL: unexpected asset size %zu\n", allFrames.size());
    return 1;
  }

  /*
   * Baseline: what fixed qp=26 gives this content (matches
   * test_encode_pframe.cpp's own real, measured 14677 bytes/10 frames).
   */
  const int kBaselineQp = 26;

  RcResult low = runAt(allFrames, numFrames, 150000);   // well below baseline
  RcResult mid = runAt(allFrames, numFrames, 300000);   // close to baseline
  RcResult high = runAt(allFrames, numFrames, 600000);  // well above baseline

  if (!low.ok || !mid.ok || !high.ok) {
    failures++;
  } else {
    printf("target=150000: qp %d->%d, %.0f bps\n", kBaselineQp, low.lastQp,
           low.actualBps);
    printf("target=300000: qp %d->%d, %.0f bps\n", kBaselineQp, mid.lastQp,
           mid.actualBps);
    printf("target=600000: qp %d->%d, %.0f bps\n", kBaselineQp, high.lastQp,
           high.actualBps);

    /*
     * Direction: a lower target must end up at a QP >= the baseline
     * (coarser or equal), a higher target at a QP <= baseline (finer or
     * equal) - the core claim rate control makes.
     */
    if (low.lastQp < kBaselineQp) {
      printf("FAIL: low-bitrate target didn't raise QP (%d, want >= %d)\n",
             low.lastQp, kBaselineQp);
      failures++;
    }
    if (high.lastQp > kBaselineQp) {
      printf("FAIL: high-bitrate target didn't lower QP (%d, want <= %d)\n",
             high.lastQp, kBaselineQp);
      failures++;
    }
    if (low.lastQp <= high.lastQp) {
      printf("FAIL: low-target QP (%d) should end up higher than "
             "high-target QP (%d)\n",
             low.lastQp, high.lastQp);
      failures++;
    }

    // Loose bound - see file header comment on why this isn't tight.
    for (auto& r : {std::make_pair(150000, low), std::make_pair(300000, mid),
                     std::make_pair(600000, high)}) {
      double ratio = r.second.actualBps / r.first;
      if (ratio < 0.4 || ratio > 2.0) {
        printf("FAIL target=%d: actual %.0f bps is %.2fx target (want "
               "0.4x-2x)\n",
               r.first, r.second.actualBps, ratio);
        failures++;
      }
    }
  }

  if (failures == 0) {
    printf("test_encode_ratecontrol: all checks passed\n");
    return 0;
  }
  printf("test_encode_ratecontrol: %d failure(s)\n", failures);
  return 1;
}
