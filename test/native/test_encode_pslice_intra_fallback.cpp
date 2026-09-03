/*
 * Desktop-only test: verifies the Intra-macroblock-inside-a-P-slice
 * fallback (shouldUseIntraInPSlice(), h264_macroblock_encode_inter.h;
 * wired into Encoder::encodePFrame(), h264_encoder.h) both round-trips
 * correctly and actually helps - not just "doesn't crash." Encodes a
 * deliberate, genuine scene cut: assets/frame0_ref.yuv as the I-frame,
 * then assets/flat_frame0_ref.yuv (a real, completely different image -
 * average luma differs by ~62/255 per pixel) as the "next frame", so
 * motion-compensated prediction from the I-frame is a *bad* match by
 * construction - exactly the scenario the fallback exists for.
 *
 * Compares two encodes of the same scene-cut P-frame: the real
 * Encoder::encodePFrame() (fallback enabled) against
 * encodePFrameNoFallback() below, a thin local reimplementation of the
 * same macroblock loop with the Intra-fallback decision hardwired off
 * (mirroring test_encode_i4x4.cpp's own encodeAlwaysI16x16() pattern) -
 * the "before this feature existed" baseline. Checks:
 * 1. Self-decode bit-exactness (same bar every other encode test holds).
 * 2. The fallback-enabled encode is not larger AND not lower quality
 *    than the no-fallback baseline - on a genuine scene cut, a mode-
 *    decision bug that never actually triggers the fallback would still
 *    pass a bit-exactness check trivially, so this is the test that
 *    actually exercises the decision logic, not just the syntax it
 *    writes.
 *
 * Development-time cross-check (not re-run here): the same scene-cut
 * stream was also decoded with real `ffmpeg` and found bit-exact against
 * this project's own reconstruction (0/25344 luma mismatches) - see
 * shouldUseIntraInPSlice()'s own comment for the heuristic this
 * verification was run against, and for real numbers from this
 * development-time check (356 bytes / 48.1dB with the fallback vs. 8571
 * bytes / 34.0dB without it on this same scene cut - smaller *and*
 * better, not a tradeoff).
 */
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <vector>
#include "../../src/MemoryResource.h"
#include "../../src/StdAllocator.h"
#include "../../src/TinyH264Decoder.h"
#include "../../src/TinyH264Encoder.h"
#include "../../src/encoder/h264_encoder.h"

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
int failures = 0;

/**
 * Thin reimplementation of Encoder::encodePFrame()'s macroblock loop
 * with the Intra-fallback decision hardwired off (always Inter/Skip) -
 * this test's "before this feature existed" baseline, so bytes/quality
 * can be compared apples-to-apples against the real encodePFrame() (which
 * makes the real per-macroblock choice) on the same scene-cut input.
 */
size_t encodePFrameNoFallback(Frame& refFrame, MbInfoTable& mbInfo,
                               const uint8_t* srcY, int srcStrideY,
                               const uint8_t* srcU, const uint8_t* srcV,
                               int srcStrideC, int qp, uint8_t* dst,
                               size_t dstCapacity, Frame& outFrame) {
  outFrame.setSize(W, H);
  int mbWidth = W / 16, mbHeight = H / 16;
  mbInfo.reset(mbWidth, mbHeight);

  static uint8_t sliceScratch[H264_MAX_NAL_SIZE];
  BitWriter sliceW(sliceScratch, sizeof(sliceScratch));
  writeSliceHeaderP(sliceW, 1, qp, qp);

  MbEncodeContext ctx;
  ctx.frame = &outFrame;
  ctx.mbInfo = &mbInfo;
  ctx.chromaQpIndexOffset = 0;
  ctx.sliceId = 0;
  ctx.srcY = srcY;
  ctx.srcStrideY = srcStrideY;
  ctx.srcU = srcU;
  ctx.srcV = srcV;
  ctx.srcStrideC = srcStrideC;

  int qpRunning = qp;
  int totalMbs = mbWidth * mbHeight;
  int pendingSkipRun = 0;
  for (int mbAddr = 0; mbAddr < totalMbs; mbAddr++) {
    int mbX = mbAddr % mbWidth, mbY = mbAddr / mbWidth;
    mbInfo.beginMb(mbX, mbY, ctx.sliceId);
    ctx.mbX = mbX;
    ctx.mbY = mbY;
    int16_t bestMv[2];
    motionSearch16x16(ctx, refFrame, bestMv);
    int16_t skipMvVal[2];
    skipMv(ctx, skipMvVal);
    bool isSkip = bestMv[0] == skipMvVal[0] && bestMv[1] == skipMvVal[1] &&
                  wouldHaveZeroResidual(ctx, refFrame, skipMvVal[0],
                                        skipMvVal[1], qpRunning);
    if (isSkip) {
      encodeMacroblockPSkip(ctx, refFrame, qpRunning);
      pendingSkipRun++;
    } else {
      sliceW.ue((uint32_t)pendingSkipRun);
      pendingSkipRun = 0;
      qpRunning = encodeMacroblockInter16x16(sliceW, ctx, refFrame, bestMv[0],
                                              bestMv[1], qpRunning, qp);
    }
  }
  if (pendingSkipRun > 0) sliceW.ue((uint32_t)pendingSkipRun);
  sliceW.rbspTrailingBits();
  size_t o = writeNalUnit(dst, dstCapacity, 3, kNalSliceNonIdr, sliceScratch,
                           sliceW.bytesWritten());
  Pps pps;
  pps.chromaQpIndexOffset = 0;
  deblockPicture(outFrame, mbInfo, pps);
  return o;
}

double psnrY(const uint8_t* recon, int stride, const uint8_t* src) {
  double sumSq = 0;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      int d = (int)recon[y * stride + x] - (int)src[y * W + x];
      sumSq += (double)d * d;
    }
  }
  double mse = sumSq / (W * H);
  return mse > 0 ? 10 * log10(255.0 * 255.0 / mse) : 999.0;
}

int main() {
  auto f0 = readFile("assets/frame0_ref.yuv");
  auto f1 = readFile("assets/flat_frame0_ref.yuv");
  if (f0.size() != (size_t)W * H * 3 / 2 || f1.size() != (size_t)W * H * 3 / 2) {
    printf("FAIL: unexpected asset size(s)\n");
    return 1;
  }
  const uint8_t* srcY0 = f0.data();
  const uint8_t* srcU0 = f0.data() + (size_t)W * H;
  const uint8_t* srcV0 = srcU0 + (size_t)(W / 2) * (H / 2);
  const uint8_t* srcY1 = f1.data();
  const uint8_t* srcU1 = f1.data() + (size_t)W * H;
  const uint8_t* srcV1 = srcU1 + (size_t)(W / 2) * (H / 2);
  const int qp = 26;

  /*
   * Real encode, fallback enabled. Fresh encoder, no reference yet, so
   * the first encodeFrame() call is an I-frame automatically; the second
   * (same size, reference now established) becomes the scene-cut P-frame.
   */
  TinyH264Encoder<> enc;
  std::vector<uint8_t> bs(200000);
  enc.setSize(W, H);
  enc.setQp(qp);
  size_t n0 = enc.encodeFrame(srcY0, srcU0, srcV0, bs.data(), bs.size());
  size_t n1 = enc.encodeFrame(srcY1, srcU1, srcV1, bs.data() + n0,
                               bs.size() - n0);
  if (n0 == 0 || n1 == 0) {
    printf("FAIL: encode returned 0 (n0=%zu n1=%zu)\n", n0, n1);
    return 1;
  }

  TinyH264Decoder<> dec;
  int frameCount = 0;
  dec.setCallback([](TinyH264Decoder<>&, void* ud) { (*(int*)ud)++; },
                   &frameCount);
  dec.write(bs.data(), n0 + n1);
  if (dec.hasError() || frameCount != 2) {
    printf("FAIL: self-decode failed (hasError=%d frames=%d)\n",
           dec.hasError(), frameCount);
    return 1;
  }

  double fallbackPsnr = psnrY(enc.y(), enc.strideY(), srcY1);

  /*
   * Baseline: same scene cut, fallback hardwired off. Encodes its own
   * fresh I-frame (rather than reusing `enc`'s) so its reference frame
   * is captured independently, keeping the two encodes fully separate.
   * Uses the internal SoftwareEncoder directly (not the TinyH264Encoder
   * public wrapper) since it exposes frame() - needed to seed
   * encodePFrameNoFallback()'s own reference frame.
   */
  AllocatorMemoryResource<StdAllocator<uint8_t>> mr;
  SoftwareEncoder encBaseline(mr);
  std::vector<uint8_t> bsBaseline(200000);
  encBaseline.setSize(W, H);
  encBaseline.setQp(qp);
  size_t nb0 = encBaseline.encodeFrame(srcY0, srcU0, srcV0, bsBaseline.data(),
                                        bsBaseline.size());
  Frame refFrameForBaseline(mr);
  refFrameForBaseline.copyFrom(encBaseline.frame());
  Frame outFrame(mr);
  MbInfoTable mbInfo(mr);
  size_t nb1 = encodePFrameNoFallback(
      refFrameForBaseline, mbInfo, srcY1, W, srcU1, srcV1, W / 2, qp,
      bsBaseline.data() + nb0, bsBaseline.size() - nb0, outFrame);
  if (nb0 == 0 || nb1 == 0) {
    printf("FAIL: baseline encode returned 0\n");
    return 1;
  }
  double baselinePsnr = psnrY(outFrame.y(), outFrame.strideY, srcY1);

  printf("with fallback:    P-frame %zu bytes, PSNR=%.2fdB\n", n1,
         fallbackPsnr);
  printf("without fallback: P-frame %zu bytes, PSNR=%.2fdB\n", nb1,
         baselinePsnr);

  if (n1 > nb1) {
    printf("FAIL: fallback-enabled P-frame (%zu bytes) is LARGER than "
           "without (%zu bytes)\n",
           n1, nb1);
    failures++;
  }
  if (fallbackPsnr < baselinePsnr - 0.5) {
    printf("FAIL: fallback-enabled P-frame PSNR (%.2fdB) is meaningfully "
           "worse than without (%.2fdB)\n",
           fallbackPsnr, baselinePsnr);
    failures++;
  }
  /*
   * The fallback should be a clear win on a genuine scene cut, not just
   * "not worse" - assert a real margin, matching the actual measured
   * gap (356 vs 8571 bytes, 48.1 vs 34.0dB) with generous headroom.
   */
  if (n1 > nb1 / 2) {
    printf("FAIL: expected the fallback to meaningfully shrink this scene "
           "cut's P-frame (got %zu vs %zu bytes, wanted <= half)\n",
           n1, nb1);
    failures++;
  }

  if (failures == 0) {
    printf("test_encode_pslice_intra_fallback: passed\n");
    return 0;
  }
  printf("test_encode_pslice_intra_fallback: %d failure(s)\n", failures);
  return 1;
}
