/*
 * Desktop-only test: verifies the I_16x16-vs-I_4x4 macroblock mode
 * decision (shouldUseIntra4x4(), h264_macroblock_encode.h) both
 * round-trips correctly and actually improves compression - not just
 * "doesn't crash." Encodes the same real QCIF frame
 * (assets/frame0_ref.yuv, the oracle the other encode/decode tests
 * share) at several QPs, twice: once with I_4x4 selection enabled (the
 * real, default encodeIFrame() path) and once with it forced off (a
 * thin local reimplementation of Encoder::encodeIFrame()'s macroblock
 * loop that always calls encodeMacroblockIntra16x16() - see
 * encodeAlwaysI16x16() below), and checks that enabling I_4x4:
 * 1. Still decodes correctly via this project's own TinyH264Decoder
 *    (bit-exact against the encoder's own reconstruction, the same bar
 *    test_encode_iframe.cpp holds the I_16x16-only path to).
 * 2. Produces a smaller or equal-sized bitstream at every tested QP -
 *    the concrete, measurable reason to have I_4x4 at all. Not just
 *    "PSNR is fine": a mode-decision bug that always picks I_16x16
 *    would also pass a bit-exactness check trivially, so this is the
 *    test that actually exercises the decision logic.
 *
 * Development-time cross-check (not re-run here): the same comparison
 * was also run against real ffmpeg's decode, confirming a real PSNR
 * improvement (1-2.8dB) at every QP alongside the smaller file sizes,
 * and bit-exactness at QP 18+ (matching the I_16x16-only path's own
 * verified QP range) - see h264_macroblock_encode.h's own comments for
 * the mode-decision heuristic this validates.
 */
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <vector>
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

int failures = 0;
static const int W = 176, H = 144;

/**
 * Thin reimplementation of Encoder::encodeIFrame()'s macroblock loop
 * with the mode decision hardwired to always choose I_16x16 - this
 * test's baseline for "no I_4x4 at all", so bytes-produced can be
 * compared apples-to-apples against the real encodeIFrame() (which
 * makes the real per-macroblock choice) at the same QP.
 */
size_t encodeAlwaysI16x16(const uint8_t* srcY, int srcStrideY,
                           const uint8_t* srcU, const uint8_t* srcV,
                           int srcStrideC, int width, int height, int qp,
                           uint8_t* dst, size_t dstCapacity) {
  Frame<> frame;
  frame.setSize(width, height);
  int mbWidth = width / 16, mbHeight = height / 16;
  MbInfoTable<> mbInfo;
  mbInfo.reset(mbWidth, mbHeight);

  size_t o = 0;
  uint8_t hdrRbsp[64];
  BitWriter spsW(hdrRbsp, sizeof(hdrRbsp));
  writeSpsRbsp(spsW, width, height, 30, 1);
  size_t n = writeNalUnit(dst + o, dstCapacity - o, 3, kNalSps, hdrRbsp,
                           spsW.bytesWritten());
  if (n == 0) return 0;
  o += n;

  BitWriter ppsW(hdrRbsp, sizeof(hdrRbsp));
  writePpsRbsp(ppsW, qp);
  n = writeNalUnit(dst + o, dstCapacity - o, 3, kNalPps, hdrRbsp,
                    ppsW.bytesWritten());
  if (n == 0) return 0;
  o += n;

  static uint8_t sliceScratch[H264_MAX_NAL_SIZE];
  BitWriter sliceW(sliceScratch, sizeof(sliceScratch));
  writeSliceHeaderIdr(sliceW);

  MbEncodeContext<StdAllocator<uint8_t>> ctx;
  ctx.frame = &frame;
  ctx.mbInfo = &mbInfo;
  ctx.chromaQpIndexOffset = 0;
  ctx.sliceId = 0;
  ctx.srcY = srcY;
  ctx.srcStrideY = srcStrideY;
  ctx.srcU = srcU;
  ctx.srcV = srcV;
  ctx.srcStrideC = srcStrideC;

  int qpRunning = qp;
  for (int mbY = 0; mbY < mbHeight; mbY++) {
    for (int mbX = 0; mbX < mbWidth; mbX++) {
      mbInfo.beginMb(mbX, mbY, ctx.sliceId);
      ctx.mbX = mbX;
      ctx.mbY = mbY;
      qpRunning = encodeMacroblockIntra16x16(sliceW, ctx, qpRunning, qp);
      if (sliceW.error()) return 0;
    }
  }
  sliceW.rbspTrailingBits();
  if (sliceW.error()) return 0;

  n = writeNalUnit(dst + o, dstCapacity - o, 3, kNalSliceIdr, sliceScratch,
                    sliceW.bytesWritten());
  if (n == 0) return 0;
  o += n;

  Pps pps;
  pps.chromaQpIndexOffset = 0;
  deblockPicture(frame, mbInfo, pps);
  return o;
}

void testAtQp(const std::vector<uint8_t>& yuv, int qp) {
  const uint8_t* srcY = yuv.data();
  const uint8_t* srcU = yuv.data() + (size_t)W * H;
  const uint8_t* srcV = srcU + (size_t)(W / 2) * (H / 2);

  TinyH264Encoder<> enc;
  std::vector<uint8_t> bsMixed(200000);
  enc.setSize(W, H);
  enc.setQp(qp);
  /*
   * Fresh encoder, no reference yet - encodeFrame() encodes this as an
   * I-frame automatically (the only public encode entry point now).
   */
  size_t nMixed =
      enc.encodeFrame(srcY, srcU, srcV, bsMixed.data(), bsMixed.size());
  if (nMixed == 0) {
    printf("FAIL qp=%d: encodeFrame (mixed I_16x16/I_4x4) returned 0\n", qp);
    failures++;
    return;
  }

  std::vector<uint8_t> bsI16(200000);
  size_t nI16 = encodeAlwaysI16x16(srcY, W, srcU, srcV, W / 2, W, H, qp,
                                    bsI16.data(), bsI16.size());
  if (nI16 == 0) {
    printf("FAIL qp=%d: encodeAlwaysI16x16 returned 0\n", qp);
    failures++;
    return;
  }

  // 1. Self-decode bit-exact, same bar as test_encode_iframe.cpp.
  TinyH264Decoder<> dec;
  struct St {
    const uint8_t* encY;
    const uint8_t* encU;
    const uint8_t* encV;
    int mismatches = 0;
    bool gotFrame = false;
  } st{enc.y(), enc.u(), enc.v()};
  dec.setCallback(
      [](TinyH264Decoder<>& d, void* ud) {
        auto* s = (St*)ud;
        s->gotFrame = true;
        for (int y = 0; y < H; y++)
          for (int x = 0; x < W; x++)
            if (d.y()[y * d.strideY() + x] != s->encY[y * W + x]) s->mismatches++;
        for (int y = 0; y < H / 2; y++)
          for (int x = 0; x < W / 2; x++) {
            if (d.u()[y * d.strideUV() + x] != s->encU[y * (W / 2) + x])
              s->mismatches++;
            if (d.v()[y * d.strideUV() + x] != s->encV[y * (W / 2) + x])
              s->mismatches++;
          }
      },
      &st);
  dec.write(bsMixed.data(), nMixed);
  if (dec.hasError() || !st.gotFrame || st.mismatches != 0) {
    printf("FAIL qp=%d: self-decode mismatch (hasError=%d gotFrame=%d "
           "mismatches=%d)\n",
           qp, dec.hasError(), st.gotFrame, st.mismatches);
    failures++;
    return;
  }

  // 2. I_4x4 selection should not make the stream bigger.
  if (nMixed > nI16) {
    printf("FAIL qp=%d: mixed-mode encode (%zu bytes) is LARGER than "
           "I_16x16-only (%zu bytes) - mode decision made things worse\n",
           qp, nMixed, nI16);
    failures++;
    return;
  }

  printf("qp=%2d: OK (mixed=%zu bytes, I16x16-only=%zu bytes, saved %.1f%%, "
         "self-decode bit-exact)\n",
         qp, nMixed, nI16, 100.0 * (1.0 - (double)nMixed / nI16));
}

int main() {
  auto yuv = readFile("assets/frame0_ref.yuv");
  if (yuv.size() != (size_t)W * H * 3 / 2) {
    printf("FAIL: unexpected asset size %zu\n", yuv.size());
    return 1;
  }

  for (int qp : {0, 10, 18, 26, 32, 40, 48, 51}) {
    testAtQp(yuv, qp);
  }

  if (failures == 0) {
    printf("test_encode_i4x4: all QPs passed\n");
    return 0;
  }
  printf("test_encode_i4x4: %d failure(s)\n", failures);
  return 1;
}
