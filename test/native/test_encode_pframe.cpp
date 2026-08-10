// Desktop-only test: encodes a real 10-frame QCIF sequence
// (assets/all_frames_ref.yuv - the same real motion-content oracle
// test_decode_multiframe.cpp uses on the decode side, so this exercises
// genuine inter-frame motion, not a synthetic/static pattern) as 1
// I-frame + 9 P-frames via TinyH264Encoder::encodeIFrame()/
// encodePFrame(), then decodes the whole sequence back with this
// project's own TinyH264Decoder and checks:
// 1. All 10 frames decode without error.
// 2. Real quality (PSNR against the original source) clears a floor -
//    catches a regression that's syntactically valid but has stopped
//    meaningfully predicting from the reference (e.g. a broken motion
//    search always landing on (0,0), or MV prediction/encoding drifting
//    off over several P-frames in a way a single-I-frame test could
//    never catch).
//
// Development-time cross-check (not re-run here, needs ffmpeg + shelling
// out): the identical encode was decoded with real `ffmpeg` and found
// bit-exact, all 10 frames, against this project's own reconstruction
// (0/25344 luma mismatches per frame) - see
// h264_macroblock_encode_inter.h's own comments for the P-frame scope
// (P_16x16/P_Skip only, single reference, integer-pel-only motion
// search) this verification was run against.
#include <cstdio>
#include <cstdlib>
#include <cmath>
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

struct DecodeState {
  const uint8_t* allFrames;
  int frameCount = 0;
  double sumSq = 0;
};

void psnrCallback(TinyH264Decoder<>& d, void* userData) {
  auto* st = (DecodeState*)userData;
  const uint8_t* src = st->allFrames + (size_t)st->frameCount * FRAME_SIZE;
  for (int y = 0; y < H; y++) {
    const uint8_t* row = d.y() + (size_t)y * d.strideY();
    for (int x = 0; x < W; x++) {
      int diff = (int)row[x] - (int)src[y * W + x];
      st->sumSq += (double)diff * diff;
    }
  }
  st->frameCount++;
}

int main() {
  auto allFrames = readFile("assets/all_frames_ref.yuv");
  int numFrames = (int)(allFrames.size() / FRAME_SIZE);
  if (numFrames < 2 || allFrames.size() % FRAME_SIZE != 0) {
    printf("FAIL: unexpected asset size %zu\n", allFrames.size());
    return 1;
  }

  TinyH264Encoder<> enc;
  std::vector<uint8_t> bitstream(2000000);
  size_t total = 0;
  const int qp = 26;
  enc.setSize(W, H);
  enc.setQp(qp);

  // Frame 0 has no reference yet, so encodeFrame() encodes it as an
  // I-frame automatically; every frame after that becomes a P-frame
  // against the previous one - encodeFrame() is the only public encode
  // entry point now (see TinyH264Encoder.h's own header comment).
  for (int i = 0; i < numFrames; i++) {
    const uint8_t* frame = allFrames.data() + (size_t)i * FRAME_SIZE;
    const uint8_t* srcY = frame;
    const uint8_t* srcU = frame + (size_t)W * H;
    const uint8_t* srcV = srcU + (size_t)(W / 2) * (H / 2);
    size_t n = enc.encodeFrame(srcY, srcU, srcV, bitstream.data() + total,
                                bitstream.size() - total);
    if (n == 0) {
      printf("FAIL: frame %d encode returned 0\n", i);
      return 1;
    }
    total += n;
  }

  TinyH264Decoder<> dec;
  DecodeState st{allFrames.data()};
  dec.setCallback(psnrCallback, &st);
  dec.write(bitstream.data(), total);
  if (dec.hasError()) {
    printf("FAIL: decode reported an error\n");
    return 1;
  }
  if (st.frameCount != numFrames) {
    printf("FAIL: decoded %d frames, expected %d\n", st.frameCount, numFrames);
    return 1;
  }

  double mse = st.sumSq / ((double)W * H * numFrames);
  double psnr = mse > 0 ? 10 * log10(255.0 * 255.0 / mse) : 999.0;
  // Floor well below the real measured value (43.14dB at qp=26, real
  // ffmpeg cross-check) - real margin, not a guessed round number.
  const double kFloor = 38.0;
  if (psnr < kFloor) {
    printf("FAIL: PSNR %.2fdB below floor %.2fdB (%d frames, %zu bytes)\n",
           psnr, kFloor, numFrames, total);
    return 1;
  }

  printf("test_encode_pframe: OK (%d frames [1 I + %d P], %zu bytes, "
         "PSNR=%.2fdB)\n",
         numFrames, numFrames - 1, total, psnr);
  return 0;
}
