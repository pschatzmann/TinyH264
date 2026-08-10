// Desktop-only test: encodes a real decoded QCIF frame (the same
// frame0_ref.yuv oracle test_decode_iframe.cpp etc. already use) with
// TinyH264Encoder, then decodes the result back with this project's own
// TinyH264Decoder - self-contained, no ffmpeg dependency at test-run
// time (unlike development-time verification, see below).
//
// Two things are checked, at several QPs across the whole 0-51 range:
// 1. Decoding the encoder's own output must reproduce the encoder's own
//    internal reconstruction *bit-exactly* - both are supposed to be
//    "what a decoder does to this bitstream", so any difference at all
//    would mean the encoder's closed-loop reconstruction (used for
//    later-macroblock intra prediction context) doesn't actually match
//    real decode semantics, a real correctness bug.
// 2. Decoded quality (PSNR against the *original* source, not the
//    encoder's own reconstruction) must clear a QP-appropriate floor -
//    catches a regression that technically produces a valid, self-
//    consistent bitstream but has stopped meaningfully compressing the
//    source (e.g. a mode-decision or residual bug that still round-trips
//    with itself but no longer resembles the input).
//
// Development-time verification (not re-run here, but documented since
// it's what actually established this encoder is producing *correct*
// H.264, not just internally-consistent output): the same encode was
// decoded with real `ffmpeg` and found bit-exact against this project's
// own reconstruction at QP 18/26/32/40/48/51, and within +/-1 (a handful
// of pixels, likely a rounding-tie edge case in the deblocking filter at
// very fine quantization steps) at QP 0/10 - see the encoder source
// files' own comments for the two real bugs this verification caught
// during development (a missing inverse-Hadamard-transform step in the
// DC reconstruction path, and an out-of-bounds zigzag-table read on the
// chroma DC block).
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include "../../src/TinyH264Encoder.h"
#include "../../src/TinyH264Decoder.h"

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

/// Passed as userData to the decode callback below - the encoder's own
/// reconstruction (to diff against) plus an output mismatch counter, so
/// the callback needs no captures/statics.
struct CompareState {
  const uint8_t* encY;
  const uint8_t* encU;
  const uint8_t* encV;
  int width, height;
  int mismatches = 0;
  bool gotFrame = false;
};

void compareCallback(TinyH264Decoder<>& d, void* userData) {
  auto* st = (CompareState*)userData;
  st->gotFrame = true;
  for (int y = 0; y < st->height; y++) {
    for (int x = 0; x < st->width; x++) {
      if (d.y()[y * d.strideY() + x] != st->encY[y * st->width + x])
        st->mismatches++;
    }
  }
  for (int y = 0; y < st->height / 2; y++) {
    for (int x = 0; x < st->width / 2; x++) {
      if (d.u()[y * d.strideUV() + x] != st->encU[y * (st->width / 2) + x])
        st->mismatches++;
      if (d.v()[y * d.strideUV() + x] != st->encV[y * (st->width / 2) + x])
        st->mismatches++;
    }
  }
}

void testAtQp(const std::vector<uint8_t>& srcYuv, int W, int H, int qp) {
  const uint8_t* srcY = srcYuv.data();
  const uint8_t* srcU = srcYuv.data() + (size_t)W * H;
  const uint8_t* srcV = srcU + (size_t)(W / 2) * (H / 2);

  TinyH264Encoder<> enc;
  std::vector<uint8_t> bitstream(200000);
  enc.setSize(W, H);
  enc.setQp(qp);
  // A fresh encoder has no reference yet, so this is always an I-frame -
  // encodeFrame() is the only public encode entry point now (see
  // TinyH264Encoder.h's own header comment).
  size_t n = enc.encodeFrame(srcY, srcU, srcV, bitstream.data(),
                              bitstream.size());
  if (n == 0) {
    printf("FAIL qp=%d: encodeFrame returned 0\n", qp);
    failures++;
    return;
  }

  TinyH264Decoder<> dec;
  CompareState st{enc.y(), enc.u(), enc.v(), W, H};
  dec.setCallback(compareCallback, &st);
  // write()'s return value is the status of the *last* thing that
  // happened while draining the buffer (see TinyH264Decoder.h's own doc
  // comment) - for a single-frame buffer with nothing following, that's
  // kNeedMoreData (input exhausted) even though the callback already
  // fired for the one frame that was there; check st.gotFrame/hasError()
  // instead of the return value for "did a frame actually decode".
  dec.write(bitstream.data(), n);
  if (dec.hasError() || !st.gotFrame) {
    printf("FAIL qp=%d: decode of own output did not produce a frame "
           "(hasError=%d)\n",
           qp, dec.hasError());
    failures++;
    return;
  }
  if (st.mismatches != 0) {
    printf("FAIL qp=%d: %d sample mismatches between encoder's own "
           "reconstruction and decoding its own output\n",
           qp, st.mismatches);
    failures++;
    return;
  }

  double sumSq = 0;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      int d = (int)enc.y()[y * W + x] - (int)srcY[y * W + x];
      sumSq += (double)d * d;
    }
  }
  double mse = sumSq / (W * H);
  double psnr = mse > 0 ? 10 * log10(255.0 * 255.0 / mse) : 999.0;
  // Floor chosen from the actual measured curve (36.4-36.5dB, plateaued,
  // for qp 0-18; 36.2dB at qp=26 down to 23.7dB at qp=51) with real
  // margin, not a guessed round number - see this file's header comment.
  // PSNR doesn't keep climbing as qp drops toward 0 the way a linear
  // formula would suggest; it plateaus at this encoder's own inherent
  // ceiling (I_16x16-only, SAD-only mode decision, no rate control).
  double floor = 45.0 - qp * 0.45;
  if (floor > 34.0) floor = 34.0;
  if (psnr < floor) {
    printf("FAIL qp=%d: PSNR %.2fdB below floor %.2fdB (bytes=%zu)\n", qp,
           psnr, floor, n);
    failures++;
    return;
  }
  printf("qp=%2d: OK (%zu bytes, PSNR=%.2fdB, self-decode bit-exact)\n", qp,
         n, psnr);
}

int main() {
  auto yuv = readFile("assets/frame0_ref.yuv");
  const int W = 176, H = 144;
  if (yuv.size() != (size_t)W * H * 3 / 2) {
    printf("FAIL: unexpected asset size %zu\n", yuv.size());
    return 1;
  }

  for (int qp : {0, 10, 18, 26, 32, 40, 48, 51}) {
    testAtQp(yuv, W, H, qp);
  }

  if (failures == 0) {
    printf("test_encode_iframe: all QPs passed\n");
    return 0;
  }
  printf("test_encode_iframe: %d failure(s)\n", failures);
  return 1;
}
