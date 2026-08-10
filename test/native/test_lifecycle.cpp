/*
 * Desktop-only test: verifies the Encoder/Decoder begin()/end() lifecycle
 * methods (encoder/h264_encoder.h's Encoder::begin()/end(), decoder/
 * h264_decoder.h's Decoder::begin()/end(), and their TinyH264Encoder/
 * TinyH264Decoder public-API wrappers). Uses the internal Encoder<>/
 * Decoder<> classes directly (not the TinyH264*<> wrappers) since this
 * test needs to inspect frame().data.empty() to confirm memory was
 * actually freed/reserved, which the public wrappers don't expose.
 *
 * Checks, against a real 10-frame QCIF motion sequence
 * (assets/all_frames_ref.yuv, the same oracle test_encode_pframe.cpp/
 * test_encode_autoframe.cpp use):
 * 1. begin() eagerly allocates frame_/refFrame_ (frame().data non-empty
 *    before any encode call) - unlike the default lazy behavior, where
 *    frame().data stays empty until the first encodeFrame() call.
 * 2. Calling begin() (with or without a prior encode) doesn't change the
 *    encoded bitstream - it's purely an eager-allocation/state-reset
 *    hook, not a new code path - by encoding the same 10-frame sequence
 *    with and without a preceding begin() call and diffing the output
 *    byte-for-byte.
 * 3. end() actually frees the memory (frame().data.empty() afterward).
 * 4. end() then begin() + re-encoding the same sequence from scratch
 *    produces byte-identical output to a fresh, never-used Encoder -
 *    i.e. end() really resets stream state, not just memory.
 * 5. The decoder side: begin() eagerly allocates curFrame_, end() frees
 *    it, and a decoder that's been end()'d can begin()/decode the same
 *    stream again from scratch with identical results (frame count, no
 *    errors) to a fresh Decoder.
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "../../src/decoder/h264_decoder.h"
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
static const size_t FRAME_SIZE = (size_t)W * H * 3 / 2;
int failures = 0;

static size_t encodeSequence(Encoder<std::allocator<uint8_t>>& enc,
                              const std::vector<uint8_t>& allFrames,
                              int numFrames, std::vector<uint8_t>& bs) {
  size_t total = 0;
  const int qp = 26;
  enc.setSize(W, H);  // idempotent - safe to call again each time this
                       // helper runs on an already-configured encoder
  enc.setQp(qp);
  for (int i = 0; i < numFrames; i++) {
    const uint8_t* f = allFrames.data() + (size_t)i * FRAME_SIZE;
    const uint8_t* srcY = f;
    const uint8_t* srcU = f + (size_t)W * H;
    const uint8_t* srcV = srcU + (size_t)(W / 2) * (H / 2);
    size_t n = enc.encodeFrame(srcY, srcU, srcV, bs.data() + total,
                                bs.size() - total);
    if (n == 0) {
      printf("FAIL: frame %d encode returned 0\n", i);
      failures++;
      return total;
    }
    total += n;
  }
  return total;
}

int main() {
  auto allFrames = readFile("assets/all_frames_ref.yuv");
  int numFrames = (int)(allFrames.size() / FRAME_SIZE);
  if (numFrames < 4 || allFrames.size() % FRAME_SIZE != 0) {
    printf("FAIL: unexpected asset size %zu\n", allFrames.size());
    return 1;
  }

  // --- 1/2. Encoder::begin() eagerly allocates, doesn't change output ---
  std::vector<uint8_t> bsPlain(2000000), bsBegin(2000000);
  size_t nPlain, nBegin;
  {
    Encoder<std::allocator<uint8_t>> encPlain;
    if (!encPlain.frame().data.empty()) {
      printf("FAIL: frame_ allocated before any encode/begin() call\n");
      failures++;
    }
    nPlain = encodeSequence(encPlain, allFrames, numFrames, bsPlain);

    Encoder<std::allocator<uint8_t>> encBegin;
    encBegin.begin(/*reserveColorConversionScratch=*/true);
    if (encBegin.frame().data.empty()) {
      printf("FAIL: begin() didn't eagerly allocate frame_\n");
      failures++;
    }
    nBegin = encodeSequence(encBegin, allFrames, numFrames, bsBegin);
  }
  if (nPlain == 0 || nBegin == 0 || nPlain != nBegin ||
      memcmp(bsPlain.data(), bsBegin.data(), nPlain) != 0) {
    printf("FAIL: begin() changed encoded output (plain=%zu begin=%zu)\n",
           nPlain, nBegin);
    failures++;
  } else {
    printf("Encoder::begin() eagerly allocates and doesn't change output "
           "(%zu bytes, %d frames)\n",
           nPlain, numFrames);
  }

  // --- 3/4. Encoder::end() frees memory and truly resets stream state ---
  {
    Encoder<std::allocator<uint8_t>> enc;
    std::vector<uint8_t> bsFirst(2000000);
    encodeSequence(enc, allFrames, numFrames, bsFirst);
    if (enc.frame().data.empty()) {
      printf("FAIL: frame_ not allocated after encoding\n");
      failures++;
    }
    enc.end();
    if (!enc.frame().data.empty()) {
      printf("FAIL: end() didn't free frame_\n");
      failures++;
    }
    enc.begin();
    std::vector<uint8_t> bsSecond(2000000);
    size_t nSecond = encodeSequence(enc, allFrames, numFrames, bsSecond);
    if (nSecond == 0 || nSecond != nPlain ||
        memcmp(bsPlain.data(), bsSecond.data(), nPlain) != 0) {
      printf("FAIL: end()+begin()+re-encode didn't match a fresh encoder's "
             "output (fresh=%zu second=%zu)\n",
             nPlain, nSecond);
      failures++;
    } else {
      printf("Encoder::end() frees memory and end()+begin() resets stream "
             "state to a fresh encoder's behavior\n");
    }
  }

  // --- 5. Decoder::begin()/end() ---
  {
    Decoder<std::allocator<uint8_t>> decPlain;
    if (!decPlain.frame().data.empty()) {
      printf("FAIL: curFrame_ allocated before any decode/begin() call\n");
      failures++;
    }

    Decoder<std::allocator<uint8_t>> decBegin;
    decBegin.begin();
    if (decBegin.frame().data.empty()) {
      printf("FAIL: Decoder::begin() didn't eagerly allocate curFrame_\n");
      failures++;
    }

    auto decodeAll = [&](Decoder<std::allocator<uint8_t>>& dec) {
      dec.setInput(bsPlain.data(), nPlain);
      int frames = 0;
      while (true) {
        DecodeStatus s = dec.next();
        if (s == DecodeStatus::kOk) {
          frames++;
        } else if (s == DecodeStatus::kNeedMoreData) {
          if (dec.inputExhausted()) break;
        } else {
          return -1;
        }
      }
      return frames;
    };

    int framesPlain = decodeAll(decPlain);
    int framesBegin = decodeAll(decBegin);
    if (framesPlain != numFrames || framesBegin != numFrames) {
      printf("FAIL: decode frame count mismatch (plain=%d begin=%d "
             "expected=%d)\n",
             framesPlain, framesBegin, numFrames);
      failures++;
    } else {
      printf("Decoder::begin() eagerly allocates and doesn't change decode "
             "behavior (%d frames)\n",
             framesPlain);
    }

    if (decBegin.frame().data.empty()) {
      printf("FAIL: curFrame_ not allocated after decoding\n");
      failures++;
    }
    decBegin.end();
    if (!decBegin.frame().data.empty()) {
      printf("FAIL: Decoder::end() didn't free curFrame_\n");
      failures++;
    }
    decBegin.begin();
    int framesAfterEnd = decodeAll(decBegin);
    if (framesAfterEnd != numFrames) {
      printf("FAIL: end()+begin()+re-decode didn't fully decode again "
             "(got %d, expected %d)\n",
             framesAfterEnd, numFrames);
      failures++;
    } else {
      printf("Decoder::end() frees memory and end()+begin() resets decode "
             "state so the same stream decodes again from scratch\n");
    }
  }

  if (failures == 0) {
    printf("test_lifecycle: all checks passed\n");
    return 0;
  }
  printf("test_lifecycle: %d failure(s)\n", failures);
  return 1;
}
