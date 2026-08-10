// Desktop-only test: exercise the public TinyH264Decoder facade (src/TinyH264Decoder.h)
// exactly as an Arduino sketch would use it - write() driving the decode
// loop internally and firing a callback per frame - on the known-good flat
// 3-frame stream (validated pixel-exact via the lower-level API in
// test_decode_flat_multi.cpp).
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../../src/TinyH264Decoder.h"

using namespace tinyh264;

static std::vector<uint8_t> readFile(const char* path) {
  FILE* f = fopen(path, "rb");
  if (!f) {
    fprintf(stderr, "cannot open %s\n", path);
    exit(1);
  }
  fseek(f, 0, SEEK_END);
  long sz = ftell(f);
  fseek(f, 0, SEEK_SET);
  std::vector<uint8_t> buf(sz);
  size_t n = fread(buf.data(), 1, sz, f);
  (void)n;
  fclose(f);
  return buf;
}

struct Context {
  const std::vector<uint8_t>* ref;
  int frameIdx = 0;
  bool allExact = true;
};

static void onFrame(TinyH264Decoder<>& decoder, void* userData) {
  Context* ctx = (Context*)userData;
  printf("frame %d: %dx%d strideY=%d strideUV=%d\n", ctx->frameIdx,
         decoder.width(), decoder.height(), decoder.strideY(),
         decoder.strideUV());
  assert(decoder.width() == 176 && decoder.height() == 144);
  const uint8_t* refFrame = ctx->ref->data() + (size_t)ctx->frameIdx * 38016;
  bool exact = true;
  for (int y = 0; y < 144 && exact; y++)
    for (int x = 0; x < 176; x++)
      if (decoder.y()[y * decoder.strideY() + x] != refFrame[y * 176 + x]) {
        exact = false;
        break;
      }
  printf("  luma exact: %s\n", exact ? "yes" : "NO");
  ctx->allExact &= exact;
  ctx->frameIdx++;
}

int main() {
  setbuf(stdout, nullptr);
  auto stream = readFile("assets/flat_test.264");
  auto ref = readFile("assets/flat_all_frames_ref.yuv");

  Context ctx;
  ctx.ref = &ref;

  TinyH264Decoder<> decoder;
  decoder.setCallback(onFrame, &ctx);
  decoder.write(stream.data(), stream.size());

  if (decoder.hasError()) {
    printf("decode failed after %d frame(s)\n", ctx.frameIdx);
    return 1;
  }
  if (ctx.frameIdx != 3 || !ctx.allExact) {
    printf("expected 3 pixel-exact frames, got %d (allExact=%d)\n",
           ctx.frameIdx, ctx.allExact);
    return 1;
  }
  printf("test_public_api: all %d frames decoded correctly via TinyH264Decoder facade\n",
         ctx.frameIdx);
  return 0;
}
