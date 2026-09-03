/*
 * Desktop-only test: decode the first (IDR) frame of the real QCIF
 * baseline/CAVLC stream with the actual TinyH264 decoder pipeline, and
 * compare it pixel-for-pixel against ffmpeg's own decode of the same
 * frame (assets/flat_frame0_ref.yuv, raw yuv420p). This is the real oracle for
 * everything built so far: CAVLC tables, macroblock layer, intra
 * prediction, and dequant/transform all have to be correct simultaneously
 * for this to match.
 */
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../../src/MemoryResource.h"
#include "../../src/StdAllocator.h"
#include "../../src/decoder/h264_decoder.h"

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

static int compareRegion(const char* name, const uint8_t* got, int gotStride,
                          const uint8_t* want, int wantStride, int w, int h) {
  int mismatches = 0;
  int firstX = -1, firstY = -1, firstGot = -1, firstWant = -1;
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      uint8_t a = got[y * gotStride + x];
      uint8_t b = want[y * wantStride + x];
      if (a != b) {
        if (mismatches == 0) {
          firstX = x;
          firstY = y;
          firstGot = a;
          firstWant = b;
        }
        mismatches++;
      }
    }
  }
  if (mismatches > 0) {
    printf("%s: %d/%d pixels mismatch (first at x=%d,y=%d got=%d want=%d, "
           "mb=(%d,%d))\n",
           name, mismatches, w * h, firstX, firstY, firstGot, firstWant,
           firstX / 16, firstY / 16);
  } else {
    printf("%s: exact match (%d pixels)\n", name, w * h);
  }
  return mismatches;
}

int main() {
  setbuf(stdout, nullptr);
  auto stream = readFile("assets/flat_test.264");
  auto ref = readFile("assets/flat_frame0_ref.yuv");

  AllocatorMemoryResource<StdAllocator<uint8_t>> mr;
  SoftwareDecoder decoder(mr);
  decoder.setInput(stream.data(), stream.size());

  DecodeStatus status;
  int nalCount = 0;
  do {
    status = decoder.next();
    nalCount++;
    if (status == DecodeStatus::kError) {
      printf("decode error after %d NALs\n", nalCount);
      return 1;
    }
    if (status == DecodeStatus::kUnsupported) {
      printf("unsupported stream feature after %d NALs\n", nalCount);
      return 1;
    }
  } while (status != DecodeStatus::kOk);

  printf("decoded first picture after %d NAL(s)\n", nalCount);

  const Frame& f = decoder.frame();
  printf("frame: %dx%d stride=%d/%d\n", f.width, f.height, f.strideY,
         f.strideC);
  assert(f.width == 176 && f.height == 144);

  const uint8_t* refY = ref.data();
  const uint8_t* refU = refY + 176 * 144;
  const uint8_t* refV = refU + 88 * 72;

  int mismatchY = compareRegion("Y", f.y(), f.strideY, refY, 176, 176, 144);
  int mismatchU = compareRegion("U", f.u(), f.strideC, refU, 88, 88, 72);
  int mismatchV = compareRegion("V", f.v(), f.strideC, refV, 88, 88, 72);

  if (mismatchY == 0 && mismatchU == 0 && mismatchV == 0) {
    printf("test_decode_iframe: PIXEL-EXACT MATCH vs ffmpeg\n");
    return 0;
  }
  printf("test_decode_iframe: MISMATCH vs ffmpeg reference\n");
  return 1;
}
