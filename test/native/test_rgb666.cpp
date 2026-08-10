// Desktop-only test: verifies h264_rgb.h's RGB666 conversion by
// self-consistency against RGB888, rather than an external oracle -
// ffmpeg has no raw 18-bit-per-pixel pixel format to cross-check against
// (unlike RGB565's rgb565le and RGB888's rgb24, both verified elsewhere
// against real ffmpeg output). Both RGB666 and RGB888 are computed from
// the exact same yuvToRgb8() values (see h264_rgb.h); RGB666 is defined
// as that 8-bit value with its bottom 2 bits cleared (left-justified in
// bits 7:2), so every RGB666 output byte must equal the corresponding
// RGB888 output byte with `& 0xFC` applied - checked here for every byte
// of a real decoded frame, exactly, not just approximately.
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../../src/decoder/h264_rgb.h"

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

int main() {
  const int W = 176, H = 144;
  auto yuv = readFile("assets/multiref_ref.yuv");  // frame 0

  const uint8_t* yPlane = yuv.data();
  const uint8_t* uPlane = yPlane + W * H;
  const uint8_t* vPlane = uPlane + (W / 2) * (H / 2);

  std::vector<uint8_t> rgb888(W * H * 3);
  std::vector<uint8_t> rgb666(W * H * 3);
  convertYuv420ToRgb888(yPlane, W, uPlane, vPlane, W / 2, W, H, rgb888.data());
  convertYuv420ToRgb666(yPlane, W, uPlane, vPlane, W / 2, W, H, rgb666.data());

  int mismatches = 0;
  int bottomBitsSet = 0;
  for (size_t i = 0; i < rgb666.size(); i++) {
    uint8_t expected = rgb888[i] & 0xFC;
    if (rgb666[i] != expected) {
      if (mismatches < 5) {
        printf("MISMATCH at byte %zu: rgb666=%d rgb888&0xFC=%d (rgb888=%d)\n",
               i, rgb666[i], expected, rgb888[i]);
      }
      mismatches++;
    }
    if (rgb666[i] & 0x03) bottomBitsSet++;
  }

  printf("test_rgb666: %zu bytes checked, %d mismatches, %d with bottom "
         "2 bits set (should be 0)\n",
         rgb666.size(), mismatches, bottomBitsSet);
  return (mismatches == 0 && bottomBitsSet == 0) ? 0 : 1;
}
