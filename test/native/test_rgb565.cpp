/*
 * Desktop-only test: converts a real decoded frame's reference YUV (the
 * same oracle used by the other pixel-exact tests) to RGB565 and compares
 * every pixel against ffmpeg's own `-pix_fmt rgb565le` conversion of the
 * same YUV data. This is the oracle for h264_rgb.h's YUV->RGB
 * conversion - the integer BT.601 coefficients were independently
 * re-derived from the published matrix, and this test confirms the
 * fixed-point rounding is *close* to a real, independent implementation,
 * not just "looks right" by inspection.
 *
 * Not held to literal bit-exact matching, unlike this project's decode
 * tests: ffmpeg's swscale uses 16-bit-precision internal conversion
 * tables, while h264_rgb.h deliberately uses a much cheaper 8-bit
 * fixed-point formula (the standard, widely-used embedded/mobile
 * approach - a 65536-entry table isn't appropriate for this library's
 * memory budget). Verified (once, by exhaustive per-pixel diff over a
 * real frame) that this never differs from ffmpeg's output by more than
 * 1 LSB in any single 5/6-bit RGB565 channel - an imperceptible,
 * expected precision difference, not a bug. This test enforces that
 * bound rather than exact equality.
 */
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
  auto yuv = readFile("assets/multiref_ref.yuv");  // frame 0 (first 38016 bytes)
  auto ref = readFile("assets/frame0_rgb565le_ref.raw");

  if (ref.size() != (size_t)W * H * 2) {
    fprintf(stderr, "unexpected ref size %zu\n", ref.size());
    return 1;
  }

  const uint8_t* yPlane = yuv.data();
  const uint8_t* uPlane = yPlane + W * H;
  const uint8_t* vPlane = uPlane + (W / 2) * (H / 2);

  std::vector<uint16_t> got(W * H);
  convertYuv420ToRgb565(yPlane, W, uPlane, vPlane, W / 2, W, H, got.data());

  int exactMatches = 0, withinTolerance = 0, tooFarOff = 0;
  int worstDiff = 0, worstX = -1, worstY = -1;
  uint16_t worstGot = 0, worstWant = 0;
  for (int row = 0; row < H; row++) {
    for (int col = 0; col < W; col++) {
      uint16_t g = got[row * W + col];
      size_t off = (size_t)(row * W + col) * 2;
      uint16_t w = (uint16_t)(ref[off] | (ref[off + 1] << 8));
      if (g == w) {
        exactMatches++;
        continue;
      }
      int dr = abs(((g >> 11) & 0x1F) - ((w >> 11) & 0x1F));
      int dg = abs(((g >> 5) & 0x3F) - ((w >> 5) & 0x3F));
      int db = abs((g & 0x1F) - (w & 0x1F));
      int diff = dr > dg ? (dr > db ? dr : db) : (dg > db ? dg : db);
      if (diff <= 1) {
        withinTolerance++;
      } else {
        tooFarOff++;
      }
      if (diff > worstDiff) {
        worstDiff = diff;
        worstX = col;
        worstY = row;
        worstGot = g;
        worstWant = w;
      }
    }
  }

  printf("test_rgb565: %d exact, %d within +/-1 LSB tolerance, %d too far off "
         "(of %d pixels); worst diff=%d",
         exactMatches, withinTolerance, tooFarOff, W * H, worstDiff);
  if (worstDiff > 0) {
    printf(" at (%d,%d) got=0x%04x want=0x%04x", worstX, worstY, worstGot,
           worstWant);
  }
  printf("\n");
  return tooFarOff == 0 ? 0 : 1;
}
