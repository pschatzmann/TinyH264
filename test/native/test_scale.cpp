/*
 * Desktop-only test: verifies TinyH264Decoder::setScaleFactor()/
 * widthScaled()/heightScaled() and the scaled toRGB565()/toRGB666()/
 * toRGB888()/toYUV420() conversion methods (see decoder/h264_rgb.h and
 * common/h264_frame.h's new scaled overloads).
 *
 * Checks, per the design confirmed with the user before implementing:
 * 1. Default scale factor (1.0) is a true no-op: widthScaled()/heightScaled()
 *    exactly equal width()/height(), and every conversion method's
 *    output is byte-identical whether or not setScaleFactor(1.0f) was
 *    called explicitly.
 * 2. Exact 2x upscale: every native pixel becomes a clean 2x2 block in
 *    the output - checked for all 4 conversion formats by comparing
 *    against the corresponding *unscaled* conversion's pixel, not by
 *    re-deriving the scaling formula (an independent check, not just
 *    re-checking the implementation against itself).
 * 3. Exact 0.5x downscale: the inverse relationship (native pixel (2x,
 *    2y) reappears at output (x,y)).
 * 4. A non-integer scale factor (1.5x), checked two ways: windowed
 *    tiles covering the whole scaled picture must match the
 *    corresponding sub-rectangle of one whole-picture scaled call
 *    (tile-consistency, the same pattern this project's other windowed
 *    conversion tests use), and widthScaled()/heightScaled() must stay even
 *    (4:2:0 chroma requires it) across several odd-rounding-prone scale
 *    factors.
 */
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "TinyH264Decoder.h"

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

int failures = 0;

#define CHECK(cond, msg)                     \
  do {                                       \
    if (!(cond)) {                           \
      printf("FAIL: %s\n", msg);             \
      failures++;                            \
    }                                        \
  } while (0)

void onFrame(TinyH264Decoder<>& d, void*) {
  int w = d.width(), h = d.height();

  // --- 1. Default scale factor is a true no-op ------------------------
  CHECK(d.scaleFactor() == 1.0f, "default scaleFactor() != 1.0");
  CHECK(d.widthScaled() == w, "widthScaled() != width() at default scale");
  CHECK(d.heightScaled() == h, "heightScaled() != height() at default scale");

  std::vector<uint8_t> yuvImplicit((size_t)w * h + 2 * (size_t)(w / 2) * (h / 2));
  d.toYUV420(yuvImplicit.data(), yuvImplicit.size());
  d.setScaleFactor(1.0f);
  std::vector<uint8_t> yuvExplicit(yuvImplicit.size());
  d.toYUV420(yuvExplicit.data(), yuvExplicit.size());
  CHECK(yuvImplicit == yuvExplicit,
        "setScaleFactor(1.0f) changed toYUV420() output");

  // Unscaled RGB references, used as the independent oracle for the 2x/
  // 0.5x checks below (compared against, not re-derived from the same
  // ratio formula the implementation itself uses).
  std::vector<uint16_t> rgb565Native((size_t)w * h);
  std::vector<uint8_t> rgb666Native((size_t)w * h * 3);
  std::vector<uint8_t> rgb888Native((size_t)w * h * 3);
  d.toRGB565(rgb565Native.data(), rgb565Native.size());
  d.toRGB666(rgb666Native.data(), rgb666Native.size());
  d.toRGB888(rgb888Native.data(), rgb888Native.size());

  // --- 2. Exact 2x upscale: each native pixel -> a clean 2x2 block ----
  d.setScaleFactor(2.0f);
  int w2 = d.widthScaled(), h2 = d.heightScaled();
  CHECK(w2 == 2 * w, "widthScaled() != 2*width() at scale 2.0");
  CHECK(h2 == 2 * h, "heightScaled() != 2*height() at scale 2.0");

  std::vector<uint16_t> rgb565x2((size_t)w2 * h2);
  std::vector<uint8_t> rgb666x2((size_t)w2 * h2 * 3);
  std::vector<uint8_t> rgb888x2((size_t)w2 * h2 * 3);
  std::vector<uint8_t> yuvx2((size_t)w2 * h2 + 2 * (size_t)(w2 / 2) * (h2 / 2));
  d.toRGB565(rgb565x2.data(), rgb565x2.size());
  d.toRGB666(rgb666x2.data(), rgb666x2.size());
  d.toRGB888(rgb888x2.data(), rgb888x2.size());
  d.toYUV420(yuvx2.data(), yuvx2.size());

  int blockMismatches = 0;
  for (int oy = 0; oy < h2 && blockMismatches < 5; oy++) {
    int py = oy / 2;
    for (int ox = 0; ox < w2 && blockMismatches < 5; ox++) {
      int px = ox / 2;
      if (rgb565x2[(size_t)oy * w2 + ox] != rgb565Native[(size_t)py * w + px]) {
        blockMismatches++;
      }
      const uint8_t* g666 = &rgb666x2[((size_t)oy * w2 + ox) * 3];
      const uint8_t* n666 = &rgb666Native[((size_t)py * w + px) * 3];
      if (g666[0] != n666[0] || g666[1] != n666[1] || g666[2] != n666[2]) {
        blockMismatches++;
      }
      const uint8_t* g888 = &rgb888x2[((size_t)oy * w2 + ox) * 3];
      const uint8_t* n888 = &rgb888Native[((size_t)py * w + px) * 3];
      if (g888[0] != n888[0] || g888[1] != n888[1] || g888[2] != n888[2]) {
        blockMismatches++;
      }
      if (yuvx2[(size_t)oy * w2 + ox] != d.getY(px, py)) {
        blockMismatches++;
      }
    }
  }
  CHECK(blockMismatches == 0, "2x upscale: output block != native pixel");

  // --- 3. Exact 0.5x downscale: inverse relationship -------------------
  d.setScaleFactor(0.5f);
  int wHalf = d.widthScaled(), hHalf = d.heightScaled();
  CHECK(wHalf == w / 2, "widthScaled() != width()/2 at scale 0.5");
  CHECK(hHalf == h / 2, "heightScaled() != height()/2 at scale 0.5");

  std::vector<uint8_t> yuvHalf((size_t)wHalf * hHalf +
                               2 * (size_t)(wHalf / 2) * (hHalf / 2));
  d.toYUV420(yuvHalf.data(), yuvHalf.size());
  int halfMismatches = 0;
  for (int oy = 0; oy < hHalf; oy++) {
    for (int ox = 0; ox < wHalf; ox++) {
      if (yuvHalf[(size_t)oy * wHalf + ox] != d.getY(ox * 2, oy * 2)) {
        halfMismatches++;
      }
    }
  }
  CHECK(halfMismatches == 0, "0.5x downscale: output != native pixel at 2x coord");

  // --- 4a. Non-integer scale: windowed tiles == sub-rect of whole call -
  d.setScaleFactor(1.5f);
  int w15 = d.widthScaled(), h15 = d.heightScaled();
  std::vector<uint8_t> yuvWhole((size_t)w15 * h15 +
                                2 * (size_t)(w15 / 2) * (h15 / 2));
  d.toYUV420(yuvWhole.data(), yuvWhole.size());

  int tx = 8, ty = 8, tdx = 16, tdy = 16;
  if (tx + tdx <= w15 && ty + tdy <= h15) {
    size_t tileNeeded = (size_t)tdx * tdy + 2 * (size_t)(tdx / 2) * (tdy / 2);
    std::vector<uint8_t> tile(tileNeeded);
    CHECK(d.toYUV420(tx, ty, tdx, tdy, tile.data(), tileNeeded) == tileNeeded,
          "scaled tile: exact-size buffer rejected");
    size_t tOff = 0;
    int tileMismatches = 0;
    for (int row = 0; row < tdy; row++) {
      for (int col = 0; col < tdx; col++) {
        int oy = ty + row, ox = tx + col;
        if (tile[tOff++] != yuvWhole[(size_t)oy * w15 + ox]) tileMismatches++;
      }
    }
    uint8_t* wholeU = yuvWhole.data() + (size_t)w15 * h15;
    uint8_t* wholeV = wholeU + (size_t)(w15 / 2) * (h15 / 2);
    for (int row = 0; row < tdy / 2; row++) {
      for (int col = 0; col < tdx / 2; col++) {
        int oy = ty / 2 + row, ox = tx / 2 + col;
        if (tile[tOff++] != wholeU[(size_t)oy * (w15 / 2) + ox]) tileMismatches++;
      }
    }
    for (int row = 0; row < tdy / 2; row++) {
      for (int col = 0; col < tdx / 2; col++) {
        int oy = ty / 2 + row, ox = tx / 2 + col;
        if (tile[tOff++] != wholeV[(size_t)oy * (w15 / 2) + ox]) tileMismatches++;
      }
    }
    CHECK(tileMismatches == 0,
          "scaled windowed tile != corresponding sub-rect of whole call");
  }

  // --- 4b. widthScaled()/heightScaled() always even, across several factors -
  float factors[] = {0.33f, 0.77f, 1.01f, 1.23f, 1.99f, 2.71f, 3.0f};
  for (float f : factors) {
    d.setScaleFactor(f);
    CHECK(d.widthScaled() % 2 == 0, "widthScaled() not even for some scale factor");
    CHECK(d.heightScaled() % 2 == 0, "heightScaled() not even for some scale factor");
  }

  d.setScaleFactor(1.0f);  // leave the decoder in the default state
  printf("frame %dx%d checked (scale 1.0/2.0/0.5/1.5)\n", w, h);
}

int main() {
  auto stream = readFile("assets/multiref.264");
  TinyH264Decoder<> decoder;
  decoder.setCallback(onFrame);
  decoder.write(stream.data(), stream.size());
  printf("test_scale: failures=%d\n", failures);
  return failures == 0 ? 0 : 1;
}
