// Desktop-only test: verifies TinyH264Decoder::toYUV420() (whole-frame
// and windowed) packs exactly the same bytes y()/u()/v() already expose,
// just concatenated into one tightly-packed buffer instead of three
// plane pointers - no color-space math involved (unlike the RGB
// converters), so this is checked for byte-exact equality, not a
// tolerance.
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

void onFrame(TinyH264Decoder<>& d, void*) {
  int w = d.width(), h = d.height();
  size_t needed = (size_t)w * h + 2 * (size_t)(w / 2) * (h / 2);

  // Size-check behavior, same pattern as the RGB converters: returns the
  // number of bytes written (== needed) on success, 0 on a too-small buffer.
  std::vector<uint8_t> buf(needed);
  if (d.toYUV420(buf.data(), needed) != needed) {
    printf("FAIL: exact-size buffer rejected\n");
    failures++;
  }
  if (d.toYUV420(buf.data(), needed - 1) != 0) {
    printf("FAIL: under-size buffer accepted\n");
    failures++;
  }

  // Byte-exact content check: reconstruct what toYUV420() *should* have
  // produced directly from y()/u()/v(), and compare.
  size_t off = 0;
  for (int row = 0; row < h; row++) {
    for (int col = 0; col < w; col++) {
      if (buf[off++] != d.getY(col, row)) {
        printf("FAIL: Y mismatch at (%d,%d)\n", col, row);
        failures++;
      }
    }
  }
  for (int row = 0; row < h / 2; row++) {
    for (int col = 0; col < w / 2; col++) {
      if (buf[off++] != d.getU(col * 2, row * 2)) {
        printf("FAIL: U mismatch at (%d,%d)\n", col, row);
        failures++;
      }
    }
  }
  for (int row = 0; row < h / 2; row++) {
    for (int col = 0; col < w / 2; col++) {
      if (buf[off++] != d.getV(col * 2, row * 2)) {
        printf("FAIL: V mismatch at (%d,%d)\n", col, row);
        failures++;
      }
    }
  }

  // Windowed variant: tile content must match the corresponding
  // sub-rectangle of the whole-frame packed buffer.
  int tx = 64, ty = 32, tdx = 32, tdy = 32;
  size_t tileNeeded = (size_t)tdx * tdy + 2 * (size_t)(tdx / 2) * (tdy / 2);
  std::vector<uint8_t> tile(tileNeeded);
  if (d.toYUV420(tx, ty, tdx, tdy, tile.data(), tileNeeded) != tileNeeded) {
    printf("FAIL: tile exact-size buffer rejected\n");
    failures++;
  }
  if (d.toYUV420(tx, ty, tdx, tdy, tile.data(), tileNeeded - 1) != 0) {
    printf("FAIL: tile under-size buffer accepted\n");
    failures++;
  }
  size_t tOff = 0;
  for (int row = 0; row < tdy; row++) {
    for (int col = 0; col < tdx; col++) {
      if (tile[tOff++] != d.getY(tx + col, ty + row)) {
        printf("FAIL: tile Y mismatch at local (%d,%d)\n", col, row);
        failures++;
      }
    }
  }
  for (int row = 0; row < tdy / 2; row++) {
    for (int col = 0; col < tdx / 2; col++) {
      if (tile[tOff++] != d.getU(tx + col * 2, ty + row * 2)) {
        printf("FAIL: tile U mismatch at local (%d,%d)\n", col, row);
        failures++;
      }
    }
  }
  for (int row = 0; row < tdy / 2; row++) {
    for (int col = 0; col < tdx / 2; col++) {
      if (tile[tOff++] != d.getV(tx + col * 2, ty + row * 2)) {
        printf("FAIL: tile V mismatch at local (%d,%d)\n", col, row);
        failures++;
      }
    }
  }

  printf("frame %dx%d checked\n", w, h);
}

int main() {
  auto stream = readFile("assets/multiref.264");
  TinyH264Decoder<> decoder;
  decoder.setCallback(onFrame);
  decoder.write(stream.data(), stream.size());
  printf("test_yuv420: failures=%d\n", failures);
  return failures == 0 ? 0 : 1;
}
