/*
 * Desktop-only test: not a correctness oracle (see the other test_*.cpp
 * files for those) - this exists purely to confirm the H264LOG wiring
 * added throughout the encoder/decoder actually compiles and runs
 * end-to-end when a caller opts in (`H264LOG.begin(LogLevel::kDebug)`),
 * rather than only ever being exercised in its default-silent (kNone)
 * state the way every other test in this suite runs it. Deliberately
 * triggers a handful of real failure paths reachable without hardware -
 * encodeFrame() before setSize(), rate control requested but never
 * configured, a too-small dst buffer on both the encoder and decoder
 * side, and a slice NAL arriving before any SPS/PPS - and just checks
 * each still reports the failure it always did (0 / DecodeStatus::kError),
 * with logging enabled, printing to stdout for a human to eyeball (no
 * capture sink exists to assert on the exact text).
 */
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../../src/TinyH264Decoder.h"
#include "../../src/TinyH264Encoder.h"
#include "../../src/common/Logger.h"

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
  std::vector<uint8_t> buf((size_t)sz);
  size_t n = fread(buf.data(), 1, (size_t)sz, f);
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

int main() {
  H264LOG.begin(LogLevel::kDebug);

  printf("-- encodeFrame() before setSize() --\n");
  {
    TinyH264Encoder<> enc;
    uint8_t srcY[16 * 16], srcU[8 * 8], srcV[8 * 8];
    uint8_t dst[4096];
    size_t n = enc.encodeFrame(srcY, srcU, srcV, dst, sizeof(dst));
    CHECK(n == 0, "encodeFrame() before setSize() should return 0");
  }

  printf("-- rate control requested but never configured --\n");
  {
    TinyH264Encoder<> enc;
    enc.setSize(176, 144);
    enc.setQp(-1);  // request rate control without setTargetBitrate()
    std::vector<uint8_t> srcY(176 * 144, 128), srcU(88 * 72, 128), srcV(88 * 72, 128);
    uint8_t dst[65536];
    size_t n = enc.encodeFrame(srcY.data(), srcU.data(), srcV.data(), dst, sizeof(dst));
    CHECK(n == 0, "encodeFrame() with unconfigured rate control should return 0");
  }

  printf("-- too-small dst buffer (encoder) --\n");
  {
    TinyH264Encoder<> enc;
    enc.setSize(176, 144);
    enc.setQp(26);
    std::vector<uint8_t> srcY(176 * 144, 128), srcU(88 * 72, 128), srcV(88 * 72, 128);
    uint8_t tinyDst[4];
    size_t n = enc.encodeFrame(srcY.data(), srcU.data(), srcV.data(), tinyDst, sizeof(tinyDst));
    CHECK(n == 0, "encodeFrame() with a too-small dst should return 0");
  }

  printf("-- too-small dst buffer (decoder toRGB888) --\n");
  {
    auto stream = readFile("assets/flat_test.264");
    TinyH264Decoder<> dec;
    dec.write(stream.data(), stream.size());
    CHECK(!dec.hasError(), "flat_test.264 should decode without error first");
    uint8_t tinyDst[4];
    size_t n = dec.toRGB888(tinyDst, sizeof(tinyDst));
    CHECK(n == 0, "toRGB888() with a too-small dst should return 0");
  }

  printf("-- slice NAL with no SPS/PPS yet (decoder) --\n");
  {
    // CAVLC/Exp-Golomb tolerates a lot of garbage without ever setting
    // br.error() (most bit patterns still decode to *some* - just wrong
    // - value), so simple byte corruption doesn't reliably reach
    // decodeSlice()'s bitstream-error sites. Feeding a slice NAL with no
    // SPS/PPS parsed first is a deterministic way to reach one of them
    // instead (decodeSlice()'s very first check).
    auto stream = readFile("assets/flat_test.264");
    // flat_test.264 is SPS, PPS, SEI, then the first (IDR) slice NAL -
    // find the start code immediately before the first NAL whose
    // nal_unit_type is 1 or 5 (slice) and feed the decoder everything
    // from there onward, skipping SPS/PPS entirely.
    size_t sliceStart = 0;
    bool found = false;
    for (size_t i = 0; i + 4 <= stream.size(); i++) {
      size_t hdrOff;
      if (stream[i] == 0 && stream[i + 1] == 0 && stream[i + 2] == 0 && stream[i + 3] == 1) {
        hdrOff = i + 4;
      } else if (stream[i] == 0 && stream[i + 1] == 0 && stream[i + 2] == 1) {
        hdrOff = i + 3;
      } else {
        continue;
      }
      if (hdrOff >= stream.size()) continue;
      int nalType = stream[hdrOff] & 0x1f;
      if (nalType == 1 || nalType == 5) {
        sliceStart = i;
        found = true;
        break;
      }
    }
    CHECK(found, "expected to find a slice NAL in flat_test.264");
    std::vector<uint8_t> sliceOnly(stream.begin() + (long)sliceStart, stream.end());
    TinyH264Decoder<> dec;
    dec.write(sliceOnly.data(), sliceOnly.size());
    CHECK(dec.hasError(), "a slice NAL with no SPS/PPS yet should report an error");
  }

  H264LOG.begin(LogLevel::kNone);  // back to silent, matching every other test's expectation

  if (failures == 0) {
    printf("test_logger_wiring: all checks passed\n");
    return 0;
  }
  printf("test_logger_wiring: %d failure(s)\n", failures);
  return 1;
}
