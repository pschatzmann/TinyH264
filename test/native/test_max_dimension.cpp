/*
 * Desktop-only test: verifies Decoder::setMaxDimension()/
 * Encoder::setMaxDimension() (the runtime alternative to `#define
 * H264_MAX_WIDTH ...`/`#define H264_MAX_HEIGHT ...` before including
 * TinyH264Decoder.h/TinyH264Encoder.h) - see h264_decoder.h/
 * h264_encoder.h for the full design rationale.
 *
 * Uses the internal SoftwareDecoder/SoftwareEncoder classes directly (not
 * the TinyH264*<> wrappers), the same pattern test_lifecycle.cpp uses, to
 * inspect frame().dataY.size() and confirm the *actual* allocated buffer
 * size tracks the runtime-configured ceiling, not just the compile-time
 * H264_MAX_WIDTH/H264_MAX_HEIGHT default.
 *
 * Checks:
 * 1. Default (never called): unchanged from before this feature existed
 *    - maxWidth()/maxHeight() equal the compile-time default, and a real
 *      stream still decodes correctly.
 * 2. Shrinking below the compile-time default: the actual heap
 *    allocation shrinks to match (not just accepted - actually smaller,
 *    the whole point), and a real QCIF stream still decodes pixel-exact
 *    at the smaller ceiling.
 * 3. Growing above the compile-time default: accepted and reflected in
 *    maxWidth()/maxHeight() and the actual allocation size - nothing
 *    downstream silently caps it back to the old compile-time bound.
 * 4. A stream whose SPS declares a resolution bigger than the
 *    (shrunk) runtime ceiling is rejected as kUnsupported, exactly like
 *    exceeding the old compile-time H264_MAX_WIDTH/H264_MAX_HEIGHT
 *    always was - confirms the bounds check itself was actually
 *    switched to the runtime value, not just the allocation size.
 * 5. Encoder side: setMaxDimension() shrinks/grows frame_'s actual
 *    allocation the same way, and encodeIFrame() rejects a picture
 *    bigger than the configured ceiling.
 */
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../../src/MemoryResource.h"
#include "../../src/StdAllocator.h"
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

int main() {
  // --- 1. Default: unchanged from the compile-time H264_MAX_WIDTH/HEIGHT ---
  {
    AllocatorMemoryResource<StdAllocator<uint8_t>> mr;
    SoftwareDecoder dec(mr);
    CHECK(dec.maxWidth() == H264_MAX_WIDTH, "default maxWidth() != H264_MAX_WIDTH");
    CHECK(dec.maxHeight() == H264_MAX_HEIGHT, "default maxHeight() != H264_MAX_HEIGHT");

    auto stream = readFile("assets/multiref.264");
    int frames = 0;
    dec.setInput(stream.data(), stream.size());
    while (true) {
      DecodeStatus s = dec.next();
      if (s == DecodeStatus::kOk) {
        frames++;
        continue;
      }
      if (s == DecodeStatus::kNeedMoreData && dec.inputExhausted()) break;
      if (s == DecodeStatus::kNeedMoreData) continue;
      printf("FAIL: default-ceiling decode failed with status %d\n", (int)s);
      failures++;
      break;
    }
    CHECK(frames == 40, "default-ceiling decode: wrong frame count");
    CHECK((int)dec.frame().dataY.size() == H264_MAX_WIDTH * H264_MAX_HEIGHT,
          "default-ceiling dataY size != H264_MAX_WIDTH*H264_MAX_HEIGHT");
  }

  // --- 2. Shrinking: real QCIF stream still decodes at a smaller ceiling ---
  {
    AllocatorMemoryResource<StdAllocator<uint8_t>> mr;
    SoftwareDecoder dec(mr);
    dec.setMaxDimension(176, 144);
    CHECK(dec.maxWidth() == 176 && dec.maxHeight() == 144,
          "setMaxDimension(176,144) not reflected in maxWidth()/maxHeight()");

    auto stream = readFile("assets/multiref.264");
    int frames = 0;
    dec.setInput(stream.data(), stream.size());
    while (true) {
      DecodeStatus s = dec.next();
      if (s == DecodeStatus::kOk) {
        frames++;
        continue;
      }
      if (s == DecodeStatus::kNeedMoreData && dec.inputExhausted()) break;
      if (s == DecodeStatus::kNeedMoreData) continue;
      printf("FAIL: shrunk-ceiling decode failed with status %d\n", (int)s);
      failures++;
      break;
    }
    CHECK(frames == 40, "shrunk-ceiling decode: wrong frame count");
    CHECK((int)dec.frame().dataY.size() == 176 * 144,
          "shrunk-ceiling dataY size != 176*144 (still allocated at the old default?)");
  }

  // --- 3. Growing: accepted, reflected in the actual allocation ---
  {
    AllocatorMemoryResource<StdAllocator<uint8_t>> mr;
    SoftwareDecoder dec(mr);
    dec.setMaxDimension(640, 480);
    CHECK(dec.maxWidth() == 640 && dec.maxHeight() == 480,
          "setMaxDimension(640,480) not reflected in maxWidth()/maxHeight()");
    dec.begin();  // forces eager allocation so dataY.size() is checkable now
    CHECK((int)dec.frame().dataY.size() == 640 * 480,
          "grown-ceiling dataY size != 640*480 (silently capped to the compile default?)");
  }

  // --- 4. A stream exceeding the (shrunk) runtime ceiling is rejected ---
  {
    AllocatorMemoryResource<StdAllocator<uint8_t>> mr;
    SoftwareDecoder dec(mr);
    dec.setMaxDimension(160, 128);  // smaller than the real 176x144 stream
    auto stream = readFile("assets/multiref.264");
    dec.setInput(stream.data(), stream.size());
    DecodeStatus s = dec.next();  // first NAL is the SPS
    CHECK(s == DecodeStatus::kUnsupported,
          "176x144 SPS not rejected against a 160x128 runtime ceiling");
  }

  // --- 5. Encoder side: shrink/grow the allocation, reject an oversized encode ---
  {
    AllocatorMemoryResource<StdAllocator<uint8_t>> mr;
    SoftwareEncoder enc(mr);
    enc.setMaxDimension(176, 144);
    CHECK(enc.maxWidth() == 176 && enc.maxHeight() == 144,
          "Encoder::setMaxDimension(176,144) not reflected");
    enc.begin();
    CHECK((int)enc.frame().dataY.size() == 176 * 144,
          "Encoder shrunk-ceiling dataY size != 176*144");

    std::vector<uint8_t> srcY(320 * 240, 128), srcU(160 * 120, 128),
        srcV(160 * 120, 128);
    std::vector<uint8_t> dst(65536);
    enc.setSize(320, 240);
    enc.setQp(26);  // fixed QP - isolates the size check from rate control
    size_t n = enc.encodeFrame(srcY.data(), srcU.data(), srcV.data(),
                                dst.data(), dst.size());
    CHECK(n == 0, "encodeFrame() accepted 320x240 against a 176x144 ceiling");

    AllocatorMemoryResource<StdAllocator<uint8_t>> mr2;
    SoftwareEncoder enc2(mr2);
    enc2.setMaxDimension(640, 480);
    enc2.setSize(320, 240);
    enc2.setQp(26);
    n = enc2.encodeFrame(srcY.data(), srcU.data(), srcV.data(), dst.data(),
                         dst.size());
    CHECK(n > 0, "encodeFrame() rejected 320x240 against a 640x480 ceiling");
    CHECK((int)enc2.frame().dataY.size() == 640 * 480,
          "Encoder grown-ceiling dataY size != 640*480");
  }

  printf("test_max_dimension: failures=%d\n", failures);
  return failures == 0 ? 0 : 1;
}
