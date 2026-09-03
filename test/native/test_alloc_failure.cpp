/*
 * Desktop-only test: correctness gate for the allocation-failure handling
 * described in src/StdAllocator.h's file comment - an out-of-memory
 * condition must surface as TinyH264Decoder::Status::kAllocationError /
 * TinyH264Encoder::encodeFrame() returning 0, not crash the process - see
 * common/h264_buffer.h's Buffer<T> and decoder/h264_decoder.h's
 * DecodeStatus::kAllocationError for the mechanics. This is the one test
 * in the suite that plugs in a custom (non-default, non-PSRAM) Allocator,
 * specifically to make an allocation fail on demand - covering both
 * Frame's picture buffers and MbInfoTable's per-macroblock metadata
 * (common/h264_mb_info.h), since both are backed by the same
 * MemoryResource, itself built from TinyH264Decoder/TinyH264Encoder's
 * own Allocator template argument (so a PSRAM allocator moves both off
 * internal SRAM together, not just the picture buffers).
 */
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../../src/TinyH264Decoder.h"
#include "../../src/TinyH264Encoder.h"

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

#define CHECK(cond, msg)                     \
  do {                                       \
    if (!(cond)) {                           \
      printf("FAIL: %s\n", msg);             \
      failures++;                            \
    }                                        \
  } while (0)

/**
 * A StdAllocator-shaped allocator (see StdAllocator.h) whose allocate()
 * fails on demand - controlled via static counters rather than instance
 * state, since std::vector/Buffer may rebind or copy an Allocator
 * instance at will and this project's Allocator concept requires
 * statelessness (see StdAllocator's own comment). Follows the same
 * "return nullptr, never throw" no-crash contract as the real
 * allocators - that contract is exactly what's under test here.
 */
template <typename T>
struct FailingAllocator {
  using value_type = T;

  static int callCount;
  static int failAtCall;  // 0 = never fail; N = fail the Nth allocate() call (1-indexed)

  FailingAllocator() noexcept = default;
  template <typename U>
  FailingAllocator(const FailingAllocator<U>&) noexcept {}

  T* allocate(std::size_t n) noexcept {
    callCount++;
    if (failAtCall != 0 && callCount == failAtCall) return nullptr;
    return static_cast<T*>(malloc(n * sizeof(T)));
  }
  void deallocate(T* p, std::size_t) noexcept { free(p); }

  template <typename U>
  bool operator==(const FailingAllocator<U>&) const noexcept {
    return true;
  }
  template <typename U>
  bool operator!=(const FailingAllocator<U>&) const noexcept {
    return false;
  }
};

template <typename T>
int FailingAllocator<T>::callCount = 0;
template <typename T>
int FailingAllocator<T>::failAtCall = 0;

static void resetFailure(int failAtCall) {
  FailingAllocator<uint8_t>::callCount = 0;
  FailingAllocator<uint8_t>::failAtCall = failAtCall;
}

int main() {
  auto stream = readFile("assets/qcif_test.264");
  const int W = 176, H = 144;

  // --- Decoder: the very first allocation write() triggers fails - that's
  //     MbInfoTable's sliceId_ buffer specifically, since decodeSlice()
  //     (h264_decoder.h) allocates the metadata table before sizing
  //     curFrame_ - see the block below for curFrame_'s own turn. ---
  {
    using FailingDecoder = TinyH264Decoder<FailingAllocator<uint8_t>>;
    resetFailure(1);  // MbInfoTable::sliceId_
    FailingDecoder dec;
    auto status = dec.write(stream.data(), stream.size());
    CHECK(status == FailingDecoder::Status::kAllocationError,
          "decoder: MbInfoTable allocation failure not reported as kAllocationError");
    CHECK(dec.lastStatus() == FailingDecoder::Status::kAllocationError,
          "decoder: lastStatus() doesn't reflect kAllocationError");
    CHECK(dec.hasError(), "decoder: hasError() false after kAllocationError");
  }

  // --- Decoder: curFrame_'s own allocation fails (the 3rd call via
  //     write() - MbInfoTable's sliceId_/mb_ are calls 1-2 and succeed
  //     first, see above) - a second, independent allocation surface
  //     reachable through the exact same public write() path. ---
  {
    using FailingDecoder = TinyH264Decoder<FailingAllocator<uint8_t>>;
    resetFailure(3);  // curFrame_::dataY, the 1st of its 3 planes
    FailingDecoder dec;
    auto status = dec.write(stream.data(), stream.size());
    CHECK(status == FailingDecoder::Status::kAllocationError,
          "decoder: curFrame_ allocation failure not reported as kAllocationError");
    CHECK(dec.hasError(), "decoder: hasError() false after kAllocationError");
  }

  // --- Decoder: begin()'s eager-allocation path reports the same failure,
  //     with no bitstream data involved at all ---
  {
    using FailingDecoder = TinyH264Decoder<FailingAllocator<uint8_t>>;
    resetFailure(1);
    FailingDecoder dec;
    bool ok = dec.begin();
    CHECK(!ok, "decoder: begin() returned true despite a failing allocator");
    CHECK(dec.lastStatus() == FailingDecoder::Status::kAllocationError,
          "decoder: begin() failure not reflected in lastStatus()");
  }

  // --- Decoder: a later allocation (not the first) fails - and once the
  //     allocator stops failing, begin() recovers rather than staying
  //     wedged in a half-allocated state (Frame::ensureAllocated()
  //     releases whatever partially succeeded on failure - see
  //     common/h264_frame.h). ---
  {
    using FailingDecoder = TinyH264Decoder<FailingAllocator<uint8_t>>;
    resetFailure(2);  // dataU, the 2nd of curFrame_'s 3 planes
    FailingDecoder dec;
    bool ok = dec.begin();
    CHECK(!ok, "decoder: begin() returned true despite a 2nd-allocation failure");
    CHECK(dec.hasError(), "decoder: hasError() false after a partial allocation failure");

    FailingAllocator<uint8_t>::failAtCall = 0;  // let allocation succeed from here on
    ok = dec.begin();
    CHECK(ok, "decoder: begin() didn't recover once allocation started succeeding again");
    CHECK(!dec.hasError(), "decoder: hasError() still true after a successful begin()");
  }

  // --- Decoder: a never-failing allocator still decodes normally through
  //     the exact same public API - the FailingAllocator plumbing itself
  //     doesn't change correct-path behavior (pixel-level correctness is
  //     already covered by test_decode_iframe.cpp; this just checks a
  //     frame actually comes through). ---
  {
    using FailingDecoder = TinyH264Decoder<FailingAllocator<uint8_t>>;
    resetFailure(0);  // never fail
    FailingDecoder dec;
    int frameCount = 0;
    dec.setCallback([](FailingDecoder&, void* userData) { (*(int*)userData)++; },
                     &frameCount);
    auto status = dec.write(stream.data(), stream.size());
    CHECK(!dec.hasError(), "decoder: hasError() true with a never-failing allocator");
    CHECK(status == FailingDecoder::Status::kFrameReady ||
              status == FailingDecoder::Status::kNeedMoreData,
          "decoder: unexpected status with a never-failing allocator");
    CHECK(frameCount > 0, "decoder: no frames decoded with a never-failing allocator");
  }

  // --- Encoder: the very first allocation encodeIFrame() triggers fails -
  //     frame_::dataY (encodeIFrame() sizes frame_ before mbInfo_, the
  //     opposite order from the decoder's decodeSlice() above - see the
  //     next block for mbInfo_'s own turn). encodeFrame() must report it
  //     the same way a too-small dst buffer or invalid width/height
  //     would (return 0), not crash. ---
  {
    using FailingEncoder = TinyH264Encoder<FailingAllocator<uint8_t>>;
    resetFailure(1);  // frame_::dataY
    FailingEncoder enc;
    std::vector<uint8_t> bitstream(200000);
    std::vector<uint8_t> srcY(W * H, 128), srcU((W / 2) * (H / 2), 128),
        srcV((W / 2) * (H / 2), 128);
    enc.setSize(W, H);
    enc.setQp(26);
    size_t n = enc.encodeFrame(srcY.data(), srcU.data(), srcV.data(),
                                bitstream.data(), bitstream.size());
    CHECK(n == 0, "encoder: encodeFrame() succeeded despite a failing allocator");
  }

  // --- Encoder: mbInfo_'s own allocation fails (the 4th call via
  //     encodeFrame() - frame_'s 3 planes are calls 1-3 and succeed
  //     first, see above) - a second, independent allocation surface
  //     reachable through the exact same public encodeFrame() path. ---
  {
    using FailingEncoder = TinyH264Encoder<FailingAllocator<uint8_t>>;
    resetFailure(4);  // mbInfo_::sliceId_
    FailingEncoder enc;
    std::vector<uint8_t> bitstream(200000);
    std::vector<uint8_t> srcY(W * H, 128), srcU((W / 2) * (H / 2), 128),
        srcV((W / 2) * (H / 2), 128);
    enc.setSize(W, H);
    enc.setQp(26);
    size_t n = enc.encodeFrame(srcY.data(), srcU.data(), srcV.data(),
                                bitstream.data(), bitstream.size());
    CHECK(n == 0, "encoder: encodeFrame() succeeded despite mbInfo_'s allocator failing");
  }

  // --- Encoder: begin() reports the same failure directly ---
  {
    using FailingEncoder = TinyH264Encoder<FailingAllocator<uint8_t>>;
    resetFailure(1);
    FailingEncoder enc;
    bool ok = enc.begin(/*reserveColorConversionScratch=*/true);
    CHECK(!ok, "encoder: begin() returned true despite a failing allocator");
  }

  // --- Encoder: a never-failing allocator still encodes normally ---
  {
    using FailingEncoder = TinyH264Encoder<FailingAllocator<uint8_t>>;
    resetFailure(0);  // never fail
    FailingEncoder enc;
    std::vector<uint8_t> bitstream(200000);
    std::vector<uint8_t> srcY(W * H, 128), srcU((W / 2) * (H / 2), 128),
        srcV((W / 2) * (H / 2), 128);
    enc.setSize(W, H);
    enc.setQp(26);
    size_t n = enc.encodeFrame(srcY.data(), srcU.data(), srcV.data(),
                                bitstream.data(), bitstream.size());
    CHECK(n > 0, "encoder: encodeFrame() failed with a never-failing allocator");
  }

  printf("test_alloc_failure: failures=%d\n", failures);
  return failures == 0 ? 0 : 1;
}
