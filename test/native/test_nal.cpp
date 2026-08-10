// Desktop-only test: parse a real ffmpeg-encoded baseline/CAVLC QCIF stream
// and check the NAL unit type sequence matches expectations (SPS, PPS, then
// one IDR slice followed by 9 non-IDR (P) slices).
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../../src/decoder/h264_nal.h"

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
  fread(buf.data(), 1, sz, f);
  fclose(f);
  return buf;
}

int main() {
  setbuf(stdout, nullptr);
  auto data = readFile("assets/qcif_test.264");

  static uint8_t scratch[H264_MAX_NAL_SIZE];
  NalReader reader(scratch, sizeof(scratch));
  reader.reset(data.data(), data.size());

  int spsCount = 0, ppsCount = 0, idrCount = 0, pCount = 0, seiCount = 0,
      total = 0;
  NalUnit nal;
  while (reader.next(&nal)) {
    total++;
    switch (nal.type) {
      case kNalSps: spsCount++; break;
      case kNalPps: ppsCount++; break;
      case kNalSliceIdr: idrCount++; break;
      case kNalSliceNonIdr: pCount++; break;
      case kNalSei: seiCount++; break;
      default: break;
    }
    printf("NAL #%d: type=%d refIdc=%d rbspSize=%zu\n", total, nal.type,
           nal.refIdc, nal.rbspSize);
    assert(nal.rbspSize > 0);
    // Sanity: emulation prevention removal never produces the illegal
    // 00 00 03 sequence's leftover, and shouldn't have shrunk below a
    // trivial minimum for slice/SPS/PPS NALs.
  }

  printf("sps=%d pps=%d idr=%d p=%d sei=%d total=%d\n", spsCount, ppsCount,
         idrCount, pCount, seiCount, total);

  assert(spsCount == 1);
  assert(ppsCount == 1);
  assert(idrCount == 1);
  assert(pCount == 9);
  assert(total == spsCount + ppsCount + idrCount + pCount + seiCount);

  printf("test_nal: all tests passed\n");
  return 0;
}
