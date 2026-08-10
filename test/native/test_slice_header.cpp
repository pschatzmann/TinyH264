/*
 * Desktop-only test: parse slice headers from the real QCIF baseline/CAVLC
 * stream and sanity-check them (first slice of each frame starts at MB 0,
 * IDR NAL -> slice_type I, non-IDR NAL -> slice_type P, QP in a sane range).
 */
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../../src/decoder/h264_nal.h"
#include "../../src/decoder/h264_slice_header.h"
#include "../../src/decoder/h264_sps_pps.h"

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
  setbuf(stdout, nullptr);
  auto data = readFile("assets/qcif_test.264");

  static uint8_t scratch[H264_MAX_NAL_SIZE];
  NalReader reader(scratch, sizeof(scratch));
  reader.reset(data.data(), data.size());

  Sps sps;
  Pps pps;
  bool haveSps = false, havePps = false;
  int idrSlices = 0, pSlices = 0;
  NalUnit nal;
  while (reader.next(&nal)) {
    if (nal.type == kNalSps) {
      BitReader br(nal.rbsp, nal.rbspSize);
      assert(parseSps(br, &sps) && sps.valid);
      haveSps = true;
    } else if (nal.type == kNalPps) {
      BitReader br(nal.rbsp, nal.rbspSize);
      assert(parsePps(br, &pps) && pps.valid);
      havePps = true;
    } else if (nal.type == kNalSliceIdr || nal.type == kNalSliceNonIdr) {
      assert(haveSps && havePps);
      BitReader br(nal.rbsp, nal.rbspSize);
      SliceHeader sh;
      bool ok = parseSliceHeader(br, nal, sps, pps, &sh);
      assert(ok);
      printf(
          "slice: idr=%d firstMb=%u type=%u frameNum=%u qp=%d bitsUsed=%zu "
          "unsupported=%d\n",
          sh.isIdr, sh.firstMbInSlice, sh.sliceType, sh.frameNum, sh.sliceQp,
          br.bitsConsumed(), sh.unsupported);
      assert(sh.valid && !sh.unsupported);
      assert(sh.firstMbInSlice == 0);
      // QP should be in the legal 0..51 range for 8-bit baseline.
      assert(sh.sliceQp >= 0 && sh.sliceQp <= 51);
      if (nal.type == kNalSliceIdr) {
        assert(sh.sliceType == kSliceI);
        idrSlices++;
      } else {
        assert(sh.sliceType == kSliceP);
        pSlices++;
      }
    }
  }

  printf("idrSlices=%d pSlices=%d\n", idrSlices, pSlices);
  assert(idrSlices == 1);
  assert(pSlices == 9);

  printf("test_slice_header: all tests passed\n");
  return 0;
}
