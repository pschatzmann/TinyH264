/*
 * Desktop-only test: parse the real SPS/PPS from the ffmpeg-encoded QCIF
 * baseline/CAVLC stream and check derived values against ground truth from
 * ffprobe (176x144, Constrained Baseline profile == profile_idc 66).
 */
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../../src/decoder/h264_nal.h"
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
  bool gotSps = false, gotPps = false;
  NalUnit nal;
  while (reader.next(&nal)) {
    if (nal.type == kNalSps) {
      BitReader br(nal.rbsp, nal.rbspSize);
      assert(parseSps(br, &sps));
      gotSps = true;
    } else if (nal.type == kNalPps) {
      BitReader br(nal.rbsp, nal.rbspSize);
      assert(parsePps(br, &pps));
      gotPps = true;
    }
  }
  assert(gotSps && gotPps);

  printf("SPS: profile=%d level=%d id=%u valid=%d unsupported=%d\n",
         sps.profileIdc, sps.levelIdc, sps.id, sps.valid, sps.unsupported);
  printf("     picWidthInMbs=%u picHeightInMbs=%u coded=%ux%u display=%ux%u\n",
         sps.picWidthInMbs, sps.picHeightInMbs, sps.codedWidth,
         sps.codedHeight, sps.displayWidth, sps.displayHeight);
  printf("     frameMbsOnly=%d log2MaxFrameNumMinus4=%u picOrderCntType=%u\n",
         sps.frameMbsOnlyFlag, sps.log2MaxFrameNumMinus4, sps.picOrderCntType);

  printf("PPS: id=%u spsId=%u entropyCabac=%d valid=%d unsupported=%d\n",
         pps.id, pps.spsId, pps.entropyCodingModeFlag, pps.valid,
         pps.unsupported);
  printf("     initQp=%d chromaQpOffset=%d constrainedIntra=%d\n",
         pps.picInitQpMinus26 + 26, pps.chromaQpIndexOffset,
         pps.constrainedIntraPredFlag);

  assert(sps.valid && !sps.unsupported);
  assert(sps.profileIdc == 66);  // Constrained Baseline
  assert(sps.codedWidth == 176);
  assert(sps.codedHeight == 144);
  assert(sps.displayWidth == 176);
  assert(sps.displayHeight == 144);
  assert(sps.frameMbsOnlyFlag);

  assert(pps.valid && !pps.unsupported);
  assert(pps.spsId == sps.id);
  assert(pps.entropyCodingModeFlag == false);  // CAVLC

  printf("test_sps_pps: all tests passed\n");
  return 0;
}
