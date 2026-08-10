// Desktop-only test: verifies TinyH264Encoder::encodeFrame() (the only
// public encode entry point - automatic I/P dispatch, encoder/
// h264_encoder.h's Encoder::encodeFrame()/needsKeyframe()) against a
// real 10-frame QCIF motion sequence (assets/all_frames_ref.yuv, the
// same oracle test_encode_pframe.cpp uses). Checks:
// 1. Encoding the whole sequence via encodeFrame() produces exactly one
//    IDR (picture 0, no reference yet) followed by numFrames-1 non-IDR
//    P-slices - checked by real NAL type, not by guessing from encoded
//    size - and the whole stream self-decodes without error.
// 2. setKeyframeInterval() lands keyframes at *exactly* the right
//    picture indices (GOP-size semantics: interval N -> keyframes at
//    0, N, 2N, ... - not N+1, an off-by-one this project's own
//    development caught by testing this exact scenario) - checked by
//    NAL type, not by guessing from encoded size.
// 3. The keyframe-interval-encoded stream still decodes cleanly.
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

static const int W = 176, H = 144;
static const size_t FRAME_SIZE = (size_t)W * H * 3 / 2;
int failures = 0;

/// True if `nalPayload` (the byte right after a 4-byte Annex-B start
/// code) is a kNalSliceIdr (5) NAL - i.e. this picture is a real
/// keyframe, not just "encoded a lot of bytes" (which a busy P-frame
/// could also do, an unreliable proxy this test deliberately avoids).
bool isIdrNal(const uint8_t* nalPayload) {
  return (nalPayload[0] & 0x1F) == kNalSliceIdr;
}

/// Scans a multi-NAL Annex-B buffer and returns the byte offset of the
/// Nth VCL (slice) NAL's start code - SPS/PPS NALs are skipped, so
/// index 0 is always the first picture's slice, index 1 the second
/// picture's, etc., regardless of how many SPS/PPS NALs preceded it.
size_t findNthSliceNal(const std::vector<uint8_t>& bs, int n) {
  int seen = -1;
  for (size_t i = 0; i + 4 < bs.size(); i++) {
    if (bs[i] == 0 && bs[i + 1] == 0 && bs[i + 2] == 0 && bs[i + 3] == 1) {
      uint8_t nalType = bs[i + 4] & 0x1F;
      if (nalType == kNalSliceIdr || nalType == kNalSliceNonIdr) {
        seen++;
        if (seen == n) return i;
      }
    }
  }
  return (size_t)-1;
}

int main() {
  auto allFrames = readFile("assets/all_frames_ref.yuv");
  int numFrames = (int)(allFrames.size() / FRAME_SIZE);
  if (numFrames < 4 || allFrames.size() % FRAME_SIZE != 0) {
    printf("FAIL: unexpected asset size %zu\n", allFrames.size());
    return 1;
  }
  const int qp = 26;

  // --- 1. encodeFrame()-only: exactly 1 IDR (picture 0) + clean decode ---
  std::vector<uint8_t> bsAuto(2000000);
  size_t totalAuto = 0;
  {
    TinyH264Encoder<> encAuto;
    encAuto.setSize(W, H);
    encAuto.setQp(qp);
    for (int i = 0; i < numFrames; i++) {
      const uint8_t* f = allFrames.data() + (size_t)i * FRAME_SIZE;
      const uint8_t* srcY = f;
      const uint8_t* srcU = f + (size_t)W * H;
      const uint8_t* srcV = srcU + (size_t)(W / 2) * (H / 2);

      size_t nAuto = encAuto.encodeFrame(srcY, srcU, srcV,
                                          bsAuto.data() + totalAuto,
                                          bsAuto.size() - totalAuto);
      if (nAuto == 0) {
        printf("FAIL: frame %d encode returned 0\n", i);
        failures++;
        break;
      }
      totalAuto += nAuto;
    }
  }
  bool autoNalTypesOk = true;
  for (int i = 0; i < numFrames; i++) {
    size_t off = findNthSliceNal(bsAuto, i);
    if (off == (size_t)-1) {
      printf("FAIL: couldn't find slice NAL for picture %d (no-interval "
             "stream)\n",
             i);
      failures++;
      autoNalTypesOk = false;
      continue;
    }
    bool isIdr = isIdrNal(&bsAuto[off + 4]);
    bool wantIdr = (i == 0);
    if (isIdr != wantIdr) {
      printf("FAIL: picture %d isIdr=%d, expected %d (no keyframe interval "
             "configured)\n",
             i, isIdr, wantIdr);
      failures++;
      autoNalTypesOk = false;
    }
  }
  TinyH264Decoder<> decAuto;
  int autoFrameCount = 0;
  decAuto.setCallback([](TinyH264Decoder<>&, void* ud) { (*(int*)ud)++; },
                       &autoFrameCount);
  decAuto.write(bsAuto.data(), totalAuto);
  if (decAuto.hasError() || autoFrameCount != numFrames) {
    printf("FAIL: encodeFrame()-only stream self-decode failed "
           "(hasError=%d frames=%d/%d)\n",
           decAuto.hasError(), autoFrameCount, numFrames);
    failures++;
  } else if (autoNalTypesOk) {
    printf("encodeFrame() produced exactly 1 IDR (picture 0) + %d P-slices "
           "and self-decoded cleanly (%zu bytes, %d frames)\n",
           numFrames - 1, totalAuto, numFrames);
  }

  // --- 2/3. Keyframe interval: exact spacing + clean decode ---
  const int kInterval = 3;
  std::vector<uint8_t> bsKf(2000000);
  size_t totalKf = 0;
  {
    TinyH264Encoder<> enc;
    enc.setSize(W, H);
    enc.setQp(qp);
    enc.setKeyframeInterval(kInterval);
    for (int i = 0; i < numFrames; i++) {
      const uint8_t* f = allFrames.data() + (size_t)i * FRAME_SIZE;
      const uint8_t* srcY = f;
      const uint8_t* srcU = f + (size_t)W * H;
      const uint8_t* srcV = srcU + (size_t)(W / 2) * (H / 2);
      size_t n = enc.encodeFrame(srcY, srcU, srcV, bsKf.data() + totalKf,
                                  bsKf.size() - totalKf);
      if (n == 0) {
        printf("FAIL: keyframe-interval frame %d encode returned 0\n", i);
        failures++;
        break;
      }
      totalKf += n;
    }
  }
  for (int i = 0; i < numFrames; i++) {
    size_t off = findNthSliceNal(bsKf, i);
    if (off == (size_t)-1) {
      printf("FAIL: couldn't find slice NAL for picture %d\n", i);
      failures++;
      continue;
    }
    bool isIdr = isIdrNal(&bsKf[off + 4]);
    bool wantIdr = (i % kInterval) == 0;
    if (isIdr != wantIdr) {
      printf("FAIL: picture %d isIdr=%d, expected %d (keyframeInterval=%d)\n",
             i, isIdr, wantIdr, kInterval);
      failures++;
    }
  }
  if (failures == 0) {
    printf("keyframeInterval=%d landed exactly at pictures 0, %d, %d, ... "
           "as expected\n",
           kInterval, kInterval, 2 * kInterval);
  }

  TinyH264Decoder<> dec;
  int frameCount = 0;
  dec.setCallback([](TinyH264Decoder<>&, void* ud) { (*(int*)ud)++; },
                   &frameCount);
  dec.write(bsKf.data(), totalKf);
  if (dec.hasError() || frameCount != numFrames) {
    printf("FAIL: keyframe-interval stream self-decode failed "
           "(hasError=%d frames=%d/%d)\n",
           dec.hasError(), frameCount, numFrames);
    failures++;
  }

  if (failures == 0) {
    printf("test_encode_autoframe: all checks passed\n");
    return 0;
  }
  printf("test_encode_autoframe: %d failure(s)\n", failures);
  return 1;
}
