/*
 * Desktop-only test: decode ALL frames (1 IDR + 9 P) of the real QCIF
 * baseline/CAVLC stream and compare each pixel-for-pixel against ffmpeg's
 * own decode of the same stream. This is the oracle for P-slice support:
 * motion vector prediction, motion compensation, and reference-frame
 * management all have to be correct simultaneously for this to match.
 */
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../../src/common/MemoryResource.h"
#include "../../src/StdAllocator.h"
#include "../../src/decoder/h264_decoder.h"

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

static int compareFrame(int frameIdx, const Frame& f, const uint8_t* ref) {
  const uint8_t* refY = ref;
  const uint8_t* refU = refY + 176 * 144;
  const uint8_t* refV = refU + 88 * 72;

  int mismatchY = 0, mismatchU = 0, mismatchV = 0;
  int firstX = -1, firstY = -1, firstGot = -1, firstWant = -1;
  const char* firstPlane = "";
  for (int y = 0; y < 144; y++)
    for (int x = 0; x < 176; x++) {
      uint8_t a = f.y()[y * f.strideY + x], b = refY[y * 176 + x];
      if (a != b) {
        if (mismatchY == 0 && mismatchU == 0 && mismatchV == 0) {
          firstX = x; firstY = y; firstGot = a; firstWant = b; firstPlane = "Y";
        }
        mismatchY++;
      }
    }
  for (int y = 0; y < 72; y++)
    for (int x = 0; x < 88; x++) {
      uint8_t a = f.u()[y * f.strideC + x], b = refU[y * 88 + x];
      if (a != b) {
        if (mismatchY == 0 && mismatchU == 0) {
          firstX = x; firstY = y; firstGot = a; firstWant = b; firstPlane = "U";
        }
        mismatchU++;
      }
      uint8_t a2 = f.v()[y * f.strideC + x], b2 = refV[y * 88 + x];
      if (a2 != b2) {
        if (mismatchY == 0 && mismatchU == 0 && mismatchV == 0) {
          firstX = x; firstY = y; firstGot = a2; firstWant = b2; firstPlane = "V";
        }
        mismatchV++;
      }
    }

  int total = mismatchY + mismatchU + mismatchV;
  if (total == 0) {
    printf("frame %d: PIXEL-EXACT MATCH\n", frameIdx);
  } else {
    printf("frame %d: MISMATCH Y=%d U=%d V=%d (first in %s at x=%d,y=%d "
           "got=%d want=%d, mb=(%d,%d))\n",
           frameIdx, mismatchY, mismatchU, mismatchV, firstPlane, firstX,
           firstY, firstGot, firstWant, firstX / 16, firstY / 16);
  }
  return total;
}

int main() {
  setbuf(stdout, nullptr);
  auto stream = readFile("assets/qcif_nodbf.264");
  auto ref = readFile("assets/all_frames_nodbf_ref.yuv");

  AllocatorMemoryResource<StdAllocator<uint8_t>> mr;
  SoftwareDecoder decoder(mr);
  decoder.setInput(stream.data(), stream.size());

  int frameIdx = 0;
  int totalMismatch = 0;
  int framesOk = 0;
  int nalBudget = 200;  // safety cap; stream has well under this many NALs
  while (frameIdx < 10 && nalBudget-- > 0) {
    DecodeStatus status = decoder.next();
    if (status == DecodeStatus::kError) {
      printf("decode error at frame %d\n", frameIdx);
      return 1;
    }
    if (status == DecodeStatus::kUnsupported) {
      printf("unsupported stream feature at frame %d\n", frameIdx);
      return 1;
    }
    if (status == DecodeStatus::kOk) {
      const uint8_t* refFrame = ref.data() + (size_t)frameIdx * 38016;
      int mism = compareFrame(frameIdx, decoder.frame(), refFrame);
      if (mism == 0) framesOk++;
      totalMismatch += mism;
      frameIdx++;
    }
  }
  if (frameIdx < 10) {
    printf("ran out of NALs before decoding 10 frames (got %d)\n", frameIdx);
    return 1;
  }

  printf("=== %d/%d frames pixel-exact, total mismatched pixels=%d ===\n",
         framesOk, frameIdx, totalMismatch);
  return (framesOk == frameIdx) ? 0 : 1;
}
