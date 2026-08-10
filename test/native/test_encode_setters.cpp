// Desktop-only test: verifies the Encoder/TinyH264Encoder setSize()/
// setStride()/setPackedStride()/setQp() setters that back encodeFrame()
// and its color-format overloads (encoder/h264_encoder.h) - the only
// public encode entry points now (encodeIFrame()/encodePFrame() and all
// explicit-width/height/stride/qp overloads were removed once every
// caller had a setter-based replacement).
//
// Checks, against a real 10-frame QCIF motion sequence
// (assets/all_frames_ref.yuv, the same oracle test_encode_pframe.cpp/
// test_encode_autoframe.cpp/test_lifecycle.cpp use):
// 1. setSize() takes effect identically whether called before or after
//    begin() (begin() deliberately doesn't reset width_/height_ - see
//    its own comment) - encoding the same sequence both ways produces
//    byte-identical output.
// 2. setPackedStride() actually takes effect for encodeFrameRgb888() -
//    encoding a deliberately padded RGB888 buffer via the override
//    matches encoding the same content unpadded with the default stride.
// 3. setStride() actually takes effect for the plain-YUV encodeFrame() -
//    same idea as check 2, for a deliberately padded Y/C buffer.
// 4. The Encoder(width, height, keyframeInterval) constructor produces
//    byte-identical output to a default-constructed Encoder with
//    setSize()/setKeyframeInterval() called separately afterward -
//    pinning down that the constructor is purely a convenience wrapper
//    around those two setters, not a new code path.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
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

int main() {
  auto allFrames = readFile("assets/all_frames_ref.yuv");
  int numFrames = (int)(allFrames.size() / FRAME_SIZE);
  if (numFrames < 4 || allFrames.size() % FRAME_SIZE != 0) {
    printf("FAIL: unexpected asset size %zu\n", allFrames.size());
    return 1;
  }
  const int qp = 26;

  // --- 1. setSize() before vs. after begin() ---
  std::vector<uint8_t> bsBefore(2000000), bsAfter(2000000);
  size_t totalBefore = 0, totalAfter = 0;
  {
    TinyH264Encoder<> encBefore;
    encBefore.setSize(W, H);  // before begin() - must still take effect
    encBefore.begin();
    encBefore.setQp(qp);

    TinyH264Encoder<> encAfter;
    encAfter.begin();
    encAfter.setSize(W, H);  // after begin()
    encAfter.setQp(qp);

    for (int i = 0; i < numFrames; i++) {
      const uint8_t* f = allFrames.data() + (size_t)i * FRAME_SIZE;
      const uint8_t* srcY = f;
      const uint8_t* srcU = f + (size_t)W * H;
      const uint8_t* srcV = srcU + (size_t)(W / 2) * (H / 2);

      size_t nB = encBefore.encodeFrame(srcY, srcU, srcV,
                                         bsBefore.data() + totalBefore,
                                         bsBefore.size() - totalBefore);
      size_t nA = encAfter.encodeFrame(srcY, srcU, srcV,
                                        bsAfter.data() + totalAfter,
                                        bsAfter.size() - totalAfter);
      if (nB == 0 || nA == 0) {
        printf("FAIL: frame %d encode returned 0 (before=%zu after=%zu)\n", i,
               nB, nA);
        failures++;
        break;
      }
      totalBefore += nB;
      totalAfter += nA;
    }
  }
  if (totalBefore == 0 || totalBefore != totalAfter ||
      memcmp(bsBefore.data(), bsAfter.data(), totalBefore) != 0) {
    printf("FAIL: setSize()-before-begin() output (%zu bytes) differs from "
           "setSize()-after-begin() output (%zu bytes)\n",
           totalBefore, totalAfter);
    failures++;
  } else {
    printf("setSize() takes effect identically before or after begin() "
           "(%zu bytes, %d frames)\n",
           totalBefore, numFrames);
  }

  // --- 2. setPackedStride() override actually takes effect (RGB888) ---
  {
    auto rgb = readFile("assets/frame0_rgb24.raw");
    if (rgb.size() != (size_t)W * H * 3) {
      printf("FAIL: unexpected frame0_rgb24.raw size %zu\n", rgb.size());
      failures++;
    } else {
      // Deliberately padded copy: rgbStride = W*3 + 48 bytes, extra
      // columns filled with garbage that must never be read if the
      // stride override works correctly.
      const int padStride = W * 3 + 48;
      std::vector<uint8_t> padded((size_t)padStride * H, 0xAA);
      for (int y = 0; y < H; y++) {
        memcpy(padded.data() + (size_t)y * padStride, rgb.data() + (size_t)y * W * 3,
               W * 3);
      }

      TinyH264Encoder<> encUnpadded, encPadded;
      encUnpadded.setSize(W, H);
      encUnpadded.setQp(qp);
      encPadded.setSize(W, H);
      encPadded.setPackedStride(padStride);
      encPadded.setQp(qp);

      std::vector<uint8_t> bsU(200000), bsP(200000);
      size_t nU =
          encUnpadded.encodeFrameRgb888(rgb.data(), bsU.data(), bsU.size());
      size_t nP =
          encPadded.encodeFrameRgb888(padded.data(), bsP.data(), bsP.size());
      if (nU == 0 || nP == 0 || nU != nP ||
          memcmp(bsU.data(), bsP.data(), nU) != 0) {
        printf("FAIL: setPackedStride()-overridden padded RGB888 encode "
               "(%zu bytes) doesn't match the unpadded default-stride "
               "encode (%zu bytes)\n",
               nP, nU);
        failures++;
      } else {
        printf("setPackedStride() override correctly reads a padded RGB888 "
               "source buffer (%zu bytes, matches unpadded reference)\n",
               nP);
      }
    }
  }

  // --- 3. setStride() override actually takes effect (plain YUV) ---
  {
    // Deliberately padded copy of frame 0: strideY = W + 32, strideC =
    // W/2 + 16, extra columns filled with garbage that must never be
    // read if the stride override works correctly.
    const int padStrideY = W + 32, padStrideC = W / 2 + 16;
    std::vector<uint8_t> paddedY((size_t)padStrideY * H, 0xAA);
    std::vector<uint8_t> paddedU((size_t)padStrideC * (H / 2), 0xAA);
    std::vector<uint8_t> paddedV((size_t)padStrideC * (H / 2), 0xAA);
    const uint8_t* f0 = allFrames.data();
    const uint8_t* srcY0 = f0;
    const uint8_t* srcU0 = f0 + (size_t)W * H;
    const uint8_t* srcV0 = srcU0 + (size_t)(W / 2) * (H / 2);
    for (int y = 0; y < H; y++) {
      memcpy(paddedY.data() + (size_t)y * padStrideY, srcY0 + (size_t)y * W, W);
    }
    for (int y = 0; y < H / 2; y++) {
      memcpy(paddedU.data() + (size_t)y * padStrideC, srcU0 + (size_t)y * (W / 2),
             W / 2);
      memcpy(paddedV.data() + (size_t)y * padStrideC, srcV0 + (size_t)y * (W / 2),
             W / 2);
    }

    TinyH264Encoder<> encUnpadded, encPadded;
    encUnpadded.setSize(W, H);
    encUnpadded.setQp(qp);
    encPadded.setSize(W, H);
    encPadded.setStride(padStrideY, padStrideC);
    encPadded.setQp(qp);

    std::vector<uint8_t> bsU(200000), bsP(200000);
    size_t nU = encUnpadded.encodeFrame(srcY0, srcU0, srcV0, bsU.data(),
                                         bsU.size());
    size_t nP = encPadded.encodeFrame(paddedY.data(), paddedU.data(),
                                       paddedV.data(), bsP.data(), bsP.size());
    if (nU == 0 || nP == 0 || nU != nP || memcmp(bsU.data(), bsP.data(), nU) != 0) {
      printf("FAIL: setStride()-overridden padded encode (%zu bytes) doesn't "
             "match the unpadded default-stride encode (%zu bytes)\n",
             nP, nU);
      failures++;
    } else {
      printf("setStride() override correctly reads a padded source buffer "
             "(%zu bytes, matches unpadded reference)\n",
             nP);
    }
  }

  // --- 4. Encoder(width, height, keyframeInterval) constructor ---
  {
    const int kInterval = 3;
    std::vector<uint8_t> bsCtor(2000000), bsSetters(2000000);
    size_t totalCtor = 0, totalSetters = 0;
    {
      TinyH264Encoder<> encCtor(W, H, kInterval);
      TinyH264Encoder<> encSetters;
      encSetters.setSize(W, H);
      encSetters.setKeyframeInterval(kInterval);
      encCtor.setQp(qp);
      encSetters.setQp(qp);

      for (int i = 0; i < numFrames; i++) {
        const uint8_t* f = allFrames.data() + (size_t)i * FRAME_SIZE;
        const uint8_t* srcY = f;
        const uint8_t* srcU = f + (size_t)W * H;
        const uint8_t* srcV = srcU + (size_t)(W / 2) * (H / 2);

        size_t nC = encCtor.encodeFrame(srcY, srcU, srcV,
                                         bsCtor.data() + totalCtor,
                                         bsCtor.size() - totalCtor);
        size_t nS = encSetters.encodeFrame(srcY, srcU, srcV,
                                            bsSetters.data() + totalSetters,
                                            bsSetters.size() - totalSetters);
        if (nC == 0 || nS == 0) {
          printf("FAIL: frame %d encode returned 0 (ctor=%zu setters=%zu)\n",
                 i, nC, nS);
          failures++;
          break;
        }
        totalCtor += nC;
        totalSetters += nS;
      }
    }
    if (totalCtor == 0 || totalCtor != totalSetters ||
        memcmp(bsCtor.data(), bsSetters.data(), totalCtor) != 0) {
      printf("FAIL: constructor-configured output (%zu bytes) differs from "
             "setter-configured output (%zu bytes)\n",
             totalCtor, totalSetters);
      failures++;
    } else {
      printf("Encoder(width, height, keyframeInterval) constructor matches "
             "setSize()+setKeyframeInterval() byte-for-byte (%zu bytes, %d "
             "frames)\n",
             totalCtor, numFrames);
    }
  }

  if (failures == 0) {
    printf("test_encode_setters: all checks passed\n");
    return 0;
  }
  printf("test_encode_setters: %d failure(s)\n", failures);
  return 1;
}
