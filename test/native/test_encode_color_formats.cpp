// Desktop-only test: exercises TinyH264Encoder's non-YUV420-plane
// encodeIFrame*() overloads (RGB888/RGB666/RGB565/YUV422) against a real
// RGB24 image (assets/frame0_rgb24.raw - ffmpeg's own `-pix_fmt yuv420p
// -> -pix_fmt rgb24` conversion of the same frame0_ref.yuv oracle the
// other decode tests use, so this is genuine image content, not a
// synthetic pattern). Each format is packed from that same RGB source
// (RGB565/RGB666 via this library's own documented bit conventions,
// matching TinyH264Decoder::toRGB565()/toRGB666(); YUV422 by duplicating
// each YUV420 chroma row across the two rows it represents, a legitimate
// - if not literally real-camera-captured - way to derive 4:2:2 content
// from real 4:2:0 data for testing), encoded, decoded back with this
// project's own TinyH264Decoder, and checked for reasonable PSNR against
// the original source. Not held to the tight bit-exact bars
// test_encode_iframe.cpp uses (that test's own YUV420-plane path is the
// oracle for encoder *bitstream* correctness) - this test's job is
// catching a real bug in the color-conversion layer (h264_color_convert.h),
// not re-verifying the encoder core.
//
// Development-time cross-check (not re-run here): h264_color_convert.h's
// RGB->YUV matrix was verified against real ffmpeg's own
// `-pix_fmt rgb24 -> -pix_fmt yuv420p` conversion of this same asset
// (max luma diff of 1, small bounded chroma diff) - see that file's own
// comments.
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include "../../src/TinyH264Encoder.h"
#include "../../src/TinyH264Decoder.h"

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
static const int W = 176, H = 144;

struct DecodeState {
  const uint8_t* srcY;
  double sumSq = 0;
  bool gotFrame = false;
};

void psnrCallback(TinyH264Decoder<>& d, void* userData) {
  auto* st = (DecodeState*)userData;
  st->gotFrame = true;
  for (int y = 0; y < H; y++) {
    const uint8_t* row = d.y() + (size_t)y * d.strideY();
    for (int x = 0; x < W; x++) {
      int diff = (int)row[x] - (int)st->srcY[y * W + x];
      st->sumSq += (double)diff * diff;
    }
  }
}

void checkEncoded(const char* label, size_t n, const std::vector<uint8_t>& bitstream,
                   const uint8_t* srcY, double minPsnr) {
  if (n == 0) {
    printf("FAIL %s: encodeIFrame* returned 0\n", label);
    failures++;
    return;
  }
  TinyH264Decoder<> dec;
  DecodeState st{srcY};
  dec.setCallback(psnrCallback, &st);
  dec.write(bitstream.data(), n);
  if (dec.hasError() || !st.gotFrame) {
    printf("FAIL %s: decode of own output did not produce a frame\n", label);
    failures++;
    return;
  }
  double mse = st.sumSq / (W * H);
  double psnr = mse > 0 ? 10 * log10(255.0 * 255.0 / mse) : 999.0;
  if (psnr < minPsnr) {
    printf("FAIL %s: PSNR %.2fdB below floor %.2fdB (%zu bytes)\n", label,
           psnr, minPsnr, n);
    failures++;
    return;
  }
  printf("%s: OK (%zu bytes, PSNR=%.2fdB)\n", label, n, psnr);
}

int main() {
  auto rgb = readFile("assets/frame0_rgb24.raw");
  auto yuv = readFile("assets/frame0_ref.yuv");
  if (rgb.size() != (size_t)W * H * 3 || yuv.size() != (size_t)W * H * 3 / 2) {
    printf("FAIL: unexpected asset size(s)\n");
    return 1;
  }
  const uint8_t* srcY = yuv.data();
  const int qp = 26;

  std::vector<uint8_t> bs(200000);

  // RGB888: direct. A fresh encoder per format (rather than reusing one
  // across all 4 blocks below) so each encodeFrame() call has no
  // reference yet and is therefore always an I-frame - reusing one
  // encoder would make the 2nd/3rd/4th blocks below become P-frames
  // against a *different* format's reconstruction, which is not what
  // this test means to compare.
  {
    TinyH264Encoder<> enc;
    enc.setSize(W, H);
    enc.setQp(qp);
    size_t n = enc.encodeFrameRgb888(rgb.data(), bs.data(), bs.size());
    checkEncoded("RGB888", n, bs, srcY, 25.0);
  }

  // RGB565: pack from the same RGB source using TinyH264Decoder's own
  // documented convention ((r&0xF8)<<8 | (g&0xFC)<<3 | b>>3).
  {
    std::vector<uint16_t> rgb565(W * H);
    for (int i = 0; i < W * H; i++) {
      uint8_t r = rgb[i * 3 + 0], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
      rgb565[i] = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    }
    TinyH264Encoder<> enc;
    enc.setSize(W, H);
    enc.setQp(qp);
    size_t n = enc.encodeFrameRgb565(rgb565.data(), bs.data(), bs.size());
    checkEncoded("RGB565", n, bs, srcY, 24.0);
  }

  // RGB666: pack from the same RGB source (r&0xFC, g&0xFC, b&0xFC).
  {
    std::vector<uint8_t> rgb666(W * H * 3);
    for (int i = 0; i < W * H; i++) {
      rgb666[i * 3 + 0] = rgb[i * 3 + 0] & 0xFC;
      rgb666[i * 3 + 1] = rgb[i * 3 + 1] & 0xFC;
      rgb666[i * 3 + 2] = rgb[i * 3 + 2] & 0xFC;
    }
    TinyH264Encoder<> enc;
    enc.setSize(W, H);
    enc.setQp(qp);
    size_t n = enc.encodeFrameRgb666(rgb666.data(), bs.data(), bs.size());
    checkEncoded("RGB666", n, bs, srcY, 25.0);
  }

  // YUV422 (YUYV): derive from the real YUV420 asset by duplicating each
  // chroma row across the 2 rows it represents (see file header comment).
  {
    const uint8_t* yPlane = yuv.data();
    const uint8_t* uPlane = yuv.data() + (size_t)W * H;
    const uint8_t* vPlane = uPlane + (size_t)(W / 2) * (H / 2);
    std::vector<uint8_t> yuyv((size_t)W * H * 2);
    for (int y = 0; y < H; y++) {
      const uint8_t* uRow = uPlane + (size_t)(y / 2) * (W / 2);
      const uint8_t* vRow = vPlane + (size_t)(y / 2) * (W / 2);
      uint8_t* dstRow = yuyv.data() + (size_t)y * W * 2;
      for (int x = 0; x < W / 2; x++) {
        dstRow[x * 4 + 0] = yPlane[y * W + x * 2];
        dstRow[x * 4 + 1] = uRow[x];
        dstRow[x * 4 + 2] = yPlane[y * W + x * 2 + 1];
        dstRow[x * 4 + 3] = vRow[x];
      }
    }
    TinyH264Encoder<> enc;
    enc.setSize(W, H);
    enc.setQp(qp);
    size_t n = enc.encodeFrameYuv422(yuyv.data(), bs.data(), bs.size());
    checkEncoded("YUV422", n, bs, srcY, 30.0);
  }

  if (failures == 0) {
    printf("test_encode_color_formats: all formats passed\n");
    return 0;
  }
  printf("test_encode_color_formats: %d failure(s)\n", failures);
  return 1;
}
