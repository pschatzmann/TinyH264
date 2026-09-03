/*
 * TinyH264Encoder + TinyH264Decoder round-trip self-test: encodes a short
 * synthetic QCIF sequence (the same shifting gradient as
 * examples/EncodeSyntheticFrame.ino - no camera/SD card needed), decodes
 * the resulting Annex-B bitstream right back with TinyH264Decoder, and
 * checks each decoded picture against the original source frame that
 * produced it (PSNR, not a bitwise memcmp() - H.264 is a lossy codec even
 * at a real, non-zero QP, so "matches the source" here means "close
 * enough that no real corruption happened", the same PSNR-based check
 * this project's own native tests use, e.g.
 * test/native/test_encode_pslice_intra_fallback.cpp).
 *
 * Useful as a real on-device sanity check for anything that changes the
 * encoder or decoder (or, on ESP32-P4, for comparing the hardware
 * encoder path against this same check via
 * TinyH264Encoder::setUseHardware() - see docs/encoding.md's "ESP32-P4
 * hardware encoding" section): if this example ever prints FAIL, either
 * the encoder produced a stream this decoder can't parse, or the decoded
 * pixels drifted far enough from the source to indicate a real bug, not
 * just ordinary lossy-compression rounding.
 *
 * Validated via arduino-cli against esp32:esp32:esp32 and
 * rp2040:rp2040:rpipico.
 */

#include <math.h>

// Compile-time resolution/reference-frame ceiling, tightened from the
// library's own default (320x240, 3 reference frames - real headroom
// for boards with more RAM) down to exactly what this sketch needs
// (QCIF, single reference) - this sketch holds both a
// TinyH264Encoder AND a TinyH264Decoder resident at once, which doesn't
// fit a plain ESP32's DRAM (no PSRAM) at the library's default ceiling.
// Must come before the TinyH264*.h includes below - see
// docs/memory-budget.md.
#define H264_MAX_WIDTH 176
#define H264_MAX_HEIGHT 144
#define H264_MAX_REF_FRAMES 1

#include <TinyH264Decoder.h>
#include <TinyH264Encoder.h>

using namespace tinyh264;

static const int kWidth = 176;    // QCIF, must be a multiple of 16
static const int kHeight = 144;
static const int kNumFrames = 8;  // 1 full GOP: I,P,P,P,I,P,P,P
static const int kKeyframeInterval = 4;
static const int kQp = 26;  // fixed QP - no rate control, simplest case

// PSNR (dB) below which a decoded frame is considered a real mismatch,
// not just ordinary lossy-compression rounding - chosen well under this
// project's own measured baseline numbers at QP 26 (I-frames ~38dB,
// P-frames ~43-45dB, see docs/optimizations.md and this project's own
// native encode tests) so it only fires on genuine corruption.
static const float kMinAcceptablePsnrDb = 25.0f;

TinyH264Encoder<> encoder(kWidth, kHeight, kKeyframeInterval);
TinyH264Decoder<> decoder;

// Doubles as both the encoder's source and, at compare time, the
// reference to check the decoded picture against - encodeFrame() takes
// its source by `const uint8_t*` and never mutates it, so the same
// buffers are still exactly the original picture when onFrame() reads
// them just afterward. Deliberately not two separate buffer sets
// (source + reference) - real on-device memory is scarce, and doubling
// up would waste it holding two copies of identical data.
//
// Heap-allocated (in setup(), below) rather than static arrays: this
// sketch holds both a TinyH264Encoder and a TinyH264Decoder resident at
// once, and on a plain ESP32 (no PSRAM) their combined static buffers
// already use most of the fixed-size `.bss`-region budget the linker
// reserves - the heap draws from a larger pool that isn't bound by that
// same fixed region, so these (real, but comparatively small next to
// the codec's own picture buffers) sketch-level buffers fit there
// instead.
static uint8_t* srcY;
static uint8_t* srcU;
static uint8_t* srcV;
static uint8_t* bitstream;
static const size_t kBitstreamCapacity = 4096;  // a QCIF frame at QP 26 is well under this

static int decodedFrameCount = 0;
static int failedFrameCount = 0;

static void makeTestPattern(int shiftX, uint8_t* y, uint8_t* u, uint8_t* v) {
  for (int py = 0; py < kHeight; py++) {
    for (int px = 0; px < kWidth; px++) {
      int val = ((px + shiftX + py) * 255) / (kWidth + kHeight);
      y[py * kWidth + px] = (uint8_t)(val & 0xFF);
    }
  }
  for (int i = 0; i < (kWidth / 2) * (kHeight / 2); i++) {
    u[i] = 128;
    v[i] = 128;
  }
}

// Mean-squared-error -> PSNR (dB) over one plane. `stride` is the
// decoded plane's row stride (may exceed `width` - Frame planes aren't
// always tightly packed); the reference plane is always tightly packed
// (`width` bytes/row), matching makeTestPattern()'s own layout.
static float planePsnr(const uint8_t* decoded, int decodedStride,
                        const uint8_t* reference, int width, int height) {
  double sumSquaredError = 0.0;
  for (int y = 0; y < height; y++) {
    const uint8_t* drow = decoded + (size_t)y * decodedStride;
    const uint8_t* rrow = reference + (size_t)y * width;
    for (int x = 0; x < width; x++) {
      int diff = (int)drow[x] - (int)rrow[x];
      sumSquaredError += (double)(diff * diff);
    }
  }
  double mse = sumSquaredError / ((double)width * (double)height);
  if (mse <= 0.0) return 99.0f;  // pixel-identical - report a large, finite value
  return (float)(10.0 * log10((255.0 * 255.0) / mse));
}

static void onFrame(TinyH264Decoder<>& dec, void* /*userData*/) {
  decodedFrameCount++;

  float psnrY = planePsnr(dec.y(), dec.strideY(), srcY, kWidth, kHeight);
  float psnrU = planePsnr(dec.u(), dec.strideUV(), srcU, kWidth / 2,
                           kHeight / 2);
  float psnrV = planePsnr(dec.v(), dec.strideUV(), srcV, kWidth / 2,
                           kHeight / 2);
  float minPsnr = psnrY;
  if (psnrU < minPsnr) minPsnr = psnrU;
  if (psnrV < minPsnr) minPsnr = psnrV;
  bool pass = minPsnr >= kMinAcceptablePsnrDb;
  if (!pass) failedFrameCount++;

  Serial.print("Frame ");
  Serial.print(decodedFrameCount);
  Serial.print(": Y=");
  Serial.print(psnrY, 1);
  Serial.print("dB U=");
  Serial.print(psnrU, 1);
  Serial.print("dB V=");
  Serial.print(psnrV, 1);
  Serial.print("dB - ");
  Serial.println(pass ? "PASS" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("TinyH264 encode -> decode round-trip self-test");

  srcY = (uint8_t*)malloc(kWidth * kHeight);
  srcU = (uint8_t*)malloc((kWidth / 2) * (kHeight / 2));
  srcV = (uint8_t*)malloc((kWidth / 2) * (kHeight / 2));
  bitstream = (uint8_t*)malloc(kBitstreamCapacity);
  if (!srcY || !srcU || !srcV || !bitstream) {
    Serial.println("RESULT: FAIL (out of memory allocating sketch buffers)");
    return;
  }

  encoder.setMaxDimension(kWidth, kHeight);
  encoder.setQp(kQp);
  // A single reference frame is all this sequence needs (no
  // multi-reference motion search here) - shrinks the decoder's
  // picture-buffer footprint from H264_MAX_REF_FRAMES' default down to
  // just what this sketch actually uses, real headroom on a plain
  // ESP32 (no PSRAM) when the encoder's own buffers are also resident
  // in the same sketch.
  decoder.setMaxDimension(kWidth, kHeight);
  decoder.setMaxRefFrames(1);
  decoder.setCallback(onFrame);

  for (int i = 0; i < kNumFrames; i++) {
    int shiftX = i * 4;  // same motion pattern as EncodeSyntheticFrame.ino
    makeTestPattern(shiftX, srcY, srcU, srcV);

    size_t n =
        encoder.encodeFrame(srcY, srcU, srcV, bitstream, kBitstreamCapacity);
    if (n == 0) {
      Serial.print("Frame ");
      Serial.print(i + 1);
      Serial.println(": ENCODE FAILED (encodeFrame returned 0)");
      failedFrameCount++;
      continue;
    }

    // onFrame() (invoked synchronously from write(), below) compares
    // the decoded picture against srcY/srcU/srcV directly - see their
    // own comment for why encodeFrame() leaves them unmodified.
    TinyH264Decoder<>::Status status = decoder.write(bitstream, n);
    if (decoder.hasError()) {
      Serial.print("Frame ");
      Serial.print(i + 1);
      Serial.print(": DECODE FAILED, status=");
      Serial.println((int)status);
      failedFrameCount++;
    }
  }

  Serial.println();
  Serial.print("Encoded ");
  Serial.print(kNumFrames);
  Serial.print(" frame(s), decoded ");
  Serial.print(decodedFrameCount);
  Serial.println(" frame(s).");
  if (decodedFrameCount != kNumFrames) {
    Serial.println(
        "MISMATCH: decoded frame count != encoded frame count - a NAL "
        "must have failed to parse.");
  }
  Serial.println(failedFrameCount == 0 && decodedFrameCount == kNumFrames
                      ? "RESULT: PASS"
                      : "RESULT: FAIL");
}

void loop() {}
