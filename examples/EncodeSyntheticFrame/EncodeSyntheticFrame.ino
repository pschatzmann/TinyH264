/*
 * TinyH264Encoder example: encodes a short synthetic QCIF sequence (a
 * shifting gradient - no camera/SD card needed) to a real Annex-B H.264
 * bitstream, and benchmarks encode speed (kBenchmarkReps repetitions,
 * avg/min/max fps - see examples/DecodeFromProgmem for the same idea on
 * the decode side). Shows the recommended way to drive the encoder:
 * configure size/keyframe interval via the constructor and bitrate via
 * setTargetBitrate() once, then just encoder.encodeFrame(srcY, srcU,
 * srcV, dst, dstCapacity) per picture - I-frame/P-frame dispatch and
 * periodic keyframes happen automatically. See docs/encoding.md for the
 * full API and encoder scope. Validated via arduino-cli against
 * esp32:esp32:esp32 and rp2040:rp2040:rpipico.
 *
 * For a real application, feed encodeFrame() with real camera frames'
 * Y/U/V planes instead of the synthetic gradient below - the calling
 * pattern is identical either way.
 */

#include <TinyH264Encoder.h>

using namespace tinyh264;

static const int kWidth = 176;   // QCIF, must be a multiple of 16
static const int kHeight = 144;
static const int kNumFrames = 8;       // 1 full GOP: I,P,P,P,I,P,P,P
static const int kKeyframeInterval = 4; // 1 I-frame every 4 pictures
static const int kTargetBps = 300000;  // rate control target
static const double kFps = 25.0;

/*
 * Encode the kNumFrames-picture sequence this many times back to back to
 * get a stable timing average instead of judging performance off 8
 * frames alone - same idea as examples/DecodeFromProgmem's
 * kBenchmarkReps.
 */
static const int kBenchmarkReps = 30;

/*
 * Width/height/keyframe interval are known at compile time here, so the
 * constructor configures them directly instead of calling setSize()/
 * setKeyframeInterval() separately in setup() below.
 */
TinyH264Encoder<> encoder(kWidth, kHeight, kKeyframeInterval);

static uint8_t srcY[kWidth * kHeight];
static uint8_t srcU[(kWidth / 2) * (kHeight / 2)];
static uint8_t srcV[(kWidth / 2) * (kHeight / 2)];
static uint8_t bitstream[16384];

static int frameCount = 0;
static uint64_t totalEncodeUs = 0;
static uint32_t minFrameUs = 0xFFFFFFFF;
static uint32_t maxFrameUs = 0;

static void printFreeHeap(const char* label) {
  Serial.print(label);
  Serial.print(": ");
#if defined(ARDUINO_ARCH_ESP32)
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
#elif defined(ARDUINO_ARCH_RP2040)
  Serial.print(rp2040.getFreeHeap());
  Serial.println(" bytes");
#else
  Serial.println("(not available on this core)");
#endif
}

/*
 * Fills the source buffers with a diagonal luma gradient shifted by
 * `shiftX` pixels (flat mid-gray chroma) - real, non-trivial content for
 * the encoder's residual/CAVLC path, and genuine frame-to-frame motion
 * (the shift) for the P-frame path's motion search to actually find and
 * compensate rather than falling back to a residual-only match every
 * frame.
 */
static void makeTestPattern(int shiftX) {
  for (int y = 0; y < kHeight; y++) {
    for (int x = 0; x < kWidth; x++) {
      int v = ((x + shiftX + y) * 255) / (kWidth + kHeight);
      srcY[y * kWidth + x] = (uint8_t)(v & 0xFF);
    }
  }
  for (int i = 0; i < (kWidth / 2) * (kHeight / 2); i++) {
    srcU[i] = 128;
    srcV[i] = 128;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("TinyH264Encoder EncodeSyntheticFrame example");
  printFreeHeap("Free heap before encode");

  /*
   * Width/height/keyframe interval were already configured by the
   * constructor above; target bitrate (qp defaults to -1 - rate control -
   * unless setQp() is also called) is the one setting left to configure
   * once, up front, instead of repeating on every encodeFrame() call
   * below.
   */
  encoder.setTargetBitrate(kTargetBps, kFps);

  /*
   * Encodes the same kNumFrames-picture GOP kBenchmarkReps times back to
   * back (see kBenchmarkReps' own comment above) - keyframes and P-frames
   * keep alternating exactly the same way across repetitions, since
   * kKeyframeInterval divides kNumFrames evenly.
   */
  for (int rep = 0; rep < kBenchmarkReps; rep++) {
    for (int i = 0; i < kNumFrames; i++) {
      makeTestPattern(i * 4);  // a little more horizontal shift each frame

      /*
       * One call per picture - encodeFrame() decides I-frame vs. P-frame on
       * its own (here: the first picture, then every kKeyframeInterval-th
       * one after that, thanks to setKeyframeInterval() above; everything
       * else becomes a P-frame), using the size/stride/qp already
       * configured above instead of taking them as parameters here.
       */
      uint32_t startUs = micros();
      size_t n =
          encoder.encodeFrame(srcY, srcU, srcV, bitstream, sizeof(bitstream));
      uint32_t encodeUs = micros() - startUs;
      frameCount++;

      if (n == 0) {
        Serial.print("Frame ");
        Serial.print(frameCount);
        Serial.println(": encode failed");
        continue;
      }

      /*
       * The very first frame overall also pays for one-time picture-
       * buffer allocation (Frame::ensureAllocated(), first triggered by
       * the first encodeFrame() call), so it isn't representative of
       * steady-state per-frame cost - excluded from the running stats,
       * still printed below.
       */
      if (frameCount > 1) {
        totalEncodeUs += encodeUs;
        if (encodeUs < minFrameUs) minFrameUs = encodeUs;
        if (encodeUs > maxFrameUs) maxFrameUs = encodeUs;
      }

      Serial.print("Frame ");
      Serial.print(frameCount);
      Serial.print(": ");
      Serial.print((unsigned)n);
      Serial.print(" bytes, qp=");
      Serial.print(encoder.lastQp());
      Serial.print(", ");
      Serial.print(encodeUs);
      Serial.print(" us (");
      Serial.print(encodeUs > 0 ? 1000000.0f / encodeUs : 0.0f, 2);
      Serial.println(" fps)");
    }
  }

  Serial.print("Encoded ");
  Serial.print(frameCount);
  Serial.print(" frame(s) over ");
  Serial.print(kBenchmarkReps);
  Serial.println(" repetition(s) of the sequence.");
  if (frameCount > 1) {
    uint32_t avgUs = (uint32_t)(totalEncodeUs / (uint32_t)(frameCount - 1));
    Serial.print("Per-frame encode time: avg=");
    Serial.print(avgUs);
    Serial.print(" us (");
    Serial.print(1000000.0f / avgUs, 1);
    Serial.print(" fps), min=");
    Serial.print(minFrameUs);
    Serial.print(" us (");
    Serial.print(1000000.0f / minFrameUs, 1);
    Serial.print(" fps), max=");
    Serial.print(maxFrameUs);
    Serial.print(" us (");
    Serial.print(1000000.0f / maxFrameUs, 1);
    Serial.println(" fps)");
  }

  printFreeHeap("Free heap after encode");
}

void loop() {}
