/*
 * TinyH264Encoder example: encodes a short synthetic QCIF sequence (a
 * shifting gradient - no camera/SD card needed) to a real Annex-B H.264
 * bitstream, and benchmarks encode speed (kBenchmarkReps repetitions,
 * avg/min/max fps - see examples/DecodeFromProgmem for the same idea on
 * the decode side). Shows the recommended way to drive the encoder:
 * configure size/keyframe interval via the constructor and QP policy
 * (fixed via setQp(), or rate control via setTargetBitrate()) once, then
 * just encoder.encodeFrame(srcY, srcU, srcV, dst, dstCapacity) per
 * picture - I-frame/P-frame dispatch and periodic keyframes happen
 * automatically. See docs/encoding.md for the full API and encoder
 * scope. Validated via arduino-cli against esp32:esp32:esp32 and
 * rp2040:rp2040:rpipico.
 *
 * Runs the same benchmark twice, back to back, once per QP policy - see
 * runBenchmark()'s own comment for why both are exercised here rather
 * than just one: on ESP32-P4, only one of the two can ever reach the
 * hardware encoder path at all.
 *
 * For a real application, feed encodeFrame() with real camera frames'
 * Y/U/V planes instead of the synthetic gradient below - the calling
 * pattern is identical either way.
 *
 */

#include <TinyH264Encoder.h>

using namespace tinyh264;

static const int kWidth = 176;   // QCIF, must be a multiple of 16
static const int kHeight = 144;
static const int kNumFrames = 8;       // 1 full GOP: I,P,P,P,I,P,P,P
static const int kKeyframeInterval = 4; // 1 I-frame every 4 pictures
static const int kFixedQp = 26;
static const int kTargetBps = 300000;  // rate control target
static const double kFps = 25.0;

// +/-pixel motion search window (default 8, same as never calling
// setMotionSearchRange() at all) - lower this to trade encode speed for
// worse compression on fast motion (search cost is roughly O(range^2) -
// see docs/optimizations.md's "Encoding" chapter for measured
// numbers at the default).
static const int kMotionSearchRange = 8;

// Exhaustive (default) checks every candidate in the +/-kMotionSearchRange
// window and always finds the true best-SAD match; Fast (Diamond Search)
// checks far fewer candidates and is faster, but can settle for a locally-
// rather than globally-best match on some content (see docs/encoding.md's
// "Motion search algorithm" section and docs/optimizations.md's
// "Encoding" chapter for the measured tradeoff) - flip this to
// MotionSearchAlgorithm::Fast to try it.
static const MotionSearchAlgorithm kMotionSearchAlgorithm =
    MotionSearchAlgorithm::Exhaustive;

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

/*
 * Runs the kNumFrames x kBenchmarkReps encode loop against whatever QP
 * policy the caller already configured on `encoder` (setQp() or
 * setTargetBitrate() - this function doesn't care which), and prints a
 * summary - shared by both of setup()'s two runs below (fixed QP, then
 * rate control) so the per-frame/summary logging logic exists once.
 *
 * Both modes are exercised, back to back, specifically because they
 * behave differently on ESP32-P4: TinyH264Encoder::ensureHardwareReady()
 * requires a resolved, non-negative QP before it will even attempt the
 * hardware encoder (qp_ < 0 - the state setTargetBitrate() leaves it in,
 * since rate control's adaptive QP lives inside the software Encoder,
 * not synced back up to TinyH264Encoder's own qp_) - so rate control
 * mode never attempts hardware at all, cheaply, every call. Fixed QP
 * mode does attempt it: on this project's current hardware driver (see
 * README's "ESP32-P4 hardware encoder" section) that attempt itself
 * still fails, but only the *first* frame pays hw_.encode()'s up-to-
 * 1-second timeout cost - encoder.h's hwEncodeFailed_ latch then skips
 * straight to the software encoder for every frame after that. Either
 * way, every frame here should encode successfully - watch the log line
 * this prints right before the loop for whether hardware was actually
 * opened (see TinyH264Encoder::ensureHardwareReady()'s own log lines).
 */
static void runBenchmark(const char* modeLabel) {
  Serial.print("-- ");
  Serial.print(modeLabel);
  Serial.println(" --");

  int frameCount = 0;
  uint64_t totalEncodeUs = 0;
  uint32_t minFrameUs = 0xFFFFFFFF;
  uint32_t maxFrameUs = 0;

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
       * the first encodeFrame() call) and - in fixed-QP mode - the
       * hardware encoder's own failed-attempt timeout (see this
       * function's own comment above), so it isn't representative of
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
      Serial.print(" fps, running avg=");
      if (frameCount > 1) {
        uint32_t avgUs = (uint32_t)(totalEncodeUs / (uint32_t)(frameCount - 1));
        Serial.print(avgUs > 0 ? 1000000.0f / avgUs : 0.0f, 2);
        Serial.println(" fps)");
      } else {
        Serial.println("n/a)");
      }
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
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("TinyH264Encoder EncodeSyntheticFrame example");
  printFreeHeap("Free heap before encode");

  // See the file header comment: sizes buffers for this sketch's actual
  // QCIF content instead of the project-wide compile-time default.
  encoder.setMaxDimension(kWidth, kHeight);
  encoder.setMotionSearchRange(kMotionSearchRange);
  encoder.setMotionSearchAlgorithm(kMotionSearchAlgorithm);

  // Fixed QP first - see runBenchmark()'s own comment for why this is
  // the mode that can actually reach ESP32-P4's hardware encoder path.
  encoder.setQp(kFixedQp);
  runBenchmark("Fixed QP (setQp() - hardware encoder path eligible on ESP32-P4)");

  // Then rate control - same encoder instance, same content, just a
  // different QP policy (setTargetBitrate() instead of setQp()).
  encoder.setQp(-1);  // re-enable rate control (setQp() above pinned it)
  encoder.setTargetBitrate(kTargetBps, kFps);
  runBenchmark("Rate control (setTargetBitrate() - always software-only on ESP32-P4)");

  printFreeHeap("Free heap after encode");
}

void loop() {}
