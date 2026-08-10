/*
 * TinyH264Encoder example: encodes a short synthetic QCIF sequence (a
 * diagonal gradient that shifts a little each frame - no camera/SD card
 * needed, generated in RAM, giving genuine if synthetic motion for
 * encodeFrame()'s P-frame path to actually exercise) to a real Annex-B
 * H.264 bitstream and prints per-frame stats to Serial. Demonstrates the
 * recommended, simplest way to drive this encoder: configure picture
 * size/keyframe interval via the constructor and target bitrate via
 * setTargetBitrate() in setup(), then just encoder.encodeFrame(srcY,
 * srcU, srcV, dst, dstCapacity) per picture - no width/height/stride/qp
 * repeated on every call - with the I-frame/P-frame decision (and, here,
 * a periodic keyframe every 4 pictures) made automatically too.
 * Validated via arduino-cli against both esp32:esp32:esp32 and
 * rp2040:rp2040:rpipico.
 * (Round-trip correctness against a real decoder - both this library's
 * own TinyH264Decoder and real ffmpeg - is already covered by the
 * desktop test suite, test/native/test_encode_autoframe.cpp and
 * friends; this example sticks to just the encoder side, both to keep
 * the static memory footprint realistic for a plain ESP32/RP2040 and
 * because a real embedded encoder use case typically streams its output
 * elsewhere for decoding rather than keeping a decoder resident on the
 * same device.)
 *
 * Current scope (see the project README's "Encoding" section):
 * I_16x16/I_4x4 intra modes, P_16x16/P_Skip inter with automatic Intra
 * fallback on a poor motion match (single reference frame, integer-pel-
 * only motion search, no P_16x8/P_8x16/P_8x8 sub-partitions), simple
 * real-time-appropriate rate control - the encoder core is plain
 * portable C++17 with no ESP32-specific dependencies, same as the
 * decoder.
 *
 * For a real application, feed encodeFrame() with real camera frames'
 * Y/U/V planes instead of the synthetic gradient below - the calling
 * pattern (one encodeFrame() call per picture, in order) is identical
 * either way. encodeFrame() is the only public encode entry point -
 * there's no explicit "force an I-frame now" call; setKeyframeInterval()
 * (periodic) and a resolution change are the only ways to influence when
 * a keyframe happens.
 */

#include <TinyH264Encoder.h>

using namespace tinyh264;

static const int kWidth = 176;   // QCIF, must be a multiple of 16
static const int kHeight = 144;
static const int kNumFrames = 8;       // long enough to see a periodic keyframe
static const int kKeyframeInterval = 4; // 1 I-frame every 4 pictures
static const int kTargetBps = 300000;  // rate control target
static const double kFps = 25.0;

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

    if (n == 0) {
      Serial.print("Frame ");
      Serial.print(i);
      Serial.println(": encode failed");
      continue;
    }

    Serial.print("Frame ");
    Serial.print(i);
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

  printFreeHeap("Free heap after encode");
}

void loop() {}
