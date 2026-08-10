/*
 * TinyH264Decoder minimal example: decodes a tiny embedded H.264 clip (3
 * frames, QCIF, flat gray - see h264_test_clip.h) and prints per-frame
 * stats to Serial. No camera, SD card, or display needed, so this runs
 * unmodified as a smoke test that the library is working - the decoder
 * core is plain portable C++17 with no ESP32-specific dependencies, so
 * this example (default std::allocator<uint8_t> - no PSRAM) builds for
 * any Arduino target with enough RAM for the QCIF picture buffers
 * (~76KB for two frames; see h264_config.h). Validated via arduino-cli
 * against both esp32:esp32:esp32 and rp2040:rp2040:rpipico.
 *
 * Also benchmarks decode speed: the short clip is decoded repeatedly
 * (kBenchmarkReps times) and per-frame wall-clock decode time is measured
 * with micros(), so you get a stable avg/min/max frames-per-second figure
 * on your actual board instead of guessing from a desktop estimate.
 *
 * For a real application, feed write() with NAL units read from an SD
 * card, a network stream (e.g. RTSP), or a camera module's H.264 output
 * instead of the embedded test clip.
 */

#include <TinyH264Decoder.h>
#include "h264_test_clip.h"

using namespace tinyh264;

TinyH264Decoder<> decoder;
int frameCount = 0;

/*
 * Decode the clip this many times back to back to get a stable timing
 * average instead of judging performance off the clip's 3 frames alone.
 */
static const int kBenchmarkReps = 30;

/*
 * Wall-clock checkpoint, reset at the *end* of onFrame() (after the
 * luma-average/print work below, which is example-only overhead, not
 * decoder cost) so each measured frameUs reflects pure decode time - not
 * diluted by ~115200-baud UART transmission time for the previous frame's
 * print line.
 */
static uint32_t checkpointUs = 0;
static uint64_t totalDecodeUs = 0;
static uint32_t minFrameUs = 0xFFFFFFFF;
static uint32_t maxFrameUs = 0;

/*
 * Free-heap reporting is not part of any Arduino-portable API - each core
 * exposes it differently (ESP32-Arduino's global `ESP` object vs.
 * arduino-pico's global `rp2040` object). Falls back to omitting the stat
 * on any other core rather than failing to build there. Uses chained
 * Serial.print() calls rather than Serial.printf() - printf() is an
 * ESP32/RP2040-Arduino convenience extension, not part of the standard
 * Arduino Print/Stream API (e.g. AVR cores don't have it), and this
 * example is meant to build on any Arduino target (see the file comment).
 */
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
 * Sum of luma samples for the current frame, used as a cheap "is this
 * picture actually different from garbage" sanity check without needing a
 * display - a real app would instead push y()/u()/v() to a screen or
 * encoder.
 */
static uint32_t averageLuma(const TinyH264Decoder<>& d) {
  uint64_t sum = 0;
  int w = d.width(), h = d.height();
  for (int row = 0; row < h; row++) {
    const uint8_t* line = d.y() + (size_t)row * d.strideY();
    for (int col = 0; col < w; col++) sum += line[col];
  }
  return (uint32_t)(sum / ((uint32_t)w * h));
}

// Called once per decoded picture, from inside decoder.write() below.
void onFrame(TinyH264Decoder<>& d, void* /*userData*/) {
  uint32_t frameUs = micros() - checkpointUs;
  frameCount++;

  /*
   * The very first frame overall also pays for one-time SPS/PPS parsing
   * and the picture buffers' first allocation (Frame::ensureAllocated()),
   * so it isn't representative of steady-state per-frame cost - excluded
   * from the running stats, still printed below.
   */
  if (frameCount > 1) {
    totalDecodeUs += frameUs;
    if (frameUs < minFrameUs) minFrameUs = frameUs;
    if (frameUs > maxFrameUs) maxFrameUs = frameUs;
  }

  Serial.print("Frame ");
  Serial.print(frameCount);
  Serial.print(": ");
  Serial.print(d.width());
  Serial.print("x");
  Serial.print(d.height());
  Serial.print(", average luma=");
  Serial.print(averageLuma(d));
  Serial.print(", decode time=");
  Serial.print(frameUs);
  Serial.print(" us (");
  Serial.print(frameUs > 0 ? 1000000.0f / frameUs : 0.0f, 1);
  Serial.println(" fps)");

  checkpointUs = micros();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("TinyH264Decoder DecodeFromProgmem example");
  printFreeHeap("Free heap before decode");

  decoder.setCallback(onFrame);

  /*
   * The embedded clip is in PROGMEM (flash); copy it to a small RAM buffer
   * since the decoder reads directly from the pointer it's given.
   */
  static uint8_t clipBuf[kTestClipSize];
  memcpy_P(clipBuf, kTestClip, kTestClipSize);

  /*
   * Decodes everything it can from the buffer, calling onFrame() once per
   * picture, before returning. Repeated kBenchmarkReps times for a stable
   * timing average (see checkpointUs comment above).
   */
  checkpointUs = micros();
  for (int rep = 0; rep < kBenchmarkReps; rep++) {
    decoder.write(clipBuf, kTestClipSize);
    if (decoder.hasError()) {
      Serial.println(decoder.lastStatus() == TinyH264Decoder<>::Status::kUnsupported
                          ? "Decoder: unsupported stream feature"
                          : "Decoder: bitstream error");
      break;
    }
  }

  Serial.print("Decoded ");
  Serial.print(frameCount);
  Serial.print(" frame(s) over ");
  Serial.print(kBenchmarkReps);
  Serial.println(" repetition(s) of the clip.");
  if (frameCount > 1) {
    uint32_t avgUs = (uint32_t)(totalDecodeUs / (uint32_t)(frameCount - 1));
    Serial.print("Per-frame decode time: avg=");
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
  printFreeHeap("Free heap after decode");
}

void loop() {
  delay(1000);
}
