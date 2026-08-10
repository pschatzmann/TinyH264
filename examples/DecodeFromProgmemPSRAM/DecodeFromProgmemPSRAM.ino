// TinyH264Decoder + PSRAM example: identical to DecodeFromProgmem, except
// the decoder's picture buffers are allocated from PSRAM instead of the
// regular heap, via PSRAMAllocatorESP32. Requires a board with PSRAM
// (e.g. most ESP32-S3 modules) and PSRAM enabled in the board config -
// on a plain ESP32 without PSRAM, use TinyH264Decoder<> (see
// DecodeFromProgmem) instead.
//
// With PSRAM absorbing the two ~38KB picture buffers, H264_MAX_WIDTH/
// H264_MAX_HEIGHT in h264_config.h can also be raised (e.g. to CIF or
// QVGA) since they no longer compete with the rest of the sketch for
// internal DRAM.
//
// Also benchmarks decode speed the same way DecodeFromProgmem does (see
// its comments for the measurement methodology) - useful for comparing
// PSRAM vs. regular-heap picture buffers' effect on decode time, since
// PSRAM access is slower than internal DRAM on most ESP32 boards.

#include <TinyH264Decoder.h>
#include "h264_test_clip.h"

using namespace tinyh264;

TinyH264Decoder<PSRAMAllocatorESP32<uint8_t>> decoder;
int frameCount = 0;

// Decode the clip this many times back to back to get a stable timing
// average instead of judging performance off the clip's 3 frames alone.
static const int kBenchmarkReps = 30;

// Wall-clock checkpoint, reset at the *end* of onFrame() (after the
// luma-average/print work below, which is example-only overhead, not
// decoder cost) so each measured frameUs reflects pure decode time - not
// diluted by ~115200-baud UART transmission time for the previous frame's
// print line.
static uint32_t checkpointUs = 0;
static uint64_t totalDecodeUs = 0;
static uint32_t minFrameUs = 0xFFFFFFFF;
static uint32_t maxFrameUs = 0;

static uint32_t averageLuma(const TinyH264Decoder<PSRAMAllocatorESP32<uint8_t>>& d) {
  uint64_t sum = 0;
  int w = d.width(), h = d.height();
  for (int row = 0; row < h; row++) {
    const uint8_t* line = d.y() + (size_t)row * d.strideY();
    for (int col = 0; col < w; col++) sum += line[col];
  }
  return (uint32_t)(sum / ((uint32_t)w * h));
}

void onFrame(TinyH264Decoder<PSRAMAllocatorESP32<uint8_t>>& d, void* /*userData*/) {
  uint32_t frameUs = micros() - checkpointUs;
  frameCount++;

  // The very first frame overall also pays for one-time SPS/PPS parsing
  // and the picture buffers' first (PSRAM) allocation, so it isn't
  // representative of steady-state per-frame cost - excluded from the
  // running stats, still printed below.
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
  Serial.println("TinyH264Decoder DecodeFromProgmemPSRAM example");
  Serial.print("Free heap before decode: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.print("Free PSRAM before decode: ");
  Serial.print(ESP.getFreePsram());
  Serial.println(" bytes");

  decoder.setCallback(onFrame);

  static uint8_t clipBuf[kTestClipSize];
  memcpy_P(clipBuf, kTestClip, kTestClipSize);

  checkpointUs = micros();
  for (int rep = 0; rep < kBenchmarkReps; rep++) {
    decoder.write(clipBuf, kTestClipSize);
    if (decoder.hasError()) {
      Serial.println(decoder.lastStatus() ==
                              TinyH264Decoder<PSRAMAllocatorESP32<uint8_t>>::Status::kUnsupported
                          ? "Decoder: unsupported stream feature"
                          : "Decoder: bitstream error");
      break;
    }
  }

  Serial.printf("Decoded %d frame(s) over %d repetition(s) of the clip.\n",
                frameCount, kBenchmarkReps);
  if (frameCount > 1) {
    uint32_t avgUs = (uint32_t)(totalDecodeUs / (uint32_t)(frameCount - 1));
    Serial.printf(
        "Per-frame decode time: avg=%lu us (%.1f fps), min=%lu us (%.1f fps), "
        "max=%lu us (%.1f fps)\n",
        (unsigned long)avgUs, 1000000.0f / avgUs, (unsigned long)minFrameUs,
        1000000.0f / minFrameUs, (unsigned long)maxFrameUs,
        1000000.0f / maxFrameUs);
  }
  Serial.printf("Free heap after decode: %u bytes\n", ESP.getFreeHeap());
  Serial.printf("Free PSRAM after decode: %u bytes\n", ESP.getFreePsram());
}

void loop() {
  delay(1000);
}
