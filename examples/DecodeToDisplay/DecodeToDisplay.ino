/*
 * TinyH264Decoder + TinyGPU (https://github.com/pschatzmann/TinyGPU)
 * display example: decodes a tiny embedded H.264 clip and streams each
 * decoded picture to an ILI9341 SPI TFT, band by band, looping forever.
 * Requires the TinyGPU library (installed the same way as TinyH264
 * itself - see its README) in addition to TinyH264.
 *
 * Board used for development and testing: a common 3.5" 240x320 ILI9341
 * SPI TFT -
 * https://github.com/pschatzmann/arduino-audio-tools/wiki/Audio-Boards#esp32-arduino-lvgl-wifibluetooth-development-board-24inch-lcd-tft-module
 *
 * Decodes at 256x192 and centers it in the 320x240 panel, rather than
 * decoding at the panel's full resolution or the project's QVGA
 * compile-time default - on a plain ESP32 (no PSRAM), QVGA's ~283KB
 * requirement (2 picture buffers + the metadata table) plus this
 * sketch's own display objects exceeded real measured free heap. See
 * docs/memory-budget.md for the full picture-buffer budget model;
 * `#define H264_MAX_WIDTH`/`H264_MAX_HEIGHT` below override the
 * project-wide default for this sketch only (h264_config.h's `#ifndef`
 * guards support this per translation unit). If your board has PSRAM,
 * remove those two #defines and switch the decoder to
 * PSRAMAllocatorESP32<uint8_t> instead (see DecodeFromProgmemPSRAM) to
 * decode at full QVGA.
 *
 * Avoids a full-screen RGB565 framebuffer (one ~150KB+ contiguous
 * allocation, which a plain ESP32's split internal DRAM often can't
 * satisfy) the same way TinyGPU's own bouncing-ball example does:
 * render/push the screen in small horizontal bands instead of one
 * whole-frame buffer, via TinyH264Decoder::toRGB565()'s windowed
 * overload (see docs/decoding.md).
 *
 *   TFT_MOSI -> GPIO13   TFT_MISO -> GPIO12   TFT_SCLK -> GPIO14
 *   TFT_CS   -> GPIO15   TFT_DC   -> GPIO2    TFT_RST  -> not connected (-1)
 *   TFT_BL   -> GPIO27 (backlight)
 * (same pinout as TinyGPU's bouncing-ball example; adjust for your own
 * wiring.)
 *
 * If the screen stays blank: check the backlight pin and SPI pins
 * first. If it lights up but the image is rotated, mirrored, or has
 * swapped colors: try a different kRotationLandscape value below (0-3,
 * see ILI9341Driver's constructor in TinyGPU) - exact MADCTL bit meaning
 * varies by panel vendor even for the same controller IC.
 *
 * ESP32-only: uses ESP32-Arduino's 4-argument SPI.begin() pin-remap
 * overload, which RP2040-Arduino's SPI class doesn't have.
 *
 * Measures decode time and convert+SPI-push time separately with
 * micros() (excluding the deliberate pacing delay() below), printing a
 * per-frame line plus a rolling avg/min/max summary once per clip loop -
 * same on-device-measurement approach as DecodeFromProgmem's benchmark.
 */

#if !defined(ARDUINO_ARCH_ESP32)
#error \
    "DecodeToDisplay is ESP32-only (see the file header comment above) - it uses ESP32-Arduino's 4-argument SPI.begin() pin-remap API."
#endif

// Overrides the project-wide QVGA default for this sketch only - see
// the file header comment above. Must precede the include below.
#define H264_MAX_WIDTH 256
#define H264_MAX_HEIGHT 192

#include <TinyGPU.h>
#include <TinyGPU/DisplayDriverSPI.h>
#include <TinyH264Decoder.h>

#include "h264_display_test_clip.h"

using namespace tinyh264;
using namespace tinygpu;

// --- display geometry -----------------------------------------------------
// Panel is 320x240; the decoded picture is 256x192, centered via
// kOffsetX/kOffsetY (see the file header comment above for why).
constexpr int kPanelWidth = 320;
constexpr int kPanelHeight = 240;
constexpr int kDisplayWidth = 256;
constexpr int kDisplayHeight = 192;
constexpr int kOffsetX = (kPanelWidth - kDisplayWidth) / 2;
constexpr int kOffsetY = (kPanelHeight - kDisplayHeight) / 2;

// --- band buffer geometry ---------------------------------------------------
// kDisplayHeight and kPanelHeight must both be evenly divisible by
// kBandHeight.
constexpr int kBandHeight = 16;
constexpr int kBandCount = kDisplayHeight / kBandHeight;

// The embedded clip was encoded at 10 fps, 10 frames total (see
// h264_display_test_clip.h) - paces playback to match and marks where
// one full loop of the clip ends, for the periodic timing summary below.
constexpr int kClipFps = 10;
constexpr int kClipFrameCount = 10;

// --- SPI / display pins -----------------------------------------------------
constexpr int8_t kPinMosi = 13;
constexpr int8_t kPinMiso = 12;
constexpr int8_t kPinSclk = 14;
constexpr int8_t kPinCs = 15;
constexpr int8_t kPinDc = 2;
constexpr int8_t kPinRst = -1;
constexpr int8_t kPinBacklight = 27;

TinyH264Decoder<> decoder;
ILI9341Driver tftDriver(SPI, kPinCs, kPinDc, kPinRst, ILI9341Driver::Rotation::kLandscape);
SurfaceRGB565 band(kDisplayWidth, kBandHeight, FontRGB565);

static uint8_t clipBuf[kDisplayTestClipSize];
static int frameCount = 0;

/*
 * Timing state. decodeCheckpointUs is reset at the very end of onFrame()
 * (after the pacing delay()), so the elapsed time until the *next*
 * onFrame() fires - measured at that call's start - is exactly the time
 * decoder.write() spent decoding that next picture, with neither the
 * delay() nor this frame's own convert+push work counted against it
 * (same reset-after-the-non-decode-work technique DecodeFromProgmem
 * uses, adapted for the extra delay() this sketch has and it doesn't).
 * displayUs is measured directly around the band conversion+SPI-push
 * loop, so it isn't diluted by decode or the pacing delay either.
 */
static uint32_t decodeCheckpointUs = 0;
static uint64_t totalDecodeUs = 0, totalDisplayUs = 0;
static uint32_t minDecodeUs = 0xFFFFFFFF, maxDecodeUs = 0;
static uint32_t minDisplayUs = 0xFFFFFFFF, maxDisplayUs = 0;
static int statsFrameCount = 0;

// One-time full-panel clear so the border around the centered picture
// isn't leftover power-on garbage. Uses its own transient, panel-width
// band, freed right after (unlike `band` above, which stays resident).
static void clearScreen() {
  SurfaceRGB565 clearBand(kPanelWidth, kBandHeight, FontRGB565);
  clearBand.begin();
  clearBand.clear(RGB565(0, 0, 0));
  for (int y = 0; y < kPanelHeight; y += kBandHeight) {
    tftDriver.writeData(clearBand, 0, y);
  }
  clearBand.end();
}

// Also prints the largest free contiguous block, not just the free-heap
// total - see docs/memory-budget.md for why that distinction matters.
static void printFreeHeap(const char* label) {
  Serial.print(label);
  Serial.print(": ");
  Serial.print(ESP.getFreeHeap());
  Serial.print(" bytes free, largest block ");
  Serial.print(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.println(" bytes");
}

// Called once per decoded picture, from inside decoder.write() below.
// Converts and pushes the picture to the display one band at a time.
void onFrame(TinyH264Decoder<>& d, void* /*userData*/) {
  uint32_t decodeUs = micros() - decodeCheckpointUs;

  uint32_t displayStartUs = micros();
  for (int bandY = 0; bandY < kDisplayHeight; bandY += kBandHeight) {
    // TinyGPU's Surface<RGB565>::pixels() returns tinygpu::RGB565* (a
    // single uint16_t member, no vtable) - reinterpret_cast to the
    // uint16_t* toRGB565() writes, matching this project's established
    // "same binary layout, safe to alias" reasoning for RGB565 buffers.
    size_t written = d.toRGB565(0, bandY, kDisplayWidth, kBandHeight,
                                reinterpret_cast<uint16_t*>(band.pixels()),
                                band.pixelCount());
    if (written == 0) {
      Serial.println("toRGB565() failed (buffer too small?) - stopping.");
      while (true) delay(1000);
    }
    tftDriver.writeData(band, kOffsetX, kOffsetY + bandY);
  }
  uint32_t displayUs = micros() - displayStartUs;

  frameCount++;
  // The very first frame overall also pays for one-time SPS/PPS parsing
  // and the picture buffers' first allocation - excluded from the
  // running stats, matching DecodeFromProgmem's identical exclusion.
  if (frameCount > 1) {
    totalDecodeUs += decodeUs;
    if (decodeUs < minDecodeUs) minDecodeUs = decodeUs;
    if (decodeUs > maxDecodeUs) maxDecodeUs = decodeUs;
    totalDisplayUs += displayUs;
    if (displayUs < minDisplayUs) minDisplayUs = displayUs;
    if (displayUs > maxDisplayUs) maxDisplayUs = displayUs;
    statsFrameCount++;
  }

  uint32_t totalUs = decodeUs + displayUs;
  Serial.print("Frame ");
  Serial.print(frameCount);
  Serial.print(": decode=");
  Serial.print(decodeUs);
  Serial.print("us, convert+push=");
  Serial.print(displayUs);
  Serial.print("us, total=");
  Serial.print(totalUs);
  Serial.print("us (");
  Serial.print(totalUs > 0 ? 1000000.0f / totalUs : 0.0f, 1);
  Serial.print(" fps unpaced)");
  Serial.print(" - free heap: ");
  Serial.println(ESP.getFreeHeap());

  // Rolling avg/min/max once per completed clip loop, so drift over a
  // long-running session (thermal throttling, WiFi/SPI contention, ...)
  // shows up instead of only ever seeing one early snapshot.
  if (statsFrameCount > 0 && frameCount % kClipFrameCount == 0) {
    Serial.print("  avg decode=");
    Serial.print((uint32_t)(totalDecodeUs / statsFrameCount));
    Serial.print("us (min=");
    Serial.print(minDecodeUs);
    Serial.print(", max=");
    Serial.print(maxDecodeUs);
    Serial.print("), avg convert+push=");
    Serial.print((uint32_t)(totalDisplayUs / statsFrameCount));
    Serial.print("us (min=");
    Serial.print(minDisplayUs);
    Serial.print(", max=");
    Serial.print(maxDisplayUs);
    Serial.println(")");
  }

  delay(1000 / kClipFps);
  decodeCheckpointUs = micros();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("TinyH264Decoder DecodeToDisplay example");
  printFreeHeap("Free heap before decode");

  if (kPinBacklight >= 0) {
    pinMode(kPinBacklight, OUTPUT);
    digitalWrite(kPinBacklight, HIGH);
  }

  SPI.begin(kPinSclk, kPinMiso, kPinMosi, kPinCs);
  tftDriver.begin();
  clearScreen();
  band.begin();

  // Required for the memory budget above: caps the decoder to 2 resident
  // picture buffers instead of the compile-time default of up to 4.
  decoder.setMaxRefFrames(1);
  decoder.setCallback(onFrame);

  // Clip is in PROGMEM (flash); copy to RAM since the decoder reads
  // directly from the pointer it's given.
  memcpy_P(clipBuf, kDisplayTestClip, kDisplayTestClipSize);

  printFreeHeap("Free heap after setup");
  decodeCheckpointUs = micros();
}

void loop() {
  /*
   * The embedded clip is short (10 frames, 1 second at kClipFps) - each
   * write() call decodes and displays the whole clip once (its first NAL
   * is an IDR, so replaying the same buffer from the start is always a
   * clean, self-contained decode, not a continuation of decoder state
   * left over from the previous pass). Looping this call for as long as
   * the sketch runs turns it into a continuously-repeating animation.
   */
  decoder.write(clipBuf, kDisplayTestClipSize);
  if (decoder.hasError()) {
    Serial.println(decoder.lastStatus() ==
                           TinyH264Decoder<>::Status::kUnsupported
                       ? "Decoder: unsupported stream feature"
                       : "Decoder: bitstream error");
    while (true) delay(1000);
  }
}
