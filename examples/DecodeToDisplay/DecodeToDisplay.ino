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
 * swapped colors: adjust kMadctlLandscape below - exact MADCTL bit
 * meaning varies by panel vendor even for the same controller IC.
 *
 * ESP32-only: uses ESP32-Arduino's 4-argument SPI.begin() pin-remap
 * overload, which RP2040-Arduino's SPI class doesn't have.
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

// The embedded clip was encoded at 10 fps - paces playback to match.
constexpr int kClipFps = 10;

// --- SPI / display pins -----------------------------------------------------
constexpr int8_t kPinMosi = 13;
constexpr int8_t kPinMiso = 12;
constexpr int8_t kPinSclk = 14;
constexpr int8_t kPinCs = 15;
constexpr int8_t kPinDc = 2;
constexpr int8_t kPinRst = -1;
constexpr int8_t kPinBacklight = 27;

// ILI9341Driver::begin() (TinyGPU) doesn't set orientation - this
// subclass sends a landscape MADCTL right after begin() (see the file
// header comment above if the image comes out rotated/mirrored).
constexpr uint8_t kMadctlLandscape = 0x28;  // MV=1, BGR=1

class ILI9341LandscapeDriver : public ILI9341Driver {
 public:
  using ILI9341Driver::ILI9341Driver;
  bool begin() override {
    if (!ILI9341Driver::begin()) return false;
    writeCommand(0x36);
    writeData8(kMadctlLandscape);
    return true;
  }
};

// TinyGPU's Surface only exposes a const data() accessor (for its usual
// per-pixel setPixel() callers); toRGB565() fills a whole band in one
// bulk call, so this subclass exposes a mutable pointer into the same
// buffer instead of copying through a setPixel() loop.
class DecodeBandSurface : public SurfaceRGB565 {
 public:
  using SurfaceRGB565::SurfaceRGB565;
  uint16_t* pixels() { return reinterpret_cast<uint16_t*>(buffer.data()); }
  size_t pixelCount() const { return buffer.size(); }
};

TinyH264Decoder<> decoder;
ILI9341LandscapeDriver tftDriver(SPI, kPinCs, kPinDc, kPinRst);
DecodeBandSurface band(kDisplayWidth, kBandHeight, FontRGB565);

static uint8_t clipBuf[kDisplayTestClipSize];
static int frameCount = 0;

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
  for (int bandY = 0; bandY < kDisplayHeight; bandY += kBandHeight) {
    size_t written = d.toRGB565(0, bandY, kDisplayWidth, kBandHeight,
                                band.pixels(), band.pixelCount());
    if (written == 0) {
      Serial.println("toRGB565() failed (buffer too small?) - stopping.");
      while (true) delay(1000);
    }
    tftDriver.writeData(band, kOffsetX, kOffsetY + bandY);
  }
  frameCount++;
  delay(1000 / kClipFps);
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
