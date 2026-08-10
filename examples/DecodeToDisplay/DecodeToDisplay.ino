/*
 * TinyH264Decoder + TinyGPU (https://github.com/pschatzmann/TinyGPU)
 * display example: decodes a tiny embedded H.264 clip and streams each
 * decoded picture to an ILI9341 SPI TFT, band by band, looping forever.
 * Requires the TinyGPU library (installed the same way as TinyH264
 * itself - see its README) in addition to TinyH264.
 *
 * We used the following board for development and testing: a common 3.5"
 * 240x320 ILI9341 SPI TFT
 * https://github.com/pschatzmann/arduino-audio-tools/wiki/Audio-Boards#esp32-arduino-lvgl-wifibluetooth-development-board-24inch-lcd-tft-module
 *
 * Decodes at 256x192, not the project's QVGA (320x240) compile-time
 * default or the ILI9341 panel's full 320x240 resolution - measured on
 * real ESP32 hardware (no PSRAM): QVGA decode (2 resident ~115KB
 * picture buffers + a ~40KB QVGA-sized per-macroblock metadata table,
 * ~283KB total) plus this sketch's own TinyGPU/SPI display objects
 * together exceeded the board's actual free heap at that point (~278KB)
 * and crashed with an uncaught std::bad_alloc from
 * Frame::ensureAllocated() - not a bug in the decoder's
 * allocation-splitting logic (see h264_frame.h/h264_mb_info.h), just a
 * genuine total-memory shortfall once a real display library is also
 * resident, not only fragmentation. 256x192 was chosen by solving
 * "2 resident frames (1.5 bytes/pixel each) + the per-macroblock
 * metadata table (~0.516 bytes/pixel) + a small band buffer ≈ 291KB
 * measured free heap" for the largest 4:3, macroblock-aligned (multiple
 * of 16) resolution with a real safety margin, not just the largest
 * that mathematically fits at zero margin (which comes out to ~326x245 -
 * i.e. QVGA itself is already right at that wall, explaining the
 * crash). 256x192's real footprint (2 buffers + metadata table + band,
 * ~181KB total - see h264_display_test_clip.h) leaves ~110KB of margin
 * against the measured ~291KB free heap, roughly 2.9x QCIF's pixel
 * count. **The "2 resident frames" half of that budget is not automatic
 * - it requires the decoder.setMaxRefFrames(1) call in setup() below.**
 * Without it, the compile-time default (H264_MAX_REF_FRAMES=3) lets the
 * decoder grow to 4 resident buffers as P-frames fill the reference
 * slots (~295KB, not ~147KB) - confirmed on real hardware: an earlier
 * revision of this example that left that call out compiled and started
 * up fine, then crashed a few frames into actual decode once the extra
 * reference slots filled.
 * `#define H264_MAX_WIDTH`/`H264_MAX_HEIGHT` below override the
 * project-wide QVGA default for *this sketch's own compiled copy* of
 * the decoder only (h264_config.h's `#ifndef` guards support this) -
 * they don't change any other sketch's budget. If your board has PSRAM,
 * remove the two #defines to decode at the full QVGA default instead -
 * see PSRAMAllocatorESP32.h / examples/DecodeFromProgmemPSRAM for the
 * pattern to place the picture buffers in PSRAM as well, which is the
 * more comfortable path for QVGA + a display library together. The
 * decoded 256x192 picture is centered within the 320x240 panel
 * (kOffsetX/kOffsetY below, 32px/24px) rather than pinned to the
 * top-left corner; clearScreen() below blanks the whole panel once at
 * startup so the uncovered border on all four sides reads as a clean
 * black frame instead of leftover power-on garbage.
 *
 * A full-screen RGB565 framebuffer (256x192x2 = ~98KB here, ~150KB at
 * QVGA) is still worth avoiding as one contiguous allocation on a plain
 * ESP32, whose internal DRAM is split into several smaller
 * non-contiguous pools - the same class of problem TinyH264's own
 * internal picture buffers hit at QVGA and were fixed for (see
 * docs/memory-budget.md). This example sidesteps it the same way
 * TinyGPU's own bouncing-ball example
 * (https://github.com/pschatzmann/TinyGPU/blob/main/examples/bouncing-ball/bouncing-ball.ino)
 * does for a full-screen redraw: render and push the screen in small
 * horizontal bands instead of one whole-frame buffer. Unlike
 * bouncing-ball's dirty-rect sprite update (only the ball's small
 * footprint changes per frame), *every* pixel changes every decoded
 * video frame, so this sketch always redraws all bands - but each
 * individual band buffer stays small regardless.
 *
 * TinyH264Decoder::toRGB565()'s windowed overload was built for exactly
 * this use case - see docs/decoding.md - it fills one band's worth of
 * pixels directly from the decoder's internal YUV picture in one bulk
 * call, no per-pixel conversion loop needed here.
 *
 *   TFT_MOSI -> GPIO13   TFT_MISO -> GPIO12   TFT_SCLK -> GPIO14
 *   TFT_CS   -> GPIO15   TFT_DC   -> GPIO2    TFT_RST  -> not connected (-1)
 *   TFT_BL   -> GPIO27 (backlight)
 * (same pinout as TinyGPU's bouncing-ball example - a common 3.5"
 * 240x320 ILI9341 SPI TFT "ESP32 LVGL WiFi & Bluetooth" dev board;
 * adjust for your own wiring.)
 *
 * If the screen stays blank: check the backlight pin and SPI pins first
 * (see bouncing-ball's own troubleshooting notes). If it lights up but
 * the image is rotated, mirrored, or has swapped colors: adjust
 * kMadctlLandscape below - exact MADCTL bit meaning varies by panel
 * vendor even for the same ILI9341 controller IC, and this value is
 * unverified against real hardware (no ILI9341 panel was attached to
 * the machine this example was written on - only the decode pipeline
 * and encoded test clip were verified, against real ffmpeg output, not
 * the display path itself).
 *
 * ESP32-only: uses ESP32-Arduino's 4-argument SPI.begin(sclk, miso, mosi,
 * cs) pin-remap overload (same as TinyGPU's own bouncing-ball reference
 * example), which RP2040-Arduino's SPI class doesn't have - unlike
 * TinyH264's own DecodeFromProgmem/EncodeSyntheticFrame examples, this
 * one isn't cross-platform, since the display wiring/pin-remap API is
 * inherently board-specific, not something the decoder itself
 * constrains.
 */

#if !defined(ARDUINO_ARCH_ESP32)
#error \
    "DecodeToDisplay is ESP32-only (see the file header comment above) - it uses ESP32-Arduino's 4-argument SPI.begin() pin-remap API."
#endif

// Overrides the project-wide QVGA compile-time default for this sketch's
// own compiled copy of the decoder only - see the file header comment
// above for why. Must come before the TinyH264Decoder.h include below.
#define H264_MAX_WIDTH 256
#define H264_MAX_HEIGHT 192

#include <TinyGPU.h>
#include <TinyGPU/DisplayDriverSPI.h>
#include <TinyH264Decoder.h>

#include "h264_display_test_clip.h"

using namespace tinyh264;
using namespace tinygpu;

// --- display geometry -----------------------------------------------------
// The ILI9341 panel itself is 320x240 (see the pinout comment below), but
// the decoded picture is only 256x192 - see the file header comment above
// for why. kPanelWidth/kPanelHeight are used by clearScreen() below (to
// blank the border the decoded picture doesn't cover) and by
// kOffsetX/kOffsetY (to center that picture in the border instead of
// pinning it to the top-left corner).
constexpr int kPanelWidth = 320;
constexpr int kPanelHeight = 240;
constexpr int kDisplayWidth = 256;
constexpr int kDisplayHeight = 192;
constexpr int kOffsetX = (kPanelWidth - kDisplayWidth) / 2;    // 32
constexpr int kOffsetY = (kPanelHeight - kDisplayHeight) / 2;  // 24

// --- band buffer geometry ---------------------------------------------------
// 256 x 16 x 2 bytes = 8,192 bytes per band - see the file header comment
// above for why this stays small instead of one whole-frame buffer.
// kDisplayHeight and kPanelHeight must both be evenly divisible by
// kBandHeight (192/16=12, 240/16=15 - both hold).
constexpr int kBandHeight = 16;
constexpr int kBandCount = kDisplayHeight / kBandHeight;

// The embedded clip was encoded at 10 fps (see h264_display_test_clip.h) -
// paces playback to roughly match instead of running as fast as decode +
// SPI transfer allow.
constexpr int kClipFps = 10;

// --- SPI / display pins -----------------------------------------------------
constexpr int8_t kPinMosi = 13;
constexpr int8_t kPinMiso = 12;
constexpr int8_t kPinSclk = 14;
constexpr int8_t kPinCs = 15;
constexpr int8_t kPinDc = 2;
constexpr int8_t kPinRst = -1;
constexpr int8_t kPinBacklight = 27;

/*
 * ILI9341Driver::begin() (TinyGPU) leaves the panel in whatever
 * orientation it powers on in - it doesn't send a MADCTL (command 0x36)
 * command at all, since TinyGPU's own examples so far have all used the
 * panel's default (portrait) orientation. This decoder always outputs
 * landscape pictures, so this subclass sends a landscape MADCTL right
 * after begin(). See the file header comment above re: kMadctlLandscape
 * not being hardware-verified.
 */
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

/*
 * TinyGPU's Surface<RGB565> (aliased SurfaceRGB565) only exposes a const
 * uint8_t* data() accessor (matching ISurface - existing TinyGPU
 * consumers write into a Surface one pixel at a time via setPixel()).
 * TinyH264Decoder::toRGB565()'s windowed overload instead fills a whole
 * band's worth of pixels in one bulk call, matching this decoder's own
 * "caller-provided buffer, no internal allocation, no per-pixel loop in
 * the hot path" design throughout - this thin subclass exposes a mutable
 * pointer into the same underlying buffer Surface already owns (its
 * protected `buffer` member), rather than converting through a
 * setPixel() loop or a second copy.
 */
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

/*
 * One-time full-panel clear: the decoded picture only covers a centered
 * 256x192 region of the physical 320x240 panel (see the file header
 * comment above), so without this the border on all four sides would be
 * left as whatever garbage was in the panel's RAM at power-on. Uses its
 * own, panel-width (320) band - unlike `band` above (256 wide, resident
 * for the whole sketch), this one is a local, transient allocation,
 * freed via end() as soon as the clear finishes, so it doesn't add to
 * the steady-state heap footprint the file header comment's math is
 * based on.
 */
static void clearScreen() {
  SurfaceRGB565 clearBand(kPanelWidth, kBandHeight, FontRGB565);
  clearBand.begin();
  clearBand.clear(RGB565(0, 0, 0));
  for (int y = 0; y < kPanelHeight; y += kBandHeight) {
    tftDriver.writeData(clearBand, 0, y);
  }
  clearBand.end();
}

/*
 * Free-heap reporting is not part of any Arduino-portable API - see
 * DecodeFromProgmem's identical helper for why this checks
 * ARDUINO_ARCH_ESP32/ARDUINO_ARCH_RP2040 explicitly rather than using
 * Serial.printf() (an ESP32/RP2040-only extension). Also prints the
 * largest free contiguous block (heap_caps_get_largest_free_block()) -
 * the number that actually matters for whether one picture buffer's
 * single allocation will succeed, not just the free-heap total; see the
 * file header comment above for why this distinction is why this
 * example decodes at 256x192 rather than the full QVGA panel resolution.
 */
static void printFreeHeap(const char* label) {
  Serial.print(label);
  Serial.print(": ");
  Serial.print(ESP.getFreeHeap());
  Serial.print(" bytes free, largest block ");
  Serial.print(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.println(" bytes");
}

// Called once per decoded picture, from inside decoder.write() below.
// Converts and pushes the picture to the display one band at a time,
// instead of converting/holding the whole frame at once. Each band is
// written at (kOffsetX, kOffsetY + bandY) rather than (0, bandY), so the
// 256x192 picture lands centered within the 320x240 panel instead of
// pinned to the top-left corner - the decoded pixel data itself is
// unaffected, only where on the panel it's addressed to.
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

  /*
   * Load-bearing for the memory budget in the file header comment above:
   * that math assumes exactly 2 resident picture buffers (curFrame_ + 1
   * reference). Without this call, the compile-time default
   * (H264_MAX_REF_FRAMES=3) lets the decoder grow to *4* resident
   * buffers as P-frames fill the reference slots - ~295KB at 256x192,
   * not ~147KB - which does not fit. (This is exactly what happened
   * when this line was accidentally left out of an earlier revision of
   * this example: setup() completed fine, since only 1-2 buffers exist
   * at that point, and it crashed a few frames into actual decode once
   * enough P-frames had filled the 3rd/4th reference slot.)
   */
  decoder.setMaxRefFrames(1);
  decoder.setCallback(onFrame);

  /*
   * The embedded clip is in PROGMEM (flash); copy it to a small RAM
   * buffer since the decoder reads directly from the pointer it's given.
   */
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
