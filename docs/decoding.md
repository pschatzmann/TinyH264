# Decoding

```cpp
#include <TinyH264Decoder.h>

using namespace tinyh264;

TinyH264Decoder<> decoder;   // <> = default allocator (std::allocator<uint8_t>, plain heap)

void onFrame(TinyH264Decoder<> &decoder, void *userData) {
  // decoder.width(), decoder.height()
  // decoder.y()/u()/v() + decoder.strideY()/strideUV(): YUV 4:2:0 planar, 8-bit
  // decoder.getY(x,y)/getU(x,y)/getV(x,y): one sample at a time, if you don't
  //   want to do the stride math yourself (see "Accessing pixel data" below)
}

void setup() {
  decoder.setCallback(onFrame);
}

void feed(const uint8_t *annexBData, size_t size) {
  // annexBData is one or more complete NAL units (00 00 01 / 00 00 00 01
  // start codes), e.g. read from an SD card, a network stream, or a
  // camera module's H.264 output.
  decoder.write(annexBData, size);   // onFrame() fires once per decoded picture
  if (decoder.hasError()) {
    // decoder.lastStatus() == kUnsupported (unimplemented stream feature)
    //                      or kError (corrupt/truncated bitstream)
  }
}
```

Everything the library exposes - `TinyH264Decoder`, `PSRAMAllocatorESP32`,
and all of the `decoder/` implementation - lives in the `tinyh264`
namespace; `using namespace tinyh264;` (as above) or explicit `tinyh264::`
qualification both work.

## Accessing pixel data

Decoded pictures are exposed as three separate 8-bit planes - Y (luma/
brightness, full resolution), U and Cb, and V/Cr (chroma/color, each at
*half* resolution in both dimensions - 4:2:0 subsampling, the same
convention `ffmpeg -pix_fmt yuv420p` uses). Available inside the frame
callback (or after `decodeNext()` returns `kFrameReady`) three ways,
depending on what you're doing with the picture:

- **Raw plane pointers** - `y()`/`u()`/`v()` plus `strideY()`/`strideUV()`
  (bytes per row - always use these rather than assuming stride == width,
  even though they're equal in this library today). Fastest option for
  processing a whole plane; you do the row/column indexing yourself:
  ```cpp
  uint8_t luma = decoder.y()[py * decoder.strideY() + px];
  uint8_t cb   = decoder.u()[(py / 2) * decoder.strideUV() + (px / 2)];
  ```
- **Single-sample getters** - `getY(x, y)`, `getU(x, y)`, `getV(x, y)` do
  that indexing (including the chroma /2 subsampling) for you, for
  one-pixel-at-a-time code (a sanity check, a naive per-pixel filter) where
  the raw pointer math would just add noise. Not bounds-checked, same as
  the plane accessors - `x`/`y` must stay within `width()`/`height()`.
- **RGB565** - `toRGB565(uint16_t *dst, size_t dstCapacity)` converts the
  whole picture at once into a caller-provided buffer, for handing
  straight to a display library (TFT_eSPI, Adafruit_GFX, LovyanGFX, ...)
  that expects 16-bit color instead of raw YUV. `dstCapacity` is `dst`'s
  size in `uint16_t` entries (not bytes) - if it's smaller than
  `width()*height()`, nothing is written and the call returns `0`
  (a raw pointer carries no size of its own, so this check is what
  catches an under-sized buffer instead of silently corrupting memory
  past its end); on success it returns the number of entries actually
  written (`width()*height()`):
  ```cpp
  static uint16_t rgb[176 * 144];  // caller-owned - write directly into
  if (decoder.toRGB565(rgb, 176 * 144) == 0) {  // your display's own framebuffer
    // buffer too small for this picture's actual width()*height()
  }
  ```
  Uses the ITU-R BT.601 limited-range conversion (matches typical H.264
  encoder output), verified against `ffmpeg`'s own conversion to within 1
  LSB per channel - see `src/decoder/h264_rgb.h`.

  For converting in smaller pieces instead - e.g. pushing to a display in
  tiles (matching how many small-display drivers already expect windowed
  `pushColors()` calls), or spreading the work across multiple `loop()`
  iterations rather than one large blocking call - `toRGB565(x, y, dx,
  dy, dst, dstCapacity)` converts just a `dx`x`dy` rectangle starting at
  `(x, y)` into a buffer (packed with no row padding), the same way
  checking `dstCapacity` against `dx*dy`:
  ```cpp
  static uint16_t tile[32 * 32];
  decoder.toRGB565(64, 32, 32, 32, tile, 32 * 32);  // 32x32 tile at (64,32)
  ```
  `x`/`y` must be even (chroma is subsampled 2x - an odd origin would
  split a shared chroma sample across two tiles, shifting color
  alignment at the boundary) - that part is *not* checked, only the
  destination buffer size is; verified tile-for-tile identical to the
  corresponding region of the whole-frame conversion above, at several
  tile positions/sizes including frame corners and a non-multiple-of-16
  size.
- **RGB666** / **RGB888** - `toRGB666(dst, dstCapacity)` (18-bit color)
  and `toRGB888(dst, dstCapacity)` (full 24-bit color) work exactly like
  `toRGB565()` above, including the windowed `(x, y, dx, dy, dst,
  dstCapacity)` overloads - only the output format differs. Both use
  `uint8_t* dst` at **3 bytes per pixel** (R, G, B order), so
  `dstCapacity` must be at least `width()*height()*3` (or `dx*dy*3` for
  the windowed form):
  ```cpp
  static uint8_t rgb888[176 * 144 * 3];
  decoder.toRGB888(rgb888, sizeof(rgb888));
  ```
  RGB888 matches `ffmpeg`'s `rgb24` (verified the same way as RGB565,
  though with a wider observed tolerance - up to 3/255 per channel, since
  RGB565's coarser 5/6-bit output happens to absorb most of the rounding
  noise that's fully visible at RGB888's full 8-bit precision; still
  imperceptible). RGB666 has no raw pixel format in `ffmpeg` to check
  against, so it's verified by self-consistency instead: every RGB666
  byte is defined as, and confirmed to exactly equal, the corresponding
  RGB888 byte with its bottom 2 bits cleared (`& 0xFC`) - i.e. each
  channel's 6 significant bits left-justified in bits 7:2, the wire
  format real 18-bit-interface TFT controllers (e.g. ILI9488 in 18-bit
  mode) expect. See `src/decoder/h264_rgb.h` for the full rationale.
- **YUV420 as one array** - `toYUV420(dst, dstCapacity)` copies the
  decoded picture into one tightly-packed buffer instead of the three
  separate `y()`/`u()`/`v()` plane pointers - standard "I420" layout (Y
  bytes, then U bytes, then V bytes, no row padding). Useful when you
  want one contiguous block to write to storage, send over a socket, or
  hand to another thread/task, rather than juggling three pointers - no
  color-space conversion happens here, just a repacking of the same
  bytes `y()`/`u()`/`v()` already expose. `dstCapacity` must be at least
  `width()*height() + 2*(width()/2)*(height()/2)`; same size-checked,
  `0`-on-too-small behavior as the RGB converters (non-zero return is the
  number of bytes written), plus the same windowed `(x, y, dx, dy, dst,
  dstCapacity)` overload:
  ```cpp
  static uint8_t yuv[176 * 144 + 2 * 88 * 72];
  decoder.toYUV420(yuv, sizeof(yuv));
  ```
  Note this is a *copy*, not a zero-copy view - unlike `y()`/`u()`/`v()`,
  which point directly into this decoder's own internal frame storage
  (itself one contiguous Y+U+V allocation internally, but at fixed
  offsets sized to the compile-time `H264_MAX_WIDTH`/`H264_MAX_HEIGHT`
  maximum, not necessarily this stream's actual resolution - so it can't
  be handed out directly as a tightly-packed buffer in general).

By default up to `H264_MAX_REF_FRAMES` (3) reference pictures are kept
resident, matching `ffmpeg`'s own default preset. If you know your source
stream needs fewer (check with `ffmpeg -i your.264 -c copy -bsf:v
trace_headers -f null - 2>&1 | grep max_num_ref_frames`, or just the
`-x264-params ref=N` you encoded with), lower it at runtime to reclaim
memory - each reference picture costs one full frame buffer (~38KB at
QCIF):

```cpp
decoder.setMaxRefFrames(1);  // call before write(); default is H264_MAX_REF_FRAMES
```

A stream that actually needs more references than the current
`maxRefFrames()` (or more than the compile-time `H264_MAX_REF_FRAMES`
upper bound) is rejected as `kUnsupported`, not mis-decoded.

`TinyH264Decoder` is templated on an allocator (`std::allocator<uint8_t>` by
default), used for the picture buffers (see [Memory budget](memory-budget.md)).
Pass a custom allocator to place them in PSRAM or a dedicated pool instead
of the default heap - `PSRAMAllocatorESP32<uint8_t>` (in
`PSRAMAllocatorESP32.h`) does this via ESP-IDF's `heap_caps_malloc()`, for
boards with PSRAM (e.g. most ESP32-S3 modules):

```cpp
#include <TinyH264Decoder.h>
#include <PSRAMAllocatorESP32.h>

using namespace tinyh264;

TinyH264Decoder<PSRAMAllocatorESP32<uint8_t>> decoder;
```

See `examples/DecodeFromProgmemPSRAM/` for the complete version. With
PSRAM absorbing the two picture buffers, `H264_MAX_WIDTH`/`H264_MAX_HEIGHT`
can also be raised beyond QCIF, since they no longer compete with the rest
of the sketch for internal DRAM.

See `examples/DecodeFromProgmem/` for a complete, self-contained sketch
(a tiny embedded test clip - no camera/SD/network needed) that runs on
any ESP32 board as a smoke test.

See `examples/DecodeToDisplay/` for a complete sketch that decodes a
test clip and streams it to an ILI9341 SPI TFT via
[TinyGPU](https://github.com/pschatzmann/TinyGPU), using the same
band-by-band `toRGB565()` windowed overload above to avoid ever needing
a full-screen RGB565 framebuffer in one contiguous allocation. Decodes
at 256x192, not the project's QVGA compile-time default or the panel's
full 320x240 resolution, via a local `#define H264_MAX_WIDTH`/
`H264_MAX_HEIGHT` override (h264_config.h's `#ifndef` guards support
this per sketch) - real-hardware testing on a plain ESP32 showed QVGA's
~283KB combined requirement (2 picture buffers + the QVGA-sized
per-macroblock metadata table) plus TinyGPU/SPI's own overhead exceeding
actual free heap and crashing with an uncaught `std::bad_alloc`, even
though no single allocation was too large on its own - a genuine
total-memory shortfall, not the fragmentation problem `toRGB565()`'s
band-streaming design solves. 256x192 was solved for as the largest
4:3, macroblock-aligned resolution leaving a real (~110KB) safety
margin against the measured free heap, rather than the largest that
fits at zero margin (~326x245 - i.e. QVGA itself was already at that
wall). See the sketch's file
header comment for the exact numbers and how to raise it back to QVGA on
a PSRAM-equipped board. ESP32-only (SPI pin-remap API); ILI9341
wiring/orientation may need adjusting for your specific panel.
