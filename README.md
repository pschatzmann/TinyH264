# TinyH264

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-blue.svg)](https://www.arduino.cc/reference/en/libraries/)
[![CMake](https://img.shields.io/badge/CMake-Supported-blue.svg)](https://cmake.org/)
[![IDF Component](https://img.shields.io/badge/IDF-Component-blue.svg)](https://github.com/pschatzmann/TinyH264)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-green.svg)](https://www.gnu.org/licenses/gpl-3.0)


A minimal H.264 (Baseline Profile, CAVLC) video encoder and decoder - written from scratch in header-only C++, for microcontrollers
such as the ESP32 and RP2040 (Raspberry Pi Pico). No dynamic memory
allocation in the hot path, no external dependencies beyond the C++
standard library headers already available on Arduino cores - validated
via `arduino-cli` against `esp32:esp32:esp32`, `esp32:esp32:esp32s3`
(PSRAM), and `rp2040:rp2040:rpipico`.

## Scope

I- and P-slice decoding (intra + inter prediction,
motion compensation, the in-loop deblocking filter) are implemented and
validated **pixel-exact for luma and chroma** against real `ffmpeg`/
x264-encoded streams, including multi-frame GOPs with deblocking active
and genuine multi-reference-frame prediction (see below).

Supported:
- Baseline profile: I-slices (I_4x4, I_16x16, I_PCM) and P-slices
  (P_16x16, P_16x8, P_8x16, P_8x8 with 8x8/8x4/4x8/4x4 sub-partitions,
  P_Skip), CAVLC entropy coding only (no CABAC)
- Quarter-pel luma motion compensation (6-tap FIR), eighth-pel chroma
  (bilinear)
- The deblocking filter, including per-block boundary-strength derivation
  for both intra and inter macroblocks, and reference-picture-differs
  detection for multi-reference streams
- Up to `H264_MAX_REF_FRAMES` reference pictures (default 3, matching
  `ffmpeg`'s own default `-preset medium` - see
  [Memory budget](#memory-budget) and [Usage](#usage) for the runtime
  `setMaxRefFrames()` knob), using the *default* reference picture list
  (clause 8.2.4.2) and *sliding window* marking (clause 8.2.5.3) - explicit
  reference list reordering (`ref_pic_list_modification()`) and adaptive
  (MMCO-based) reference marking are parsed but rejected as unsupported
  rather than implemented, since real Baseline encoders overwhelmingly use
  the default/sliding-window behavior this decoder does implement

Not supported (by design, for this decoder's target: small, single-camera-
style Baseline streams on constrained MCUs):
- CABAC, B-slices, weighted prediction, explicit reference list reordering,
  adaptive (MMCO) reference marking, FMO/ASO slice groups, interlaced/MBAFF
  content, High-profile-and-above features (8x8 transform, scaling lists)

Streams using any of the above are detected and rejected
(`DecodeStatus::kUnsupported`) rather than mis-decoded.

`TinyH264Encoder`, the matching encoder, supports I_16x16/I_4x4 intra
and P_16x16/P_Skip inter macroblocks with automatic per-macroblock
Intra fallback on a poor motion match (single reference frame, integer-
pel-only motion search - no P_16x8/P_8x16/P_8x8 sub-partitions, no
multi-reference) plus simple rate control - see [Encoding](#encoding)
for full scope and usage.


## Memory budget

Both `TinyH264Decoder` and `TinyH264Encoder` are sized by default for a
plain ESP32 (no PSRAM), QCIF (176x144) - the budget below applies to
*both*, not just the decoder, since `TinyH264Encoder` holds its own
closed-loop reconstruction picture buffer internally (see
[Encoding](#encoding)) with the same per-frame cost as the decoder's.
Figures below are real `arduino-cli` compiles (`esp32:esp32:esp32`), not
estimates - an empty sketch alone already uses ~22KB of static RAM
(Arduino/ESP-IDF framework baseline), so that's the floor everything
below is measured against:

| Object (static RAM, before any picture is actually decoded/encoded) | Cost |
|---|---|
| `TinyH264Decoder<>` | ~47KB |
| `TinyH264Encoder<>` | ~46KB |
| Both together (e.g. a decode-then-re-encode relay) | ~93KB (purely additive) |

**Decoder**: one current + up to `H264_MAX_REF_FRAMES` (default 3)
reference YUV 4:2:0 frame buffers at ~38KB each - ~152KB total by
default - heap-allocated on first use (not counted in the static figures
above), plus the ~47KB static cost (NAL scratch buffer, per-macroblock
metadata table) shown above. See `src/decoder/h264_decoder.h`'s
`setMaxRefFrames()` (or the `TinyH264Decoder` wrapper of the same name)
to lower the *runtime-active* reference count below the compile-time
`H264_MAX_REF_FRAMES` and reclaim some of that ~152KB without rebuilding
(each reference picture not needed saves ~38KB at QCIF).

**Encoder**: two picture buffers - the current reconstruction and the
single reference frame P-frames motion-compensate against (see
[Encoding](#encoding)) - each ~38KB, ~76KB total, both heap-allocated the
first time `encodeFrame()` is called (not counted in the static figures
above). Both are allocated unconditionally on that first call, even for
a caller whose stream never actually produces a P-frame - a real, if
modest, inefficiency in exchange for a simple "copy the just-finished
picture into the reference slot" design rather than a more complex
buffer-swapping scheme; ~38KB is the price of that simplicity for an
I-frame-only use case. On top of the heap cost, the ~46KB static cost
shown above (a slice scratch buffer, sized the same as the decoder's NAL
scratch buffer, plus the same per-macroblock metadata table - unchanged
by P-frame/rate-control support, both add only a handful of `int`/`bool`
member fields). The `encodeFrameRgb888()`/`encodeFrameRgb666()`/
`encodeFrameRgb565()`/`encodeFrameYuv422()` convenience overloads add
~38KB more, but only if actually called - their conversion scratch
buffers are heap-allocated lazily on first use (or eagerly via
`begin(true)` - see below), so a sketch that only calls the plain
`encodeFrame()` never pays for them.

**Explicit lifecycle control**: both classes allocate lazily by default
(as described above - first real encode/decode call), but both also
expose `begin()`/`end()` if you'd rather control exactly when that
happens: `begin()` reserves the picture buffers (and, on the encoder
side, optionally the RGB/YUV422 conversion scratch too - pass
`begin(true)`) up front instead of waiting for the first call, so an
allocation failure surfaces deterministically in `setup()` rather than
mid-stream; `end()` releases everything back to the heap and resets the
object to a fresh state, for reclaiming that memory before doing
something else memory-hungry without destructing and reconstructing the
whole object. Neither is required - the lazy default and the
destructor's own cleanup are enough for most sketches.

See `src/h264_config.h` to change the compile-time
`H264_MAX_REF_FRAMES`/`H264_MAX_WIDTH`/`H264_MAX_HEIGHT` upper bounds
(shared by both classes), e.g. to raise resolution beyond QCIF if
targeting a board with PSRAM (e.g. ESP32-S3) - see the `PSRAMAllocatorESP32`
example in [Usage](#usage), which works identically for `TinyH264Encoder<PSRAMAllocatorESP32<uint8_t>>`.

## Performance

Measured on real hardware, not estimated: QCIF (176x144), `examples/
DecodeFromProgmem`'s built-in benchmark (30 repetitions of the embedded
test clip, `micros()`-timed per frame - see that sketch for the
methodology, including why the timing checkpoint is taken around, not
across, the `Serial` output).

| Board | avg | min | max |
|---|---|---|---|
| ESP32 | 20494 us (48.8 fps) | 20296 us (49.3 fps) | 20647 us (48.4 fps) |
| ESP32-S3 | 16045 us (62.3 fps) | 15243 us (65.6 fps) | 16564 us (60.4 fps) |
| RP2040 | 29008 us (34.5 fps) | 22700 us (44.1 fps) | 32263 us (31.0 fps) |
| RP2350 | 26193 us (38.2 fps) | 19782 us (50.6 fps) | 29511 us (33.9 fps) |
| STM32F723 | 13926 us (71.8 fps) | 11377 us (87.9 fps) | 15567 us (64.2 fps) |

All comfortably clear real-time (15-30 fps) for QCIF at Baseline/CAVLC
with the deblocking filter active. None of this decoder's hot paths
(CAVLC entropy decoding, motion-compensation interpolation, the
deblocking filter) are hand-optimized for any of these targets - see
`src/decoder/h264_cavlc.h`'s `decodeVlc()` and `src/common/h264_motion.h`
for what a profiling-driven optimization pass would likely target first
if you need more headroom than this.

## Decoding

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

### Accessing pixel data

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
default), used for the picture buffers (see [Memory budget](#memory-budget)).
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

## Encoding

`TinyH264Encoder` is a matching Baseline-profile CAVLC *encoder*,
deliberately kept simple for a real-time embedded target rather than
maximally compression-efficient: I_16x16 and I_4x4 intra macroblocks
(SAD-based mode decision, no full rate-distortion optimization), P_16x16
and P_Skip inter macroblocks against a single reference frame (small
integer-pel-only motion search - no sub-pel refinement, no P_16x8/
P_8x16/P_8x8 sub-partitions, no multi-reference), automatic per-
macroblock Intra fallback within a P-slice when motion-compensated
prediction is a poor match (scene cuts, occlusion, content entering/
leaving the search range), and simple real-time-appropriate rate
control.

**`encodeFrame()` is the only public encode entry point** - configure
picture size/keyframe interval (via the constructor) and QP policy
(via `setQp()`/`setTargetBitrate()`) once, then call `encodeFrame()`
once per picture, in order, and it decides I-frame vs. P-frame for you.
There's no explicit "encode an I-frame now" call - `encodeFrame()` is
the whole API surface for driving the encoder, deliberately, so there's
nothing to choose between and no width/height/stride/qp to repeat on
every call:

```cpp
#include <TinyH264Encoder.h>

using namespace tinyh264;

uint8_t bitstream[16384];

// width/height must each be a multiple of 16 (no cropping support yet);
// the third argument is an optional periodic keyframe interval (a GOP
// size - 0, the default, means "only re-key when unavoidable": no
// reference yet, or a resolution change).
TinyH264Encoder<> encoder(width, height, /*keyframeInterval=*/0);
encoder.setQp(26);  // 0-51, lower = higher quality/larger output

for (...) {
  // One call per picture, in order - the first call (and any call after
  // width/height change) becomes a self-contained I-frame (SPS + PPS +
  // IDR slice); every other call becomes a motion-compensated P-frame
  // against the previous picture (just the P-slice, no SPS/PPS resent).
  size_t n = encoder.encodeFrame(y, u, v, bitstream, sizeof(bitstream));
  if (n == 0) {
    // buffer too small, or width/height not a multiple of 16
  }
  // bitstream[0..n) is a complete Annex-B NAL unit (or units) - decodable
  // by this library's own TinyH264Decoder or any conformant H.264
  // decoder, verified against real ffmpeg
  // (test/native/test_encode_autoframe.cpp and friends).
}
```

If width/height/keyframe interval aren't known at construction time,
`setSize(width, height)`/`setKeyframeInterval(frames)` set them
afterward instead - the constructor is purely a convenience wrapper
around those two setters (`TinyH264Encoder<> encoder;` with no
arguments still works, matching this class's behavior before the
constructor existed; `setSize()` must then be called before the first
`encodeFrame()` call). `setStride(strideY, strideC)` overrides the Y/C
plane row strides `encodeFrame()` uses (they otherwise default to
tightly packed - `strideY == width`, `strideC == width/2` - the common
case for a camera/framebuffer with no row padding); `setPackedStride(stride)`
is the equivalent for the RGB/YUV422 overloads below.

`setKeyframeInterval(frames)` (or the constructor's third argument)
inserts a periodic keyframe automatically (a GOP size, in the standard
sense - keyframes land at picture 0, `frames`, `2*frames`, ...), useful
for stream resilience or letting a decoder join mid-stream; it's off by
default (`encodeFrame()` still always re-keys on the two cases that are
never optional - no reference yet, or a resolution change). There is
deliberately no way to force a keyframe on a specific call beyond that -
a resolution change (`setSize()` with a new width/height) is the only
other trigger.

I_16x16-vs-I_4x4, Inter-vs-Skip, and (within a P-slice) Inter-vs-Intra
mode decisions are all automatic (no separate call needed) - every
macroblock picks whichever the encoder's SAD-based heuristics estimate
will compress better. I_16x16-vs-I_4x4 gives a real, measurable
improvement over the simpler always-I_16x16 path (10-18% smaller files
at most QPs, `test/native/test_encode_i4x4.cpp`); the P-slice Intra
fallback is dramatically more consequential on an actual scene cut - a
real test (one real image abruptly followed by a completely different
one, `test/native/test_encode_pslice_intra_fallback.cpp`) measured
**356 bytes at 48.1dB PSNR with the fallback vs. 8571 bytes at only
34.0dB without it** for that one P-frame - smaller *and* better, not a
tradeoff, since forcing motion compensation from an unrelated reference
frame is strictly worse than just intra-coding the new content directly.
None of this is full rate-distortion optimization, just fast SAD-based
heuristics - see each decision function's own comment
(`h264_macroblock_encode.h`/`h264_macroblock_encode_inter.h`) for the
exact thresholds and why they're not empirically tuned against a rate-
distortion curve.

**Rate control**: call `setTargetBitrate()` and leave `qp` at its
default (or call `setQp(-1)` explicitly - same sentinel meaning) to let
the encoder pick its own QP each frame, adapted toward a target bitrate
instead of a fixed QP:

```cpp
encoder.setTargetBitrate(300000, /*fps=*/25.0);  // ~300kbps at 25fps
encoder.setQp(-1);  // rate control (also the default if setQp() is never called)

for (...) {
  size_t n = encoder.encodeFrame(y, u, v, bitstream, sizeof(bitstream));
}

encoder.lastQp();  // what QP the most recent call actually used
```

A simple, real-time-appropriate proportional feedback controller (not a
two-pass/lookahead one): after each frame, QP is nudged up (coarser,
smaller future frames) if that frame came out well over target, or down
(finer) if well under, with a small dead zone around the target so it
doesn't chase every frame's natural content-driven size variation, and a
capped per-frame step to avoid visible quality swings between adjacent
frames. Verified against a real 10-frame sequence at three very
different targets (well below, close to, and well above what a fixed
QP=26 encode of that content naturally produces): QP moves in the
correct direction every time and settles within a real, if not tightly
converged after only 10 frames, range of the target
(`test/native/test_encode_ratecontrol.cpp`) - a longer real sequence
converges more precisely than this project's own short test streams can
demonstrate.

For source data that isn't already three separate Y/U/V planes,
`encodeFrameRgb888()`/`encodeFrameRgb666()`/`encodeFrameRgb565()`/
`encodeFrameYuv422()` take a single packed-pixel source buffer instead
(same automatic I/P dispatch and setSize()/setQp()/setPackedStride()
configuration as `encodeFrame()`) - useful when a camera module or
framebuffer already produces RGB or packed YUV 4:2:2 rather than planar
YUV 4:2:0:

```cpp
// RGB888: 3 bytes/pixel, R/G/B order (matches TinyH264Decoder::toRGB888()).
encoder.encodeFrameRgb888(rgb, bitstream, sizeof(bitstream));

// RGB565: uint16_t/pixel, 5-6-5 packed (matches toRGB565()).
encoder.encodeFrameRgb565(rgb565, bitstream, sizeof(bitstream));

// YUYV-order packed YUV 4:2:2 (Y0 U0 Y1 V0 per pixel pair) - the common
// camera-module convention (e.g. OV2640/OV7670 output).
encoder.encodeFrameYuv422(yuyv, bitstream, sizeof(bitstream));
```

These convert internally to YUV 4:2:0 (see `src/encoder/h264_color_convert.h`)
before encoding - a real, if modest, extra precision loss on top of the
usual DCT quantization, not a lossless passthrough. The conversion
matrix is the standard ITU-R BT.601 limited-range forward transform (the
algebraic inverse of the matrix `h264_rgb.h` already uses for decode-side
RGB output), verified both by round-tripping through that existing,
ffmpeg-cross-checked matrix and against real ffmpeg's own `-pix_fmt
rgb24 -> -pix_fmt yuv420p` conversion of a real image
(`test/native/test_encode_color_formats.cpp`); typical quality impact on
top of the plain YUV-planes path is well under 1 dB PSNR. The RGB/YUV422
conversion scratch buffers are allocated lazily on first use of one of
these RGB/YUV422 methods (matching `Frame`'s own allocate-on-first-use
convention, or eagerly via `begin(true)` - see "Memory budget" above) -
calling only the plain `encodeFrame()` never pays for them.

Verified round-trip against real `ffmpeg` decode: I-frames bit-exact at
QP 18 and above across the whole QP range 0-51, within +/-1 (a handful
of pixels, a rounding-tie edge case in the deblocking filter at very
fine quantization steps) at QP 0/10 (`test/native/test_encode_iframe.cpp`)
- see the encoder source files' own comments for the two real bugs this
verification caught during development (a missing inverse-Hadamard-
transform step in the DC reconstruction path, and an out-of-bounds
zigzag-table read on the chroma DC block - both now regression-tested).
A real 10-frame motion sequence (1 I-frame + 9 P-frames) is **bit-exact
against real ffmpeg across all 10 frames** at QP 26
(`test/native/test_encode_pframe.cpp`) - P-frames on that sequence
measured 42.9-44.8dB PSNR against the source, meaningfully higher than
the I-frame's own 38.4dB, the expected benefit of temporal (motion-
compensated) prediction on top of spatial coding.

See `examples/EncodeSyntheticFrame/` for a complete, self-contained
sketch (a synthetic gradient test pattern that shifts a little each
frame, giving P-frames genuine motion to compensate - no camera/
SD/network needed) demonstrating the constructor, `encodeFrame()`,
periodic keyframes, and rate control together, that runs on any
ESP32/RP2040 board as a smoke test.

## Preparing input with ffmpeg

This decoder only implements a deliberately small subset of H.264 (see
[Status](#status)); source video needs to be transcoded to match it. The
command below (verified against this decoder, not just written from spec
memory - including re-verifying after the reference-frame changes below)
produces a compatible stream from any input `ffmpeg` can read:

```sh
ffmpeg -i input.mp4 \
  -vf scale=176:144 -pix_fmt yuv420p \
  -c:v libx264 -profile:v baseline -level 3.0 \
  -g 25 \
  -f h264 output.264
```

What each part is doing, and why it's required rather than just a good
default:

- **`-f h264` (and a plain `output.264`/`.h264` filename)** - forces
  ffmpeg's *raw Annex-B* H.264 muxer, i.e. NAL units delimited by
  `00 00 01`/`00 00 00 01` start codes. This is the only container this
  decoder understands (`h264_nal.h`'s `NalReader`) - muxing into
  `.mp4`/`.mkv`/etc. instead stores NALs length-prefixed with no start
  codes, which this decoder cannot parse directly (you'd need to
  demux/repackage first).
- **`-profile:v baseline`** - by itself already forces off everything
  this decoder doesn't implement *except* the reference-frame count (see
  below): no B-slices, no CABAC, no 8x8 transform, no weighted
  prediction - these are hard restrictions of H.264's Baseline profile
  itself (profile_idc 66), not just x264 defaults, so no separate
  `-x264-params` overrides are needed for them (confirmed via `ffprobe
  -show_streams output.264 | grep profile` reading
  `profile=Constrained Baseline`, and by decoding the result through this
  decoder). An older version of this recipe also passed
  `-x264-params cabac=0:bframes=0:weightp=0:8x8dct=0` explicitly - still
  harmless if you're used to writing it, just redundant.
- **Reference frame count** - the one thing `-profile:v baseline` does
  *not* constrain (Baseline still permits multiple reference pictures).
  This decoder supports up to `H264_MAX_REF_FRAMES` (default 3) stored
  references, matching ffmpeg's own default `-preset medium` - so as of
  that default preset, no extra flag is needed here either. This stops
  being true if you pick a slower preset: `-preset slow`/`slower`/
  `veryslow`/`placebo` use 5/6/8/16 reference frames respectively
  (verified via `ffmpeg -i output.264 -c copy -bsf:v trace_headers -f
  null - 2>&1 | grep max_num_ref_frames` across presets), all above the
  default cap - such a stream is rejected as `kUnsupported` unless you
  either add `-x264-params ref=N` (N <= `H264_MAX_REF_FRAMES`) at encode
  time, or raise `H264_MAX_REF_FRAMES` in `h264_config.h` and rebuild.
  Memory-constrained targets can go the other way and pass `ref=1` to
  minimize picture-buffer RAM regardless of `H264_MAX_REF_FRAMES` - see
  [Memory budget](#memory-budget) and `TinyH264Decoder::setMaxRefFrames()`
  in [Usage](#usage).
- **`-pix_fmt yuv420p`** - 4:2:0 chroma subsampling; the only chroma
  format this decoder's SPS parser accepts (`chromaFormatIdc != 1` is
  flagged unsupported in `h264_sps_pps.h`).
- **`-vf scale=176:144`** - match `H264_MAX_WIDTH`/`H264_MAX_HEIGHT` in
  `h264_config.h` (QCIF by default; raise both the encode resolution and
  those constants together if targeting a board with more RAM/PSRAM -
  see [Memory budget](#memory-budget)). Both dimensions must already be
  multiples of 16 (QCIF is); ffmpeg will not pad a non-multiple-of-16
  size for you here.
- **`-g 25`** - GOP length / keyframe interval (one IDR frame every 25
  pictures here). Any value works; a shorter GOP means more I-frames
  (larger file, cheaper to seek/recover from a dropped frame), a longer
  one means more P-frames (smaller file, but every picture after the IDR
  depends on unbroken decode of everything before it, and errors in one
  picture propagate into every picture referencing it afterward).

To also disable the in-loop deblocking filter (useful when isolating
whether a decode mismatch is in prediction/motion/CAVLC vs. specifically
the deblocking filter - the same reason this project's own test suite has
`test_decode_multiframe_nodbf.cpp` alongside the deblocking-enabled
version), add `-x264-params deblock=0`:

```sh
-x264-params deblock=0
```

The decoder handles both cases automatically (`disable_deblocking_filter_
idc` is read per-slice from the bitstream, see `h264_deblock.h`) - no
code change needed to feed it a deblock-disabled stream.

To generate a synthetic test clip instead of transcoding a real file
(handy for a quick smoke test with no source video on hand):

```sh
ffmpeg -f lavfi -i testsrc=size=176x144:rate=25:duration=2 \
  -pix_fmt yuv420p -c:v libx264 -profile:v baseline -level 3.0 \
  -g 25 -f h264 test.264
```


## Testing

Correctness is validated by decoding real streams generated with
`ffmpeg`/libx264 and diffing every pixel against `ffmpeg`'s own decode -
not by conformance-suite guesswork.

Build and run the whole suite (25 tests) with CMake + CTest from the repo
root:

```sh
cmake -B build && cmake --build build -j && 
```

This also defines a `TinyH264::TinyH264` CMake target (header-only,
C++17 propagated to consumers) that another CMake project can pull in
directly via `add_subdirectory()`:

```cmake
add_subdirectory(path/to/TinyH264)
target_link_libraries(your_target PRIVATE TinyH264::TinyH264)
```

`add_subdirectory()`-ing it this way does *not* also build TinyH264's own
test suite - `test/native/` only builds when this repo is the top-level
CMake project (`TINYH264_BUILD_TESTS`, on by default in that case, off
otherwise - override either way with `-DTINYH264_BUILD_TESTS=ON/OFF`).

The same `CMakeLists.txt` also doubles as an **ESP-IDF component**
manifest - `if(ESP_PLATFORM)` (the variable ESP-IDF's own build system
sets) routes straight to `idf_component_register()` instead, before any
`project()` call, so no separate copy needs to be kept in sync. To use it
from an ESP-IDF project, either drop (or symlink) this repo into that
project's `components/` directory, or point `EXTRA_COMPONENT_DIRS` at it
in the project's top-level `CMakeLists.txt`:

```cmake
set(EXTRA_COMPONENT_DIRS "path/to/TinyH264")
```

then `REQUIRES TinyH264` (or `PRIV_REQUIRES`) in whichever component's
`CMakeLists.txt` needs the headers, and `#include <TinyH264Decoder.h>` as
usual. Verified with a real `idf.py build` (ESP-IDF v6.2, `esp32` target)
against a minimal component consuming `TinyH264Decoder<>` - not just
read from the ESP-IDF docs. `idf_component.yml` at the repo root carries
the component's registry metadata (version, license, description) for
projects that pull it in via the IDF Component Manager instead.

For quicker one-off iteration on a single test without reconfiguring,
the equivalent direct `g++` invocation still works (from `test/native/`):

```sh
g++ -std=c++17 -O2 -I../../src test_decode_multiframe.cpp -o /tmp/t && /tmp/t
```

Other `test_*.cpp` files cover individual layers (bitstream reader, NAL
parsing, SPS/PPS, slice headers, CAVLC table validity). Test assets
(`assets/*.264`, `assets/*.yuv`) are pre-generated with `ffmpeg`/libx264
using the same parameters documented in
[Preparing input with ffmpeg](#preparing-input-with-ffmpeg) above (the
`_nodbf` variants add `deblock=0`; `multiref.264` uses `-x264-params
ref=3:me=umh:subme=8` and complex synthetic motion content specifically to
force genuine use of reference indices other than 0 - confirmed via a
debug build tallying actual ref_idx usage across the decode, not just
that the stream declares multiple references - see `test_decode_
multiref.cpp`, the oracle for the multi-reference-frame feature); the
`.yuv` reference files are `ffmpeg`'s own raw decode of the same `.264`
stream, used as the pixel-exact oracle each test diffs against.

