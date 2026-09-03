# Encoding

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
    // buffer too small, width/height not a multiple of 16, or a picture
    // buffer allocation failed (out of memory - see "Allocation failure
    // handling" below; not a crash)
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

**Motion search range**: `setMotionSearchRange(range)` overrides the
+/-`range`-pixel window each P-macroblock's motion search checks
(default 8). Search cost is roughly `O(range^2)`, so this is a real
speed/compression tradeoff, not a quality-only setting - a smaller range
encodes faster but can't represent motion larger than `range`
pixels/frame (the encoder still produces a correct bitstream regardless,
just with a bigger residual instead of a matching motion vector for
motion beyond the window):

```cpp
encoder.setMotionSearchRange(4);  // faster, worse compression on fast motion
```

See [Optimizations](optimizations.md#encoding) for
the measured numbers this default (8) costs on real hardware, and why a
smaller range is the safest lever to pull first if you need more
headroom than the applied SAD fast-path already gives you.

**Motion search algorithm**: `setMotionSearchAlgorithm(MotionSearchAlgorithm::Fast)`
switches from the default exhaustive full search (`Exhaustive`, guaranteed
to find the true best-SAD match within `range`) to a Diamond Search
(`Fast`) that checks far fewer candidates - roughly 15-30 on typical
content instead of `(2*range+1)^2` (289 at the default range=8):

```cpp
encoder.setMotionSearchAlgorithm(MotionSearchAlgorithm::Fast);  // opt-in
```

Unlike `setMotionSearchRange()`, this is not a pure speed/compression
dial: Diamond Search is a *local* search (it follows whichever neighbor
looks best from wherever it currently is), so on content whose SAD
surface has more than one local minimum it can converge on a match that
isn't the true global best - a real, data-dependent quality/compression
tradeoff, not just slower-but-equivalent. The bitstream is still always
valid and self-decodable regardless (a suboptimal MV just costs a bigger
residual, never correctness - `test/native/test_motion_search_fast.cpp`).
Defaults to `Exhaustive` so existing behavior/bit-exactness is unchanged
for anyone not opting in. See
[Optimizations](optimizations.md#encoding) for the
measured candidate-count/speed tradeoff.

`setAllOptimizationsActive(true)` is a shorthand for "turn on every
optional, opt-in performance optimization this encoder has" - currently
just `setMotionSearchAlgorithm(MotionSearchAlgorithm::Fast)`, since that's
the only one with a real behavior tradeoff to opt into (permanently-applied
fixes with no tradeoff, like the SAD branch-elimination and duplicate
motion-compensation/transform eliminations documented in
[Optimizations](optimizations.md#encoding), are always on - nothing to
switch). Doesn't touch `setMotionSearchRange()`, a continuous dial rather
than an on/off optimization. `setAllOptimizationsActive(false)` reverts to
`Exhaustive`. A convenience for not having to track each optional
optimization's own setter individually as more get added over time:

```cpp
encoder.setAllOptimizationsActive(true);  // == setMotionSearchAlgorithm(Fast) today
```

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
convention, or eagerly via `begin(true)` - see
[Memory budget](memory-budget.md)) - calling only the plain
`encodeFrame()` never pays for them.

### Hardware encoder (ESP32-P4)

On ESP32-P4 builds (chip revision < 3.0), `TinyH264Encoder` also has a
dedicated hardware H.264 encoder path - `HwEncoderP4`
(`src/encoder/h264_hw_encoder_p4.h`), a genuinely different
implementation with a narrower feature set, ~140x faster than software
at QCIF (see the [README](../README.md#performance) for measured
numbers and the [investigation writeup](esp32-p4-hardware-encoder-investigation.md)
for how it was root-caused). It's on by default wherever it's available
- `hardwareAvailable()` reports whether this build has it at all (a
compile-time answer, safe to call unconditionally); `setUseHardware(bool)`
toggles it and `useHardware()` reads the current state:

```cpp
if (TinyH264Encoder<>::hardwareAvailable()) {
  encoder.setUseHardware(true);  // the default already, shown explicitly here
}
```

While hardware mode is active:
- `setQp()` sets the one fixed QP used for the whole stream - the
  hardware driver has no rate-control mode, so **`setTargetBitrate()`
  only ever affects the software fallback**, not hardware itself
  (`qp = -1`, `setTargetBitrate()`'s sentinel, makes `encodeFrame()`
  skip the hardware attempt entirely and go straight to software - see
  the README's own "Known limitation" note for why the two can't be
  synced).
- `setKeyframeInterval()` becomes the hardware's own GOP size.
- `setMotionSearchRange()`/`setMotionSearchAlgorithm()`/
  `setAllOptimizationsActive()`/`setTargetBitrate()` are software-only -
  each now **returns `false`** while hardware mode is active (the call
  still takes effect for the inherited software fallback, just not for
  hardware itself), instead of the `void` these all used to be:
  ```cpp
  if (!encoder.setTargetBitrate(300000, 25.0)) {
    // ignored - hardware mode is active; setQp() controls quality instead
  }
  ```
- A real hardware encode failure falls back to the software encoder
  automatically from then on, rather than returning `0` to the caller -
  see [Logging](logging.md) for how that (and every other failure path
  in this library) reports through `H264LOG`.

`examples/EncodeSyntheticFrame` runs both the hardware and software
paths back to back against the same content, labeled, in one sketch.

### Allocation failure handling

`TinyH264Encoder` is templated on an allocator (`StdAllocator<uint8_t>`
by default, see `src/StdAllocator.h`) the same way `TinyH264Decoder` is -
see [Decoding](decoding.md#allocation-failure-handling) for the full
rationale. Unlike `std::allocator`, it never throws `std::bad_alloc` on
an out-of-memory condition; instead, every `encodeFrame()`-family call
(and its color-format overloads) reports it the same way a too-small
`dst` buffer or invalid width/height/qp already were - a `0` return, with
nothing usable written to `dst` - and `begin()` returns `false`. No new
failure mode for callers to handle differently; an existing "did this
call actually produce output?" check already covers it. Every one of
these failures also logs via `H264LOG` - see [Logging](logging.md).

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
