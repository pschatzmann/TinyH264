# Optimizations

Performance work on this project's encoder/decoder: what's been applied
(with real measurements, not just reasoning), and what's been
investigated and set aside (with why) - so neither gets re-derived from
scratch in a future session.

## Encoding

### Motion search performance

`motionSearch16x16()` (`src/encoder/h264_macroblock_encode_inter.h`)
defaults to an exhaustive full search (289 candidates/macroblock at the
default `range=8`) - a "correctness first" starting point, not a fast
one (see that file's header comment).

**Branch-elimination in `sad16x16At()`** (applied) - bounds-checks a
candidate once instead of once per pixel, branch-free inner loop when
the 16x16 block is fully in-picture. Bit-exact (full test suite), pure
win: ~4.4x on x86, ~2.5x on real ESP32/RP2040 (QCIF P-frame ~1.82s ->
~0.73s - embedded sees less benefit since weaker branch prediction
matters less there to begin with).

**`setMotionSearchRange(range)`** (applied) - runtime +/-pixel search
window, default 8 (unchanged behavior unless a caller opts in).
Correctness never depends on `range` - a narrower window just costs a
bigger residual instead of a matching MV
(`test/native/test_motion_search_range.cpp`). See [Encoding](encoding.md).

**`setMotionSearchAlgorithm(MotionSearchAlgorithm)`** (applied) - opt-in
Diamond Search (`Fast`; default stays `Exhaustive`). LDSP radius-2 probe
around the current center, repeating while it improves; one SDSP
radius-1 refinement once it converges; capped at `2*range+1` iterations
so pathological content can't loop unboundedly. A *local* search - can
settle for a locally- rather than globally-best match on some content
(a real data-dependent tradeoff, not just slower). Demonstrated even on
a zero-true-motion test case
(`test/native/test_motion_search_fast.cpp`): P-frame search matches the
raw source against the *lossy reconstructed* reference, not the raw
previous frame, so the SAD surface isn't flat even when the true motion
is exactly zero - `Exhaustive` itself picked a non-zero MV there. Both
algorithms always produce a valid, self-decodable bitstream regardless.

**Measured, ESP32-S3** (`examples/EncodeSyntheticFrame`, QCIF):

| range | algorithm | avg | min (I-frame) | max (P-frame) |
|---|---|---|---|---|
| 8 | Exhaustive (baseline) | 486647 us (2.1 fps) | 205299 us (4.9 fps) | 584823 us (1.7 fps) |
| 4 | Exhaustive | 241217 us (4.1 fps) | 200952 us (5.0 fps) | 256509 us (3.9 fps) |
| 8 | Fast | 177094 us (5.6 fps) | 151597 us (6.6 fps) | 206981 us (4.8 fps) |
| 4 | Fast | 172003 us (5.8 fps) | 136400 us (7.3 fps) | 206974 us (4.8 fps) |

- `range` matters a lot for `Exhaustive` (`O(range^2)`, 2.02-2.28x from
  8->4) but barely at all for `Fast` (177094 -> 172003, ~3%): `range`
  only bounds Diamond Search's *worst case* - real content converges in
  a handful of iterations well under that cap regardless. `range` is the
  lever to pull under `Exhaustive`; under `Fast` it mainly guards against
  pathological content.
- `Fast` alone is 2.75-2.83x faster than the `Exhaustive`/range=8
  baseline - bigger than the ~2.07x an x86 microbenchmark predicted,
  plausibly because cutting SAD candidates (memory traffic) matters more
  on a core with a weaker memory subsystem than cutting branches does
  (the opposite pattern from the branch-elimination fix above; not
  root-caused with on-device profiling).
- `min` (I-frame) shifting under `Fast` isn't I-frames getting faster -
  they never call either search function. It's the fastest *P*-frame in
  the run overtaking the I-frame as the sequence-wide minimum once
  P-frames get fast enough.

**Eliminated a redundant motion-compensation + transform pass** (applied)
- found via a more realistic benchmark (static checkerboard background +
a genuinely moving 48x48 block; the panning-only benchmark above has
almost no static content, which hid this). Whenever a macroblock's best
MV equaled the predicted skip MV (`skipMv()` - measured at 63-94% of all
macroblocks, not rare), the encoder ran a full motion-compensation +
16-block luma transform/quantize + chroma transform/quantize pass
*twice*: once as a throwaway skip-check trial
(`wouldHaveZeroResidual()`), and again for real
(`encodeMacroblockInter16x16()`) whenever that trial found nonzero
residual. `computeInterResidual()`/`finishInterMacroblock()`
(`h264_macroblock_encode_inter.h`) split the old function so the trial's
result is kept and reused instead of recomputed. Also removed a smaller
duplicate: `encodeMacroblockPSkip()` used to redo `skipMv()` +
`motionCompensate16x16()` even though the trial had just computed the
identical result.

One wrinkle worth recording: the Intra-mode trial
(`chooseIntra16x16Mode()`) overwrites `ctx.frame`'s luma plane between
the skip check and the final decision - reusing the residual trial's
coefficients without restoring luma first would have silently corrupted
this encoder's own reconstruction (not the encoded bitstream, just its
reference-frame state) - caught before it reached the test suite, fixed
with a luma-only `motionCompLuma()` restore (chroma is never touched by
the Intra trial, so only luma needs it).

Bit-exact (full native test suite, including the ffmpeg comparison in
`test_encode_pframe.cpp`). Wall-clock timing on the build machine was too
noisy to trust (system load >3 on 4 cores, 50%+ run-to-run variance on
an *unchanged* binary); verified instead via `motionCompensate16x16()`
call counts on the mixed-content benchmark (1485 macroblocks total):

| | before | after |
|---|---|---|
| Exhaustive | 2422 calls (1.63/MB) | 1484 calls (~1/MB, **38.7% fewer**) |
| Fast | 2871 calls (1.93/MB) | 1481 calls (~1/MB, **48.4% fewer**) |

The saved-call counts (938, 1390) match this benchmark's `mvEqSkip`
counts exactly - the fix removes precisely the identified redundancy and
nothing else.

**Confirmed on real STM32H750VBT6** (`-O3`,
`setAllOptimizationsActive(true)` = `Fast` + this fix together, vs. the
original `Exhaustive`/no-fix baseline - can't cleanly separate which
optimization contributes how much, both are real and both are included):

| | avg | min (I-frame) | max (P-frame) |
|---|---|---|---|
| before | 88616 us (11.3 fps) | 15744 us (63.5 fps) | 114245 us (8.8 fps) |
| after | 21461 us (46.6 fps) | 15528 us (64.4 fps) | 24322 us (41.1 fps) |
| change | **4.13x faster** | ~1% (noise) | **4.70x faster** |

### Already considered

Approaches investigated and set aside, with why - so they aren't
re-investigated from scratch later without new information.

#### Hardware SIMD acceleration (ESP-DSP / CMSIS-DSP)

Not a good fit for this project's actual targets, verified against each
library's real source:

- Neither library ships a fused SAD/abs-diff-accumulate primitive -
  either way this means hand-writing a custom kernel, not calling a
  ready function.
- **ESP-DSP**: real SIMD for 8-bit pixel data (`dsps_sub_s8`) is gated
  to `_aes3` (ESP32-S3's 128-bit vector instructions) only - plain
  ESP32 (LX6) has a separate `_ae32` tier, but only for 16-bit/float
  data, not the 8-bit pixels SAD needs. Would only help ESP32-S3.
- **CMSIS-DSP**: RP2040 is genuinely ARM, but its Cortex-M0+ cores are
  ARMv6-M, lacking the ARMv7-M DSP extension entirely. `arm_sub_q7.c`'s
  SIMD path is gated on `ARM_MATH_DSP` (`__QSUB8`, M4/M7/M33 only);
  without it, falls back to the same plain loop `-O2` already generates.
  Zero benefit on RP2040.
- **Net**: no DSP library meaningfully helps this project's primary
  targets (plain ESP32, RP2040) without hand-written per-platform SIMD
  intrinsics. Worth revisiting only if ESP32-S3 became a primary target.

#### ESP32-P4 hardware (H.264 encoder, PPA, ISP)

The only ESP32-family chip with real video hardware (not "ESP32-H3" -
that chip doesn't exist; the H-series is low-power Thread/Zigbee/Matter
silicon). Worth a closer look than the other entries here, because it
solves a different kind of problem: even after the optimizations above,
encoding tops out at single-digit fps at QCIF on every current target -
no further *software* work on Xtensa/ARM closes that gap, but P4's
hardware (1080p@30fps claimed) is a difference in kind, not degree.

**H.264 encoder** (*ESP32-P4 Series Datasheet* v0.7, §4.2.1.5): Baseline
profile, I/P-frame only (matches this project's own scope), up to
1080p@30fps YUV420 input (also accepts RGB888/RGB565/YUV444/YUV422/GRAY),
GOP mode, dual-stream mode. More capable than this project's software
encoder in ways that map directly onto the feature gaps below: inter
sub-partitioning down to 4x4 (this project is P_16x16-only) and 1/2- and
1/4-pel motion estimation (this project is integer-pel only), plus an
asymmetric, wider search window (H [-29.75,+16.75]px, V
[-13.75,+13.75]px) and 8-region ROI QP control this project has no
equivalent of at all. One place the two designs already agree: the
datasheet lists "P slice supporting I macroblock" as a hardware feature
- exactly this project's own P-slice Intra-fallback
(`shouldUseIntraInPSlice()`), arrived at independently. No hardware
decode on any ESP32 chip, including P4 - Espressif's own SDK decodes in
software there too.

**Is the hardware driver actually usable, or closed-source?** Split.
`espressif/esp-h264-component`'s `hw/`, `interface/`, and `port/`
directories (HAL/DMA/ISR, API dispatch, alloc/cache/mutex glue) are
real, substantive, Apache-2.0 C source (`esp_h264_enc_single_hw.c` alone
is ~18KB of real driver logic) - GPLv3-compatible. Only
`sw/libs/*/{libopenh264,libtinyh264}.a` (the *software* encode/decode
fallback used on chips without hardware, e.g. ESP32-S3) are closed,
headers-only binary blobs - a separate part of the same component, not
what P4's hardware path needs. (Aside: that software decoder's
"tinyH264" credit is
[github.com/udevbe/tinyh264](https://github.com/udevbe/tinyh264), an
unrelated project - not this one.)

The `esp_h264` component itself is distributed via the ESP-IDF Component
Registry, not as an Arduino library - using it as-is would mean the
"Arduino as an ESP-IDF component" build pattern, a different
build/validate story than every other example in this repo (plain
`.ino` + `arduino-cli compile`). Since the driver is open, that's likely
avoidable by vendoring the `hw`/`interface`/`port` files directly instead
of depending on the component - real adaptation work remains
(P4-specific DMA/cache alignment, wiring FreeRTOS/interrupt calls to
whatever Arduino-ESP32 3.x, itself ESP-IDF-based, actually exposes), but
it isn't blocked by unavailable source. Arduino-ESP32 3.3.10 already has
real P4 board definitions and compiles a plain sketch today (confirmed
via `arduino-cli`), so the toolchain isn't the blocker either.

**PPA** (Pixel-Processing Accelerator): a generic 2D accelerator (scale/
rotate/mirror/blend/fill) operating on arbitrary memory buffers, not
just camera/display DMA - closer to something this project could
actually use than the H.264 block, since it would accelerate *existing*
glue code rather than requiring a parallel codec. Its formats
(RGB888/RGB565/ARGB8888/YUV420/YUV422/YUV444/GRAY8) line up with
`encodeFrameRgb888()`/`encodeFrameRgb565()`/`encodeFrameYuv422()` and
`toRGB565()`/`toRGB888()` (RGB666 is the one gap - not a PPA format at
all), and its scaling op is a direct match for `setScaleFactor()`. Real
caveats: DMA-driven async API (blocking mode is a reasonable fit for
this project's synchronous style), and performance is bandwidth-bound
when buffers live in PSRAM, not free.

**ISP**: irrelevant - a raw-Bayer-camera-sensor pipeline (demosaic,
black-level correction, etc.), strictly upstream of anything this
project ever sees (`encodeFrame()`'s input is always already-demosaiced
YUV420/RGB/YUV422). Only relevant to application code feeding a raw
sensor in, never to code inside this library.

**Net**: parked, not ruled out. No P4 hardware available to validate
against yet. If picked up: scope as a separate, opt-in P4-only encoder
path (new class/backend, built from the vendored `hw` driver) rather
than touching the existing portable `SoftwareEncoder` - PPA would be
the more natural first piece to adopt, since it accelerates existing
code rather than replacing it.

### Other known gaps

Feature-completeness gaps (not performance) - see [Scope](scope.md) for
the full, authoritative list: P_16x8/P_8x16/P_8x8 sub-partitions,
multi-reference P-frames, and sub-pel motion search are the main ones
not yet implemented for the encoder.

## Decoding

Decode performance hasn't needed the same attention as encoding - real
hardware already measures well above real-time (~49 fps at QCIF on
plain ESP32, see the Performance table in the [README](../README.md)) -
so nothing below has been implemented or even prioritized; this is a
lower-confidence list than the encoding chapter above.

### Performance

No hardware decode path exists on any ESP32 chip, including ESP32-P4
(the only one with a hardware H.264 *encoder*) - see "ESP32-P4 hardware"
above for the full investigation. Nothing to accelerate this way on any
target, current or hypothetical.

Assessed which decode hot paths would actually benefit from DSP/SIMD
acceleration (lower confidence than the encoder-side ESP-DSP/CMSIS-DSP
check above - worth re-verifying the same way if this is picked up):
- **Good fit**: motion compensation's 6-tap FIR luma interpolation
  (`h264_motion.h`'s `tap6()`/`rawHalfH()`/`rawHalfV()`) - a textbook
  fixed-coefficient FIR filter, currently scalar per-pixel.
- **Poor fit**: CAVLC entropy decoding (`h264_cavlc.h`'s bit-serial
  `decodeVlc()`) - inherently sequential, no DSP/SIMD library can help.
  A LUT-based fast VLC matcher (instead of the current bit-by-bit
  generic prefix matcher) would be the real fix there.
- **Marginal**: IDCT/Hadamard and the deblocking filter's pixel math -
  vectorizable arithmetic, but wrapped in per-pixel branching that would
  need restructuring first.

No per-stage profiling has confirmed which of these is actually the
bottleneck in real measured decode time - worth doing before committing
effort to any of them.

### Feature gaps

Not supported by design, tracked authoritatively in [Scope](scope.md):
CABAC, B-slices, weighted prediction, explicit reference list
reordering, adaptive (MMCO) reference marking, FMO/ASO slice groups,
interlaced/MBAFF content, and High-profile-and-above features (8x8
transform, scaling lists). Deliberate scope choices for this decoder's
target (small, single-camera-style Baseline streams on constrained
MCUs), not gaps anyone has evaluated implementing - listed only for
completeness, not as a prioritized roadmap.
