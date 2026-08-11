# Future potential improvements

Ideas investigated but deliberately not implemented (yet) - kept here
with what was actually found, so a future session doesn't re-derive the
same investigation from scratch.

## Encoding

### Motion search performance

`motionSearch16x16()` (`src/encoder/h264_macroblock_encode_inter.h`) does
an exhaustive integer-pel full search over a +/-8 pixel window (289
candidates/macroblock), each scored by a 256-pixel SAD - a deliberate
"correctness and simplicity first" choice over a fast search algorithm
(see that file's own header comment).

**Applied**: `sad16x16At()` used to run all 256 pixels of every
candidate through `clampedSample()` (4 bounds-check branches/pixel),
even though most macroblocks/candidates never actually go outside the
picture. Now checks bounds once per candidate and uses a branch-free
inner loop when the whole 16x16 block is safely in-picture, falling back
to the clamped path only near edges - bit-exact identical output
(confirmed via the full native test suite), pure performance win.
Measured: ~4.4x on native x86, **~2.5x on real ESP32/RP2040 hardware**
(QCIF P-frame: ~1.82s -> ~0.73s). Real embedded hardware sees a smaller
win than x86 because eliminating branches matters less on cores with
weaker branch prediction to begin with.

Even with that fix, P-frame encode is still ~730ms (~1.4 fps) at QCIF on
real hardware - the remaining cost is the search *algorithm* itself
(289 SAD evaluations by design), not implementation slack. Two further
options are still open (not applied yet); a third was investigated and
ruled out (see "Already considered" below).

1. **Shrink the search range** (e.g. +/-8 -> +/-4): ~3.6x fewer
   candidates (81 vs 289), simple and low-risk, but caps how much
   frame-to-frame motion the encoder can track - worse compression
   (bigger residuals) on fast-moving content. Would need re-verification
   against the existing PSNR/bit-exactness tests, but is a small,
   mechanical change.
2. **Fast search algorithm** (diamond/hex-style, checking ~15-25
   candidates instead of 289): the biggest potential win, but real
   implementation complexity, and can miss the true best match on some
   content (data-dependent behavior change, not just a speed/quality
   dial). **User's stated preference for how to eventually add this: as
   an opt-in choice via the public API** (e.g. a
   `setMotionSearch(Exhaustive | Fast)`-style setter), not a silent
   default-behavior change - so the existing exhaustive search stays the
   default and existing bit-exactness guarantees for anyone not opting
   in are undisturbed.

### Already considered

Approaches investigated and set aside, with why - so they aren't
re-investigated from scratch later without new information.

#### Hardware SIMD acceleration (ESP-DSP / CMSIS-DSP)

**Not a good fit for this project's actual targets**, verified against
each library's real source (not assumed from memory):

- Neither library ships a fused SAD/abs-diff-accumulate primitive - both
  only have separate subtract and separate abs functions, so either way
  this means hand-writing a custom kernel, not calling a ready function.
- **ESP-DSP**: real SIMD acceleration for 8-bit pixel data
  (`dsps_sub_s8`) is gated to `_aes3` (ESP32-S3's 128-bit vector
  instructions) only - plain ESP32 (LX6) has a separate `_ae32`
  acceleration tier (Xtensa Loop+MAC16 extensions), but only for
  16-bit/float data, not the 8-bit pixel workload SAD actually needs. So
  this would only help ESP32-S3, one of this project's several targets.
- **CMSIS-DSP**: relevant in principle since RP2040 is genuinely ARM
  (unlike ESP32), but RP2040's Cortex-M0+ cores implement ARMv6-M, which
  lacks the ARMv7-M DSP extension entirely. Checked `arm_sub_q7.c`
  directly: its SIMD path is gated on `ARM_MATH_DSP` (uses `__QSUB8`, a
  4-lane packed-byte subtract instruction available on M4/M7/M33 - not
  M0+); without that, it falls back to the same plain per-byte loop the
  compiler already generates at `-O2`. Zero real benefit on RP2040.
- **Net**: no DSP library meaningfully helps this project's primary
  targets (plain ESP32, RP2040) for this workload without hand-written
  per-platform SIMD intrinsics. Would only be worth revisiting if
  ESP32-S3 became a primary (not secondary) target.

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

An earlier session assessed which decode hot paths would actually
benefit from DSP/SIMD acceleration (ESP-DSP specifically, before this
session's more rigorous ESP-DSP/CMSIS-DSP source-level check on the
encoder side - worth re-verifying the same way if this is picked up):
- **Good fit**: motion compensation's 6-tap FIR luma interpolation
  (`h264_motion.h`'s `tap6()`/`rawHalfH()`/`rawHalfV()`) - a textbook
  fixed-coefficient FIR filter, currently scalar per-pixel.
- **Poor fit**: CAVLC entropy decoding (`h264_cavlc.h`'s bit-serial
  `decodeVlc()`) - inherently sequential (each bit's meaning depends on
  the previous ones), no DSP/SIMD library can help. The real fix there
  would be a LUT-based fast VLC matcher instead of the current
  bit-by-bit generic prefix matcher.
- **Marginal**: IDCT/Hadamard and the deblocking filter's pixel math -
  vectorizable arithmetic, but wrapped in per-pixel branching that would
  need restructuring first.

No per-stage profiling has been done to confirm which of these is
actually the bottleneck in real measured decode time - worth doing
before committing effort to any of them.

### Feature gaps

Not supported by design, tracked authoritatively in
[Scope](scope.md): CABAC, B-slices, weighted prediction, explicit
reference list reordering, adaptive (MMCO) reference marking, FMO/ASO
slice groups, interlaced/MBAFF content, and High-profile-and-above
features (8x8 transform, scaling lists). These are deliberate scope
choices for this decoder's target (small, single-camera-style Baseline
streams on constrained MCUs), not gaps anyone has evaluated implementing
- listed here only for completeness, not as a prioritized roadmap.
