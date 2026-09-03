# ESP32-P4 Hardware Encoder: Investigation History

See the main [README](../README.md#performance)'s "ESP32-P4 hardware
encoder" section for current status and measured performance - this
document is the detailed root-cause writeup and investigation history
behind it, kept separately since the README itself was getting long.

## Root cause

The root cause, once found, was a single, precise bug: the hardware's
"8-byte-alignment slice-header split" mechanism (which holds back up to
7 bytes of the slice header into three hardware registers, ported
verbatim from Espressif's own `esp_h264_enc_hw_slice_header_align8()`)
computes its split point relative to the *absolute* start of the output
buffer, not relative to the slice header alone. For I-frames, the
preceding SPS/PPS always keeps that computed split point safely past the
slice NAL's own 4-byte Annex-B start code. P-frames have no such prefix
- and once the held-back region reaches its documented 7-byte maximum,
it reaches backward into the start code itself. The hardware's own CAVLC
emulation-prevention logic then treats those bytes as ordinary RBSP
content rather than a start code, and inserts a spurious `0x03`
byte right after the first two `0x00` bytes - confirmed directly by
dumping the DMA-written bytes against what was written pre-encode, byte
for byte. Fixed by prepending a small (9-byte) AUD NAL - Espressif's own
driver uses the identical technique for a related alignment case - before
every P-frame slice, sized (re-derived from the split formula, not
guessed) so the split point can never land inside the real start code
again; a runtime check right after the split computation catches it
explicitly if that ever stops holding, rather than silently corrupting
output. See `HwEncoderP4::prepareAndStartFrame()`'s own comment for the
full derivation.

## How it was found

Finding it took a long path, summarized here for anyone hitting a
similar class of bug on this hardware. Cross-checked against Espressif's
official, chip-revision-matched Technical Reference Manual, the
`esp_h264` component's published docs, and - most valuably -
`codec-h264-ESP32P4`'s vendored copy of Espressif's actual driver
*source*, used as a live, on-the-same-board comparison target via a
series of hybrid bisection tests. Along the way, five real,
independently-confirmed bugs were found and fixed before the actual root
cause: TX channel 2 was gated on an interrupt the TRM documents but the
real driver doesn't use; a missing `ENC_CORE` reset pulse the TRM
doesn't document but the real driver issues anyway (real source beat
documentation prose both times); every `H264DmaDesc` allocated without
its required 8-byte alignment; `dmaClearAllInterrupts()` missing
entirely (a whole class of DMA channel interrupt latches never cleared);
ROI never force-enabled on the unused second video-stream channel
(Espressif's own driver does, unconditionally); and an unrelated,
already-latent `esp_cache_msync()` alignment bug in the post-encode
cache invalidate, found and fixed in the same investigation that
uncovered the real cause. A stray `#pragma GCC optimize("O3")` leak
(missing `push_options`/`pop_options`, silently forcing this whole
driver to a different optimization level than the rest of the project)
was also found and fixed - genuinely worth keeping, but not the cause
either, confirmed via disassembly.

What actually broke the case open, after code-level parity with the real
driver was established and ruled out as the difference: reaching the
hardware's own *success* path at all. Every earlier test exercised
`encode()`/`encodeDiagnostic()` directly against `HwEncoderP4`, always at
a fixed fake QP with no rate-control interaction - and, unrelated to the
real cause, `TinyH264Encoder::useHardware_` had at one point regressed
to defaulting `false` (a real, independent bug in itself, since the
intent has always been "on automatically when hardware exists" - fixed
alongside the fail-open fallback described in the README). Once
genuinely fixed QP was driven through the full `TinyH264Encoder` ->
`HwEncoderP4` pipeline (`examples/EncodeDecodeRoundTrip`, `setQp()`, no
rate control), I-frames decoded correctly on the very first real run -
proving the hardware, DMA, and cache-coherency work were all already
correct - and P-frames, still failing but now with `finishFrame()`'s own
post-encode byte dump to compare directly against what was written
pre-encode, pinpointed the exact corrupted byte within a few iterations.

## Why it took so long

1. **The bug only manifests on the success path.** It lives in
   `finishFrame()`, reached only after a real `FRAME_DONE`. Every
   earlier test failed *before* getting there, for separate, real bugs
   (see above) - none of which could reveal a bug in code that never
   ran.
2. **A second, unrelated regression stacked on top of the first.**
   `TinyH264Encoder::useHardware_` had regressed to defaulting `false`,
   so even testing through the normal API silently skipped hardware
   entirely. Fixing one bug didn't show progress while the other still
   blocked things.
3. **The failure signal was maximally uninformative.** `encode()`
   returning 0 with no crash and no partial output gives almost no clue
   what's wrong - ruling things out meant testing entire categories
   (register content, DMA sequencing order, compiler optimization,
   calling context, timing, cache coherency) one at a time rather than
   following a specific lead.
4. **No public documentation for this exact register-level sequence** -
   Espressif's own SDK abstracts it away, so every hypothesis had to be
   verified empirically on real hardware, and each round trip (edit,
   compile, flash, physically reset, capture) takes real time.
5. **The final bug is invisible unless you're looking at raw bytes.**
   From the decoder's side it just looks like generic corruption
   (`slice_type=4` instead of `0`) - nothing hints that the mechanism is
   "a start code got eaten by an emulation-prevention pass that doesn't
   know it's a start code." Finding it required first reaching a state
   where *anything* beyond total silence was observable, then doing
   byte-for-byte, bit-level forensics comparing pre-encode and
   post-encode buffer contents.
