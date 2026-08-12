# Memory budget

Both `TinyH264Decoder` and `TinyH264Encoder` are sized by default for
QVGA (320x240) - the budget below applies to *both*, not just the
decoder, since `TinyH264Encoder` holds its own closed-loop reconstruction
picture buffer internally (see [Encoding](encoding.md)) with the same
per-frame cost as the decoder's. Figures below are real `arduino-cli`
compiles (`esp32:esp32:esp32`), not estimates - an empty sketch alone
already uses ~22KB of static RAM (Arduino/ESP-IDF framework baseline), so
that's the floor everything below is measured against:

| Object (static RAM, before any picture is actually decoded/encoded) | Cost |
|---|---|
| `TinyH264Decoder<>` | ~34KB |
| `TinyH264Encoder<>` | ~33KB |
| Both together (e.g. a decode-then-re-encode relay) | ~67KB (purely additive) |

Picture buffers (the actual per-frame cost, see below) are *not* part of
these static figures - they're heap-allocated, and each picture's Y/U/V
planes are 3 separate allocations rather than one merged buffer, so that
each individual allocation (largest: the luma plane) stays small enough
to still find a home even on a heap fragmented by WiFi/BT - see
`src/h264_config.h` and `src/common/h264_frame.h` for why this matters
in practice, not just in theory.

**Decoder**: one current + up to `H264_MAX_REF_FRAMES` (default 3)
reference YUV 4:2:0 frame buffers, each ~115KB at QVGA (76800 Y + 19200 U
+ 19200 V, 3 separate allocations) - ~460KB total for all 4 by default -
heap-allocated on first use, on top of the ~34KB static cost (NAL scratch
buffer, per-macroblock metadata table - the metadata table is also
heap-allocated, ~40KB at QVGA, included in this ~34KB *static* figure
only in the sense that it's allocated once and kept resident, same
allocate-once-not-per-frame idiom as the picture buffers). ~460KB rarely
fits a plain ESP32's ~320KB total heap - call
`src/decoder/h264_decoder.h`'s `setMaxRefFrames()` (or the
`TinyH264Decoder` wrapper of the same name) to lower the *runtime-active*
reference count below the compile-time `H264_MAX_REF_FRAMES` without
rebuilding; `setMaxRefFrames(1)` caps it to 2 resident frames (~230KB),
comfortably fitting a plain ESP32 alongside everything else - see
`examples/DecodeFromProgmem` for a worked example (each reference picture
not needed saves ~115KB at QVGA, ~38KB at QCIF).

**Encoder**: two picture buffers - the current reconstruction and the
single reference frame P-frames motion-compensate against (see
[Encoding](encoding.md)) - each ~115KB at QVGA, ~230KB total, both
heap-allocated the first time `encodeFrame()` is called. Both are
allocated unconditionally on that first call, even for a caller whose
stream never actually produces a P-frame - a real, if modest,
inefficiency in exchange for a simple "copy the just-finished picture
into the reference slot" design rather than a more complex
buffer-swapping scheme. On top of the heap cost, the ~33KB static cost
shown above (a slice scratch buffer, sized the same as the decoder's NAL
scratch buffer, plus the same heap-allocated per-macroblock metadata
table). The `encodeFrameRgb888()`/`encodeFrameRgb666()`/
`encodeFrameRgb565()`/`encodeFrameYuv422()` convenience overloads add
~115KB more at QVGA (~38KB at QCIF), but only if actually called - their
conversion scratch buffers are heap-allocated lazily on first use (or
eagerly via `begin(true)` - see below), so a sketch that only calls the
plain `encodeFrame()` never pays for them.

**Plain-ESP32 QVGA total heap budget, worked example**: decoder with
`setMaxRefFrames(1)` (~230KB) leaves comfortable headroom against a
typical ~320-334KB free-heap ESP32 (less once WiFi/BT are active) for
the ~40KB metadata table, NAL scratch, and whatever else the sketch
needs (camera driver buffers, network buffers, etc.). Encoder's fixed
2-buffer design (~230KB, no ref-count knob to turn) is the same
ballpark. Running *both* decoder and encoder resident at once at QVGA on
a plain ESP32 (e.g. a decode-transcode-encode relay) is tight even with
`setMaxRefFrames(1)` - PSRAM (below) is the more comfortable choice for
that combination.

**Explicit lifecycle control**: both classes allocate lazily by default
(as described above - first real encode/decode call), but both also
expose `begin()`/`end()` if you'd rather control exactly when that
happens: `begin()` reserves the picture buffers (and, on the encoder
side, optionally the RGB/YUV422 conversion scratch too - pass
`begin(true)`) up front instead of waiting for the first call, so an
allocation failure surfaces deterministically in `setup()` rather than
mid-stream - as a `false` return (`begin()` on either class now returns
`bool`) and `Status::kAllocationError`/a `0` `encodeFrame()` return
thereafter, not a crash - see the note below and `StdAllocator.h`'s file
comment for the mechanics. `end()` releases everything back to the heap
and resets the object to a fresh state, for reclaiming that memory before
doing something else memory-hungry without destructing and reconstructing
the whole object. Neither is required - the lazy default and the
destructor's own cleanup are enough for most sketches.

**Allocation failure doesn't crash.** Every heap allocation in this
library - the picture buffers (`Frame<Allocator>`'s Y/U/V planes),
`MbInfoTable`'s per-macroblock metadata, and the encoder's RGB/YUV422
scratch buffers alike, `begin()`'s eager path and the lazy first-use path
alike - goes through `StdAllocator<uint8_t>` (the default `Allocator`,
see `src/StdAllocator.h`) or `PSRAMAllocatorESP32`, both of which return
`nullptr` on failure instead of throwing `std::bad_alloc`; that matters
because many embedded C++ toolchains build with exceptions disabled
entirely, where a thrown exception would call `std::terminate()`
regardless of any `try`/`catch`. None of these go through `std::vector`
either, for the same reason - see `src/common/h264_buffer.h`'s file
comment for why a null-returning allocator and `std::vector` don't mix
safely. `TinyH264Decoder::write()`/`decodeNext()` report
`Status::kAllocationError` (`hasError()` returns `true` for it, the same
as `kError`/`kUnsupported`); `TinyH264Encoder::encodeFrame()` (and its
color-format overloads) return `0`, the same convention already used for
a too-small `dst` buffer or invalid width/height/qp. This is what "the
QVGA-crash story" referenced below (and in the Estimated-max-resolution
section) would report today instead of taking the whole sketch down.

See `src/h264_config.h` to change the compile-time
`H264_MAX_REF_FRAMES`/`H264_MAX_WIDTH`/`H264_MAX_HEIGHT` upper bounds
(shared by both classes), e.g. to lower resolution back to QCIF for an
even tighter plain-ESP32 budget, or to raise it further if targeting a
board with PSRAM (e.g. ESP32-S3) - see the `PSRAMAllocatorESP32` example
in [Decoding](decoding.md#accessing-pixel-data), which works identically
for `TinyH264Encoder<PSRAMAllocatorESP32<uint8_t>>`. PSRAM moves *every*
heap allocation this library makes off the regular heap, via the
`Allocator` template parameter: the picture buffers, `MbInfoTable`'s
per-macroblock metadata (~0.516 bytes/pixel, ~38.7KB at QVGA - by far the
largest of the "static-looking" costs above once resolution grows), and
the encoder's RGB/YUV422 scratch buffers all follow whichever `Allocator`
the `Decoder<Allocator>`/`Encoder<Allocator>` was instantiated with.
Only the NAL/slice scratch buffer (`H264_MAX_NAL_SIZE`, a fixed-size
array, not a heap allocation at all) and the SPS/PPS tables stay on
internal SRAM regardless - see the Estimated-max-resolution section
below for what that leaves as the real ceiling on a PSRAM board.

## Estimated max resolution by board

Decoder-only budget (`setMaxRefFrames(1)`, i.e. 2 resident picture
buffers, plus the per-macroblock metadata table: ~3.52 bytes/pixel
combined), solved for the largest ~4:3, macroblock-aligned (multiple of
16) resolution that leaves **real safety margin** against each board's
available heap - not the largest that fits at zero margin, which is a
mistake this project made once already (see the QVGA-crash story in
git history/`examples/DecodeToDisplay`'s commits): a resolution that
"just barely" fits on paper is not safe in practice once fragmentation,
other libraries, and normal runtime overhead are accounted for.

| Board | Total SRAM | Available heap | Max resolution | Confidence |
|---|---|---|---|---|
| ESP32 (no PSRAM) | ~520KB | ~284KB | **256x192** | Measured on real hardware (this project's `DecodeToDisplay` example) |
| ESP32 with PSRAM† | ~520KB + 2-8MB PSRAM | ~284KB internal | **≥640x480** (VGA) | Stale lower bound, not a re-verified ceiling - see note † |
| ESP32-S3 (no PSRAM) | ~512KB | ~272KB | **256x192** | Estimated - `arduino-cli` static-RAM report only, not measured free-heap |
| ESP32-S3 with PSRAM† | ~512KB + 2-8MB PSRAM | ~272KB internal | **≥640x480** (VGA) | Stale lower bound, not a re-verified ceiling - see note † |
| RP2040 | 264KB | ~219KB | **240x160** (HQVGA) | Estimated - `arduino-cli` report only, no RP2040 hardware tested |
| RP2350 (Pico 2) | 520KB | ~480KB | **320x240** (QVGA) | Estimated - `arduino-cli` report only, no RP2350 hardware tested |
| STM32H750VBT6 (WeAct) | 1MB* | ~488KB | **320x240** (QVGA) | Estimated - `arduino-cli` report only, no STM32 hardware tested |

† `PSRAMAllocatorESP32<uint8_t>` (see [Decoding](decoding.md#accessing-pixel-data)) moves every heap allocation this library makes to PSRAM via the `Allocator` template parameter - as of this table's last update, that includes `MbInfoTable` (the per-macroblock metadata table), not just the picture buffers: an earlier version of this library kept `MbInfoTable` hardcoded to internal SRAM regardless of `Allocator` (the same limitation this footnote used to describe); it's now templated on `Allocator` the same way `Frame` is, via `std::allocator_traits`'s rebind mechanism (`sliceId_`/`mb_` need `int16_t`/`MacroblockInfo` allocators, not `uint8_t` - see `src/common/h264_mb_info.h`'s file comment for the mechanics, and `test/native/test_alloc_failure.cpp` for it verified working through the public API, not just compiling). That removes the resolution-scaling internal-SRAM cost (~0.516 bytes/pixel) the VGA figure below was originally derived from, so the real ceiling on a PSRAM board is now dominated by PSRAM capacity itself (typically 2-8MB, vastly more than 2-4 picture buffers plus one metadata table need at any sane resolution) and the small, fixed (not resolution-scaling) internal-SRAM cost of `H264_MAX_NAL_SIZE` (32KB by default) plus the SPS/PPS tables - nothing left on internal SRAM that grows with resolution at all. **The 640x480 (VGA) figure in this table predates that change and is now a stale lower bound, not a re-verified ceiling** - the real number is very likely considerably higher, but this project hasn't measured it and won't publish a guessed replacement (see this file's own opening note: every figure here is either a real `arduino-cli`/hardware measurement or clearly labeled speculative/estimated) - treat VGA as "confirmed to still fit," not "the limit," until someone actually measures higher. RP2040/RP2350/STM32H750 don't get a "with PSRAM" row here because this project doesn't currently implement a PSRAM allocator for those cores - only `PSRAMAllocatorESP32.h` exists today.

\* STM32H750's 1MB SRAM is split across several regions (DTCM, AXI SRAM,
SRAM1-4); the Arduino core's linker script addresses ~512KB of it as one
heap-usable pool, which is the figure this table's estimate is based on.
Its bigger caveat is **flash, not RAM**: only 128KB internal flash (a
minimal decode-only sketch already uses ~34% of it) - a fuller sketch
(encode+decode+display together) is unlikely to fit without running code
from the board's external QSPI flash (XIP), which this project has never
built or tested against.

**"Available heap" methodology**: for ESP32, this is a real
`ESP.getFreeHeap()` reading from `examples/DecodeToDisplay` on real
hardware. For every other board, it's `arduino-cli`'s own
"leaving N bytes for local variables" static-RAM report for
`examples/DecodeFromProgmem` (a decode-only sketch) - a real compiler
output, but not a live free-heap measurement, and it conflates stack and
heap into one number. Cross-checking the one board with both numbers
(ESP32: `arduino-cli` estimated ~271KB available, the real device
measured ~284-293KB) suggests this proxy runs slightly *conservative*
relative to reality, which is the direction you want an estimate to err
in - but "estimated" rows in this table have not been run on real
hardware and should be verified with `arduino-cli`/a real device before
being relied on, the same way the ESP32 figures in this document were
established the hard way (see `examples/DecodeToDisplay`'s development
history for what a `setMaxRefFrames()` oversight or a fragmented-heap
allocation failure actually looks like at runtime).

