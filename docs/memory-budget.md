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
mid-stream; `end()` releases everything back to the heap and resets the
object to a fresh state, for reclaiming that memory before doing
something else memory-hungry without destructing and reconstructing the
whole object. Neither is required - the lazy default and the
destructor's own cleanup are enough for most sketches.

See `src/h264_config.h` to change the compile-time
`H264_MAX_REF_FRAMES`/`H264_MAX_WIDTH`/`H264_MAX_HEIGHT` upper bounds
(shared by both classes), e.g. to lower resolution back to QCIF for an
even tighter plain-ESP32 budget, or to raise it further if targeting a
board with PSRAM (e.g. ESP32-S3) - see the `PSRAMAllocatorESP32` example
in [Decoding](decoding.md#accessing-pixel-data), which works identically
for `TinyH264Encoder<PSRAMAllocatorESP32<uint8_t>>`. PSRAM only moves the
*picture buffers* off the regular heap (via the `Allocator` template
parameter) - the metadata table and scratch buffers described above stay
on regular heap either way, since they're small enough (tens of KB, not
hundreds) not to need it.
