# Memory budget

Both `TinyH264Decoder` and `TinyH264Encoder` are sized by default for a
plain ESP32 (no PSRAM), QCIF (176x144) - the budget below applies to
*both*, not just the decoder, since `TinyH264Encoder` holds its own
closed-loop reconstruction picture buffer internally (see
[Encoding](encoding.md)) with the same per-frame cost as the decoder's.
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
[Encoding](encoding.md)) - each ~38KB, ~76KB total, both heap-allocated the
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
example in [Decoding](decoding.md#accessing-pixel-data), which works
identically for `TinyH264Encoder<PSRAMAllocatorESP32<uint8_t>>`.
