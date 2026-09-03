# Logging

Every failure path across `TinyH264Encoder`/`TinyH264Decoder` and their
internals - allocation failures, a corrupt/truncated bitstream, an
unsupported stream feature, a too-small destination buffer, a hardware
encode timeout or open failure - reports through a single shared logger
instance, `tinyh264::H264LOG` (`src/common/Logger.h`), silent by default:

```cpp
#include <TinyH264Encoder.h>
using namespace tinyh264;

void setup() {
  Serial.begin(115200);
  H264LOG.begin(LogLevel::kInfo);  // opt in - see levels below
}
```

`H264LOG` is a `Logger` (also `src/common/Logger.h`) - a small,
leveled, printf-style logger usable both in an Arduino sketch (writes
to any `Print`, defaulting to `Serial`) and in a plain desktop/CI build
(writes to `stdout` via `printf`, e.g. this project's own
`test/native/test_logger_wiring.cpp`) - `#ifdef ARDUINO` inside the
class picks the right one, so the same `H264LOG.error(...)`/`.warn(...)`
call sites work unmodified either way. `begin()`'s own signature differs
slightly between the two: `begin(LogLevel, Print& = Serial)` on Arduino,
`begin(LogLevel)` off it (there's no `Print` concept there to pass in).

## Why a single shared instance, not a per-object logger

`TinyH264Encoder`/`TinyH264Decoder` (and everything underneath them -
`SoftwareEncoder`, `HwEncoderP4`, `SoftwareDecoder`, macroblock/slice-
header/SPS-PPS parsing) call `H264LOG` directly wherever a failure
happens, with no `Logger` member to construct, no `setLogger()` call to
remember, and no parameter threaded through the many free functions
(macroblock decode, CAVLC residual decode) that would otherwise need
one just to log a single error. `H264LOG` is declared `inline` (a C++17
feature) in the header, so it resolves to exactly one shared instance
regardless of how many translation units end up including
`Logger.h` - the correct header-only-library way to share state like
this without a getter function or a `.cpp` file this project doesn't
otherwise have.

The tradeoff, worth being explicit about: this is one logger for the
whole program, not one per `TinyH264Encoder`/`TinyH264Decoder` instance
- a sketch running two decoders at once can't give them independently
configured loggers. For this project's target (a single encoder and/or
decoder per sketch, logging to `Serial` for diagnostics during
development) that's the right tradeoff; it would not be if this were a
library meant to run many independent codec instances with per-instance
log routing.

## Levels

```cpp
enum class LogLevel : uint8_t {
  kNone = 0,   // logging disabled entirely (the default)
  kError = 1,  // error() only
  kWarn = 2,   // + warn()
  kInfo = 3,   // + info()
  kDebug = 4,  // + debug() - everything
};
```

Each level includes every level above it in severity - the usual
`-v`/`-vv`/`-vvv` model. Nothing in this library currently calls
`info()`/`debug()` itself (both exist on `Logger` for a sketch's own
use); `H264LOG.begin(LogLevel::kInfo)` is enough to see everything this
library itself logs. A default-constructed, never-`begin()`'d `H264LOG`
is silent - on Arduino, that also avoids writing to `Serial` before the
sketch has necessarily called `Serial.begin()`.

- **`error()`**: allocation failure (out of memory - see
  `Buffer::allocate()`, the single choke point every heap allocation in
  this library goes through, so this is the one call site that catches
  all of them), a bitstream/writer destination buffer too small, a
  corrupt/truncated bitstream (a CAVLC/Exp-Golomb decode failure, an
  invalid syntax element value), an internal invariant violation (the
  ESP32-P4 hardware encoder's AUD-padding safety net - see
  [the investigation writeup](esp32-p4-hardware-encoder-investigation.md)),
  a hardware encode timeout or `open()` failure.
- **`warn()`**: a recognized-but-intentionally-unimplemented stream
  feature (`DecodeStatus::kUnsupported` - CABAC, B/SP/SI slices,
  interlaced content, FMO, weighted prediction, adaptive/MMCO reference
  marking, explicit reference list reordering, High-profile scaling
  lists - see [Scope](scope.md) for the full, authoritative list) and
  the ESP32-P4 hardware encoder falling back to the software path after
  a real hardware failure - both are "didn't do what was asked, but not
  corrupted or crashed" outcomes.

## What's deliberately not instrumented

A few genuinely hot, low-level spots are left silent on purpose, since
logging there would either fire on essentially every byte of a
truncated stream (spam, not diagnosis) or cost real per-call overhead
in code that runs thousands of times per frame:

- `BitReader::readBit()`/`ue()` (`src/decoder/h264_bitreader.h`) - the
  single hottest point in the whole decoder (called for every bit of
  every syntax element). Every caller already checks `br.error()` once
  per syntax element/group instead, which is where the corresponding
  `H264LOG.error()` call actually lives.
- `src/decoder/h264_cavlc.h`'s internals
  (`decodeVlc()`/`decodeCoeffToken()`/`decodeLevels()`/
  `decodeTotalZeros()`/`decodeRunBefore()`) and
  `src/decoder/h264_macroblock_inter.h`'s `decodeMvd()`/`decodeRefIdx()`
  - called per-coefficient/per-partition, with no macroblock-address
  context available at that level anyway. Logged once at the nearest
  caller that does have it (`decodeMacroblockIntraWithType()`/
  `decodeMacroblockInter()`, `ctx.mbX`/`ctx.mbY` in the message) instead
  of inside each of these.
- Free functions with no real failure path at all (confirmed, not
  assumed) - `h264_nal.h`, `h264_deblock.h`, `h264_rgb.h`,
  `h264_cavlc_tables.h`, `h264_color_convert.h`,
  `h264_macroblock_encode.h`/`h264_macroblock_encode_inter.h` (pure
  write paths - a `BitWriter` overflow is checked by the *caller*
  afterward, where it's logged instead) - nothing to instrument.

## Verifying the wiring

`test/native/test_logger_wiring.cpp` attaches `H264LOG` at `kDebug` and
deliberately triggers a handful of real failure paths reachable without
hardware (`encodeFrame()` before `setSize()`, rate control requested but
never configured, a too-small destination buffer on both the encoder
and decoder side, a slice NAL arriving before any SPS/PPS) - confirming
the wiring actually compiles and runs end-to-end with logging enabled,
not just that it compiles dormant (`H264LOG` defaults to `kNone`, and
every other test in the suite runs that way, so this is the one test
that exercises the non-default path). Not a correctness oracle like the
rest of `test/native/` - see [Testing](testing.md).
