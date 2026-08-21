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

## Performance

**Decoding:** Measured on real hardware, not estimated: QCIF (176x144), `examples/
DecodeFromProgmem`'s built-in benchmark (30 repetitions of the embedded
test clip, `micros()`-timed per frame - see that sketch for the
methodology, including why the timing checkpoint is taken around, not
across, the `Serial` output).

| Board | avg | min | max |
|---|---|---|---|
| ESP32 | 20494 us (48.8 fps) | 20296 us (49.3 fps) | 20647 us (48.4 fps) |
| ESP32-S3 | 16045 us (62.3 fps) | 15243 us (65.6 fps) | 16564 us (60.4 fps) |
| ESP32-P4 | 4720 us (211.9 fps) | 4484 us (223.0 fps) | 5018 us (199.3 fps) |
| RP2040 | 29008 us (34.5 fps) | 22700 us (44.1 fps) | 32263 us (31.0 fps) |
| RP2350 | 26193 us (38.2 fps) | 19782 us (50.6 fps) | 29511 us (33.9 fps) |
| STM32F723 | 13926 us (71.8 fps) | 11377 us (87.9 fps) | 15567 us (64.2 fps) |
| x86 (native)* | 280 us (3569.8 fps) | 181 us (5524.3 fps) | 429 us (2328.7 fps) |

\* Not a board - a native desktop build of the same decode path (no
Arduino framework, `Serial`, or UART involved), same embedded clip and
30-repetition loop, `std::chrono`-timed instead of `micros()`-timed.
Measured on an Intel i7-4650U (single core, `-O2`). Included as a rough
ceiling for how much of each embedded figure above is decode work versus
that core's own overhead, not a fair apples-to-apples comparison with
the boards above it.

All comfortably clear real-time (15-30 fps) for QCIF at Baseline/CAVLC
with the deblocking filter active. None of this decoder's hot paths
(CAVLC entropy decoding, motion-compensation interpolation, the
deblocking filter) are hand-optimized for any of these targets - see
`src/decoder/h264_cavlc.h`'s `decodeVlc()` and `src/common/h264_motion.h`
for what a profiling-driven optimization pass would likely target first
if you need more headroom than this.

**Decode + display, real hardware**: `examples/DecodeToDisplay` (256x192,
decode + RGB565 conversion + SPI push to an ILI9341 TFT, plain ESP32, no
PSRAM) measured avg decode 56742 us (min 50161, max 79416 - content-
dependent) and avg convert+push 55324 us (min 55314, max 55349 - a fixed
SPI-transfer cost, essentially content-independent) - combined, ~112066
us/frame, ~8.9 fps if run back-to-back with no pacing delay. Slower than
the QCIF-only figures above since it's 2.9x the pixel count *and*
includes the display push, not just decode - see that sketch's own
on-device timing instrumentation for the methodology.

**Encoding**: QCIF (176x144), `examples/EncodeSyntheticFrame`'s built-in
benchmark (30 repetitions of an 8-frame synthetic-gradient GOP,
`micros()`-timed per frame, same methodology as the decode table above).
Encoding is far more expensive than decoding - The timings below are w/o any optimizations.

| Board | avg | min | max |
|---|---|---|---|
| ESP32 | 600503 us (1.7 fps) | 256325 us (3.9 fps) | 721200 us (1.4 fps) |
| ESP32-S3 | 486647 us (2.1 fps) | 205299 us (4.9 fps) | 584823 us (1.7 fps) |
| ESP32-P4 |136026  us (7.4 fps) | 27687 us (36.1 fps) | 174451 us (5.7 fps) |
| RP2040* | 401440 us (2.5 fps) | 122553 us (8.2 fps) | 499529 us (2.0 fps) |
| RP2350* | 808982 us (1.2 fps) | 303344 us (3.3 fps) | 983456 us (1.0 fps) |
| STM32H750VBT6* | 88616 us (11.3 fps) | 15744 us (63.5 fps) | 114245 us (8.8 fps) |
| x86 (native)† | 5908 us (169.3 fps) | 2506 us (399.1 fps) | 11483 us (87.1 fps) |

\* Built with the `-O3` ("Optimize Even More") board-menu option;
ESP32/ESP32-S3 above use the Arduino-ESP32 core's fixed `-Os` (that core
has no user-selectable optimization level), so those two rows aren't a
like-for-like comparison with the `*` rows. The RP2350 figure being
slower than RP2040 here is also suspect rather than a real chip
comparison - RP2350's `arduino-pico` board defaults to a lower `CPU
Speed` than RP2040's (150 MHz vs. 200 MHz) and has an ARM-vs-RISC-V
`CPU Architecture` menu RP2040 doesn't even have, either of which could
explain it; not yet root-caused.

† Not a board - a native desktop build of the same encode path (same
rate control target, keyframe interval, and synthetic-gradient GOP
`EncodeSyntheticFrame` uses), `-O2`, same Intel i7-4650U as the decode
table's x86 row - not a fair apples-to-apples comparison with the
boards above it.

These numbers are all too low for streaming live video. Therefore different __optimizations__ have been set in place which will turn fast microcontrollers into a viable option. For slow microcontrollers the functionality is still useful for some special scenaios like Time-Lapse Recording, AI Vision Agents and Streaming Analysis, Low-Storage CCTV & Dashcams, Remote Wildlife Traps ...


**`setAllOptimizationsActive(true)`,  real hardware**: confirmed on
STM32H750VBT6 (`-O3`), combining `Fast` with this session's other encoder
work (a duplicate motion-compensation/transform pass eliminated - see
[Optimizations](docs/optimizations.md#encoding)): avg 88616 -> 21461 us (= 46.6 fps)
(**4.13x faster**).


## Containers

`TinyH264Encoder`/`TinyH264Decoder` speak raw H.264 elementary streams
only (Annex-B NAL units, see
[Preparing input with ffmpeg](docs/preparing-input-with-ffmpeg.md)) -
there's no container muxing/demuxing in this library, deliberately. To
produce a playable video file/stream from `encodeFrame()`'s output, or
to extract the raw H.264 stream from an existing file to feed into
`TinyH264Decoder`, use the
[AudioTools](https://github.com/pschatzmann/arduino-audio-tools) project's
`MuxerMP4`/`DemuxerMP4` (or `MuxerAVI`/`DemuxerAVI`) classes - both work
directly against Annex-B access units, no transcoding needed. Prefer MP4
over AVI unless you have a specific reason not to: classic AVI has no
official H.264 standardization, so "H.264-in-AVI" plays fine in VLC/
ffplay/mpv but not in a browser `<video>` tag.

## Documentation

- [Scope](docs/scope.md) - what's implemented and validated (I/P-slice
  decoding, motion compensation, deblocking, multi-reference) vs. what's
  deliberately out of scope (CABAC, B-slices, High profile, ...), for
  both the decoder and the encoder.
- [Memory budget](docs/memory-budget.md) - real `arduino-cli`-measured
  static/heap RAM cost for both `TinyH264Decoder` and `TinyH264Encoder`,
  and the `begin()`/`end()` explicit-lifecycle-control API.
- [Decoding](docs/decoding.md) - `TinyH264Decoder` usage, accessing
  decoded pixel data (raw planes, single samples, RGB565/666/888,
  packed YUV420), reference-frame count tuning, and PSRAM placement.
- [Encoding](docs/encoding.md) - `TinyH264Encoder` usage, `encodeFrame()`
  and its RGB/YUV422 overloads, automatic I-frame/P-frame dispatch,
  periodic keyframes, and rate control.
- [Preparing input with ffmpeg](docs/preparing-input-with-ffmpeg.md) -
  the exact `ffmpeg`/libx264 command line (and why each flag is needed)
  to produce a stream this decoder can read.
- [Testing](docs/testing.md) - running the native CMake/CTest suite,
  consuming this library as a CMake target or ESP-IDF component, and
  how the test assets themselves were generated.
- [Optimizations](docs/optimizations.md) - encoding
  and decoding chapters covering motion-search performance findings
  (what was optimized, what's still slow and why, options considered for
  going further, and why ESP-DSP/CMSIS-DSP don't help this project's
  targets for that workload), plus other known gaps.

## Installation

For Arduino, download this library as a zip and use Library ->
Include Library -> Add .ZIP Library. Or git clone this project into
your Arduino libraries folder, e.g.

```
cd ~/Documents/Arduino/libraries
git clone https://github.com/pschatzmann/TinyH264.git
```

No external Arduino library dependencies - the only include beyond the
C++ standard library is `Arduino.h` itself (transitively, via the
Arduino build), and PSRAM support (`PSRAMAllocatorESP32.h`) is opt-in
and self-contained.

For CMake or ESP-IDF projects instead, see [Testing](docs/testing.md)
for `add_subdirectory()`/`EXTRA_COMPONENT_DIRS` usage.
