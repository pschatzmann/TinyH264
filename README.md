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

**Decode + display, real hardware**: `examples/DecodeToDisplay` (256x192,
decode + RGB565 conversion + SPI push to an ILI9341 TFT, plain ESP32, no
PSRAM) measured avg decode 56742 us (min 50161, max 79416 - content-
dependent) and avg convert+push 55324 us (min 55314, max 55349 - a fixed
SPI-transfer cost, essentially content-independent) - combined, ~112066
us/frame, ~8.9 fps if run back-to-back with no pacing delay. Slower than
the QCIF-only figures above since it's 2.9x the pixel count *and*
includes the display push, not just decode - see that sketch's own
on-device timing instrumentation for the methodology.


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
