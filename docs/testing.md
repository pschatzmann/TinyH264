# Testing

Correctness is validated by decoding real streams generated with
`ffmpeg`/libx264 and diffing every pixel against `ffmpeg`'s own decode -
not by conformance-suite guesswork.

Build and run the whole suite (31 tests) with CMake + CTest from the repo
root:

```sh
cmake -B build && cmake --build build -j && 
```

This also defines a `TinyH264::TinyH264` CMake target (header-only,
C++17 propagated to consumers) that another CMake project can pull in
directly via `add_subdirectory()`:

```cmake
add_subdirectory(path/to/TinyH264)
target_link_libraries(your_target PRIVATE TinyH264::TinyH264)
```

`add_subdirectory()`-ing it this way does *not* also build TinyH264's own
test suite - `test/native/` only builds when this repo is the top-level
CMake project (`TINYH264_BUILD_TESTS`, on by default in that case, off
otherwise - override either way with `-DTINYH264_BUILD_TESTS=ON/OFF`).

The same `CMakeLists.txt` also doubles as an **ESP-IDF component**
manifest - `if(ESP_PLATFORM)` (the variable ESP-IDF's own build system
sets) routes straight to `idf_component_register()` instead, before any
`project()` call, so no separate copy needs to be kept in sync. To use it
from an ESP-IDF project, either drop (or symlink) this repo into that
project's `components/` directory, or point `EXTRA_COMPONENT_DIRS` at it
in the project's top-level `CMakeLists.txt`:

```cmake
set(EXTRA_COMPONENT_DIRS "path/to/TinyH264")
```

then `REQUIRES TinyH264` (or `PRIV_REQUIRES`) in whichever component's
`CMakeLists.txt` needs the headers, and `#include <TinyH264Decoder.h>` as
usual. Verified with a real `idf.py build` (ESP-IDF v6.2, `esp32` target)
against a minimal component consuming `TinyH264Decoder<>` - not just
read from the ESP-IDF docs. `idf_component.yml` at the repo root carries
the component's registry metadata (version, license, description) for
projects that pull it in via the IDF Component Manager instead.

For quicker one-off iteration on a single test without reconfiguring,
the equivalent direct `g++` invocation still works (from `test/native/`):

```sh
g++ -std=c++17 -O2 -I../../src test_decode_multiframe.cpp -o /tmp/t && /tmp/t
```

Other `test_*.cpp` files cover individual layers (bitstream reader, NAL
parsing, SPS/PPS, slice headers, CAVLC table validity) or specific
behaviors (`test_alloc_failure.cpp`: a mock `Allocator` that fails
`allocate()` on demand, checking that both `TinyH264Decoder` and
`TinyH264Encoder` report the failure cleanly - `Status::kAllocationError`/
a `0` return - rather than crashing; see `src/StdAllocator.h`'s file
comment). Test assets
(`assets/*.264`, `assets/*.yuv`) are pre-generated with `ffmpeg`/libx264
using the same parameters documented in
[Preparing input with ffmpeg](preparing-input-with-ffmpeg.md) above (the
`_nodbf` variants add `deblock=0`; `multiref.264` uses `-x264-params
ref=3:me=umh:subme=8` and complex synthetic motion content specifically to
force genuine use of reference indices other than 0 - confirmed via a
debug build tallying actual ref_idx usage across the decode, not just
that the stream declares multiple references - see `test_decode_
multiref.cpp`, the oracle for the multi-reference-frame feature); the
`.yuv` reference files are `ffmpeg`'s own raw decode of the same `.264`
stream, used as the pixel-exact oracle each test diffs against.
