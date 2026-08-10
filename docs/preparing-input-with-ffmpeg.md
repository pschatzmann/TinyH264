# Preparing input with ffmpeg

This decoder only implements a deliberately small subset of H.264 (see
[Scope](scope.md)); source video needs to be transcoded to match it. The
command below (verified against this decoder, not just written from spec
memory - including re-verifying after the reference-frame changes below)
produces a compatible stream from any input `ffmpeg` can read:

```sh
ffmpeg -i input.mp4 \
  -vf scale=176:144 -pix_fmt yuv420p \
  -c:v libx264 -profile:v baseline -level 3.0 \
  -g 25 \
  -f h264 output.264
```

What each part is doing, and why it's required rather than just a good
default:

- **`-f h264` (and a plain `output.264`/`.h264` filename)** - forces
  ffmpeg's *raw Annex-B* H.264 muxer, i.e. NAL units delimited by
  `00 00 01`/`00 00 00 01` start codes. This is the only container this
  decoder understands (`h264_nal.h`'s `NalReader`) - muxing into
  `.mp4`/`.mkv`/etc. instead stores NALs length-prefixed with no start
  codes, which this decoder cannot parse directly (you'd need to
  demux/repackage first).
- **`-profile:v baseline`** - by itself already forces off everything
  this decoder doesn't implement *except* the reference-frame count (see
  below): no B-slices, no CABAC, no 8x8 transform, no weighted
  prediction - these are hard restrictions of H.264's Baseline profile
  itself (profile_idc 66), not just x264 defaults, so no separate
  `-x264-params` overrides are needed for them (confirmed via `ffprobe
  -show_streams output.264 | grep profile` reading
  `profile=Constrained Baseline`, and by decoding the result through this
  decoder). An older version of this recipe also passed
  `-x264-params cabac=0:bframes=0:weightp=0:8x8dct=0` explicitly - still
  harmless if you're used to writing it, just redundant.
- **Reference frame count** - the one thing `-profile:v baseline` does
  *not* constrain (Baseline still permits multiple reference pictures).
  This decoder supports up to `H264_MAX_REF_FRAMES` (default 3) stored
  references, matching ffmpeg's own default `-preset medium` - so as of
  that default preset, no extra flag is needed here either. This stops
  being true if you pick a slower preset: `-preset slow`/`slower`/
  `veryslow`/`placebo` use 5/6/8/16 reference frames respectively
  (verified via `ffmpeg -i output.264 -c copy -bsf:v trace_headers -f
  null - 2>&1 | grep max_num_ref_frames` across presets), all above the
  default cap - such a stream is rejected as `kUnsupported` unless you
  either add `-x264-params ref=N` (N <= `H264_MAX_REF_FRAMES`) at encode
  time, or raise `H264_MAX_REF_FRAMES` in `h264_config.h` and rebuild.
  Memory-constrained targets can go the other way and pass `ref=1` to
  minimize picture-buffer RAM regardless of `H264_MAX_REF_FRAMES` - see
  [Memory budget](memory-budget.md) and `TinyH264Decoder::setMaxRefFrames()`
  in [Decoding](decoding.md#accessing-pixel-data).
- **`-pix_fmt yuv420p`** - 4:2:0 chroma subsampling; the only chroma
  format this decoder's SPS parser accepts (`chromaFormatIdc != 1` is
  flagged unsupported in `h264_sps_pps.h`).
- **`-vf scale=176:144`** - match `H264_MAX_WIDTH`/`H264_MAX_HEIGHT` in
  `h264_config.h` (QCIF by default; raise both the encode resolution and
  those constants together if targeting a board with more RAM/PSRAM -
  see [Memory budget](memory-budget.md)). Both dimensions must already be
  multiples of 16 (QCIF is); ffmpeg will not pad a non-multiple-of-16
  size for you here.
- **`-g 25`** - GOP length / keyframe interval (one IDR frame every 25
  pictures here). Any value works; a shorter GOP means more I-frames
  (larger file, cheaper to seek/recover from a dropped frame), a longer
  one means more P-frames (smaller file, but every picture after the IDR
  depends on unbroken decode of everything before it, and errors in one
  picture propagate into every picture referencing it afterward).

To also disable the in-loop deblocking filter (useful when isolating
whether a decode mismatch is in prediction/motion/CAVLC vs. specifically
the deblocking filter - the same reason this project's own test suite has
`test_decode_multiframe_nodbf.cpp` alongside the deblocking-enabled
version), add `-x264-params deblock=0`:

```sh
-x264-params deblock=0
```

The decoder handles both cases automatically (`disable_deblocking_filter_
idc` is read per-slice from the bitstream, see `h264_deblock.h`) - no
code change needed to feed it a deblock-disabled stream.

To generate a synthetic test clip instead of transcoding a real file
(handy for a quick smoke test with no source video on hand):

```sh
ffmpeg -f lavfi -i testsrc=size=176x144:rate=25:duration=2 \
  -pix_fmt yuv420p -c:v libx264 -profile:v baseline -level 3.0 \
  -g 25 -f h264 test.264
```
