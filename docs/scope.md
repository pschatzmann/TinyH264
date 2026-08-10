# Scope

I- and P-slice decoding (intra + inter prediction,
motion compensation, the in-loop deblocking filter) are implemented and
validated **pixel-exact for luma and chroma** against real `ffmpeg`/
x264-encoded streams, including multi-frame GOPs with deblocking active
and genuine multi-reference-frame prediction (see below).

Supported:
- Baseline profile: I-slices (I_4x4, I_16x16, I_PCM) and P-slices
  (P_16x16, P_16x8, P_8x16, P_8x8 with 8x8/8x4/4x8/4x4 sub-partitions,
  P_Skip), CAVLC entropy coding only (no CABAC)
- Quarter-pel luma motion compensation (6-tap FIR), eighth-pel chroma
  (bilinear)
- The deblocking filter, including per-block boundary-strength derivation
  for both intra and inter macroblocks, and reference-picture-differs
  detection for multi-reference streams
- Up to `H264_MAX_REF_FRAMES` reference pictures (default 3, matching
  `ffmpeg`'s own default `-preset medium` - see
  [Memory budget](memory-budget.md) and
  [Decoding](decoding.md#accessing-pixel-data) for the runtime
  `setMaxRefFrames()` knob), using the *default* reference picture list
  (clause 8.2.4.2) and *sliding window* marking (clause 8.2.5.3) - explicit
  reference list reordering (`ref_pic_list_modification()`) and adaptive
  (MMCO-based) reference marking are parsed but rejected as unsupported
  rather than implemented, since real Baseline encoders overwhelmingly use
  the default/sliding-window behavior this decoder does implement

Not supported (by design, for this decoder's target: small, single-camera-
style Baseline streams on constrained MCUs):
- CABAC, B-slices, weighted prediction, explicit reference list reordering,
  adaptive (MMCO) reference marking, FMO/ASO slice groups, interlaced/MBAFF
  content, High-profile-and-above features (8x8 transform, scaling lists)

Streams using any of the above are detected and rejected
(`DecodeStatus::kUnsupported`) rather than mis-decoded.

`TinyH264Encoder`, the matching encoder, supports I_16x16/I_4x4 intra
and P_16x16/P_Skip inter macroblocks with automatic per-macroblock
Intra fallback on a poor motion match (single reference frame, integer-
pel-only motion search - no P_16x8/P_8x16/P_8x8 sub-partitions, no
multi-reference) plus simple rate control - see [Encoding](encoding.md)
for full scope and usage.
