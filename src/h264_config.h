#pragma once
/*
 * Compile-time resource limits. Tuned for a plain ESP32 (no/little PSRAM).
 *
 * Frame buffers are YUV 4:2:0 planar and sized H264_MAX_WIDTH x H264_MAX_HEIGHT
 * (rounded up to a macroblock, i.e. multiples of 16). curFrame_ (being
 * reconstructed) plus up to H264_MAX_REF_FRAMES stored reference pictures
 * are kept resident - see H264_MAX_REF_FRAMES below for how that count is
 * chosen and how it can be lowered at runtime. Each picture's Y/U/V planes
 * are 3 separate heap allocations (see Frame in common/h264_frame.h), not
 * one merged buffer - this matters on a plain ESP32, where WiFi/BT can
 * fragment the heap badly enough that the *largest single contiguous
 * block* is well below total free heap; splitting keeps each individual
 * allocation small enough to still find a home.
 *
 * QCIF (176x144): 38016 bytes/frame (25344 Y + 6336 U + 6336 V) -> ~76KB
 * for 2 buffers (1 current + 1 reference), ~152KB for 4 (this file's
 * H264_MAX_REF_FRAMES=3 default).
 * QVGA (320x240): 115200 bytes/frame (76800 Y + 19200 U + 19200 V) ->
 * ~230KB for 2 buffers. Each individual plane allocation (largest: Y at
 * 76800 bytes) is small enough to fit a fragmented plain-ESP32 heap in
 * practice, but the *total* (up to ~460KB at the 4-buffer default) may
 * exceed free heap - call TinyH264Decoder::setMaxRefFrames(1) at QVGA on
 * a plain ESP32 to cap it to 2 resident buffers; see
 * examples/DecodeFromProgmem for a worked example. PSRAM (e.g. ESP32-S3
 * via PSRAMAllocatorESP32, see H264_FRAME_ALLOC_PSRAM below) removes the
 * budget pressure entirely and isn't required just to make QVGA fit.
 */

#ifndef H264_MAX_WIDTH
#define H264_MAX_WIDTH 320
#endif

#ifndef H264_MAX_HEIGHT
#define H264_MAX_HEIGHT 240
#endif

/*
 * Compile-time *upper bound* on stored reference pictures - sizes a static
 * array of Frame<Allocator> (Decoder::refFrames_), so it must be fixed at
 * compile time like the other limits in this file. The *active* count used
 * by any given decode is a separate, runtime-adjustable value (default:
 * this constant) - see TinyH264Decoder::setMaxRefFrames() /
 * Decoder::setMaxRefFrames(), clamped to [1, H264_MAX_REF_FRAMES]. Raising
 * the runtime value can never exceed this compile-time bound; raise this
 * constant (via -D or #define before including any TinyH264 header) if you
 * need more headroom than the default.
 *
 * H.264 Baseline profile permits multiple reference frames (it restricts
 * which *coding tools* are allowed, not the DPB size) - libx264's actual
 * default reference-frame count depends on -preset, not on
 * -profile:v baseline alone (verified via ffmpeg's trace_headers filter):
 * - ultrafast          1
 * - fast                2
 * - medium (ffmpeg default) 3
 * - slow                5
 * - veryslow           16
 * 3 is chosen as this library's default to match ffmpeg's own default
 * preset out of the box, without requiring every caller to pass -preset
 * ultrafast/-x264-params ref=1 just to stay decodable.
 */
#ifndef H264_MAX_REF_FRAMES
#define H264_MAX_REF_FRAMES 3
#endif

// Macroblocks are always 16x16 luma / 8x8 chroma in 4:2:0.
#define H264_MB_SIZE 16
#define H264_MAX_MB_WIDTH ((H264_MAX_WIDTH + H264_MB_SIZE - 1) / H264_MB_SIZE)
#define H264_MAX_MB_HEIGHT ((H264_MAX_HEIGHT + H264_MB_SIZE - 1) / H264_MB_SIZE)
#define H264_MAX_MBS (H264_MAX_MB_WIDTH * H264_MAX_MB_HEIGHT)

/*
 * Maximum size of a single NAL unit's RBSP payload we will buffer at once
 * (after emulation-prevention removal). Slice data for QCIF frames is well
 * under this even at low QP; raise if you see H264_ERR_NAL_TOO_LARGE.
 */
#ifndef H264_MAX_NAL_SIZE
#define H264_MAX_NAL_SIZE 32768
#endif

/*
 * Number of SPS/PPS entries retained simultaneously (ids 0..N-1). Most
 * streams use a single SPS/PPS pair.
 */
#define H264_MAX_SPS 4
#define H264_MAX_PPS 4
