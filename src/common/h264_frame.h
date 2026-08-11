#pragma once
#include <stdint.h>
#include <string.h>
#include <memory>
#include <vector>
#include "h264_config.h"

/*
 * Header-only. A single YUV 4:2:0 planar picture buffer, MB-aligned
 * (coded_width x coded_height, both multiples of 16), sized up to
 * H264_MAX_WIDTH x H264_MAX_HEIGHT; see h264_config.h for the memory
 * budget this implies.
 *
 * Y, U, and V live in *three separate* std::vector<uint8_t, Allocator>
 * buffers rather than one merged contiguous allocation. A merged buffer
 * was tried first (one allocator call per Frame, whole picture DMA/bulk-
 * copy friendly), but at larger resolutions (e.g. QVGA, 115200 bytes for
 * one frame) that single allocation can exceed the *largest available
 * contiguous block* on a plain ESP32 even when total free heap would be
 * enough - heap fragmentation from WiFi/BT and other subsystems limits
 * the biggest single malloc, not just the running total. Splitting into
 * 3 smaller allocations (luma 76800 bytes, each chroma plane 19200 bytes
 * at QVGA) makes each individually much more likely to fit a fragmented
 * heap's largest block, at the cost of losing the single-contiguous-
 * buffer property. y()/u()/v() below return pointers into whichever of
 * the three buffers is relevant; nothing outside this struct needs to
 * know how many allocations back it. A fixed-size array isn't used
 * instead: a Decoder holds several Frames (current + up to
 * H264_MAX_REF_FRAMES references), and on a plain ESP32 those as
 * *static* struct members overflow the DRAM .bss segment before
 * anything else in the sketch even gets a chance to run - confirmed by
 * actually compiling the example against esp32:esp32 with arduino-cli,
 * not just by the theoretical budget math in h264_config.h. The
 * Allocator template parameter (propagated from
 * TinyH264Decoder<Allocator>, default std::allocator<uint8_t>) lets
 * callers point these buffers at PSRAM or a custom pool instead of the
 * default heap, without touching decoder internals. Sizing happens once
 * at startup, never per-frame, so this doesn't reintroduce allocation
 * into the decode hot path.
 */

namespace tinyh264 {

/**
 * A single YUV 4:2:0 planar decoded picture (ITU-T H.264 clause 6.2:
 * "Source, decoded, and output pictures"), MB-aligned to
 * coded_width x coded_height. A Decoder<Allocator> keeps several of
 * these resident: the picture currently being reconstructed, plus up to
 * H264_MAX_REF_FRAMES stored reference pictures. The Allocator template
 * parameter controls where the backing storage lives (regular heap by
 * default; see PSRAMAllocatorESP32.h to place it in PSRAM instead).
 */
template <typename Allocator = std::allocator<uint8_t>>
struct Frame {
  /**
   * Upper bound on the luma plane's element count, from the compile-time
   * H264_MAX_WIDTH/H264_MAX_HEIGHT budget in h264_config.h.
   */
  static const int kMaxLumaSize = H264_MAX_WIDTH * H264_MAX_HEIGHT;
  /**
   * Upper bound on each chroma plane's element count (quarter-resolution
   * luma, per 4:2:0 subsampling).
   */
  static const int kMaxChromaSize = (H264_MAX_WIDTH / 2) * (H264_MAX_HEIGHT / 2);
  /// Upper bound on the combined Y+U+V byte count (for memory-budget docs only - not one allocation, see dataY/dataU/dataV below).
  static const int kMaxTotalSize = kMaxLumaSize + 2 * kMaxChromaSize;

  /**
   * Y, U, and V samples, each its own separate allocation (unlike a
   * merged single buffer, this keeps every individual allocation small
   * enough to fit a fragmented heap's largest free block - see the file
   * header comment above). Prefer y()/u()/v() (or yRow()/uRow()/vRow())
   * over indexing these directly.
   */
  std::vector<uint8_t, Allocator> dataY;
  std::vector<uint8_t, Allocator> dataU;
  std::vector<uint8_t, Allocator> dataV;

  int width = 0;   ///< coded (MB-aligned) luma width in samples
  int height = 0;  ///< coded (MB-aligned) luma height in samples
  int strideY = 0; ///< samples per luma row (== width; no row padding)
  int strideC = 0; ///< samples per chroma row (== width / 2)

  // Picture order / bookkeeping used by the reference-picture logic.
  uint32_t frameNum = 0;   ///< slice header frame_num of this picture (clause 7.4.3)
  bool isReference = false; ///< true if this picture was coded with nal_ref_idc != 0

  /**
   * Allocates the three backing buffers at their maximum
   * (H264_MAX_WIDTH x H264_MAX_HEIGHT-derived) sizes if not already
   * done. Safe to call more than once (e.g. once per setSize()); a
   * no-op after the first call, so it never re-allocates in the
   * per-frame decode hot path.
   */
  void ensureAllocated() {
    if (!dataY.empty()) return;
    dataY.resize(kMaxLumaSize);
    dataU.resize(kMaxChromaSize);
    dataV.resize(kMaxChromaSize);
  }

  /**
   * Sets this picture's actual coded dimensions (from the active SPS) and
   * derives the plane strides. Must be called before any row accessor or
   * copyFrom(); allocates backing storage on first use via
   * ensureAllocated().
   */
  void setSize(int w, int h) {
    ensureAllocated();
    width = w;
    height = h;
    strideY = w;
    strideC = w / 2;
  }

  /**
   * Releases the three backing buffers back to the allocator (clear() +
   * shrink_to_fit() each) and resets dimensions to zero - the
   * counterpart to ensureAllocated()/setSize(), for callers that want to
   * reclaim this picture's ~38KB+ (H264_MAX_WIDTH x H264_MAX_HEIGHT-
   * sized) buffers when done decoding/encoding instead of leaving them
   * resident for the rest of the program's lifetime. setSize() (via
   * ensureAllocated()) reallocates on the next use, same as before this
   * Frame's first setSize() call.
   */
  void release() {
    dataY.clear();
    dataY.shrink_to_fit();
    dataU.clear();
    dataU.shrink_to_fit();
    dataV.clear();
    dataV.shrink_to_fit();
    width = height = strideY = strideC = 0;
  }

  /// Pointer to the start of the luma plane.
  uint8_t* y() { return dataY.data(); }
  /// Pointer to the start of the Cb (blue-difference chroma) plane.
  uint8_t* u() { return dataU.data(); }
  /// Pointer to the start of the Cr (red-difference chroma) plane.
  uint8_t* v() { return dataV.data(); }
  /// const overload of y().
  const uint8_t* y() const { return dataY.data(); }
  /// const overload of u().
  const uint8_t* u() const { return dataU.data(); }
  /// const overload of v().
  const uint8_t* v() const { return dataV.data(); }

  /// Pointer to the first luma sample of row `row`.
  uint8_t* yRow(int row) { return y() + (size_t)row * strideY; }
  /// Pointer to the first Cb sample of chroma row `row`.
  uint8_t* uRow(int row) { return u() + (size_t)row * strideC; }
  /// Pointer to the first Cr sample of chroma row `row`.
  uint8_t* vRow(int row) { return v() + (size_t)row * strideC; }
  /// const overload of yRow(), for reading a reference picture.
  const uint8_t* yRow(int row) const { return y() + (size_t)row * strideY; }
  /// const overload of uRow(), for reading a reference picture.
  const uint8_t* uRow(int row) const { return u() + (size_t)row * strideC; }
  /// const overload of vRow(), for reading a reference picture.
  const uint8_t* vRow(int row) const { return v() + (size_t)row * strideC; }

  /**
   * Deep-copies `other`'s dimensions and sample data into this Frame.
   * Used to snapshot a just-decoded picture into a reference-picture
   * slot (Decoder::refFrames_) once it has been fully reconstructed and
   * deblocked, so subsequent P-slice motion compensation reads a stable
   * copy even while the next picture starts overwriting the "current"
   * buffer.
   */
  void copyFrom(const Frame& other) {
    ensureAllocated();
    width = other.width;
    height = other.height;
    strideY = other.strideY;
    strideC = other.strideC;
    frameNum = other.frameNum;
    isReference = other.isReference;
    memcpy(y(), other.y(), (size_t)strideY * height);
    memcpy(u(), other.u(), (size_t)strideC * (height / 2));
    memcpy(v(), other.v(), (size_t)strideC * (height / 2));
  }
};

/**
 * Packs a Frame's Y/U/V planes into one tightly-packed, contiguous
 * buffer - standard "I420" layout (Y bytes, then U bytes, then V bytes),
 * sized to this picture's *actual* width/height, no row padding. Frame's
 * own internal storage is already one contiguous allocation (see the
 * struct comment above), but at fixed offsets sized to the compile-time
 * H264_MAX_WIDTH/HEIGHT maximum, not this picture's actual dimensions -
 * so it can't just be handed out directly in general (there could be a
 * gap between the actual Y data and where U starts, if the compile-time
 * maximum is larger than this stream's real resolution). This produces
 * a minimally-sized, portable copy instead, the same role h264_rgb.h's
 * converters play for their output formats - see
 * TinyH264Decoder::toYUV420() for the public-API wrapper. `dst` must
 * have room for width*height + 2*(width/2)*(height/2) uint8_t entries;
 * never allocates. (The whole-Y-plane-in-one-memcpy() below relies on
 * strideY == width, which setSize() always establishes in this library -
 * there's no row padding to skip over.)
 */
template <typename Allocator>
inline void convertFrameToYuv420(const Frame<Allocator>& frame, uint8_t* dst) {
  size_t ySize = (size_t)frame.strideY * frame.height;
  size_t cSize = (size_t)frame.strideC * (frame.height / 2);
  memcpy(dst, frame.y(), ySize);
  memcpy(dst + ySize, frame.u(), cSize);
  memcpy(dst + ySize + cSize, frame.v(), cSize);
}

/**
 * Windowed/tiled counterpart of convertFrameToYuv420() above - packs
 * just a `width` x `height` rectangle starting at (originX, originY)
 * into `dst` (I420 layout, no row padding, sized to width*height +
 * 2*(width/2)*(height/2)). Unlike the whole-frame overload, this can't
 * bulk-memcpy a whole plane at once (a sub-rectangle's rows aren't
 * contiguous in the source, due to the frame's own stride), so it
 * copies row by row instead. `originX`/`originY` must be even (chroma
 * is subsampled 2x - see h264_rgb.h's identical constraint for why) and,
 * together with width/height, must stay within the source frame - not
 * bounds-checked.
 */
template <typename Allocator>
inline void convertFrameToYuv420(const Frame<Allocator>& frame, int originX,
                                  int originY, int width, int height,
                                  uint8_t* dst) {
  uint8_t* dstY = dst;
  uint8_t* dstU = dst + (size_t)width * height;
  uint8_t* dstV = dstU + (size_t)(width / 2) * (height / 2);
  for (int row = 0; row < height; row++) {
    memcpy(dstY + (size_t)row * width, frame.yRow(originY + row) + originX,
           width);
  }
  for (int row = 0; row < height / 2; row++) {
    memcpy(dstU + (size_t)row * (width / 2),
           frame.uRow((originY >> 1) + row) + (originX >> 1), width / 2);
    memcpy(dstV + (size_t)row * (width / 2),
           frame.vRow((originY >> 1) + row) + (originX >> 1), width / 2);
  }
}

/**
 * Scaled counterpart of the windowed convertFrameToYuv420() above -
 * `originX`/`originY`/`width`/`height` describe the requested tile *in
 * the scaled output picture's coordinate space* (scaledWidth x
 * scaledHeight overall, see TinyH264Decoder::setScaleFactor()/
 * widthScaled()/heightScaled()), not the decoded picture's own native
 * resolution. Each output sample (luma and chroma both) is
 * nearest-neighbor sampled from the decoded picture via the same
 * integer-ratio mapping h264_rgb.h's scaled RGB converters use - no
 * floating point, no interpolation. When scaledWidth/scaledHeight equal
 * frame.width/frame.height (scale factor 1.0, the default), this
 * delegates straight to the existing unscaled (bulk-memcpy) path above -
 * zero behavior or performance change for callers who never touch
 * setScaleFactor(). `originX`/`originY` must be even (chroma is
 * subsampled 2x); `dst` must have room for width*height +
 * 2*(width/2)*(height/2) uint8_t entries.
 */
template <typename Allocator>
inline void convertFrameToYuv420(const Frame<Allocator>& frame,
                                  int scaledWidth, int scaledHeight,
                                  int originX, int originY, int width,
                                  int height, uint8_t* dst) {
  if (scaledWidth == frame.width && scaledHeight == frame.height) {
    convertFrameToYuv420(frame, originX, originY, width, height, dst);
    return;
  }
  int srcWidth = frame.width;
  int srcHeight = frame.height;
  int cWidth = width / 2, cHeight = height / 2;
  uint8_t* dstY = dst;
  uint8_t* dstU = dst + (size_t)width * height;
  uint8_t* dstV = dstU + (size_t)cWidth * cHeight;

  for (int row = 0; row < height; row++) {
    int py = (int)((int64_t)(originY + row) * srcHeight / scaledHeight);
    if (py >= srcHeight) py = srcHeight - 1;
    const uint8_t* srcRow = frame.yRow(py);
    uint8_t* dstRow = dstY + (size_t)row * width;
    for (int col = 0; col < width; col++) {
      int px = (int)((int64_t)(originX + col) * srcWidth / scaledWidth);
      if (px >= srcWidth) px = srcWidth - 1;
      dstRow[col] = srcRow[px];
    }
  }

  int srcCWidth = srcWidth / 2, srcCHeight = srcHeight / 2;
  int scaledCWidth = scaledWidth / 2, scaledCHeight = scaledHeight / 2;
  int cOriginX = originX / 2, cOriginY = originY / 2;
  for (int row = 0; row < cHeight; row++) {
    int py = (int)((int64_t)(cOriginY + row) * srcCHeight / scaledCHeight);
    if (py >= srcCHeight) py = srcCHeight - 1;
    const uint8_t* srcURow = frame.uRow(py);
    const uint8_t* srcVRow = frame.vRow(py);
    uint8_t* dstURow = dstU + (size_t)row * cWidth;
    uint8_t* dstVRow = dstV + (size_t)row * cWidth;
    for (int col = 0; col < cWidth; col++) {
      int px = (int)((int64_t)(cOriginX + col) * srcCWidth / scaledCWidth);
      if (px >= srcCWidth) px = srcCWidth - 1;
      dstURow[col] = srcURow[px];
      dstVRow[col] = srcVRow[px];
    }
  }
}

/**
 * Whole-picture convenience overload of the scaled/windowed
 * convertFrameToYuv420() above - `dst` must have room for
 * scaledWidth*scaledHeight + 2*(scaledWidth/2)*(scaledHeight/2)
 * uint8_t entries.
 */
template <typename Allocator>
inline void convertFrameToYuv420(const Frame<Allocator>& frame,
                                  int scaledWidth, int scaledHeight,
                                  uint8_t* dst) {
  convertFrameToYuv420(frame, scaledWidth, scaledHeight, 0, 0, scaledWidth,
                        scaledHeight, dst);
}

}  // namespace tinyh264
