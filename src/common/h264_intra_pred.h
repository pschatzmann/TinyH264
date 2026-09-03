#pragma once
#include <stdint.h>
#include <string.h>
#ifdef TINYH264_DEBUG_MB
#include <stdio.h>
#endif
#include "h264_frame.h"
#include "h264_mb_info.h"
#include "h264_tables.h"

/*
 * Header-only. Intra prediction, ITU-T H.264 clause 8.3. Operates directly
 * on the reconstructed pixels already sitting in the Frame buffer (raster
 * decode order guarantees any neighbor this code reads has already been
 * fully reconstructed - see h264_mb_info.h for why the luma 4x4 top-right
 * exception set {3,7,11,13,15} needs no separate bookkeeping).
 */

namespace tinyh264 {

/**
 * Intra_4x4 prediction modes (clause 8.3.1.2, Table 8-2). The nine
 * directional/DC modes for a single 4x4 luma block; see predictIntra4x4().
 */
enum Intra4x4PredMode {
  kI4Vertical = 0,
  kI4Horizontal = 1,
  kI4Dc = 2,
  kI4DiagDownLeft = 3,
  kI4DiagDownRight = 4,
  kI4VerticalRight = 5,
  kI4HorizontalDown = 6,
  kI4VerticalLeft = 7,
  kI4HorizontalUp = 8,
};

/**
 * Intra_16x16 prediction modes (clause 8.3.3, Table 7-11's mb_type
 * decomposition); see predictIntra16x16().
 */
enum Intra16x16PredMode {
  kI16Vertical = 0,
  kI16Horizontal = 1,
  kI16Dc = 2,
  kI16Plane = 3,
};

/**
 * intra_chroma_pred_mode values (clause 8.3.4, Table 8-5); see
 * predictIntraChromaPlane().
 */
enum IntraChromaPredMode {
  kChromaDc = 0,
  kChromaHorizontal = 1,
  kChromaVertical = 2,
  kChromaPlane = 3,
};

/**
 * Luma 4x4 blocks whose "top-right" neighbor is never available, because
 * (raster decode order within the MB, plus right-neighbor-MB never being
 * decoded yet) it can't be - see h264_mb_info.h header comment.
 */
inline bool blk4x4TopRightAlwaysUnavailable(int blkIdx) {
  return blkIdx == 3 || blkIdx == 7 || blkIdx == 11 || blkIdx == 13 ||
         blkIdx == 15;
}

// --- 4x4 luma prediction --------------------------------------------------

/**
 * Reconstructed-sample neighborhood for one 4x4 luma block: the 4 left,
 * top, and top-right samples plus the single top-left corner sample, and
 * which of them are actually available (clause 6.4.9 neighbor
 * derivation). Built by gatherNeighbors4x4(), consumed by
 * predictIntra4x4().
 */
struct Neighbors4x4 {
  bool haveLeft = false, haveTop = false, haveTopLeft = false,
       haveTopRight = false;
  uint8_t left[4] = {0};       // left[y]
  uint8_t top[4] = {0};        // top[x]
  uint8_t topRight[4] = {0};   // top[4+x], substituted if unavailable
  uint8_t topLeft = 0;
};

/**
 * Gathers the Neighbors4x4 for luma 4x4 block `blkIdx` at pixel position
 * (px,py), given the enclosing macroblock's edge availability. Handles
 * both purely-local neighbors (already reconstructed earlier blocks
 * within this same macroblock) and cross-macroblock neighbors, plus the
 * clause 8.3.1.2.1 top-right substitution (repeat the last top sample)
 * when the true top-right neighbor doesn't exist.
 */
inline Neighbors4x4 gatherNeighbors4x4(const Frame& f, int px, int py,
                                        int blkIdx, bool mbLeftAvail,
                                        bool mbTopAvail, bool mbTopLeftAvail,
                                        bool mbTopRightAvail) {
  Neighbors4x4 n;
  int bx = kBlk4x4X[blkIdx], by = kBlk4x4Y[blkIdx];

  n.haveLeft = (bx > 0) || mbLeftAvail;
  n.haveTop = (by > 0) || mbTopAvail;
  n.haveTopLeft = (bx > 0 && by > 0) || (bx == 0 && by > 0 && mbLeftAvail) ||
                  (bx > 0 && by == 0 && mbTopAvail) ||
                  (bx == 0 && by == 0 && mbTopLeftAvail);

  if (blk4x4TopRightAlwaysUnavailable(blkIdx)) {
    n.haveTopRight = false;
  } else if (by == 0) {
    n.haveTopRight = (bx == 3) ? mbTopRightAvail : mbTopAvail;
  } else {
    n.haveTopRight = true;  // local, already decoded (see header comment)
  }

  if (n.haveLeft) {
    for (int i = 0; i < 4; i++) n.left[i] = f.yRow(py + i)[px - 1];
  }
  if (n.haveTop) {
    const uint8_t* row = f.yRow(py - 1);
    for (int i = 0; i < 4; i++) n.top[i] = row[px + i];
  }
  if (n.haveTopLeft) {
    n.topLeft = f.yRow(py - 1)[px - 1];
  }
  if (n.haveTopRight) {
    const uint8_t* row = f.yRow(py - 1);
    for (int i = 0; i < 4; i++) n.topRight[i] = row[px + 4 + i];
  } else if (n.haveTop) {
    for (int i = 0; i < 4; i++) n.topRight[i] = n.top[3];
  }
  return n;
}

/**
 * Predicts and writes one 4x4 luma block at pixel position (px,py) using
 * the given mode (clause 8.3.1.2's nine Intra_4x4 modes) and neighbor
 * samples, writing directly into the Frame (the caller adds the residual
 * on top afterward). `n.haveTop`/`haveLeft` are assumed true for any mode
 * other than kI4Dc that needs them - the mb_type/pred_mode signaling
 * itself guarantees a non-DC mode is only ever coded when its required
 * neighbors are available (clause 8.3.1.1), so only kI4Dc needs the
 * explicit availability branches.
 */
inline void predictIntra4x4(Frame& f, int px, int py, int mode,
                             const Neighbors4x4& n) {
  uint8_t pred[16];  // raster, [y*4+x]
  auto topAt = [&](int i) -> int { return i < 0 ? n.topLeft : n.top[i]; };
  auto leftAt = [&](int i) -> int { return i < 0 ? n.topLeft : n.left[i]; };
  // Extended top row t[0..7] = top[0..3] followed by topRight[0..3].
  uint8_t t[8] = {n.top[0],      n.top[1],      n.top[2],      n.top[3],
                  n.topRight[0], n.topRight[1], n.topRight[2], n.topRight[3]};

  switch (mode) {
    case kI4Vertical:
      for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++) pred[y * 4 + x] = n.top[x];
      break;
    case kI4Horizontal:
      for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++) pred[y * 4 + x] = n.left[y];
      break;
    case kI4Dc: {
      int dc;
      if (n.haveTop && n.haveLeft) {
        int s = n.top[0] + n.top[1] + n.top[2] + n.top[3] + n.left[0] +
                n.left[1] + n.left[2] + n.left[3];
        dc = (s + 4) >> 3;
      } else if (n.haveTop) {
        int s = n.top[0] + n.top[1] + n.top[2] + n.top[3];
        dc = (s + 2) >> 2;
      } else if (n.haveLeft) {
        int s = n.left[0] + n.left[1] + n.left[2] + n.left[3];
        dc = (s + 2) >> 2;
      } else {
        dc = 128;
      }
      for (int i = 0; i < 16; i++) pred[i] = (uint8_t)dc;
      break;
    }
    case kI4DiagDownLeft:
      for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
          int v;
          if (x == 3 && y == 3) {
            v = (t[6] + 3 * t[7] + 2) >> 2;
          } else {
            int i = x + y;
            v = (t[i] + 2 * t[i + 1] + t[i + 2] + 2) >> 2;
          }
          pred[y * 4 + x] = (uint8_t)v;
        }
      }
      break;
    case kI4DiagDownRight:
      for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
          int v;
          if (x > y) {
            int i = x - y;
            v = (topAt(i - 2) + 2 * topAt(i - 1) + topAt(i) + 2) >> 2;
          } else if (x < y) {
            int i = y - x;
            v = (leftAt(i - 2) + 2 * leftAt(i - 1) + leftAt(i) + 2) >> 2;
          } else {
            v = (n.top[0] + 2 * n.topLeft + n.left[0] + 2) >> 2;
          }
          pred[y * 4 + x] = (uint8_t)v;
        }
      }
      break;
    case kI4VerticalRight:
      for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
          int zVR = 2 * x - y;
          int v;
          if (zVR >= 0 && (zVR & 1) == 0) {
            int i = x - (y >> 1) - 1;
            v = (topAt(i) + topAt(i + 1) + 1) >> 1;
          } else if (zVR >= 1) {
            int i = x - (y >> 1) - 2;
            v = (topAt(i) + 2 * topAt(i + 1) + topAt(i + 2) + 2) >> 2;
          } else if (zVR == -1) {
            v = (n.left[0] + 2 * n.topLeft + n.top[0] + 2) >> 2;
          } else {
            v = (leftAt(y - 1) + 2 * leftAt(y - 2) + leftAt(y - 3) + 2) >> 2;
          }
          pred[y * 4 + x] = (uint8_t)v;
        }
      }
      break;
    case kI4HorizontalDown:
      for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
          int zHD = 2 * y - x;
          int v;
          if (zHD >= 0 && (zHD & 1) == 0) {
            int i = y - (x >> 1) - 1;
            v = (leftAt(i) + leftAt(i + 1) + 1) >> 1;
          } else if (zHD >= 1) {
            int i = y - (x >> 1) - 2;
            v = (leftAt(i) + 2 * leftAt(i + 1) + leftAt(i + 2) + 2) >> 2;
          } else if (zHD == -1) {
            v = (n.left[0] + 2 * n.topLeft + n.top[0] + 2) >> 2;
          } else {
            v = (topAt(x - 1) + 2 * topAt(x - 2) + topAt(x - 3) + 2) >> 2;
          }
          pred[y * 4 + x] = (uint8_t)v;
        }
      }
      break;
    case kI4VerticalLeft:
      for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
          int i = x + (y >> 1);
          int v = (y & 1) == 0 ? (t[i] + t[i + 1] + 1) >> 1
                                : (t[i] + 2 * t[i + 1] + t[i + 2] + 2) >> 2;
          pred[y * 4 + x] = (uint8_t)v;
        }
      }
      break;
    case kI4HorizontalUp:
      for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
          int zHU = x + 2 * y;
          int v;
          if (zHU <= 4 && (zHU & 1) == 0) {
            int i = y + (x >> 1);
            v = (n.left[i] + n.left[i + 1] + 1) >> 1;
          } else if (zHU <= 4) {
            int i = y + (x >> 1);
            v = (n.left[i] + 2 * n.left[i + 1] + n.left[i + 2] + 2) >> 2;
          } else if (zHU == 5) {
            v = (n.left[2] + 3 * n.left[3] + 2) >> 2;
          } else {
            v = n.left[3];
          }
          pred[y * 4 + x] = (uint8_t)v;
        }
      }
      break;
  }

  for (int y = 0; y < 4; y++) {
    uint8_t* row = f.yRow(py + y);
    for (int x = 0; x < 4; x++) row[px + x] = pred[y * 4 + x];
  }
}

/*
 * --- Generic block DC/Vertical/Horizontal/Plane, shared by 16x16 luma and
 *     8x8 chroma (which don't have the 4x4's quirky diagonal modes) ------
 */

/**
 * Computes the Plane-mode value at (x,y) within a size x size block
 * (clause 8.3.3.4 for 16x16 luma, 8.3.4.4 for 8x8 chroma - same formula,
 * different rounding constants selected by `size`), given full-length
 * top[]/left[] arrays (size entries each) and topLeft. Shared by
 * predictIntra16x16() and predictIntraChromaPlane() (the two block sizes
 * that support Plane mode; 4x4 luma does not).
 */
inline int planePredict(int x, int y, int size, const uint8_t* top,
                         const uint8_t* left, uint8_t topLeft) {
  int half = size / 2;
  auto topAt = [&](int i) -> int { return i < 0 ? topLeft : top[i]; };
  auto leftAt = [&](int i) -> int { return i < 0 ? topLeft : left[i]; };
  int H = 0, V = 0;
  for (int i = 0; i < half; i++) {
    H += (i + 1) * (topAt(half + i) - topAt(half - 2 - i));
    V += (i + 1) * (leftAt(half + i) - leftAt(half - 2 - i));
  }
  int a = 16 * (top[size - 1] + left[size - 1]);
  int b, c;
  if (size == 16) {
    b = (5 * H + 32) >> 6;
    c = (5 * V + 32) >> 6;
  } else {  // size == 8 (chroma)
    b = (17 * H + 16) >> 5;
    c = (17 * V + 16) >> 5;
  }
  int v = (a + b * (x - (half - 1)) + c * (y - (half - 1)) + 16) >> 5;
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  return v;
}

// --- 16x16 luma prediction ------------------------------------------------

/**
 * Predicts and writes a whole 16x16 luma macroblock (clause 8.3.3, the
 * four Intra_16x16 modes), reading the macroblock's actual top/left
 * neighbor availability directly (unlike 4x4, which relies on
 * mode-signaling constraints, 16x16 DC handles unavailable neighbors
 * explicitly since kI16Dc doesn't have that same guarantee at the whole-MB
 * level for the topAvail-only/leftAvail-only/neither cases).
 */
inline void predictIntra16x16(Frame& f, int mbX, int mbY, int mode,
                               bool leftAvail, bool topAvail,
                               bool topLeftAvail) {
  int px = mbX * 16, py = mbY * 16;
  uint8_t top[16], left[16];
  uint8_t topLeft = topLeftAvail ? f.yRow(py - 1)[px - 1] : 0;
  if (topAvail) memcpy(top, f.yRow(py - 1) + px, 16);
  if (leftAvail)
    for (int i = 0; i < 16; i++) left[i] = f.yRow(py + i)[px - 1];

  for (int y = 0; y < 16; y++) {
    uint8_t* row = f.yRow(py + y);
    for (int x = 0; x < 16; x++) {
      int v;
      switch (mode) {
        case kI16Vertical:
          v = top[x];
          break;
        case kI16Horizontal:
          v = left[y];
          break;
        case kI16Plane:
          v = planePredict(x, y, 16, top, left, topLeft);
          break;
        case kI16Dc:
        default: {
          if (topAvail && leftAvail) {
            int s = 0;
            for (int i = 0; i < 16; i++) s += top[i] + left[i];
            v = (s + 16) >> 5;
          } else if (topAvail) {
            int s = 0;
            for (int i = 0; i < 16; i++) s += top[i];
            v = (s + 8) >> 4;
          } else if (leftAvail) {
            int s = 0;
            for (int i = 0; i < 16; i++) s += left[i];
            v = (s + 8) >> 4;
          } else {
            v = 128;
          }
          break;
        }
      }
      row[px + x] = (uint8_t)v;
    }
  }
}

// --- Chroma (8x8 per plane, 4:2:0) prediction ------------------------------

/**
 * Predicts and writes one 8x8 chroma plane (Cb or Cr) for a macroblock,
 * clause 8.3.4 (the four chroma prediction modes). Takes a raw plane
 * pointer rather than a Frame, since it's called once per plane (u and
 * v) with otherwise-identical arguments. The DC mode (clause 8.3.4.1)
 * computes each of the four 4x4 quadrants' DC value independently rather
 * than one 8x8-wide average - see the FFmpeg-cross-checked formulas
 * inline below for exactly which neighbor samples each quadrant uses
 * (this is the code that had a bottom-right-quadrant bug, since fixed:
 * when both top and left are available, the bottom-right quadrant's DC
 * is the combined average of top[4..7] AND left[4..7], not one side
 * alone - matching FFmpeg's pred8x8_dc/h264pred_template.c).
 */
inline void predictIntraChromaPlane(uint8_t* plane, int stride, int px,
                                     int py, int mode, bool leftAvail,
                                     bool topAvail, bool topLeftAvail) {
  uint8_t top[8], left[8];
  uint8_t topLeft =
      topLeftAvail ? plane[(py - 1) * stride + (px - 1)] : 0;
  if (topAvail) memcpy(top, plane + (py - 1) * stride + px, 8);
  if (leftAvail)
    for (int i = 0; i < 8; i++) left[i] = plane[(py + i) * stride + px - 1];

  for (int y = 0; y < 8; y++) {
    uint8_t* row = plane + (py + y) * stride;
    for (int x = 0; x < 8; x++) {
      int v;
      switch (mode) {
        case kChromaVertical:
          v = top[x];
          break;
        case kChromaHorizontal:
          v = left[y];
          break;
        case kChromaPlane:
          v = planePredict(x, y, 8, top, left, topLeft);
          break;
        case kChromaDc:
        default: {
          /*
           * Each 4x4 quadrant of the 8x8 chroma block computes its own DC,
           * clause 8.3.4.1 - not a simple 8x8-wide average.
           */
          int qx = (x >> 2) * 4, qy = (y >> 2) * 4;  // quadrant origin 0 or 4
          bool topQuad = (qx == 4 && qy == 0);
          bool leftQuad = (qx == 0 && qy == 4);
          bool cornerQuad = (qx == 0 && qy == 0);
          if (cornerQuad) {
            if (topAvail && leftAvail) {
              int s = top[0] + top[1] + top[2] + top[3] + left[0] + left[1] +
                      left[2] + left[3];
              v = (s + 4) >> 3;
            } else if (topAvail) {
              v = (top[0] + top[1] + top[2] + top[3] + 2) >> 2;
            } else if (leftAvail) {
              v = (left[0] + left[1] + left[2] + left[3] + 2) >> 2;
            } else {
              v = 128;
            }
          } else if (topQuad) {
            if (topAvail) {
              v = (top[4] + top[5] + top[6] + top[7] + 2) >> 2;
            } else if (leftAvail) {
              v = (left[0] + left[1] + left[2] + left[3] + 2) >> 2;
            } else {
              v = 128;
            }
          } else if (leftQuad) {
            if (leftAvail) {
              v = (left[4] + left[5] + left[6] + left[7] + 2) >> 2;
            } else if (topAvail) {
              v = (top[0] + top[1] + top[2] + top[3] + 2) >> 2;
            } else {
              v = 128;
            }
          } else {  // bottom-right quadrant
            if (topAvail && leftAvail) {
              int s = top[4] + top[5] + top[6] + top[7] + left[4] + left[5] +
                      left[6] + left[7];
              v = (s + 4) >> 3;
            } else if (topAvail) {
              v = (top[4] + top[5] + top[6] + top[7] + 2) >> 2;
            } else if (leftAvail) {
              v = (left[4] + left[5] + left[6] + left[7] + 2) >> 2;
            } else {
              v = 128;
            }
          }
          break;
        }
      }
      row[px + x] = (uint8_t)v;
    }
  }
}

}  // namespace tinyh264
