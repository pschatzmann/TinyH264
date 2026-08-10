#pragma once
#include <stdint.h>
#include <stddef.h>
#include "h264_config.h"
#include "../common/h264_nal_types.h"

namespace tinyh264 {

/**
 * One demuxed, emulation-prevention-stripped NAL unit: the header fields
 * (clause 7.3.1, nal_unit()) plus a pointer to its RBSP payload, ready to
 * be handed to a BitReader for syntax parsing.
 */
struct NalUnit {
  uint8_t type = 0;          ///< nal_unit_type, 5 bits (see NalUnitType)
  uint8_t refIdc = 0;        ///< nal_ref_idc, 2 bits (0 = picture is never used as a reference)
  const uint8_t* rbsp = nullptr;  ///< payload with emulation prevention removed
  size_t rbspSize = 0;            ///< size in bytes of `rbsp`
};

/**
 * Scans `buf` (Annex-B byte stream, clause B.1: NAL units separated by
 * 00 00 01 or 00 00 00 01 start codes) for the next NAL unit starting at
 * or after `pos`. Stateless/free function; use NalReader below for
 * stateful iteration over a whole stream.
 *
 * Returns the byte offset of the start of the NAL unit's header byte
 * (i.e. just past the start code), or (size_t)-1 if no more start codes
 * are found. `nalEnd` receives the offset one past the end of this NAL
 * unit's data (i.e. the start of the next start code, or `size`).
 */
inline size_t findNextStartCode(const uint8_t* buf, size_t size, size_t pos,
                                 size_t* nalEnd) {
  size_t i = pos;
  // find 00 00 01
  while (i + 2 < size) {
    if (buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 1) {
      size_t nalStart = i + 3;
      size_t j = nalStart;
      while (j + 2 < size) {
        if (buf[j] == 0 && buf[j + 1] == 0 && buf[j + 2] == 1) break;
        j++;
      }
      size_t end = (j + 2 < size) ? j : size;
      /*
       * trim trailing zero bytes belonging to the *next* start code's
       * leading zeros (four-byte start code 00 00 00 01 case) is handled
       * naturally since we search for 00 00 01 and back up below.
       */
      while (end > nalStart && buf[end - 1] == 0) end--;
      *nalEnd = end;
      return nalStart;
    }
    i++;
  }
  return (size_t)-1;
}

/**
 * Removes emulation prevention bytes (clause 7.4.1: emulation_prevention_
 * three_byte, the 0x03 in the sequence 00 00 03) from [src, src+srcSize)
 * into dst, turning a raw NAL payload into an RBSP (dst may alias src as
 * long as dst <= src, which is always true here since we only ever
 * remove bytes). Returns the number of bytes written to dst.
 */
inline size_t stripEmulationPrevention(const uint8_t* src, size_t srcSize,
                                        uint8_t* dst, size_t dstCap) {
  size_t o = 0;
  int zeroRun = 0;
  for (size_t i = 0; i < srcSize && o < dstCap; i++) {
    uint8_t b = src[i];
    if (zeroRun >= 2 && b == 0x03) {
      /*
       * drop this byte, reset run (per spec the byte *after* 0x03 is real
       * data and could itself start a new zero run)
       */
      zeroRun = 0;
      continue;
    }
    dst[o++] = b;
    zeroRun = (b == 0) ? zeroRun + 1 : 0;
  }
  return o;
}

/**
 * Stateful iterator over an in-memory Annex-B buffer. Each call to next()
 * strips emulation prevention into a caller-owned scratch buffer sized
 * H264_MAX_NAL_SIZE, so only one NAL's worth of RAM is needed regardless of
 * total stream length - suitable for streaming a file/socket in chunks
 * where the whole stream is not resident in RAM (the caller is responsible
 * for feeding `buf` with enough lookahead to contain full NAL units; for a
 * file read into a moderate-size buffer this is straightforward).
 */
class NalReader {
 public:
  /**
   * `scratch`/`scratchCap` is the caller-owned buffer next() will strip
   * each NAL's emulation prevention into; must be at least as large as
   * the largest NAL unit's payload the caller expects (H264_MAX_NAL_SIZE
   * by convention). Not copied or freed by this class.
   */
  NalReader(uint8_t* scratch, size_t scratchCap)
      : scratch_(scratch), scratchCap_(scratchCap) {}

  /**
   * Rewinds iteration to the start of a new (or the same) Annex-B buffer.
   * `buf` is not copied - it must remain valid until the next reset() or
   * for as long as next() is called.
   */
  void reset(const uint8_t* buf, size_t size) {
    buf_ = buf;
    size_ = size;
    pos_ = 0;
  }

  /**
   * Advances to and decodes the next NAL unit in the buffer set by
   * reset(), stripping emulation prevention into the scratch buffer.
   * Returns true and fills `nal` if another NAL unit was found, false at
   * end of buffer (leaving `nal` untouched).
   */
  bool next(NalUnit* nal) {
    size_t nalEnd;
    size_t start = findNextStartCode(buf_, size_, pos_, &nalEnd);
    if (start == (size_t)-1 || start >= nalEnd) return false;
    pos_ = nalEnd;

    uint8_t header = buf_[start];
    nal->refIdc = (header >> 5) & 0x3;
    nal->type = header & 0x1F;

    size_t payloadSize = nalEnd - (start + 1);
    size_t written = stripEmulationPrevention(buf_ + start + 1, payloadSize,
                                               scratch_, scratchCap_);
    nal->rbsp = scratch_;
    nal->rbspSize = written;
    return true;
  }

 private:
  uint8_t* scratch_;
  size_t scratchCap_;
  const uint8_t* buf_ = nullptr;
  size_t size_ = 0;
  size_t pos_ = 0;
};

}  // namespace tinyh264
