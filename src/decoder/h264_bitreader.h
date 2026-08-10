#pragma once
#include <stdint.h>
#include <stddef.h>

namespace tinyh264 {

// MSB-first bit reader over an RBSP buffer (i.e. NAL payload with emulation
// prevention bytes already stripped - see NalUnit in h264_nal.h).
//
// Header-only / all-inline: this sits on the hottest path of the decoder
// (every syntax element in every macroblock goes through here), so we let
// the compiler inline everything rather than paying call overhead on a
// single-core MCU.

/// MSB-first bitstream reader over a single NAL unit's RBSP (Raw Byte
/// Sequence Payload - emulation-prevention bytes already stripped, see
/// NalUnit in h264_nal.h). Implements the elementary descriptors the H.264
/// syntax tables are written in terms of (clause 7.2, Table 7-1):
/// u(n), ue(v), se(v), plus the byte-alignment and end-of-data helpers
/// (more_rbsp_data(), rbsp_trailing_bits()) needed to walk a whole slice.
/// Every parser in this library (SPS/PPS, slice header, macroblock layer,
/// CAVLC residuals) is built directly on top of this class.
class BitReader {
 public:
  /// Wraps `size` bytes at `data` (not copied/owned - caller must keep the
  /// buffer alive for the BitReader's lifetime) for reading from bit 0.
  BitReader(const uint8_t* data, size_t size)
      : data_(data), size_(size) {}

  /// u(n): reads the next n bits (0 <= n <= 32) as an unsigned integer,
  /// MSB first, per clause 7.2's bit string parsing.
  uint32_t u(int n) {
    uint32_t val = 0;
    while (n-- > 0) {
      val = (val << 1) | readBit();
    }
    return val;
  }

  /// u(1) read as a bool - convenience for the many one-bit "_flag" syntax
  /// elements (e.g. entropy_coding_mode_flag, frame_mbs_only_flag).
  bool flag() { return u(1) != 0; }

  /// ue(v): Exp-Golomb unsigned integer, ITU-T H.264 clause 9.1. Used
  /// throughout the syntax for values with no fixed range (e.g. seq/pic
  /// parameter set ids, mb_type, coded_block_pattern, and indirectly by
  /// se() for signed residuals).
  uint32_t ue() {
    int zeros = 0;
    while (zeros < 32) {
      if (readBit() != 0) break;
      zeros++;
      if (error_) break;
    }
    if (zeros >= 32 || error_) {
      error_ = true;
      return 0;
    }
    uint32_t info = zeros > 0 ? u(zeros) : 0;
    return ((1u << zeros) - 1) + info;
  }

  /// se(v): Exp-Golomb signed integer, clause 9.1.1 (the "mapping of ue(v)
  /// to se(v)" table: codeNum k maps to +ceil(k/2) for odd k, -k/2 for
  /// even k). Used for mb_qp_delta, mvd, and other signed residuals.
  int32_t se() {
    uint32_t k = ue();
    int32_t v = (int32_t)((k + 1) >> 1);
    return (k & 1) ? v : -v;
  }

  /// True if the read position is currently on a byte boundary (bit 0 of
  /// the current byte).
  bool byteAligned() const { return bitPos_ == 0; }

  /// Advances the read position to the start of the next byte if not
  /// already aligned. Used before byte-oriented syntax like I_PCM sample
  /// data (clause 7.3.5, `while( !byte_aligned() )`).
  void byteAlign() {
    if (bitPos_ != 0) {
      bitPos_ = 0;
      bytePos_++;
    }
  }

  // clause 7.2 more_rbsp_data(): true if there is another syntax element
  // before the rbsp_trailing_bits() (stop bit + zero padding) at the end
  // of the buffer.
  /// Callers (e.g. the residual block parser, the macroblock loop's
  /// end-of-slice check) use this instead of a raw end-of-buffer test so
  /// the trailing stop bit itself is never misparsed as data.
  bool moreRbspData() {
    size_t curBit = bytePos_ * 8 + (size_t)bitPos_;
    if (curBit >= size_ * 8) return false;
    if (stopBitIndex_ < 0) stopBitIndex_ = (long)findStopBitIndex();
    if (stopBitIndex_ < 0) return false;
    return (long)curBit < stopBitIndex_;
  }

  /// Total number of bits consumed so far from the start of the buffer.
  size_t bitsConsumed() const { return bytePos_ * 8 + (size_t)bitPos_; }
  /// Index of the byte the next bit will be read from (0-based).
  size_t bytePos() const { return bytePos_; }
  /// The underlying RBSP buffer pointer (not owned).
  const uint8_t* data() const { return data_; }
  /// Size in bytes of the underlying RBSP buffer.
  size_t size() const { return size_; }

  /// Set when a read has run past the end of the buffer (truncated /
  /// corrupt NAL). Callers should abandon the current slice/frame when
  /// this is set, since all subsequently-read values are meaningless.
  bool error() const { return error_; }

 private:
  /// Reads and consumes a single bit; sets error_ and returns 0 past
  /// end-of-buffer rather than reading out of bounds.
  uint32_t readBit() {
    if (bytePos_ >= size_) {
      error_ = true;
      return 0;
    }
    uint32_t bit = (data_[bytePos_] >> (7 - bitPos_)) & 1u;
    if (++bitPos_ == 8) {
      bitPos_ = 0;
      bytePos_++;
    }
    return bit;
  }

  /// Index (bit offset from start, MSB-first numbering) of the
  /// rbsp_stop_one_bit: the last '1' bit in the buffer. Returns -1 if the
  /// buffer is all zero (malformed). Computed lazily and cached on first
  /// moreRbspData() call, since it requires a backward scan from the end
  /// of the buffer.
  size_t findStopBitIndex() const {
    size_t last = size_;
    while (last > 0 && data_[last - 1] == 0) last--;
    if (last == 0) return (size_t)-1;
    uint8_t b = data_[last - 1];
    int lsbPos = 0;  // position of the lowest set bit, counted from LSB=0
    for (int i = 0; i < 8; i++) {
      if (b & (1u << i)) {
        lsbPos = i;
        break;
      }
    }
    return (last - 1) * 8 + (7 - lsbPos);
  }

  const uint8_t* data_;
  size_t size_;
  size_t bytePos_ = 0;
  int bitPos_ = 0;  // next bit to read within data_[bytePos_], 0 = MSB
  bool error_ = false;
  long stopBitIndex_ = -1;  // cached, computed on first moreRbspData() call
};

}  // namespace tinyh264
