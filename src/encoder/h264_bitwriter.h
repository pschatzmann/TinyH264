#pragma once
#include <stdint.h>
#include <stddef.h>

namespace tinyh264 {

/// MSB-first bitstream writer into a caller-owned RBSP buffer - the
/// encode-side mirror of BitReader (h264_bitreader.h). Implements the same
/// elementary descriptors used throughout the H.264 syntax tables (clause
/// 7.2, Table 7-1): u(n), ue(v), se(v), plus rbspTrailingBits() (clause
/// 7.3.2.11) to close out a syntax structure. Never allocates - `data`/
/// `capacity` is a fixed buffer sized by the caller (see
/// H264_MAX_NAL_SIZE), matching this library's whole allocation-free
/// hot-path philosophy on the decode side.
class BitWriter {
 public:
  /// Wraps `capacity` bytes at `data` (not owned - caller must keep the
  /// buffer alive for the BitWriter's lifetime) for writing from bit 0.
  BitWriter(uint8_t* data, size_t capacity)
      : data_(data), capacity_(capacity) {}

  /// u(n): writes the low n bits (0 <= n <= 32) of `val`, MSB first.
  void u(uint32_t val, int n) {
    for (int i = n - 1; i >= 0; i--) writeBit((val >> i) & 1u);
  }

  /// u(1) written from a bool - convenience for the many one-bit "_flag"
  /// syntax elements.
  void flag(bool b) { writeBit(b ? 1u : 0u); }

  /// ue(v): Exp-Golomb unsigned integer, clause 9.1 - codeNum `val` is
  /// written as leadingZeroBits zero bits, then a 1 bit, then
  /// leadingZeroBits info bits, where leadingZeroBits = floor(log2(val+1))
  /// and the info bits are (val+1) with its leading 1 stripped.
  void ue(uint32_t val) {
    uint32_t codeNum = val + 1;
    int leadingZeroBits = 0;
    // Find the bit-length of codeNum minus 1 (position of its MSB).
    for (uint32_t t = codeNum; t > 1; t >>= 1) leadingZeroBits++;
    for (int i = 0; i < leadingZeroBits; i++) writeBit(0);
    u(codeNum, leadingZeroBits + 1);
  }

  /// se(v): Exp-Golomb signed integer, clause 9.1.1 - the exact inverse of
  /// BitReader::se()'s "mapping of se(v) to ue(v)" (positive v -> codeNum
  /// 2v-1, non-positive v -> codeNum -2v).
  void se(int32_t val) {
    uint32_t codeNum = (val > 0) ? (uint32_t)(2 * val - 1)
                                  : (uint32_t)(-2 * val);
    ue(codeNum);
  }

  /// True if the write position is currently on a byte boundary.
  bool byteAligned() const { return bitPos_ == 0; }

  /// rbsp_trailing_bits() (clause 7.3.2.11): writes the mandatory
  /// rbsp_stop_one_bit (a single 1) followed by cabac_zero_word-style zero
  /// padding up to the next byte boundary. Must be the last thing written
  /// to any RBSP (SPS/PPS/slice data) so BitReader::moreRbspData() /
  /// findStopBitIndex() on the decode side can find the end correctly.
  void rbspTrailingBits() {
    writeBit(1);
    while (bitPos_ != 0) writeBit(0);
  }

  /// Total number of bits written so far.
  size_t bitsWritten() const { return bytePos_ * 8 + (size_t)bitPos_; }
  /// Number of whole bytes written so far, rounding up a partial byte -
  /// only meaningful after rbspTrailingBits() has byte-aligned the stream.
  size_t bytesWritten() const { return bytePos_ + (bitPos_ != 0 ? 1 : 0); }

  /// Set when a write has run past the end of the buffer (undersized
  /// caller-provided capacity). Callers should treat the buffer contents
  /// as incomplete/unusable when this is set, same convention as
  /// BitReader::error().
  bool error() const { return error_; }

 private:
  /// Writes a single bit; sets error_ instead of writing out of bounds.
  void writeBit(uint32_t bit) {
    if (bytePos_ >= capacity_) {
      error_ = true;
      return;
    }
    if (bitPos_ == 0) data_[bytePos_] = 0;
    data_[bytePos_] |= (uint8_t)((bit & 1u) << (7 - bitPos_));
    if (++bitPos_ == 8) {
      bitPos_ = 0;
      bytePos_++;
    }
  }

  uint8_t* data_;
  size_t capacity_;
  size_t bytePos_ = 0;
  int bitPos_ = 0;
  bool error_ = false;
};

}  // namespace tinyh264
