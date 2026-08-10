#pragma once
#include <stdint.h>
#include <stddef.h>

/*
 * Header-only. Annex-B NAL unit emission - the encoder-side counterpart to
 * decoder/h264_nal.h's NalReader/stripEmulationPrevention(). Kept as a
 * separate concern from BitWriter (h264_bitwriter.h), same architecture as
 * the decode side: BitWriter only ever produces a plain RBSP byte buffer,
 * with no emulation-prevention awareness of its own; this file wraps that
 * RBSP into a real Annex-B NAL unit (start code + NAL header byte +
 * emulation-prevention-escaped payload) as a distinct final step.
 */

namespace tinyh264 {

/**
 * Writes one complete Annex-B NAL unit (start code + NAL header byte +
 * emulation-prevention-escaped RBSP) for `rbsp[0..rbspSize)` into `dst`.
 * `nalRefIdc` (0-3) and `nalType` (see NalUnitType, h264_nal.h) become
 * the NAL header byte's own fields (clause 7.3.1). Emulation prevention
 * (clause 7.4.1.1) is inserted here - a 0x03 byte after any run of two
 * 0x00 bytes immediately followed by a byte <= 0x03 - the exact inverse
 * of stripEmulationPrevention() (h264_nal.h): tracked with the same
 * zero-run-counter approach, reset after each inserted 0x03 the same way
 * stripEmulationPrevention() resets after consuming one. Always uses the
 * 4-byte start code (00 00 00 01) - simpler and universally valid,
 * unlike the 3-byte form which is only safe when the *previous* NAL's
 * trailing bytes are known not to end in a pattern that would fuse with
 * it. Returns the number of bytes written to `dst`, or 0 if `dstCapacity`
 * was too small (nothing written in that case - same fail-safe
 * convention as TinyH264Decoder's to*() converters).
 */
inline size_t writeNalUnit(uint8_t* dst, size_t dstCapacity,
                            uint8_t nalRefIdc, uint8_t nalType,
                            const uint8_t* rbsp, size_t rbspSize) {
  size_t o = 0;
  auto put = [&](uint8_t b) -> bool {
    if (o >= dstCapacity) return false;
    dst[o++] = b;
    return true;
  };

  if (!put(0) || !put(0) || !put(0) || !put(1)) return 0;
  uint8_t header = (uint8_t)(((nalRefIdc & 0x3) << 5) | (nalType & 0x1F));
  if (!put(header)) return 0;

  int zeroRun = 0;
  for (size_t i = 0; i < rbspSize; i++) {
    uint8_t b = rbsp[i];
    if (zeroRun >= 2 && b <= 0x03) {
      if (!put(0x03)) return 0;
      zeroRun = 0;
    }
    if (!put(b)) return 0;
    zeroRun = (b == 0) ? zeroRun + 1 : 0;
  }
  return o;
}

}  // namespace tinyh264
