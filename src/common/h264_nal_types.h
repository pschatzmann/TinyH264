#pragma once

// Header-only. nal_unit_type constants (ITU-T H.264 Table 7-1) - a wire-
// format enum needed identically by both the decoder's NAL demuxer
// (decoder/h264_nal.h's NalReader) and the encoder's NAL writer
// (encoder/h264_nal_writer.h/h264_encoder.h), so it lives in common/
// rather than being duplicated or owned by one side.

namespace tinyh264 {

/// nal_unit_type values this library either handles (decode) or emits
/// (encode). Values not listed (e.g. 2-4 data-partitioned slices, 13-18
/// auxiliary/extension types) never appear in a Baseline-profile CAVLC
/// stream and are simply ignored by the decoder's top-level NAL dispatch
/// (Decoder::next()) - this encoder never emits them either.
enum NalUnitType {
  kNalSliceNonIdr = 1,  ///< coded slice of a non-IDR picture (clause 7.3.2.8/7.3.3)
  kNalSliceIdr = 5,     ///< coded slice of an IDR picture - resets reference state
  kNalSei = 6,          ///< supplemental enhancement information (skipped)
  kNalSps = 7,          ///< sequence parameter set (clause 7.3.2.1)
  kNalPps = 8,          ///< picture parameter set (clause 7.3.2.2)
  kNalAud = 9,          ///< access unit delimiter (skipped)
  kNalEndOfSeq = 10,    ///< end of sequence (skipped)
  kNalEndOfStream = 11, ///< end of stream (skipped)
  kNalFiller = 12,      ///< filler data (skipped)
};

}  // namespace tinyh264
