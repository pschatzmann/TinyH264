#pragma once
#include <stdint.h>
#include <memory>
#include <utility>
#include "h264_config.h"
#include "h264_deblock.h"
#include "../common/h264_frame.h"
#include "h264_macroblock.h"
#include "h264_macroblock_inter.h"
#include "../common/h264_mb_info.h"
#include "h264_nal.h"
#include "h264_slice_header.h"
#include "h264_sps_pps.h"

/*
 * Header-only. Top-level decoder: NAL demux -> SPS/PPS -> slice header ->
 * macroblock loop -> reconstructed Frame. Decodes I and P slices (Baseline
 * profile: CAVLC), with up to H264_MAX_REF_FRAMES stored reference
 * pictures (runtime-adjustable via setMaxRefFrames(), see h264_config.h
 * for the compile-time upper bound and rationale for its default).
 * Reference picture management implements only the *default* reference
 * list (clause 8.2.4.2) and *sliding window* marking (clause 8.2.5.3) -
 * explicit list reordering and adaptive (MMCO) marking are rejected as
 * unsupported by parseSliceHeader() (h264_slice_header.h) rather than
 * implemented.
 *
 * Templated on Allocator (default std::allocator<uint8_t>, propagated from
 * TinyH264Decoder<Allocator>) purely because it owns curFrame_ plus an
 * array of reference Frame<Allocator> pictures; see h264_frame.h for why
 * the frame buffers are allocator-parameterized in the first place.
 */

namespace tinyh264 {

/// Outcome of one Decoder::next() call (one NAL unit processed).
enum class DecodeStatus {
  kOk,           ///< a new picture was decoded into frame()
  kNeedMoreData, ///< this NAL wasn't a picture (SPS/PPS/SEI/etc.), keep going
  kUnsupported,  ///< stream uses a feature this decoder doesn't implement
  kError,        ///< corrupt/truncated bitstream
};

/**
 * Top-level H.264 Baseline Profile (CAVLC) decoder: owns NAL demuxing,
 * SPS/PPS state, the per-picture macroblock metadata table, and the
 * resident picture buffers (current + up to H264_MAX_REF_FRAMES
 * references). This is the engine behind the public
 * TinyH264Decoder<Allocator> facade (see TinyH264Decoder.h); most callers
 * should use that instead of this class directly, unless they want to
 * drive NAL-by-NAL decoding themselves without the callback wrapper.
 */
template <typename Allocator = std::allocator<uint8_t>>
class Decoder {
 public:
  Decoder() : nalReader_(nalScratch_, sizeof(nalScratch_)) {}

  /**
   * Sets the runtime-active maximum number of stored reference pictures,
   * clamped to [1, H264_MAX_REF_FRAMES] (the compile-time upper bound -
   * see h264_config.h). Defaults to H264_MAX_REF_FRAMES if never called.
   * Takes effect on the very next reference picture stored (see
   * decodeSlice()'s sliding-window insert) - lowering it immediately
   * starts evicting down to the new cap, no need to wait for an IDR.
   */
  void setMaxRefFrames(int n) {
    if (n < 1) n = 1;
    if (n > H264_MAX_REF_FRAMES) n = H264_MAX_REF_FRAMES;
    maxRefFrames_ = n;
  }
  /// The current runtime-active maximum reference-picture count.
  int maxRefFrames() const { return maxRefFrames_; }

  /**
   * Feeds one full Annex-B buffer (as many NAL units as are in it) and
   * decodes NAL units one at a time via next(). Call setInput() again
   * (with the next chunk, or a new stream) once inputExhausted() is true;
   * there is no separate reset() - passing a fresh buffer is enough.
   */
  void setInput(const uint8_t* data, size_t size) {
    nalReader_.reset(data, size);
    inputExhausted_ = false;
  }

  /**
   * True once next() has found no further NAL units in the current input
   * buffer (as opposed to kNeedMoreData from processing a non-picture NAL
   * like SPS/PPS/SEI, which just means "call next() again").
   */
  bool inputExhausted() const { return inputExhausted_; }

  /**
   * Processes exactly one NAL unit from the current input (SPS/PPS get
   * parsed and cached, VCL slice NALs get decoded into curFrame_). Returns
   * kOk once a full picture has been reconstructed into frame(); call
   * repeatedly to drain the input set by setInput().
   */
  DecodeStatus next() {
    NalUnit nal;
    if (!nalReader_.next(&nal)) {
      inputExhausted_ = true;
      return DecodeStatus::kNeedMoreData;
    }

    if (nal.type == kNalSps) {
      BitReader br(nal.rbsp, nal.rbspSize);
      Sps sps;
      if (!parseSps(br, &sps)) return DecodeStatus::kError;
      if (sps.unsupported) return DecodeStatus::kUnsupported;
      if (sps.codedWidth > H264_MAX_WIDTH ||
          sps.codedHeight > H264_MAX_HEIGHT) {
        return DecodeStatus::kUnsupported;
      }
      spsTable_[sps.id % H264_MAX_SPS] = sps;
      haveSps_ = true;
      return DecodeStatus::kNeedMoreData;
    }

    if (nal.type == kNalPps) {
      BitReader br(nal.rbsp, nal.rbspSize);
      Pps pps;
      if (!parsePps(br, &pps)) return DecodeStatus::kError;
      if (pps.unsupported) return DecodeStatus::kUnsupported;
      ppsTable_[pps.id % H264_MAX_PPS] = pps;
      havePps_ = true;
      return DecodeStatus::kNeedMoreData;
    }

    if (nal.type == kNalSliceIdr || nal.type == kNalSliceNonIdr) {
      if (nal.type == kNalSliceIdr) refFrameCount_ = 0;  // IDR resets the DPB
      return decodeSlice(nal);
    }

    return DecodeStatus::kNeedMoreData;  // SEI, AUD, filler, etc.
  }

  /**
   * The most recently completed picture (valid after next() returns
   * DecodeStatus::kOk).
   */
  const Frame<Allocator>& frame() const { return curFrame_; }

  /**
   * Reserves this decoder's picture buffers (curFrame_ plus all
   * H264_MAX_REF_FRAMES reference slots, each up to their compile-time
   * H264_MAX_WIDTH x H264_MAX_HEIGHT maximum - see h264_frame.h) up
   * front, instead of the default allocate-on-first-decode behavior
   * (Frame::ensureAllocated(), otherwise first triggered by the first
   * picture next() actually decodes). Entirely optional - next() still
   * allocates lazily on its own if this was never called - but useful on
   * an embedded target that wants any allocation failure to surface
   * deterministically during setup() rather than mid-stream, and to know
   * the decoder's full resident memory footprint is already committed
   * before the first real frame arrives. Safe to call more than once
   * (Frame::ensureAllocated() is itself idempotent).
   */
  void begin() {
    curFrame_.ensureAllocated();
    for (int i = 0; i < H264_MAX_REF_FRAMES; i++) {
      refFrames_[i].ensureAllocated();
    }
  }

  /**
   * Releases every picture buffer (curFrame_ and all reference slots,
   * via Frame::release()) and resets this decoder's stream state (cached
   * SPS/PPS presence, reference-picture bookkeeping, input-exhaustion
   * flag) back to how a freshly constructed Decoder starts - the
   * counterpart to begin(), for callers that want to reclaim this
   * decoder's resident memory (several times H264_MAX_WIDTH x
   * H264_MAX_HEIGHT - see h264_frame.h) before starting an unrelated
   * stream, or before doing something else memory-hungry, rather than
   * keeping it allocated for the rest of the program's lifetime. Not
   * required before destruction - the members' own destructors free
   * everything regardless - only useful for freeing memory *before*
   * that, while this object is still alive. Safe to call begin()/next()
   * again afterward to start over.
   */
  void end() {
    curFrame_.release();
    for (int i = 0; i < H264_MAX_REF_FRAMES; i++) refFrames_[i].release();
    haveSps_ = false;
    havePps_ = false;
    inputExhausted_ = false;
    refFrameCount_ = 0;
    sliceCount_ = 0;
  }

 private:
  /**
   * Parses and decodes one VCL (slice) NAL unit's macroblocks into
   * curFrame_, clause 7.3.3/7.3.4 (slice_header/slice_data). On the first
   * slice of a picture (first_mb_in_slice == 0) this also resets the
   * per-picture macroblock metadata and sizes curFrame_ from the active
   * SPS. Runs the deblocking filter and snapshots into refFrame_ once
   * the picture's last macroblock has been decoded (pictureComplete).
   */
  DecodeStatus decodeSlice(const NalUnit& nal) {
    if (!haveSps_ || !havePps_) return DecodeStatus::kError;

    BitReader br(nal.rbsp, nal.rbspSize);
    /*
     * Peek pps_id without committing: slice header needs sps/pps looked up
     * by id first, per clause 7.3.3 (first_mb_in_slice, slice_type,
     * pic_parameter_set_id come before anything SPS/PPS-dependent).
     */
    uint32_t firstMb = br.ue();
    uint32_t sliceTypeRaw = br.ue();
    uint32_t ppsId = br.ue();
    (void)firstMb;
    (void)sliceTypeRaw;

    const Pps& pps = ppsTable_[ppsId % H264_MAX_PPS];
    if (!pps.valid || pps.id != ppsId) return DecodeStatus::kError;
    const Sps& sps = spsTable_[pps.spsId % H264_MAX_SPS];
    if (!sps.valid || sps.id != pps.spsId) return DecodeStatus::kError;

    /*
     * Re-parse from the start now that sps/pps are known (parseSliceHeader
     * expects a fresh reader).
     */
    BitReader br2(nal.rbsp, nal.rbspSize);
    SliceHeader sh;
    if (!parseSliceHeader(br2, nal, sps, pps, &sh)) return DecodeStatus::kError;
    if (sh.unsupported) return DecodeStatus::kUnsupported;
    int numActiveRefs = (int)sh.numRefIdxL0ActiveMinus1 + 1;
    if (sh.sliceType == kSliceP) {
      if (refFrameCount_ == 0) return DecodeStatus::kError;
      /*
       * Checked in this order deliberately: refFrameCount_ can never
       * exceed maxRefFrames_ (the sliding-window insert caps it there),
       * so an "exceeds maxRefFrames_" check placed *after* an "exceeds
       * refFrameCount_" check would be unreachable dead code once the DPB
       * saturates to the cap - every over-limit stream would incorrectly
       * report kError (implying corruption) instead of kUnsupported
       * (implying "raise H264_MAX_REF_FRAMES/call setMaxRefFrames()").
       */
      if (numActiveRefs > maxRefFrames_) return DecodeStatus::kUnsupported;
      /*
       * Above the cap is ruled out above; a conformant encoder also never
       * signals more active refs than actually exist in the DPB yet
       * (clause 8.2.4 implicitly assumes enough reference pictures exist
       * before num_ref_idx_l0_active_minus1 is raised) - seeing that here
       * means a corrupt/non-conformant stream.
       */
      if (numActiveRefs > refFrameCount_) return DecodeStatus::kError;
    }

    if (sh.firstMbInSlice == 0) {
      mbInfo_.reset(sps.picWidthInMbs, sps.picHeightInMbs);
      curFrame_.setSize((int)sps.codedWidth, (int)sps.codedHeight);
      sliceCount_ = 0;
    }
    int sliceId = sliceCount_++;

    int mbWidth = (int)sps.picWidthInMbs;
    int mbHeight = (int)sps.picHeightInMbs;
    int totalMbs = mbWidth * mbHeight;
    int qpY = sh.sliceQp;

    MbDecodeContext<Allocator> ctx;
    ctx.frame = &curFrame_;
    if (sh.sliceType == kSliceP) {
      /*
       * Default reference picture list (clause 8.2.4.2): refFrames_ is
       * already maintained most-recent-first (see the sliding-window
       * insert below), so the active list is just its first numActiveRefs
       * entries, as-is - no separate construction/sort needed.
       */
      ctx.numActiveRefs = numActiveRefs;
      for (int i = 0; i < numActiveRefs; i++) ctx.refList[i] = &refFrames_[i];
    }
    ctx.mbInfo = &mbInfo_;
    ctx.sps = &sps;
    ctx.pps = &pps;
    ctx.sliceId = sliceId;

    int mbAddr = (int)sh.firstMbInSlice;
    while (mbAddr < totalMbs) {
      if (sh.sliceType == kSliceP) {
        uint32_t skipRun = br2.ue();
        if (br2.error()) return DecodeStatus::kError;
        for (uint32_t s = 0; s < skipRun && mbAddr < totalMbs; s++) {
          ctx.mbX = mbAddr % mbWidth;
          ctx.mbY = mbAddr / mbWidth;
          mbInfo_.beginMb(ctx.mbX, ctx.mbY, sliceId);
          decodePSkipMacroblock(ctx, qpY);
          tagDeblockParams(ctx, sh);
          mbAddr++;
        }
        if (mbAddr >= totalMbs) break;
        if (skipRun > 0 && !br2.moreRbspData()) break;
      }

      ctx.mbX = mbAddr % mbWidth;
      ctx.mbY = mbAddr / mbWidth;
      mbInfo_.beginMb(ctx.mbX, ctx.mbY, sliceId);

      MacroblockDecodeResult result;
      bool ok = (sh.sliceType == kSliceP)
                    ? decodeMacroblockInter(br2, ctx, &qpY, &result)
                    : decodeMacroblockIntra(br2, ctx, &qpY, &result);
      if (!ok) return DecodeStatus::kError;
      if (result.unsupported) return DecodeStatus::kUnsupported;
      tagDeblockParams(ctx, sh);

      mbAddr++;
      if (mbAddr >= totalMbs) break;
      if (!br2.moreRbspData()) break;
    }

    bool pictureComplete = (mbAddr >= totalMbs);
    if (pictureComplete) {
      deblockPicture(curFrame_, mbInfo_, pps);
      curFrame_.frameNum = sh.frameNum;
      curFrame_.isReference = (nal.refIdc != 0);
      if (curFrame_.isReference) {
        /*
         * Sliding window reference marking (clause 8.2.5.3, default
         * process): insert as the new most-recent entry (index 0),
         * shifting older entries back; if already at the runtime cap,
         * the oldest (last) entry is evicted first. The shift is done
         * with std::swap rather than Frame::copyFrom() - Frame's
         * y/u/v members are std::vector, so swapping is an O(1)
         * pointer exchange, not a ~38KB+ memcpy; only the actual new
         * entry (refFrames_[0] <- curFrame_) needs a real copy, exactly
         * one per stored reference picture regardless of maxRefFrames_.
         */
        int keep = refFrameCount_ < maxRefFrames_ ? refFrameCount_
                                                    : maxRefFrames_ - 1;
        for (int i = keep; i > 0; i--) {
          std::swap(refFrames_[i], refFrames_[i - 1]);
        }
        refFrames_[0].copyFrom(curFrame_);
        refFrameCount_ = keep + 1;
      }
      return DecodeStatus::kOk;
    }
    return DecodeStatus::kNeedMoreData;  // more slices still to come
  }

  /**
   * Stashes this macroblock's slice-level deblocking filter parameters
   * (clause 7.4.3: disable_deblocking_filter_idc and the alpha/beta
   * offsets) into its MacroblockInfo, since deblockPicture() runs after
   * the whole picture (potentially spanning several slices with
   * different parameters) has been decoded and needs per-MB values.
   */
  static void tagDeblockParams(MbDecodeContext<Allocator>& ctx, const SliceHeader& sh) {
    MacroblockInfo& mb = ctx.mbInfo->at(ctx.mbX, ctx.mbY);
    mb.disableDeblockIdc = (uint8_t)sh.disableDeblockingFilterIdc;
    mb.alphaC0OffsetDiv2 = (int8_t)sh.sliceAlphaC0OffsetDiv2;
    mb.betaOffsetDiv2 = (int8_t)sh.sliceBetaOffsetDiv2;
  }

  uint8_t nalScratch_[H264_MAX_NAL_SIZE]; ///< emulation-prevention-stripped scratch for the current NAL
  NalReader nalReader_;                   ///< Annex-B demuxer over the buffer given to setInput()

  Sps spsTable_[H264_MAX_SPS];  ///< cached SPS entries, indexed by id % H264_MAX_SPS
  Pps ppsTable_[H264_MAX_PPS];  ///< cached PPS entries, indexed by id % H264_MAX_PPS
  bool haveSps_ = false, havePps_ = false; ///< true once at least one SPS/PPS has been parsed
  bool inputExhausted_ = false; ///< mirrors inputExhausted()

  MbInfoTable mbInfo_;         ///< per-picture macroblock metadata (see h264_mb_info.h)
  Frame<Allocator> curFrame_;  ///< picture currently being reconstructed
  /**
   * Stored reference pictures, index 0 = most recently decoded (clause
   * 8.2.5.3 sliding window) - see decodeSlice()'s picture-complete block.
   */
  Frame<Allocator> refFrames_[H264_MAX_REF_FRAMES];
  int refFrameCount_ = 0;  ///< how many of refFrames_[] are currently valid (0..maxRefFrames_)
  int maxRefFrames_ = H264_MAX_REF_FRAMES;  ///< runtime-active cap, see setMaxRefFrames()
  int sliceCount_ = 0;         ///< slices seen so far in the current picture, doubles as the next sliceId
};

}  // namespace tinyh264
