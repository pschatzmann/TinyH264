#pragma once
#include <stdint.h>
#include <array>
#include <utility>
#include "../common/Logger.h"
#include "../common/MemoryResource.h"
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
 * Nothing here is templated - SoftwareDecoder holds plain Frame/
 * MbInfoTable members, both non-templated types backed by a
 * MemoryResource chosen at construction (see ../MemoryResource.h) rather
 * than a compile-time Allocator parameter. TinyH264Decoder<Allocator>
 * (see TinyH264Decoder.h) builds an AllocatorMemoryResource<Allocator>
 * from its own Allocator template argument and passes it in here. An
 * allocation failure (out of memory) is reported as
 * DecodeStatus::kAllocationError rather than crashing.
 */

namespace tinyh264 {

/// Outcome of one Decoder::next() call (one NAL unit processed).
enum class DecodeStatus {
  kOk,           ///< a new picture was decoded into frame()
  kNeedMoreData, ///< this NAL wasn't a picture (SPS/PPS/SEI/etc.), keep going
  kUnsupported,  ///< stream uses a feature this decoder doesn't implement
  kError,        ///< corrupt/truncated bitstream
  kAllocationError, ///< a picture buffer allocation failed (out of memory) - see StdAllocator.h; not a bitstream problem, but treated as terminal the same way kError is
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
class SoftwareDecoder {
 public:
  explicit SoftwareDecoder(MemoryResource& memRes)
      : nalReader_(nalScratch_, sizeof(nalScratch_)),
        mbInfo_(memRes),
        curFrame_(memRes),
        refFrames_(makeFrameArray(memRes,
                                   std::make_index_sequence<H264_MAX_REF_FRAMES>{})) {}

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
   * Overrides the H264_MAX_WIDTH/H264_MAX_HEIGHT compile-time defaults
   * (h264_config.h) for this Decoder instance's own picture-buffer and
   * per-macroblock-metadata allocation ceiling - an alternative to a
   * `#define H264_MAX_WIDTH ...`/`#define H264_MAX_HEIGHT ...` before
   * `#include`ing this header, for callers that would rather configure
   * this at runtime (e.g. once in setup()) than via the preprocessor.
   * Propagates to curFrame_, every H264_MAX_REF_FRAMES reference slot,
   * and the macroblock metadata table (see Frame::setMaxDimension()/
   * MbInfoTable::setMaxDimension()) - not just a smaller bound, either
   * direction works, since nothing downstream of this assumes a fixed
   * compile-time size anymore (H264_MAX_REF_FRAMES, sizing the fixed
   * `refFrames_` array itself, and H264_MAX_NAL_SIZE are the only
   * remaining genuinely compile-time-fixed limits). A stream whose SPS
   * declares a resolution bigger than this ceiling is rejected as
   * kUnsupported (see next()'s SPS handling below), same as exceeding
   * the old compile-time H264_MAX_WIDTH/H264_MAX_HEIGHT always was. If
   * picture buffers are already allocated, this releases and reclaims
   * them so the next decode reallocates at the new size - safe to call
   * at any time, not just before the first decode.
   */
  void setMaxDimension(int maxWidth, int maxHeight) {
    maxWidth_ = maxWidth;
    maxHeight_ = maxHeight;
    curFrame_.setMaxDimension(maxWidth, maxHeight);
    for (int i = 0; i < H264_MAX_REF_FRAMES; i++) {
      refFrames_[i].setMaxDimension(maxWidth, maxHeight);
    }
    mbInfo_.setMaxDimension(maxWidth, maxHeight);
  }
  /// The current allocation-ceiling width - see setMaxDimension().
  int maxWidth() const { return maxWidth_; }
  /// The current allocation-ceiling height - see setMaxDimension().
  int maxHeight() const { return maxHeight_; }

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
      if (!parseSps(br, &sps)) {
        H264LOG.error("SoftwareDecoder: SPS parse failed (corrupt/truncated)");
        return DecodeStatus::kError;
      }
      if (sps.unsupported) {
        H264LOG.warn("SoftwareDecoder: SPS id=%d uses an unsupported feature", sps.id);
        return DecodeStatus::kUnsupported;
      }
      if (sps.codedWidth > (uint32_t)maxWidth_ ||
          sps.codedHeight > (uint32_t)maxHeight_) {
        H264LOG.warn("SoftwareDecoder: SPS resolution %ux%u exceeds the allocation ceiling %dx%d",
                      sps.codedWidth, sps.codedHeight, maxWidth_, maxHeight_);
        return DecodeStatus::kUnsupported;
      }
      spsTable_[sps.id % H264_MAX_SPS] = sps;
      haveSps_ = true;
      return DecodeStatus::kNeedMoreData;
    }

    if (nal.type == kNalPps) {
      BitReader br(nal.rbsp, nal.rbspSize);
      Pps pps;
      if (!parsePps(br, &pps)) {
        H264LOG.error("SoftwareDecoder: PPS parse failed (corrupt/truncated)");
        return DecodeStatus::kError;
      }
      if (pps.unsupported) {
        H264LOG.warn("SoftwareDecoder: PPS id=%d uses an unsupported feature", pps.id);
        return DecodeStatus::kUnsupported;
      }
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
  const Frame& frame() const { return curFrame_; }

  /**
   * The SPS in effect for the most recently decoded picture's slices
   * (valid after next() returns DecodeStatus::kOk) - e.g. for its
   * declared frame rate, Sps::frameRate(). Default-constructed
   * (Sps::valid == false, frameRate() == 0.0) before any slice has been
   * decoded.
   */
  const Sps& sps() const { return currentSps_; }

  /**
   * Reserves this decoder's picture buffers (curFrame_ plus all
   * H264_MAX_REF_FRAMES reference slots, each up to their maxWidth()/
   * maxHeight() maximum - see setMaxDimension()/h264_frame.h) up
   * front, instead of the default allocate-on-first-decode behavior
   * (Frame::ensureAllocated(), otherwise first triggered by the first
   * picture next() actually decodes). Entirely optional - next() still
   * allocates lazily on its own if this was never called - but useful on
   * an embedded target that wants any allocation failure to surface
   * deterministically during setup() rather than mid-stream, and to know
   * the decoder's full resident memory footprint is already committed
   * before the first real frame arrives. Safe to call more than once
   * (Frame::ensureAllocated() is itself idempotent). Returns false if any
   * allocation failed (out of memory) - see StdAllocator.h; whichever
   * buffers succeeded before the first failure are left allocated
   * (harmless - end() releases them, or a retried begin() call skips them
   * and just attempts the rest again).
   */
  bool begin() {
    bool ok = curFrame_.ensureAllocated();
    for (int i = 0; i < H264_MAX_REF_FRAMES; i++) {
      ok = refFrames_[i].ensureAllocated() && ok;
    }
    if (!ok) {
      H264LOG.error("SoftwareDecoder::begin: one or more picture buffer allocations failed");
    }
    return ok;
  }

  /**
   * Releases every picture buffer (curFrame_ and all reference slots,
   * via Frame::release()) and resets this decoder's stream state (cached
   * SPS/PPS presence, reference-picture bookkeeping, input-exhaustion
   * flag) back to how a freshly constructed Decoder starts - the
   * counterpart to begin(), for callers that want to reclaim this
   * decoder's resident memory (several times maxWidth() x maxHeight() -
   * see h264_frame.h) before starting an unrelated
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
    currentSps_ = Sps();
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
    if (!haveSps_ || !havePps_) {
      H264LOG.error("decodeSlice: slice NAL arrived before SPS/PPS (haveSps=%d, havePps=%d)",
                     (int)haveSps_, (int)havePps_);
      return DecodeStatus::kError;
    }

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
    if (!pps.valid || pps.id != ppsId) {
      H264LOG.error("decodeSlice: referenced PPS id=%u not found/cached", ppsId);
      return DecodeStatus::kError;
    }
    const Sps& sps = spsTable_[pps.spsId % H264_MAX_SPS];
    if (!sps.valid || sps.id != pps.spsId) {
      H264LOG.error("decodeSlice: referenced SPS id=%u not found/cached", pps.spsId);
      return DecodeStatus::kError;
    }
    currentSps_ = sps;  // see sps(), e.g. TinyH264Decoder::fps()

    /*
     * Re-parse from the start now that sps/pps are known (parseSliceHeader
     * expects a fresh reader).
     */
    BitReader br2(nal.rbsp, nal.rbspSize);
    SliceHeader sh;
    if (!parseSliceHeader(br2, nal, sps, pps, &sh)) {
      H264LOG.error("decodeSlice: slice header parse failed (corrupt/truncated)");
      return DecodeStatus::kError;
    }
    if (sh.unsupported) {
      H264LOG.warn("decodeSlice: slice header uses an unsupported feature");
      return DecodeStatus::kUnsupported;
    }
    int numActiveRefs = (int)sh.numRefIdxL0ActiveMinus1 + 1;
    if (sh.sliceType == kSliceP) {
      if (refFrameCount_ == 0) {
        H264LOG.error("decodeSlice: P-slice with no reference pictures available yet");
        return DecodeStatus::kError;
      }
      /*
       * Checked in this order deliberately: refFrameCount_ can never
       * exceed maxRefFrames_ (the sliding-window insert caps it there),
       * so an "exceeds maxRefFrames_" check placed *after* an "exceeds
       * refFrameCount_" check would be unreachable dead code once the DPB
       * saturates to the cap - every over-limit stream would incorrectly
       * report kError (implying corruption) instead of kUnsupported
       * (implying "raise H264_MAX_REF_FRAMES/call setMaxRefFrames()").
       */
      if (numActiveRefs > maxRefFrames_) {
        H264LOG.warn("decodeSlice: stream wants %d active refs, exceeds maxRefFrames_=%d",
                      numActiveRefs, maxRefFrames_);
        return DecodeStatus::kUnsupported;
      }
      /*
       * Above the cap is ruled out above; a conformant encoder also never
       * signals more active refs than actually exist in the DPB yet
       * (clause 8.2.4 implicitly assumes enough reference pictures exist
       * before num_ref_idx_l0_active_minus1 is raised) - seeing that here
       * means a corrupt/non-conformant stream.
       */
      if (numActiveRefs > refFrameCount_) {
        H264LOG.error("decodeSlice: stream wants %d active refs, only %d exist in the DPB",
                       numActiveRefs, refFrameCount_);
        return DecodeStatus::kError;
      }
    }

    if (sh.firstMbInSlice == 0) {
      if (!mbInfo_.reset(sps.picWidthInMbs, sps.picHeightInMbs)) {
        H264LOG.error("decodeSlice: mbInfo_.reset(%u,%u) failed",
                       sps.picWidthInMbs, sps.picHeightInMbs);
        return DecodeStatus::kAllocationError;
      }
      if (!curFrame_.setSize((int)sps.codedWidth, (int)sps.codedHeight)) {
        H264LOG.error("decodeSlice: curFrame_.setSize(%u,%u) failed",
                       sps.codedWidth, sps.codedHeight);
        return DecodeStatus::kAllocationError;
      }
      sliceCount_ = 0;
    }
    int sliceId = sliceCount_++;

    int mbWidth = (int)sps.picWidthInMbs;
    int mbHeight = (int)sps.picHeightInMbs;
    int totalMbs = mbWidth * mbHeight;
    int qpY = sh.sliceQp;

    MbDecodeContext ctx;
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
        if (br2.error()) {
          H264LOG.error("decodeSlice: bitstream error reading mb_skip_run near mb(%d,%d)",
                         mbAddr % mbWidth, mbAddr / mbWidth);
          return DecodeStatus::kError;
        }
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
      if (!ok) {
        H264LOG.error("decodeSlice: macroblock decode failed at mb(%d,%d)", ctx.mbX, ctx.mbY);
        return DecodeStatus::kError;
      }
      if (result.unsupported) {
        H264LOG.warn("decodeSlice: macroblock at mb(%d,%d) uses an unsupported feature",
                      ctx.mbX, ctx.mbY);
        return DecodeStatus::kUnsupported;
      }
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
         * y/u/v members are Buffer<uint8_t>, so swapping is an O(1)
         * pointer exchange, not a ~38KB+ memcpy; only the actual new
         * entry (refFrames_[0] <- curFrame_) needs a real copy, exactly
         * one per stored reference picture regardless of maxRefFrames_.
         */
        int keep = refFrameCount_ < maxRefFrames_ ? refFrameCount_
                                                    : maxRefFrames_ - 1;
        for (int i = keep; i > 0; i--) {
          std::swap(refFrames_[i], refFrames_[i - 1]);
        }
        /*
         * copyFrom() can only fail (out of memory) the first time a given
         * refFrames_[0] slot is ever populated - once allocated, its
         * storage is kept for the life of the Decoder (see Buffer::
         * release()'s only other caller, end()), so ensureAllocated()
         * inside copyFrom() is a no-op on every later call. Reported as
         * kAllocationError rather than kOk: curFrame_ itself decoded
         * successfully, but silently delivering it while failing to
         * store it as a reference would let a later P-slice reference a
         * picture that was never actually saved - "detect and reject,
         * don't guess" (see this file's own DecodeStatus::kUnsupported
         * policy) applies here too.
         */
        if (!refFrames_[0].copyFrom(curFrame_)) {
          H264LOG.error("decodeSlice: refFrames_[0].copyFrom() failed - picture decoded but not saved as a reference");
          return DecodeStatus::kAllocationError;
        }
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
  static void tagDeblockParams(MbDecodeContext& ctx, const SliceHeader& sh) {
    MacroblockInfo& mb = ctx.mbInfo->at(ctx.mbX, ctx.mbY);
    mb.disableDeblockIdc = (uint8_t)sh.disableDeblockingFilterIdc;
    mb.alphaC0OffsetDiv2 = (int8_t)sh.sliceAlphaC0OffsetDiv2;
    mb.betaOffsetDiv2 = (int8_t)sh.sliceBetaOffsetDiv2;
  }

  /**
   * Builds refFrames_: an array of H264_MAX_REF_FRAMES Frame objects, all
   * sharing the same MemoryResource, none of them default-constructible
   * (Frame requires a MemoryResource& at construction) - the
   * index_sequence pack expansion is the standard idiom for
   * "construct N identical, non-default-constructible elements" without
   * hardcoding N at the call site. Relies on C++17 guaranteed copy
   * elision to materialize the returned std::array directly into
   * refFrames_, no move/copy of the Frame elements involved.
   */
  template <size_t... Is>
  static std::array<Frame, sizeof...(Is)> makeFrameArray(
      MemoryResource& memRes, std::index_sequence<Is...>) {
    return {{((void)Is, Frame(memRes))...}};
  }

  uint8_t nalScratch_[H264_MAX_NAL_SIZE]; ///< emulation-prevention-stripped scratch for the current NAL
  NalReader nalReader_;                   ///< Annex-B demuxer over the buffer given to setInput()

  Sps spsTable_[H264_MAX_SPS];  ///< cached SPS entries, indexed by id % H264_MAX_SPS
  Sps currentSps_;  ///< SPS used by the most recently decoded slice - see sps()
  Pps ppsTable_[H264_MAX_PPS];  ///< cached PPS entries, indexed by id % H264_MAX_PPS
  bool haveSps_ = false, havePps_ = false; ///< true once at least one SPS/PPS has been parsed
  bool inputExhausted_ = false; ///< mirrors inputExhausted()

  MbInfoTable mbInfo_;  ///< per-picture macroblock metadata (see h264_mb_info.h)
  Frame curFrame_;  ///< picture currently being reconstructed
  /**
   * Stored reference pictures, index 0 = most recently decoded (clause
   * 8.2.5.3 sliding window) - see decodeSlice()'s picture-complete block.
   */
  std::array<Frame, H264_MAX_REF_FRAMES> refFrames_;
  int refFrameCount_ = 0;  ///< how many of refFrames_[] are currently valid (0..maxRefFrames_)
  int maxRefFrames_ = H264_MAX_REF_FRAMES;  ///< runtime-active cap, see setMaxRefFrames()
  int sliceCount_ = 0;         ///< slices seen so far in the current picture, doubles as the next sliceId
  int maxWidth_ = H264_MAX_WIDTH;    ///< allocation-ceiling width, see setMaxDimension()
  int maxHeight_ = H264_MAX_HEIGHT;  ///< allocation-ceiling height, see setMaxDimension()
};

}  // namespace tinyh264
