#pragma once

// ESP32-P4 dedicated hardware H.264 encoder, driven directly at the
// register/DMA-descriptor level - a from-scratch C++ port with ZERO
// build or link dependency on Espressif's `esp_h264` or `esp_video`
// components (this repo's own constraint: no dependency, direct or
// transitive, on Espressif's own H.264 codec components). Everything in
// this file talks only to: (1) the H.264 core/DMA hardware registers
// directly, at their fixed physical addresses; (2) genuinely generic,
// public ESP-IDF infrastructure that ships in every ESP32-P4 build,
// Arduino or plain IDF alike - `esp_intr_alloc()`, `esp_cache_msync()`,
// FreeRTOS semaphores, and the public `soc/interrupts.h` interrupt-source
// enum. None of that is H.264-specific. Verified: `esp_intr_alloc.h` and
// `esp_cache.h` both compile clean under a plain `arduino-cli` build for
// `esp32:esp32:esp32p4`.
//
// Methodology: the register layout, DMA descriptor format, and per-frame
// sequencing below were learned by reading Espressif's real
// Apache-2.0-licensed `esp-h264-component` source
// (https://github.com/espressif/esp-h264-component) - not guessed, and
// not derived from public documentation (there isn't any for this
// hardware block). Apache-2.0 permits reading and porting logic with
// attribution; this project already does the equivalent for other
// reference sources throughout (e.g. cross-checking algorithms and
// numeric tables against FFmpeg's GPL-licensed `libavcodec` source while
// writing its own independent decoder). This file's structs and
// sequencing are an independent re-implementation, not a copy-paste.
//
// Register access strategy, deliberately NOT padded struct overlays:
// an earlier draft of this file used hand-typed structs with explicit
// `uint32_t reservedNN[...]` padding arrays to match real hardware
// register spacing - and a padding-count mistake was caught partway
// through (an array sized one word short, which would have compiled
// clean and produced the right total struct size while putting every
// later field at the wrong address - the exact kind of bug a
// `sizeof`-based `static_assert` cannot catch). To eliminate that whole
// risk class rather than just being more careful within it, every
// register here is instead accessed via a plain
// `reinterpret_cast<volatile T*>(base + byte_offset)` helper, with the
// byte offset for every single register independently re-derived (not
// estimated) by summing real field sizes from Espressif's actual struct
// headers, and cross-checked against those headers' own authoritative
// `_Static_assert(sizeof(...) == <total>)` lines (the H.264 core struct
// totals `0xf4` bytes, the DMA struct `0xb64` bytes, both confirmed to
// match a from-scratch field-by-field summation before any offset below
// was trusted). No struct spans more than a handful of fields, so there
// is no padding-count arithmetic left to get wrong.
//
// Chip revision scope: this targets ESP32-P4 chip revision <3.0 ("hw_ver1"
// register layout - real early-silicon boards, including the one this
// was developed and smoke-tested against). Revision >=3.0 ("hw_ver3")
// has a materially different register layout in places (more input
// pixel formats via registers hw_ver1 doesn't have; a dedicated
// bitstream-overflow interrupt bit hw_ver1 lacks) and is deliberately
// NOT supported here - gated via `CONFIG_ESP32P4_REV_MIN_FULL` (this
// build's configured minimum chip revision) being < 300, matching
// exactly how Espressif's own component gates the same split.
//
// **Validation status - read before trusting this file**: unlike every
// other part of this project (pixel-diffed against real ffmpeg output),
// there is no reference decoder to diff this file's hardware output
// against, and no way to single-step real hardware register behavior.
// This file is validated only by: (1) careful source-reading
// cross-referenced against Espressif's real driver, matching its
// register write sequences field-for-field; (2) a real compile against
// `arduino-cli`/`esp32:esp32:esp32p4`; (3) a real flash-and-run smoke
// test on one physical early-silicon ESP32-P4 board. That is a much
// lower rigor bar than this project's usual standard, and hardware-facing
// bugs here can hang or misbehave the chip in ways a software bug can't
// (DMA descriptor errors, unbounded register-poll loops). Treat this
// file's register sequencing as needing extra scrutiny before being
// trusted in a safety- or reliability-sensitive application.
//
// One deliberate, documented deviation from Espressif's own sequencing,
// for safety: their DMA channel reset helper polls a
// `*_reset_avail` status bit in an unbounded `while(!ready);` loop with
// no timeout - if a channel never asserts that bit (a real hardware
// hazard, not just hypothetical), the calling task hangs forever with no
// recovery. This port bounds every such poll with a large iteration cap
// and returns failure instead of spinning forever - see
// `dmaResetChannel()`.
//
// Scope, deliberately narrower than Espressif's own driver:
// - Fixed QP only, no hardware or software rate control (matches this
//   project's own use case, and Espressif's own driver skips its whole
//   rate-control subsystem in the fixed-QP case too - confirmed from
//   their source: `qp_min == qp_max` never creates their RC handle at
//   all). One QP for the whole stream (PPS's `pic_init_qp`,
//   `slice_qp_delta` always 0).
// - No ROI (region-of-interest per-macroblock QP), no motion-vector
//   telemetry output (MVM) - real hardware features, deliberately left
//   disabled rather than ported (neither is needed for a valid encoded
//   stream, and both add real register-sequencing surface area for no
//   benefit here). Espressif's own P-frame start sequence unconditionally
//   starts an MVM DMA channel even when MVM is disabled and no
//   descriptor was ever configured for it - this port intentionally does
//   NOT replicate that (starting a DMA channel with no valid descriptor
//   address configured is an unnecessary risk for a feature this project
//   never uses) - see `startFrameDma()`.
// - Only the packed pseudo-planar pixel format hw_ver1 silicon actually
//   accepts (`O_UYY_E_VYY` - chroma-row-interleaved, not standard planar
//   YUV420) - this file converts from TinyH264's own planar YUV420
//   source into that format; see `convertYuv420ToPackedOuyyEvyy()`.

#if defined(CONFIG_IDF_TARGET_ESP32P4) &&     \
    (!defined(CONFIG_ESP32P4_REV_MIN_FULL) || \
     CONFIG_ESP32P4_REV_MIN_FULL < 300)
#define TINYH264_HW_ENCODER_P4_AVAILABLE 1
#endif

#ifdef TINYH264_HW_ENCODER_P4_AVAILABLE

#include <cstdint>
#include <cstring>

#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "soc/interrupts.h"

#include "h264_bitwriter.h"
#include "h264_sps_pps_writer.h"

namespace tinyh264 {
namespace hw_p4_detail {

// ===========================================================================
// Register access - every offset below is a byte offset from one of the
// two fixed peripheral base addresses, independently re-derived from
// Espressif's real register struct headers (see file header comment for
// the method and cross-check). No padded overlay structs.
// ===========================================================================

constexpr uintptr_t kH264Base = 0x50084000;
constexpr uintptr_t kH264DmaBase = 0x500A7000;
// DR_REG_HP_SYS_CLKRST_BASE = DR_REG_HPPERIPH1_BASE(0x500C0000) + 0x26000,
// resolved from ESP-IDF's own public
// soc/esp32p4/register/hw_ver1/soc/reg_base.h - a generic SoC
// clock/reset controller, not H.264-specific.
constexpr uintptr_t kHpSysClkrstBase = 0x500E6000;

template <typename T>
inline volatile T* reg(uintptr_t base, uintptr_t byteOffset) {
  return reinterpret_cast<volatile T*>(base + byteOffset);
}

// --- H.264 core registers (base kH264Base) ---
// Every offset below independently summed from h264_struct.h's real
// h264_dev_t/h264_ctrl_regs_t field lists and cross-checked against that
// header's own `_Static_assert(sizeof(h264_dev_t) == 0xf4, ...)`.
constexpr uintptr_t kOffSysCtrl = 0x00;
constexpr uintptr_t kOffGopConf = 0x04;
// ctrl[0] starts at 0x08; ctrl[1] (unused - this driver only uses
// stream/channel 0) would start at 0x08 + 0x4c = 0x54.
constexpr uintptr_t kOffCtrl0Base = 0x08;
constexpr uintptr_t kOffCtrl0SysMbRes = kOffCtrl0Base + 0x00;
constexpr uintptr_t kOffCtrl0SysConf = kOffCtrl0Base + 0x04;
constexpr uintptr_t kOffCtrl0DeciScore = kOffCtrl0Base + 0x08;
constexpr uintptr_t kOffCtrl0DeciScoreOffset = kOffCtrl0Base + 0x0C;
constexpr uintptr_t kOffCtrl0RcConf0 = kOffCtrl0Base + 0x10;
constexpr uintptr_t kOffCtrl0RcConf1 = kOffCtrl0Base + 0x14;
constexpr uintptr_t kOffCtrl0DbBypass = kOffCtrl0Base + 0x18;
constexpr uintptr_t kOffCtrl0NoRoiRegionQpOffset = kOffCtrl0Base + 0x44;
constexpr uintptr_t kOffCtrl0RoiConfig = kOffCtrl0Base + 0x48;
// ctrl[1] (video stream B, never otherwise touched by this
// single-stream-only driver) - base 0x08 + 0x4c = 0x54, matching this
// file's own long-standing comment on ctrl[0]/ctrl[1] spacing.
constexpr uintptr_t kOffCtrl1Base = 0x54;
constexpr uintptr_t kOffCtrl1NoRoiRegionQpOffset = kOffCtrl1Base + 0x44;
constexpr uintptr_t kOffCtrl1RoiConfig = kOffCtrl1Base + 0x48;
constexpr uintptr_t kOffSliceHeaderRemain = 0xAC;
constexpr uintptr_t kOffSliceHeaderByteLength = 0xB0;
constexpr uintptr_t kOffSliceHeader0 = 0xB8;
constexpr uintptr_t kOffSliceHeader1 = 0xBC;
constexpr uintptr_t kOffIntRaw = 0xC0;
constexpr uintptr_t kOffIntSt = 0xC4;
constexpr uintptr_t kOffIntEna = 0xC8;
constexpr uintptr_t kOffIntClr = 0xCC;
constexpr uintptr_t kOffConf = 0xD0;
constexpr uintptr_t kOffSysStatus = 0xDC;
constexpr uintptr_t kOffFrameCodeLength = 0xE0;
constexpr uintptr_t kOffDebugInfo0 = 0xE4;
constexpr uintptr_t kOffDebugInfo1 = 0xE8;

// --- H.264 2D-DMA registers (base kH264DmaBase) ---
// Every out/in channel block is 0x100 bytes (independently summed from
// h264_dma_struct.h's h264_dma_out_chn_regs_t/h264_dma_in_chn_regs_t;
// both sum to exactly 64 words), and in_ch5 is a third, differently
// laid out 0x100-byte block right after them - cross-checked against
// h264_dma_struct.h's own `_Static_assert(sizeof(h264_dma_dev_t) ==
// 0xb64, ...)` (5*0x100 + 5*0x100 + 0x100 + 0x64 tail == 0xb64).
constexpr uintptr_t kDmaChnStride = 0x100;
constexpr uintptr_t kOffOutCh0 = 0x000;               // out_ch[i] = kOffOutCh0 + i*kDmaChnStride
constexpr uintptr_t kOffInCh0 = 5 * kDmaChnStride;    // in_ch[i]  = kOffInCh0  + i*kDmaChnStride
constexpr uintptr_t kOffInCh5 = 10 * kDmaChnStride;   // 0xA00
// Relative offsets within an out_ch[i]/in_ch[i] block (identical layout
// for both, confirmed field-by-field from the real struct).
constexpr uintptr_t kChnConf0 = 0x00;
constexpr uintptr_t kChnIntClr = 0x10;
constexpr uintptr_t kChnLinkConf = 0x1C;
constexpr uintptr_t kChnLinkAddr = 0x20;
constexpr uintptr_t kChnState = 0x24;
constexpr uintptr_t kCh5IntClr = kOffInCh5 + 0x1C;
// in_ch5 has its own, different layout.
constexpr uintptr_t kCh5Conf0 = kOffInCh5 + 0x00;
constexpr uintptr_t kCh5Conf1 = kOffInCh5 + 0x04;  // block_start_addr
constexpr uintptr_t kCh5Conf2 = kOffInCh5 + 0x08;  // block_row_length_{12,4}line
constexpr uintptr_t kCh5Conf3 = kOffInCh5 + 0x0C;  // block_length_{12,4}line
constexpr uintptr_t kCh5State = kOffInCh5 + 0x28;
// Tail registers after in_ch5 (kOffInCh5 + 0x100 = 0xB00).
constexpr uintptr_t kOffInterMemAddr0Start = 0xB0C;
constexpr uintptr_t kOffInterMemAddr0End = 0xB10;
constexpr uintptr_t kOffInterMemAddr1Start = 0xB14;
constexpr uintptr_t kOffInterMemAddr1End = 0xB18;
constexpr uintptr_t kOffExterMemAddr0Start = 0xB20;
constexpr uintptr_t kOffExterMemAddr0End = 0xB24;
constexpr uintptr_t kOffExterMemAddr1Start = 0xB28;
constexpr uintptr_t kOffExterMemAddr1End = 0xB2C;
constexpr uintptr_t kOffCounterRst = 0xB50;

// --- HP_SYS_CLKRST (base kHpSysClkrstBase) ---
// Byte offsets confirmed directly from ESP-IDF's real
// soc/esp32p4/register/hw_ver1/soc/hp_sys_clkrst_reg.h:
// HP_SYS_CLKRST_SOC_CLK_CTRL1_REG, _PERI_CLK_CTRL26_REG, _HP_RST_EN2_REG.
constexpr uintptr_t kOffSocClkCtrl1 = 0x18;
constexpr uintptr_t kOffPeriClkCtrl26 = 0xAC;
constexpr uintptr_t kOffHpRstEn2 = 0xC8;
// Single-bit field positions within those three 32-bit registers,
// confirmed from the same header:
// reg_h264_sys_clk_en bitpos[31], reg_h264_clk_src_sel bitpos[18],
// reg_h264_clk_en bitpos[19], reg_rst_en_h264 bitpos[24].
constexpr uint32_t kBitH264SysClkEn = 1u << 31;
constexpr uint32_t kBitH264ClkSrcSel = 1u << 18;
constexpr uint32_t kBitH264ClkEn = 1u << 19;
constexpr uint32_t kBitRstEnH264 = 1u << 24;

inline void h264ClockAndResetInit() {
  volatile uint32_t* rstEn2 = reg<uint32_t>(kHpSysClkrstBase, kOffHpRstEn2);
  volatile uint32_t* clkCtrl1 = reg<uint32_t>(kHpSysClkrstBase, kOffSocClkCtrl1);
  volatile uint32_t* periClkCtrl26 =
      reg<uint32_t>(kHpSysClkrstBase, kOffPeriClkCtrl26);
  *rstEn2 |= kBitRstEnH264;
  *rstEn2 &= ~kBitRstEnH264;
  *clkCtrl1 |= kBitH264SysClkEn;
  *periClkCtrl26 |= (kBitH264ClkEn | kBitH264ClkSrcSel);
}

// --- 2D-DMA descriptor - the one real hardware struct in this file,
// kept as an actual struct rather than offset accessors since it's a
// small, flat, non-padded 16-byte layout with no room for a
// stride-miscount (every field is used, nothing skipped) - verified
// field-by-field against h264_dma_ll.h's own `h264_dma_desc_t`.
struct H264DmaDesc {
  volatile uint32_t vb : 14;
  volatile uint32_t hb : 14;
  volatile uint32_t errEof : 1;
  volatile uint32_t dma2dEn : 1;
  volatile uint32_t eof : 1;
  volatile uint32_t owner : 1;
  volatile uint32_t va : 14;
  volatile uint32_t ha : 15;
  volatile uint32_t mode : 1;
  volatile uint32_t reserved30 : 2;
  volatile uint8_t* buf;
  H264DmaDesc* nextDesc;
};
#if UINTPTR_MAX == 0xffffffffu
static_assert(sizeof(H264DmaDesc) == 16,
              "H264DmaDesc must match the real 16-byte hardware descriptor "
              "layout (only checked on a 32-bit target, matching real "
              "hardware - a 64-bit host syntax-check build has wider "
              "pointers and is expected to differ in size)");
#endif

// ===========================================================================
// H.264 core register primitives - ported from esp_h264's `h264_ll.h`.
// ===========================================================================

// Ported from `h264_ll_set_sys()`: force every internal RAM clock gate
// open, clear the run/reset/DMA-move control bits, disable and clear
// every interrupt latch - the baseline state after the clock/reset
// sequence and before any per-open configuration.
inline void h264SetSys() {
  *reg<uint32_t>(kH264Base, kOffConf) = 0xffffffffu;
  volatile uint32_t* sysCtrl = reg<uint32_t>(kH264Base, kOffSysCtrl);
  *sysCtrl &= ~(1u << 0);  // frame_start = 0
  *sysCtrl &= ~(1u << 1);  // dma_move_start = 0
  *sysCtrl |= (1u << 3);   // sys_rst_pulse = 1 (self-clearing WT bit)
  *reg<uint32_t>(kH264Base, kOffIntEna) = 0;
  *reg<uint32_t>(kH264Base, kOffIntClr) = 0xffffffffu;
}

// Ported from `h264_ll_set_gop()`. `gopModeEn` true selects GOP mode
// (this driver's only mode - matches Espressif's own single-frame
// encode API, which always passes true here).
inline void h264SetGop(uint8_t gop, bool gopModeEn) {
  volatile uint32_t* gopConf = reg<uint32_t>(kH264Base, kOffGopConf);
  *gopConf = (*gopConf & ~(0xffu << 1)) | ((uint32_t)gop << 1);
  volatile uint32_t* sysCtrl = reg<uint32_t>(kH264Base, kOffSysCtrl);
  if (gopModeEn) {
    *sysCtrl &= ~(1u << 2);  // frame_mode = 0 (GOP mode)
  } else {
    *sysCtrl |= (1u << 2);
  }
  *gopConf &= ~(1u << 0);  // dual_stream_mode = 0
}

/// Ported from `h264_ll_reset()`: pulses the whole-IP reset bit.
inline void h264Reset() {
  *reg<uint32_t>(kH264Base, kOffSysCtrl) |= (1u << 3);
}

/// Ported from `h264_ll_set_frame_start()`: the actual "go" bit for one
/// frame's encode - `sys_ctrl.frame_start`, a self-clearing WT bit.
inline void h264SetFrameStart() {
  *reg<uint32_t>(kH264Base, kOffSysCtrl) |= (1u << 0);
}

/// Ported from `h264_ll_dma_move_start()`: `sys_ctrl.dma_move_start`.
inline void h264DmaMoveStart() {
  *reg<uint32_t>(kH264Base, kOffSysCtrl) |= (1u << 1);
}

/// Ported from `h264_ll_set_mb()`. `dbTmpReadyTriggerMbNum` and
/// `recReadyTriggerMbLines` (hardcoded 4, "min is 4" per the real
/// register doc comment) match Espressif's own fixed choices exactly.
inline void h264SetMb(uint8_t mbWidth, uint8_t mbHeight) {
  volatile uint32_t* mbRes = reg<uint32_t>(kH264Base, kOffCtrl0SysMbRes);
  *mbRes = ((uint32_t)mbWidth << 7) | (uint32_t)mbHeight;
  volatile uint32_t* sysConf = reg<uint32_t>(kH264Base, kOffCtrl0SysConf);
  uint32_t dbTmpTrigger = (uint32_t)(mbWidth >> 1) & 0x7f;
  uint32_t recTrigger = 4u & 0x7f;
  *sysConf = (*sysConf & ~0x3fffu) | dbTmpTrigger | (recTrigger << 7);
}

/// Ported from `h264_ll_set_qp()`: `rc_conf0.qp` (6-bit field, 0-51 used).
inline void h264SetQp(uint8_t qp) {
  volatile uint32_t* rcConf0 = reg<uint32_t>(kH264Base, kOffCtrl0RcConf0);
  *rcConf0 = (*rcConf0 & ~0x3fu) | (qp & 0x3fu);
}

/// Ported from `h264_ll_set_db_bypass()`. `bypass` true disables
/// deblocking - this driver always calls this with `false` (deblocking
/// stays on), matching the hardware's own reset default, but the
/// function is written to take the flag explicitly rather than assuming
/// it's never called, in case that changes later.
inline void h264SetDbBypass(bool bypass) {
  volatile uint32_t* dbBypass = reg<uint32_t>(kH264Base, kOffCtrl0DbBypass);
  if (bypass) {
    *dbBypass |= 1u;
  } else {
    *dbBypass &= ~1u;
  }
}

/// Ported from `h264_ll_set_roi_cfg()` - a REAL, previously-missing piece
/// of `HwEncoderP4::open()`'s setup, found by directly comparing this
/// port's register writes against the vendored source of
/// `codec-h264-ESP32P4` (a sibling project wrapping Espressif's own,
/// confirmed-working hardware driver - see that project's README for
/// why it exists). Despite its call site in Espressif's own
/// `h264_hal_init()` being commented `/* Disable ROI */`, this function
/// unconditionally sets `roi_config.roi_en = 1` - the comment describes
/// the CALLER's intent (no actual ROI regions ever get configured), not
/// what the register write itself does. Every real encode this
/// project's driver ever attempted left `roi_en` at its power-on-reset
/// default (0, disabled) instead - the "roi" module is a real pipeline
/// stage in ENC_CORE's own architecture (Figure 37.4-1/37.5-1 in the
/// TRM: "roi: Receives QPs from the rate_ctrl module... adjusts the
/// QPs, and uses the adjusted QPs to process the current MB"), so
/// leaving it in a state Espressif's own driver never actually
/// exercises is a real, plausible explanation for a total pipeline
/// stall with zero macroblocks ever processed - `roi_mode=0` (fixed QP)
/// with `no_roi_qp=0` matches Espressif's own `h264_hal_init()` call
/// exactly (`h264_ll_set_roi_cfg(&hal->dev->ctrl[i], cfg->cfg_ch[i]
/// .roi_mode, cfg->cfg_ch[i].roi_none_roi_qp)` with both fields 0 from
/// a zero-initialized config struct).
inline void h264SetRoiFixedQpPassthrough() {
  volatile uint32_t* noRoiQp =
      reg<uint32_t>(kH264Base, kOffCtrl0NoRoiRegionQpOffset);
  *noRoiQp = 0;
  volatile uint32_t* roiConfig = reg<uint32_t>(kH264Base, kOffCtrl0RoiConfig);
  *roiConfig = 1u;  // roi_en = 1, roi_mode = 0 (fixed QP)
}

/// Same as h264SetRoiFixedQpPassthrough(), but for ctrl[1] (video stream
/// B) - a REAL, confirmed-via-direct-register-diff gap: Espressif's own
/// `h264_hal_init()` loops over BOTH channels
/// (`for (i = 0; i < H264_SUP_MAX_CHANNEL; i++) h264_ll_set_roi_cfg(&hal
/// ->dev->ctrl[i], ...)`), even though this driver (like Espressif's own
/// single-stream mode) never uses stream B at all. A full register-bank
/// dump taken right before `frame_start`, diffed directly against
/// `codec-h264-ESP32P4`'s own matching capture on the same board (see
/// `examples/HwEncoderP4RegisterDump` / that project's
/// `examples/RegisterDump`), showed ctrl[1]'s `roi_config` register
/// reading 1 or the real driver and 0 for this one - every other
/// register in ctrl[1] matched (both end up 0, since
/// `esp_h264_enc_hw_new()` only ever populates `cfg_ch[0]`, leaving
/// `cfg_ch[1]` zero-initialized - the same effective value this driver
/// already leaves ctrl[1]'s other registers at by never touching them).
inline void h264SetRoiFixedQpPassthroughCtrl1() {
  volatile uint32_t* noRoiQp =
      reg<uint32_t>(kH264Base, kOffCtrl1NoRoiRegionQpOffset);
  *noRoiQp = 0;
  volatile uint32_t* roiConfig = reg<uint32_t>(kH264Base, kOffCtrl1RoiConfig);
  *roiConfig = 1u;
}

/// Ported from `h264_ll_set_decimate_score()`. A REAL, previously-missing
/// piece of `HwEncoderP4::open()`'s setup, found via
/// `encodeDiagnostic()`'s FSM trace (see this project's own README.md/
/// git history for that investigation): Espressif's own driver always
/// explicitly writes this before ever starting an encode (defaults
/// `H264_SCORE_LUMA=6`/`H264_SCORE_CHROMA=7` - not 0), but this port
/// never had until now, leaving it at the hardware's own power-on-reset
/// value instead. `l_deci_score`/`c_deci_score` gate the macroblock
/// mode-decision "decimate" logic directly inside the intra/inter
/// top-control blocks (`top_ctrl_intra`/`top_ctrl_inter` in
/// `debug_info0`) - exactly the FSM sub-block observed stalling.
inline void h264SetDecimateScore(uint16_t scoreLuma, uint16_t scoreChroma) {
  volatile uint32_t* deciScore = reg<uint32_t>(kH264Base, kOffCtrl0DeciScore);
  *deciScore = ((uint32_t)scoreLuma & 0x3ffu) << 10 |
               ((uint32_t)scoreChroma & 0x3ffu);
}

/// Ported from `h264_ll_set_decimate_offset()` - see
/// `h264SetDecimateScore()`'s own comment for why this (previously
/// missing) group of registers matters. Espressif's own driver always
/// calls this too, with every offset 0 in the common case (this port's
/// only case) - writing 0 here is a real, deliberate "make sure it's
/// actually 0, not just hoping the power-on-reset value already is"
/// write, not a no-op.
inline void h264SetDecimateOffset(uint8_t intraLumaOffset,
                                   uint8_t intraChromaOffset,
                                   uint8_t interLumaOffset,
                                   uint8_t interChromaOffset) {
  *reg<uint32_t>(kH264Base, kOffCtrl0DeciScoreOffset) =
      ((uint32_t)intraLumaOffset & 0x3fu) |
      (((uint32_t)intraChromaOffset & 0x3fu) << 6) |
      (((uint32_t)interLumaOffset & 0x3fu) << 12) |
      (((uint32_t)interChromaOffset & 0x3fu) << 18);
}

/// Ported from `h264_ll_set_ip_cost_thres()`: `sys_conf.intra_cost_cmp_offset`,
/// bits [29:14] of the same `sys_conf` register `h264SetMb()` already
/// partially writes (bits [13:0]) - a real, previously-missing piece,
/// same rationale as `h264SetDecimateScore()`'s own comment. Must be
/// called AFTER `h264SetMb()` (this does a read-modify-write of the
/// bits `h264SetMb()` doesn't touch, so call order between the two
/// doesn't itself matter for correctness, but this is written assuming
/// `h264SetMb()` already ran at least once so the register isn't in an
/// undefined intermediate state).
inline void h264SetIpCostThres(uint16_t thres) {
  volatile uint32_t* sysConf = reg<uint32_t>(kH264Base, kOffCtrl0SysConf);
  *sysConf =
      (*sysConf & ~(0xffffu << 14)) | (((uint32_t)thres & 0xffffu) << 14);
}

/// Ported from `h264_ll_set_slice_header()`. `header[0]`/`header[1]` are
/// the reversed "extra complete bytes" past the 8-byte alignment
/// boundary (see `spliceSliceHeaderAt8ByteBoundary()`), `header2Byte` is
/// the trailing partial byte, `sliceBitLen` the total remaining bit
/// count (its low 3 bits = bits valid in the partial byte, the rest = 8
/// * how many of header[0]/header[1]'s bytes are valid).
inline void h264SetSliceHeaderRegs(uint32_t header0, uint32_t header1,
                                    uint8_t header2Byte,
                                    uint16_t sliceBitLen) {
  *reg<uint32_t>(kH264Base, kOffSliceHeader0) = header0;
  *reg<uint32_t>(kH264Base, kOffSliceHeader1) = header1;
  volatile uint32_t* remain =
      reg<uint32_t>(kH264Base, kOffSliceHeaderRemain);
  *remain = ((uint32_t)header2Byte << 3) | (sliceBitLen & 0x7u);
  *reg<uint32_t>(kH264Base, kOffSliceHeaderByteLength) =
      (sliceBitLen >> 3) & 0xfu;
}

// Interrupt bit positions, matching `H264_INTR_*` (hw_ver1: only these
// four exist - `H264_INTR_MASK == 0xf` on this silicon, confirmed from
// h264_ll.h's own `#if CHIP_SUPPORT_MIN_REV < 300` branch).
constexpr uint32_t kIntrDbTmpReady = 1u << 0;
constexpr uint32_t kIntrRecReady = 1u << 1;
constexpr uint32_t kIntrFrameDone = 1u << 2;
constexpr uint32_t kIntr2mbLineDone = 1u << 3;
constexpr uint32_t kIntrMask = 0xfu;

/// Ported from `h264_ll_set_intr()` (a plain assignment, not an OR - see
/// that function's own doc comment - matches upstream's own one-time,
/// non-additive use at open()).
inline void h264EnableInterrupts(uint32_t mask) {
  *reg<uint32_t>(kH264Base, kOffIntEna) = mask;
}

/// Ported from `h264_ll_get_intr_st()`.
inline uint32_t h264GetInterruptStatus() {
  return *reg<uint32_t>(kH264Base, kOffIntSt) & kIntrMask;
}

/// Ported from `h264_ll_clr_intr_st()`.
inline void h264ClearInterrupts(uint32_t mask) {
  *reg<uint32_t>(kH264Base, kOffIntClr) |= mask;
}

/// Ported from `h264_ll_get_coded_len()`: `frame_code_length` (24-bit).
inline uint32_t h264GetCodedLength() {
  return *reg<uint32_t>(kH264Base, kOffFrameCodeLength) & 0xffffffu;
}

/// Ported from `h264_ll_get_bs_bit_overflow()` (hw_ver1 only):
/// `debug_info1.bs_buffer_debug_state`, bit 18 - a debug-designated
/// register bit that is nonetheless Espressif's only hw_ver1 overflow
/// signal (see file header's own note on this being unreliable near the
/// boundary - the software length check in `HwEncoderP4::encode()` is
/// the real safety net, this is a best-effort supplement).
inline bool h264GetBsBitOverflowDebugBit() {
  return (*reg<uint32_t>(kH264Base, kOffDebugInfo1) & (1u << 18)) != 0;
}

/// Not part of the ported driver logic - a read-only diagnostic hook
/// (see `HwEncoderP4::encodeDiagnostic()`) for inspecting the hardware's
/// own internal per-block-FSM state and picture-level status registers
/// directly, independent of whether any interrupt ever fires. Field
/// meanings are documented in `h264_struct.h`'s
/// `h264_debug_info0_reg_t`/`h264_debug_info1_reg_t`/
/// `h264_sys_status_reg_t` (each named FSM sub-state field is 3-4 bits,
/// 0 conventionally means "idle/reset" for these - not independently
/// re-verified beyond that, since this project's own driver never needed
/// to interpret them for anything beyond the one bit
/// `h264GetBsBitOverflowDebugBit()` already uses).
struct H264DebugSnapshot {
  uint32_t debugInfo0;
  uint32_t debugInfo1;
  uint32_t sysStatus;
};
inline H264DebugSnapshot h264GetDebugSnapshot() {
  H264DebugSnapshot s;
  s.debugInfo0 = *reg<uint32_t>(kH264Base, kOffDebugInfo0);
  s.debugInfo1 = *reg<uint32_t>(kH264Base, kOffDebugInfo1);
  s.sysStatus = *reg<uint32_t>(kH264Base, kOffSysStatus);
  return s;
}

// ===========================================================================
// 2D-DMA primitives - ported from esp_h264's `h264_dma_ll.h`.
// ===========================================================================

constexpr int kDmaResetAvailTimeoutIters = 100000;

inline volatile uint32_t* dmaChnConf0(uintptr_t chnBase) {
  return reg<uint32_t>(kH264DmaBase, chnBase + kChnConf0);
}
inline volatile uint32_t* dmaChnLinkConf(uintptr_t chnBase) {
  return reg<uint32_t>(kH264DmaBase, chnBase + kChnLinkConf);
}
inline volatile uint32_t* dmaChnLinkAddr(uintptr_t chnBase) {
  return reg<uint32_t>(kH264DmaBase, chnBase + kChnLinkAddr);
}
inline volatile uint32_t* dmaChnState(uintptr_t chnBase) {
  return reg<uint32_t>(kH264DmaBase, chnBase + kChnState);
}

/// Ported from `h264_dma_ll_set_out_conf0()`/`set_in_conf0()`: a whole-word
/// configure write (Espressif's own `conf0` is a union with a plain
/// `uint32_t val` alias - this is that same whole-word write, done via
/// the same offset-based accessor as everything else here).
inline void dmaSetChnConf0(uintptr_t chnBase, uint32_t cfg0) {
  *dmaChnConf0(chnBase) = cfg0;
}
inline void dmaSetCh5Conf0(uint32_t cfg0) {
  *reg<uint32_t>(kH264DmaBase, kCh5Conf0) = cfg0;
}

/// Ported from `h264_dma_ll_set_all_burst_size()`: sets
/// `*_mem_burst_length` (bits [8:6] of conf0, same position for every
/// channel including ch5) on every channel to `burstSize` (4 == 128
/// bytes/burst, matching Espressif's own `H264_DMA_BURST_SIZE`).
inline void dmaSetAllBurstSize(uint8_t burstSize) {
  uint32_t field = ((uint32_t)burstSize & 0x7u) << 6;
  for (int i = 0; i < 5; i++) {
    uintptr_t outBase = kOffOutCh0 + (uintptr_t)i * kDmaChnStride;
    uintptr_t inBase = kOffInCh0 + (uintptr_t)i * kDmaChnStride;
    *dmaChnConf0(outBase) = (*dmaChnConf0(outBase) & ~(0x7u << 6)) | field;
    *dmaChnConf0(inBase) = (*dmaChnConf0(inBase) & ~(0x7u << 6)) | field;
  }
  volatile uint32_t* ch5Conf0 = reg<uint32_t>(kH264DmaBase, kCh5Conf0);
  *ch5Conf0 = (*ch5Conf0 & ~(0x7u << 6)) | field;
}

/// Ported from `h264_dma_ll_clear_all_intr()` - a REAL, previously-
/// missing piece found via a hybrid bisection test (substituting
/// Espressif's real, working `h264_hal_*`/`h264_dma_hal_*` functions for
/// this port's own DMA-start/frame-start sequence, on top of this port's
/// OWN setup/buffers/descriptors, immediately produced real encoding
/// progress - `frame_enc_bits`/`frame_mad_sum`/`frame_code_length` all
/// went from a permanent 0 to real, nonzero values for the first time in
/// this project's entire hardware-encoder investigation - narrowing the
/// bug specifically to this port's DMA-start-sequence code). This clears
/// the per-DMA-channel interrupt-status latch (`int_clr`, a separate
/// register from the H.264 core's own top-level interrupt register this
/// port already clears via `h264ClearInterrupts()`) for every TX/RX
/// channel including ch5 - never previously cleared anywhere in this
/// port, on any channel, for the whole lifetime of the driver. A stale
/// latched interrupt condition left over from a previous frame/reset is
/// a real, plausible way for a DMA channel's own internal state machine
/// to end up gated even though its conf0/link/descriptor registers all
/// look perfectly correct - exactly matching this project's own
/// long-standing symptom (real data movement, confirmed via a live
/// register-bank diff, but completion never signaled).
inline void dmaClearAllInterrupts() {
  for (int i = 0; i < 5; i++) {
    *reg<uint32_t>(kH264DmaBase, kOffOutCh0 + (uintptr_t)i * kDmaChnStride + kChnIntClr) = 0xffffffffu;
    *reg<uint32_t>(kH264DmaBase, kOffInCh0 + (uintptr_t)i * kDmaChnStride + kChnIntClr) = 0xffffffffu;
  }
  *reg<uint32_t>(kH264DmaBase, kCh5IntClr) = 0xffffffffu;
}

/// Ported from `h264_dma_ll_set_exter_mem_addr()`/`set_inter_mem_addr()`:
/// unrestricted (`H264_DMA_END_ADDR == ~0`) access range on both
/// internal and external memory - matches Espressif's own default (no
/// address-range restriction imposed by this driver).
inline void dmaSetUnrestrictedMemRange() {
  *reg<uint32_t>(kH264DmaBase, kOffInterMemAddr0Start) = 0;
  *reg<uint32_t>(kH264DmaBase, kOffInterMemAddr0End) = 0xffffffffu;
  *reg<uint32_t>(kH264DmaBase, kOffInterMemAddr1Start) = 0;
  *reg<uint32_t>(kH264DmaBase, kOffInterMemAddr1End) = 0xffffffffu;
  *reg<uint32_t>(kH264DmaBase, kOffExterMemAddr0Start) = 0;
  *reg<uint32_t>(kH264DmaBase, kOffExterMemAddr0End) = 0xffffffffu;
  *reg<uint32_t>(kH264DmaBase, kOffExterMemAddr1Start) = 0;
  *reg<uint32_t>(kH264DmaBase, kOffExterMemAddr1End) = 0xffffffffu;
}

/// Ported from `h264_dma_ll_reset_counter{0,1,2,5}()`: pulse (1 then 0)
/// the named channel's counter-reset bit. Bit positions from
/// `h264_dma_counter_rst_reg_t`: rx_ch0=bit0, rx_ch1=bit1, rx_ch2=bit2,
/// rx_ch5=bit3.
inline void dmaResetCounter(int bit) {
  volatile uint32_t* counterRst =
      reg<uint32_t>(kH264DmaBase, kOffCounterRst);
  *counterRst |= (1u << bit);
  *counterRst &= ~(1u << bit);
}
inline void dmaResetCounterDb() {  // matches `h264_dma_hal_reset_counter_db`
  dmaResetCounter(0);
  dmaResetCounter(1);
}
inline void dmaResetCounterDbtmp() {
  dmaResetCounter(2);
}
inline void dmaResetCounterRef() {
  dmaResetCounter(3);  // rx_ch5_inter_counter_rst, bit 3
}

/// Bounded stand-in for `h264_dma_ll_reset_out()`/`reset_in()`/
/// `reset_in5()`'s infinite `while(!reset_avail);` spin - see file
/// header's "deliberate deviation" note. `cmdDisableBit`/`rstBit`
/// locate the two relevant conf0 bits (same 24/25 bit positions for
/// every channel including ch5, confirmed from the real register
/// layouts), `resetAvailBit` locates the state register's ready bit
/// (bit 24 for out channels, bit 23 for in channels 0-4, bit 3 for
/// ch5 - genuinely different positions, not a typo, confirmed from
/// each channel's own distinct state-register struct).
inline bool dmaResetChannelGeneric(uintptr_t conf0Addr, uintptr_t stateAddr,
                                    int resetAvailBit) {
  volatile uint32_t* conf0 = reinterpret_cast<volatile uint32_t*>(conf0Addr);
  volatile uint32_t* state = reinterpret_cast<volatile uint32_t*>(stateAddr);
  *conf0 |= (1u << 25);  // cmd_disable = 1
  int iters = 0;
  while (((*state >> resetAvailBit) & 1u) == 0) {
    if (++iters > kDmaResetAvailTimeoutIters) return false;
  }
  *conf0 |= (1u << 24);   // rst = 1
  *conf0 &= ~(1u << 24);  // rst = 0
  *conf0 &= ~(1u << 25);  // cmd_disable = 0
  return true;
}
inline bool dmaResetOutChannel(uintptr_t chnBase) {
  return dmaResetChannelGeneric(
      (uintptr_t)dmaChnConf0(chnBase), (uintptr_t)dmaChnState(chnBase), 24);
}
inline bool dmaResetInChannel(uintptr_t chnBase) {
  return dmaResetChannelGeneric(
      (uintptr_t)dmaChnConf0(chnBase), (uintptr_t)dmaChnState(chnBase), 23);
}
inline bool dmaResetIn5Channel() {
  return dmaResetChannelGeneric((uintptr_t)reg<uint32_t>(kH264DmaBase, kCh5Conf0),
                                 (uintptr_t)reg<uint32_t>(kH264DmaBase, kCh5State),
                                 3);
}

/// Ported from `h264_dma_ll_set_out_link_stop/start()` and the
/// `h264_dma_hal_start_*_dma()` stop->reset->start dance those compose
/// into. Returns false if the reset step timed out (see
/// `dmaResetChannelGeneric()`).
inline bool dmaStartOutChannel(uintptr_t chnBase) {
  *dmaChnLinkConf(chnBase) |= (1u << 20);  // outlink_stop = 1
  if (!dmaResetOutChannel(chnBase)) return false;
  *dmaChnLinkConf(chnBase) |= (1u << 21);  // outlink_start = 1
  return true;
}
inline bool dmaStartInChannel(uintptr_t chnBase) {
  *dmaChnLinkConf(chnBase) |= (1u << 21);  // inlink_stop = 1
  if (!dmaResetInChannel(chnBase)) return false;
  *dmaChnLinkConf(chnBase) |= (1u << 22);  // inlink_start = 1
  return true;
}

inline void dmaSetOutLinkAddr(uintptr_t chnBase, uint32_t addr) {
  *dmaChnLinkAddr(chnBase) = addr;
}
inline void dmaSetInLinkAddr(uintptr_t chnBase, uint32_t addr) {
  *dmaChnLinkAddr(chnBase) = addr;
}

constexpr uintptr_t kOutCh(int i) { return kOffOutCh0 + (uintptr_t)i * kDmaChnStride; }
constexpr uintptr_t kInCh(int i) { return kOffInCh0 + (uintptr_t)i * kDmaChnStride; }

/// Ported from `h264_dma_ll_set_in5_block()`: tells the reference-frame
/// RX channel (ch5) the destination address and per-row-block byte
/// lengths it needs to correctly de-interleave the reference picture
/// data the H.264 core streams out mid-frame. `H264_DMA_DB_12_LINES_ROW_LENGTH`
/// (256) / `_4_LINES_ROW_LENGTH` (128) are fixed geometry constants
/// (12 or 4 luma macroblock-rows' worth of Y plus the matching chroma,
/// per macroblock - independent of picture size), ported from
/// `h264_dma_ll.h`'s own `#define`s.
constexpr uint32_t kDbtmp12LinesRowLength = 16 * 12 + 8 * 4 * 2;  // 256
constexpr uint32_t kDbtmp4LinesRowLength = 16 * 4 + 8 * 4 * 2;    // 128
inline void dmaSetIn5Block(uint32_t bufAddr, int mbWidth) {
  *reg<uint32_t>(kH264DmaBase, kCh5Conf1) = bufAddr;
  *reg<uint32_t>(kH264DmaBase, kCh5Conf2) =
      (kDbtmp12LinesRowLength * (uint32_t)mbWidth & 0xffffu) |
      ((kDbtmp4LinesRowLength * (uint32_t)mbWidth & 0xffffu) << 16);
  *reg<uint32_t>(kH264DmaBase, kCh5Conf3) =
      (kDbtmp12LinesRowLength & 0x3fffu) |
      ((kDbtmp4LinesRowLength & 0x3fffu) << 14);
}

/// Writes back one descriptor's cache lines to physical memory - the DMA
/// engine reads physical RAM, not CPU cache, so every descriptor field
/// write must be followed by this before the DMA engine is told to use
/// it. Ported from `esp_h264_enc_hw_param.c`'s `cfg_dsc()` (which does
/// this as the last step of every descriptor field assignment).
inline void dmaWritebackDesc(H264DmaDesc* dsc) {
  esp_cache_msync(reinterpret_cast<void*>(dsc), sizeof(H264DmaDesc),
                   ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}
inline void cacheWriteback(void* addr, size_t len) {
  esp_cache_msync(addr, len,
                   ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}
inline void cacheInvalidate(void* addr, size_t len) {
  esp_cache_msync(addr, len, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
}

/// Fills one `H264DmaDesc` and writes it back - ported from
/// `esp_h264_enc_hw_param.c`'s `cfg_dsc()`.
inline void configureDesc(H264DmaDesc* dsc, bool en2d, uint8_t mode,
                           uint16_t vb, uint16_t hb, uint8_t eof,
                           uint8_t owner, uint16_t va, uint16_t ha,
                           uint8_t* buf, H264DmaDesc* nextDsc) {
  dsc->dma2dEn = en2d ? 1 : 0;
  dsc->mode = mode;
  dsc->vb = vb;
  dsc->hb = hb;
  dsc->eof = eof;
  dsc->owner = owner;
  dsc->va = va;
  dsc->ha = ha;
  dsc->buf = buf;
  dsc->nextDesc = nextDsc;
  dmaWritebackDesc(dsc);
}

// ===========================================================================
// Pixel format conversion: TinyH264's own planar YUV 4:2:0 (separate Y/U/V
// planes, any stride) -> the packed pseudo-planar format
// (`ESP_H264_RAW_FMT_O_UYY_E_VYY` in Espressif's own naming) that this
// hardware revision is documented to require - confirmed from
// `esp_h264_types.h`'s own capability table: hw_ver1 silicon
// (`CONFIG_ESP_REV_MIN_FULL < 300`) accepts ONLY this one input pixel
// format, not standard planar YUV420 - a real, load-bearing finding from
// reading that header, not an assumption. Row layout (from that same
// header's own doc comment): row 2k is "u y y u y y ..." (a chroma-row
// U sample followed by 2 luma samples, repeating), row 2k+1 is the same
// with V instead of U - i.e. each pair of output rows packs one 4:2:0
// chroma row (shared, subsampled 2x vertically, exactly like standard
// 4:2:0) together with the two luma rows it corresponds to. `width` must
// be even (already guaranteed elsewhere - TinyH264's own width-multiple-
// of-16 requirement).
// ===========================================================================
inline void convertYuv420ToPackedOuyyEvyy(const uint8_t* srcY, int strideY,
                                           const uint8_t* srcU,
                                           const uint8_t* srcV, int strideC,
                                           int width, int height,
                                           uint8_t* dst) {
  int chromaW = width / 2;
  for (int y = 0; y < height; y++) {
    const uint8_t* yRow = srcY + (size_t)y * strideY;
    const uint8_t* cRow = (y & 1) ? (srcV + (size_t)(y / 2) * strideC)
                                   : (srcU + (size_t)(y / 2) * strideC);
    uint8_t* outRow = dst + (size_t)y * (size_t)chromaW * 3;
    for (int x = 0; x < chromaW; x++) {
      outRow[x * 3 + 0] = cRow[x];
      outRow[x * 3 + 1] = yRow[x * 2 + 0];
      outRow[x * 3 + 2] = yRow[x * 2 + 1];
    }
  }
}

}  // namespace hw_p4_detail

// ===========================================================================
// Public driver class.
// ===========================================================================

/**
 * Drives the ESP32-P4 hardware H.264 encoder for one fixed picture size
 * at a time, fixed QP, no rate control - see this file's header comment
 * for the full scope/validation-status disclaimer. Mirrors
 * `h264_hw_enc_gop_mode_process()`/`h264_start_gop_mode_enc()`/
 * `h264_gop_isr()` from Espressif's real `esp_h264_enc_single_hw.c`
 * (read for reference, ported independently - see file header), with
 * ROI/motion-vector-telemetry/rate-control left out entirely (this
 * project's own scope, not needed for a valid stream) and the unbounded
 * DMA-reset spin-wait replaced with a bounded one.
 */
// Diagnostic-only hook (weak, no-op unless a sketch defines it) - fires
// right before frame_start is written, at the exact "armed and
// configured, not yet started" point codec-h264-ESP32P4's own matching
// hook (see that project's examples/RegisterDump) fires at too - see
// this project's own examples/HwEncoderP4RegisterDump. extern "C" so
// both sides use plain C linkage regardless of being compiled as C++.
extern "C" void h264_debug_before_frame_start_hook(void) __attribute__((weak));

class HwEncoderP4 {
 public:
  ~HwEncoderP4() { close(); }

  bool isOpen() const { return opened_; }
  int width() const { return width_; }
  int height() const { return height_; }

  /**
   * Diagnostic-only (see `encodeDiagnostic()`'s own comment for the
   * general rationale): the raw addresses of every buffer this
   * instance's DMA descriptors point at. Real ESP32-P4 boards can have
   * PSRAM mapped into a completely different address range than
   * internal SRAM (typically `0x48xxxxxx`+ for PSRAM vs. internal RAM
   * elsewhere), and this project's own memory notes flag PSRAM-vs-DMA
   * reachability as a real, board-dependent question this project has
   * hit before on other peripherals - this exists purely so a caller
   * can check "did any of these end up somewhere the H.264 DMA engine
   * might not actually be able to reach" without needing to add ad-hoc
   * prints inside the class itself.
   */
  struct BufferAddresses {
    uintptr_t refBuf, dbBuf, dbTmpBuf, packedYuvScratch, dscYuv, dscBs;
  };
  BufferAddresses debugBufferAddresses() const {
    BufferAddresses a;
    a.refBuf = (uintptr_t)refBuf_;
    a.dbBuf = (uintptr_t)dbBuf_;
    a.dbTmpBuf = (uintptr_t)dbTmpBuf_;
    a.packedYuvScratch = (uintptr_t)packedYuvScratch_;
    a.dscYuv = (uintptr_t)dscYuv_;
    a.dscBs = (uintptr_t)dscBs_;
    return a;
  }

  /**
   * Opens the hardware encoder for a fixed `width`x`height` (each must
   * be a multiple of 16, matching the software encoder's own
   * macroblock-alignment requirement), fixed `qp` (0-51, used for every
   * frame - no rate control), and periodic-keyframe `gop` (0 means "only
   * the first frame is a keyframe", matching the software encoder's own
   * `setKeyframeInterval(0)` convention). Returns false on any failure
   * (bad parameters, allocation failure, interrupt-registration
   * failure) - safe to call again, since it always starts with close().
   */
  bool open(int width, int height, int qp, int gop) {
    close();
    if (width <= 0 || height <= 0 || (width % 16) != 0 || (height % 16) != 0)
      return false;
    if (qp < 0) qp = 0;
    if (qp > 51) qp = 51;

    width_ = width;
    height_ = height;
    mbWidth_ = (width + 15) >> 4;
    mbHeight_ = (height + 15) >> 4;
    qp_ = qp;
    gop_ = gop;
    frameNum_ = 0;

    hw_p4_detail::h264ClockAndResetInit();
    hw_p4_detail::h264SetSys();
    hw_p4_detail::h264SetGop((uint8_t)(gop_ > 0 ? gop_ : 0), true);
    hw_p4_detail::h264SetMb((uint8_t)mbWidth_, (uint8_t)mbHeight_);
    hw_p4_detail::h264SetQp((uint8_t)qp_);
    hw_p4_detail::h264SetDbBypass(false);
    hw_p4_detail::h264SetRoiFixedQpPassthrough();
    hw_p4_detail::h264SetRoiFixedQpPassthroughCtrl1();
    // Matches Espressif's own driver's real defaults - see
    // h264SetDecimateScore()'s own comment for why this was a real,
    // previously-missing gap in this port (found via encodeDiagnostic()
    // observing top_ctrl_intra stall shortly after frame_start).
    hw_p4_detail::h264SetDecimateScore(/*scoreLuma=*/6, /*scoreChroma=*/7);
    hw_p4_detail::h264SetDecimateOffset(0, 0, 0, 0);
    hw_p4_detail::h264SetIpCostThres(0);

    hw_p4_detail::dmaSetUnrestrictedMemRange();
    for (int i = 0; i < 5; i++) {
      uint32_t conf0 = (1u << 1);  // EOF_EN, every out channel
      if (i == 0) conf0 |= (1u << 16);  // + REORDER_EN, YUV source only
      hw_p4_detail::dmaSetChnConf0(hw_p4_detail::kOutCh(i), conf0);
      hw_p4_detail::dmaSetChnConf0(hw_p4_detail::kInCh(i), 0);
    }
    hw_p4_detail::dmaSetCh5Conf0(0);
    hw_p4_detail::dmaSetAllBurstSize(4);  // 128-byte bursts

    if (!allocateBuffersAndDescriptors()) {
      close();
      return false;
    }
    configureStaticDescriptors();

    esp_err_t err = esp_intr_alloc(ETS_H264_REG_INTR_SOURCE, 0, &HwEncoderP4::isrThunk,
                                    this, &intrHandle_);
    if (err != ESP_OK) {
      close();
      return false;
    }
    frameDoneSem_ = xSemaphoreCreateBinary();
    if (!frameDoneSem_) {
      close();
      return false;
    }
    hw_p4_detail::h264EnableInterrupts(
        hw_p4_detail::kIntrDbTmpReady | hw_p4_detail::kIntrRecReady |
        hw_p4_detail::kIntr2mbLineDone | hw_p4_detail::kIntrFrameDone);

    opened_ = true;
    return true;
  }

  /// Releases every resource open() acquired - safe to call even if
  /// open() was never called or partially failed.
  void close() {
    if (intrHandle_) {
      esp_intr_free(intrHandle_);
      intrHandle_ = nullptr;
    }
    if (frameDoneSem_) {
      vSemaphoreDelete(frameDoneSem_);
      frameDoneSem_ = nullptr;
    }
    free(dscYuvRaw_); dscYuvRaw_ = nullptr; dscYuv_ = nullptr;
    free(dscBsRaw_); dscBsRaw_ = nullptr; dscBs_ = nullptr;
    free(dscRefRaw_); dscRefRaw_ = nullptr; dscRef_ = nullptr;
    for (int i = 0; i < 4; i++) {
      free(dscDbRaw_[i]); dscDbRaw_[i] = nullptr; dscDb_[i] = nullptr;
    }
    for (int i = 0; i < 2; i++) {
      free(dscDbtmpRaw_[i]); dscDbtmpRaw_[i] = nullptr; dscDbtmp_[i] = nullptr;
    }
    free(dbTmpBuf_); dbTmpBuf_ = nullptr;
    free(refBuf_); refBuf_ = nullptr;
    free(dbBuf_); dbBuf_ = nullptr;
    free(packedYuvScratch_); packedYuvScratch_ = nullptr;
    packedYuvScratchSize_ = 0;
    opened_ = false;
    width_ = height_ = 0;
  }

  /**
   * Encodes one YUV 4:2:0 planar picture (separate Y/U/V plane
   * pointers/strides - matches `TinyH264Encoder`'s own `encodeFrame()`
   * source shape) into a complete Annex-B bitstream at `dst`. Internally
   * converts to the packed pixel format this hardware revision actually
   * requires (see `convertYuv420ToPackedOuyyEvyy()`) before handing the
   * picture to the DMA engine. Returns the number of bytes written, or 0
   * on any failure (not open, wrong size, `dst` too small for what the
   * hardware produced, a DMA-reset timeout, or the hardware reporting a
   * timeout/hang on this frame - the encoder's internal state is reset
   * back to a fresh IDR-next-frame either way, so a caller can just keep
   * calling `encode()` again after a 0 return).
   */
  size_t encode(const uint8_t* srcY, int strideY, const uint8_t* srcU,
                const uint8_t* srcV, int strideC, uint8_t* dst,
                size_t dstCapacity) {
    bool isIdr = false;
    size_t nalStartOffset = 0, outFrameLen = 0;
    if (!prepareAndStartFrame(srcY, strideY, srcU, srcV, strideC, dst,
                               dstCapacity, isIdr, nalStartOffset,
                               outFrameLen)) {
      return 0;
    }

    if (xSemaphoreTake(frameDoneSem_, pdMS_TO_TICKS(1000)) != pdTRUE) {
      // Timeout - matches Espressif's own recovery: force a core/DMA
      // reset so the next open()/encode() (or the next frame, since we
      // don't close() here) starts from a clean state.
      hw_p4_detail::h264Reset();
      hw_p4_detail::dmaResetCounterDbtmp();
      hw_p4_detail::dmaResetCounterDb();
      hw_p4_detail::dmaResetCounterRef();
      frameNum_ = 0;
      return 0;
    }

    return finishFrame(dst, dstCapacity, nalStartOffset, outFrameLen);
  }

  /**
   * Diagnostic variant of encode() - NOT part of this class's supported
   * API, only for narrowing down exactly where the hardware pipeline
   * stalls (see this project's own current, open "encode() always times
   * out waiting for FRAME_DONE" issue). Identical to encode() up through
   * starting the frame, but instead of blocking on the interrupt-driven
   * semaphore with one 1-second timeout, polls the raw interrupt status
   * register directly (bypassing the ISR/semaphore path entirely,
   * running the exact same status-handling logic `isr()` would via the
   * shared `pumpInterruptStatus()` helper - see that method) for up to
   * `timeoutUs`, calling `onEvent(elapsedUs, rawStatus, debugSnapshot)`
   * (if non-null) both every time the observed raw interrupt status
   * value changes AND at least once every `sampleIntervalUs` regardless
   * (so the hardware's own internal FSM debug registers - see
   * `h264GetDebugSnapshot()` - are visible evolving over time even if
   * the interrupt status itself never changes at all). This can tell
   * "the hardware pipeline itself never does anything after
   * frame_start" (interrupt status AND every FSM debug field stay at
   * their reset value) apart from "the hardware pipeline runs (FSM
   * fields move) but no interrupt ever reaches the CPU" (a real,
   * different bug - `esp_intr_alloc()`/interrupt routing, not the
   * DMA/register sequence itself) apart from "some pipeline stages
   * complete (interrupt status changes through some of DB_TMP_READY/
   * REC_READY/2MB_LINE_DONE) but it stalls before FRAME_DONE" - the
   * ISR-driven encode() can't distinguish any of these on its own, since
   * it never runs at all if the interrupt never reaches the CPU, and
   * looks identical from outside (always exactly the ~1s timeout)
   * either way.
   */
  size_t encodeDiagnostic(
      const uint8_t* srcY, int strideY, const uint8_t* srcU,
      const uint8_t* srcV, int strideC, uint8_t* dst, size_t dstCapacity,
      void (*onEvent)(uint32_t elapsedUs, uint32_t rawStatus,
                       hw_p4_detail::H264DebugSnapshot snapshot) = nullptr,
      uint32_t timeoutUs = 2000000, uint32_t sampleIntervalUs = 50000) {
    bool isIdr = false;
    size_t nalStartOffset = 0, outFrameLen = 0;
    if (!prepareAndStartFrame(srcY, strideY, srcU, srcV, strideC, dst,
                               dstCapacity, isIdr, nalStartOffset,
                               outFrameLen)) {
      return 0;
    }

    uint32_t t0 = micros();
    uint32_t lastStatus = 0xffffffffu;
    uint32_t lastSampleUs = 0;
    bool done = false;
    while ((uint32_t)(micros() - t0) < timeoutUs) {
      uint32_t status = hw_p4_detail::h264GetInterruptStatus();
      uint32_t elapsed = (uint32_t)(micros() - t0);
      if (status != lastStatus || (elapsed - lastSampleUs) >= sampleIntervalUs) {
        if (onEvent) {
          onEvent(elapsed, status, hw_p4_detail::h264GetDebugSnapshot());
        }
        lastStatus = status;
        lastSampleUs = elapsed;
      }
      if (status) {
        if (pumpInterruptStatus(status, isIdr, /*fromIsr=*/false, nullptr)) {
          done = true;
          break;
        }
      }
    }
    if (!done) {
      hw_p4_detail::h264Reset();
      hw_p4_detail::dmaResetCounterDbtmp();
      hw_p4_detail::dmaResetCounterDb();
      hw_p4_detail::dmaResetCounterRef();
      frameNum_ = 0;
      return 0;
    }
    return finishFrame(dst, dstCapacity, nalStartOffset, outFrameLen);
  }

 private:
  /**
   * The shared setup every encode()-family method needs: converts the
   * source picture, writes SPS/PPS (if this is an IDR)/the slice header
   * into `dst`, splits it at the 8-byte hardware boundary, configures
   * the per-frame YUV-source/bitstream-dest descriptors, and starts the
   * DMA/hardware pipeline (`startFrameDma()`) - everything encode() and
   * encodeDiagnostic() do identically, up until the point where they
   * differ in how they wait for completion. Returns false (nothing
   * started) on any failure; `outIsIdr`/`outNalStartOffset`/
   * `outOutFrameLen` are only meaningful on a true return, needed by
   * `finishFrame()` afterward.
   */
  bool prepareAndStartFrame(const uint8_t* srcY, int strideY,
                             const uint8_t* srcU, const uint8_t* srcV,
                             int strideC, uint8_t* dst, size_t dstCapacity,
                             bool& outIsIdr, size_t& outNalStartOffset,
                             size_t& outOutFrameLen) {
    if (!opened_ || !dst) return false;

    bool isIdr = (frameNum_ == 0) || (gop_ > 0 && (frameNum_ % gop_) == 0);
    if (isIdr) frameNum_ = 0;

    if (!ensurePackedScratch()) return 0;
    hw_p4_detail::convertYuv420ToPackedOuyyEvyy(
        srcY, strideY, srcU, srcV, strideC, width_, height_,
        packedYuvScratch_);
    hw_p4_detail::cacheWriteback(packedYuvScratch_, packedYuvScratchSize_);

    size_t o = 0;
    if (isIdr) {
      uint8_t rbsp[64];
      BitWriter spsBw(rbsp, sizeof(rbsp));
      // levelIdc: a fixed, generously-sized choice (42 == level 4.2,
      // comfortably covers every resolution this hardware supports up
      // to 1920x1080-class) rather than porting Espressif's own
      // resolution/fps -> level lookup table - a real simplification,
      // not a correctness requirement (decoders don't reject a stream
      // for declaring a higher level than it strictly needs).
      writeSpsRbsp(spsBw, width_, height_, /*levelIdc=*/42,
                   /*maxNumRefFrames=*/1);
      size_t n = writeNalUnit(dst + o, dstCapacity - o, /*nalRefIdc=*/3,
                               /*nalType=*/7, rbsp, spsBw.bytesWritten());
      if (n == 0) return 0;
      o += n;

      BitWriter ppsBw(rbsp, sizeof(rbsp));
      writePpsRbsp(ppsBw, qp_);
      n = writeNalUnit(dst + o, dstCapacity - o, /*nalRefIdc=*/3,
                        /*nalType=*/8, rbsp, ppsBw.bytesWritten());
      if (n == 0) return 0;
      o += n;
    }

    size_t nalStartOffset = o;
    if (o + 5 > dstCapacity) return 0;
    dst[o++] = 0; dst[o++] = 0; dst[o++] = 0; dst[o++] = 1;
    uint8_t nalRefIdc = isIdr ? 3 : 2;
    uint8_t nalType = isIdr ? 5 : 1;
    dst[o++] = (uint8_t)(((nalRefIdc & 3) << 5) | (nalType & 0x1f));

    uint8_t rawHeader[16] = {0};
    BitWriter hbw(rawHeader, sizeof(rawHeader));
    if (isIdr) {
      writeSliceHeaderIdr(hbw);
    } else {
      // Fixed QP: ppsBaseQp == qp_ always, so slice_qp_delta is always 0
      // - no live rate control, matching this file's documented scope.
      writeSliceHeaderP(hbw, frameNum_, qp_, qp_);
    }
    if (hbw.error()) return 0;
    size_t rawBits = hbw.bitsWritten();
    size_t wholeBytes = rawBits / 8;
    int remainderBits = (int)(rawBits % 8);

    int zeroRun = 0;
    for (size_t i = 0; i < wholeBytes; i++) {
      uint8_t b = rawHeader[i];
      if (zeroRun >= 2 && b <= 3) {
        if (o >= dstCapacity) return 0;
        dst[o++] = 0x03;
        zeroRun = 0;
      }
      if (o >= dstCapacity) return 0;
      dst[o++] = b;
      zeroRun = (b == 0) ? zeroRun + 1 : 0;
    }
    size_t escapedWholeByteCount = o - (nalStartOffset + 5);
    if (remainderBits > 0) {
      if (o >= dstCapacity) return 0;
      dst[o++] = rawHeader[wholeBytes];
    }

    // 8-byte-alignment split (relative to dst[0], matching Espressif's
    // own `esp_h264_enc_hw_slice_header_align8()` - see file header's
    // "Validation status" note on why this specific transform is copied
    // verbatim rather than re-derived): the hardware's slice-header
    // registers can only hold up to 7 "extra" complete bytes past the
    // last 8-byte boundary; anything before that boundary is left as-is
    // in `dst` for the DMA engine to read, everything from that boundary
    // onward gets held back into three hardware registers instead (the
    // DMA-written bitstream physically starts there once the hardware
    // begins encoding).
    size_t bitLen = nalStartOffset * 8 + 32 + 8 + escapedWholeByteCount * 8 +
                     (size_t)remainderBits;
    size_t blen = bitLen >> 3;
    size_t unalignedBlen = blen & 7;
    uint8_t* src = dst + (blen & ~(size_t)7);
    uint32_t header[3] = {0, 0, 0};
    uint8_t* hdst = reinterpret_cast<uint8_t*>(&header[0]);
    for (size_t i = 0; i < unalignedBlen; i++) hdst[7 - i] = src[i];
    header[2] = dst[((bitLen + 7) >> 3) - 1];
    uint16_t sliceHeaderBits =
        (uint16_t)((unalignedBlen << 3) + (bitLen & 7));
    hw_p4_detail::h264SetSliceHeaderRegs(header[0], header[1],
                                          (uint8_t)(header[2] & 0xff),
                                          sliceHeaderBits);

    size_t outFrameLen = (size_t)(src - dst);
    hw_p4_detail::cacheWriteback(dst, (bitLen + 7) >> 3);

    if (dstCapacity <= outFrameLen) return 0;
    // eof=0 - REVERTED from an earlier "eof=1" change. That change was
    // itself wrong: cross-checked directly against Espressif's real,
    // confirmed-working driver source (`esp_h264_enc_hw_param.c`'s
    // `esp_h264_enc_hw_cfg_dma_yuv_bs()`), which explicitly passes
    // `H264_DMA_EOF_CONTINUE` (0), not `H264_DMA_EOF_END` (1), for this
    // exact descriptor - a real, surprising, non-obvious exception to
    // the general "a lone/non-chained descriptor should have eof=1"
    // rule (which every other descriptor in this file correctly
    // follows). Presumably because TX channel 0 runs in 2D block-repeat
    // mode (`mode=1`, reading `hb`x`vb` blocks until `HA`x`VA` is fully
    // covered) - EOF's own meaning ("this is the last descriptor in the
    // linked list") is a different concept from "when has this
    // descriptor's own repeated-block read finished", so forcing eof=1
    // here was based on a plausible but incorrect generalization from
    // the other (1D, single-shot) descriptors in this file.
    hw_p4_detail::configureDesc(
        dscYuv_, /*en2d=*/true, /*mode=*/1, /*vb=*/16,
        /*hb=*/(uint16_t)(96 / 1.5f), /*eof=*/0, /*owner=*/1,
        (uint16_t)height_, (uint16_t)width_, packedYuvScratch_, nullptr);
    hw_p4_detail::dmaSetOutLinkAddr(hw_p4_detail::kOutCh(0),
                                     (uint32_t)(uintptr_t)dscYuv_);
    uint32_t bufBsLen = (uint32_t)(dstCapacity - outFrameLen);
    hw_p4_detail::configureDesc(dscBs_, /*en2d=*/false, /*mode=*/0,
                                 (uint16_t)(bufBsLen & 0x3fff), 0,
                                 /*eof=*/1, /*owner=*/1,
                                 (uint16_t)(bufBsLen >> 14), 0,
                                 src, nullptr);
    hw_p4_detail::dmaSetInLinkAddr(hw_p4_detail::kInCh(4),
                                    (uint32_t)(uintptr_t)dscBs_);

    if (!startFrameDma(isIdr)) {
      frameNum_ = 0;
      return false;
    }

    outIsIdr = isIdr;
    outNalStartOffset = nalStartOffset;
    outOutFrameLen = outFrameLen;
    return true;
  }

  /**
   * The shared tail every encode()-family method needs, once completion
   * (real or diagnostic-polled) has been observed: reads the real coded
   * length, invalidates `dst`'s cache line(s) so the CPU sees what the
   * DMA engine actually wrote, force-rewrites the real Annex-B start
   * code (see the comment at the call site below for why), and applies
   * the overflow safety net. Mirrors exactly what encode() used to do
   * inline before `prepareAndStartFrame()`/`finishFrame()` were split
   * out to share this logic with `encodeDiagnostic()`.
   */
  size_t finishFrame(uint8_t* dst, size_t dstCapacity, size_t nalStartOffset,
                      size_t outFrameLen) {
    uint32_t codedLen = hw_p4_detail::h264GetCodedLength();
    hw_p4_detail::cacheInvalidate(dst, dstCapacity);
    // The hardware's CAVLC engine has some undocumented rule against
    // continuous zero bytes that can corrupt the placeholder bytes at
    // this exact position - Espressif's own driver unconditionally
    // rewrites the real Annex-B start code here after every encode,
    // regardless of whether corruption is visible, and this port does
    // the same rather than trusting the DMA'd bytes at this offset.
    dst[nalStartOffset + 0] = 0;
    dst[nalStartOffset + 1] = 0;
    dst[nalStartOffset + 2] = 0;
    dst[nalStartOffset + 3] = 1;
    hw_p4_detail::cacheWriteback(dst + nalStartOffset, 4);

    bool overflow = hw_p4_detail::h264GetBsBitOverflowDebugBit();
    // Safety net beyond the (documented-unreliable) hardware flag: never
    // report success with a length larger than the buffer supplied.
    if ((size_t)codedLen + outFrameLen > dstCapacity) overflow = true;

    frameNum_++;
    if (overflow) return 0;
    return (size_t)codedLen + outFrameLen;
  }
  int width_ = 0, height_ = 0, mbWidth_ = 0, mbHeight_ = 0;
  int qp_ = 26, gop_ = 0;
  uint8_t frameNum_ = 0;
  bool opened_ = false;

  hw_p4_detail::H264DmaDesc* dscYuv_ = nullptr;
  hw_p4_detail::H264DmaDesc* dscBs_ = nullptr;
  hw_p4_detail::H264DmaDesc* dscRef_ = nullptr;
  hw_p4_detail::H264DmaDesc* dscDb_[4] = {nullptr, nullptr, nullptr, nullptr};
  hw_p4_detail::H264DmaDesc* dscDbtmp_[2] = {nullptr, nullptr};
  // Raw (pre-alignment) allocation pointers for the above - see
  // `internalAllocAligned8()`'s own comment for why these exist
  // separately from the (8-byte-aligned) pointers actually used above.
  uint8_t* dscYuvRaw_ = nullptr;
  uint8_t* dscBsRaw_ = nullptr;
  uint8_t* dscRefRaw_ = nullptr;
  uint8_t* dscDbRaw_[4] = {nullptr, nullptr, nullptr, nullptr};
  uint8_t* dscDbtmpRaw_[2] = {nullptr, nullptr};
  uint8_t* dbTmpBuf_ = nullptr;
  uint8_t* refBuf_ = nullptr;
  uint8_t* dbBuf_ = nullptr;
  uint8_t* packedYuvScratch_ = nullptr;
  size_t packedYuvScratchSize_ = 0;

  intr_handle_t intrHandle_ = nullptr;
  SemaphoreHandle_t frameDoneSem_ = nullptr;

  // Every DMA-touched buffer/descriptor in this class MUST live in
  // internal RAM, never PSRAM - found the hard way (see
  // `debugBufferAddresses()`'s own comment and this project's git
  // history): with PSRAM enabled, plain `malloc()` can and does route a
  // ~38KB allocation like the packed-YUV scratch buffer into PSRAM
  // (confirmed via a real address readback - `0x48xxxxxx`, ESP32-P4's
  // PSRAM range), and the H.264 hardware's DMA engine then stalls
  // forever trying to fetch from it (exactly the "intra_top_ctrl gets
  // stuck at state 3" symptom this project spent real effort
  // diagnosing). Espressif's own real driver allocates every equivalent
  // buffer with `ESP_H264_MEM_INTERNAL` explicitly for exactly this
  // reason - this project missed that requirement in its own from-
  // scratch port until this was tracked down. `heap_caps_malloc(...,
  // MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)` is the portable ESP-IDF
  // equivalent, available in both plain-IDF and Arduino builds.
  static uint8_t* internalAlloc(size_t size) {
    return (uint8_t*)heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  static uint8_t* alignedAlloc8(size_t size) {
    // +8 slack so ALIGN_UP(ptr,8) always has `size` real bytes available
    // after it, matching Espressif's own `ALIGN_UP(malloc'd ptr, 8)`
    // pattern (their allocator itself over-aligns to the cache line; a
    // plain heap_caps_malloc() here does not, hence the extra slack
    // instead).
    return internalAlloc(size + 8);
  }
  static uint8_t* alignUp8(uint8_t* p) {
    uintptr_t v = (uintptr_t)p;
    return (uint8_t*)((v + 7u) & ~(uintptr_t)7u);
  }
  /// Allocates `size` bytes, 8-byte aligned, and hands back BOTH the
  /// aligned pointer (return value) and the real, freeable allocation
  /// pointer (`*outRaw`) - needed for `H264DmaDesc`, whose address is
  /// written directly into an `OUTLINK_ADDR_CHx`/`INLINK_ADDR_CHx`
  /// register. A REAL, previously-missing requirement found via the
  /// official ESP32-P4 v1.3 TRM (chapter 37.5.2.3, "Transfer
  /// Initialization"): "H264_DMA_OUTLINK/INLINK_ADDR_CHx: ... the
  /// address requires 8-byte alignment" - `heap_caps_malloc()` does not
  /// document 8-byte alignment as guaranteed for an arbitrary allocation
  /// size (16 bytes, `sizeof(H264DmaDesc)`, in every caller here), so an
  /// unaligned descriptor address was a real, silent possibility this
  /// port never accounted for. This project's own picture/reference/
  /// deblock BUFFERS already went through the equivalent
  /// `alignedAlloc8()`/`alignUp8()` pattern - this was a real gap
  /// specifically in the descriptor allocations, which used plain
  /// `internalAlloc()` instead.
  static uint8_t* internalAllocAligned8(size_t size, uint8_t** outRaw) {
    uint8_t* raw = internalAlloc(size + 8);
    *outRaw = raw;
    return raw ? alignUp8(raw) : nullptr;
  }

  bool ensurePackedScratch() {
    size_t needed = (size_t)width_ * (size_t)height_ * 3 / 2;
    if (packedYuvScratch_ && packedYuvScratchSize_ == needed) return true;
    free(packedYuvScratch_);
    packedYuvScratch_ = internalAlloc(needed);
    packedYuvScratchSize_ = packedYuvScratch_ ? needed : 0;
    return packedYuvScratch_ != nullptr;
  }

  bool allocateBuffersAndDescriptors() {
    using hw_p4_detail::H264DmaDesc;
    dscYuv_ = (H264DmaDesc*)internalAllocAligned8(sizeof(H264DmaDesc), &dscYuvRaw_);
    dscBs_ = (H264DmaDesc*)internalAllocAligned8(sizeof(H264DmaDesc), &dscBsRaw_);
    dscRef_ = (H264DmaDesc*)internalAllocAligned8(sizeof(H264DmaDesc), &dscRefRaw_);
    for (int i = 0; i < 4; i++)
      dscDb_[i] = (H264DmaDesc*)internalAllocAligned8(sizeof(H264DmaDesc), &dscDbRaw_[i]);
    for (int i = 0; i < 2; i++)
      dscDbtmp_[i] = (H264DmaDesc*)internalAllocAligned8(sizeof(H264DmaDesc), &dscDbtmpRaw_[i]);
    if (!dscYuv_ || !dscBs_ || !dscRef_) return false;
    for (int i = 0; i < 4; i++) if (!dscDb_[i]) return false;
    for (int i = 0; i < 2; i++) if (!dscDbtmp_[i]) return false;

    // Buffer sizing formulas ported from
    // `esp_h264_enc_hw_param.c`'s `max_refame_buffer_size()`/
    // `max_db_buffer_size()`/`esp_h264_enc_hw_max_db_tmp_buffer_size()`.
    uint32_t refSize = 3u * 16u * (16u + 8u) * (uint32_t)mbWidth_ + 7u;
    uint32_t dbSize = (uint32_t)mbWidth_ * (uint32_t)mbHeight_ * 384u + 16u;
    uint32_t dbTmpSize = (uint32_t)mbWidth_ * hw_p4_detail::kDbtmp4LinesRowLength + 79u;
    refBuf_ = alignedAlloc8(refSize);
    dbBuf_ = alignedAlloc8(dbSize);
    dbTmpBuf_ = alignedAlloc8(dbTmpSize);
    return refBuf_ && dbBuf_ && dbTmpBuf_;
  }

  void configureStaticDescriptors() {
    using namespace hw_p4_detail;
    uint8_t* refAligned = alignUp8(refBuf_);
    configureDesc(dscRef_, /*en2d=*/true, /*mode=*/1, /*vb=*/3, /*hb=*/256,
                  /*eof=*/1, /*owner=*/1, /*va=*/3,
                  (uint16_t)(256 * mbWidth_), refAligned, dscRef_);
    dmaSetOutLinkAddr(kOutCh(1), (uint32_t)(uintptr_t)dscRef_);
    dmaSetIn5Block((uint32_t)(uintptr_t)refAligned, mbWidth_);

    uint32_t size12_4 = kDbtmp12LinesRowLength * (uint32_t)mbWidth_ *
                             (uint32_t)(mbHeight_ - 1) +
                         (kDbtmp12LinesRowLength + kDbtmp4LinesRowLength) *
                             (uint32_t)mbWidth_;
    uint8_t* dbAligned = alignUp8(dbBuf_);
    configureDesc(dscDb_[0], false, 0, (uint16_t)(size12_4 & 0x3fff),
                  (uint16_t)(size12_4 & 0x3fff), 1, 1,
                  (uint16_t)(size12_4 >> 14), (uint16_t)(size12_4 >> 14),
                  dbAligned, dscDb_[0]);
    configureDesc(dscDb_[1], false, 0, (uint16_t)(size12_4 & 0x3fff),
                  (uint16_t)(size12_4 & 0x3fff), 1, 1,
                  (uint16_t)(size12_4 >> 14), (uint16_t)(size12_4 >> 14),
                  dbAligned, dscDb_[1]);
    uint8_t* dbAligned2 = alignUp8(dbAligned + size12_4);
    uint32_t size4 =
        kDbtmp4LinesRowLength * (uint32_t)mbWidth_ * (uint32_t)(mbHeight_ - 1);
    configureDesc(dscDb_[2], false, 0, (uint16_t)(size4 & 0x3fff),
                  (uint16_t)(size4 & 0x3fff), 1, 1, (uint16_t)(size4 >> 14),
                  (uint16_t)(size4 >> 14), dbAligned2, dscDb_[2]);
    configureDesc(dscDb_[3], false, 0, (uint16_t)(size4 & 0x3fff),
                  (uint16_t)(size4 & 0x3fff), 1, 1, (uint16_t)(size4 >> 14),
                  (uint16_t)(size4 >> 14), dbAligned2, dscDb_[3]);
    dmaSetOutLinkAddr(kOutCh(3), (uint32_t)(uintptr_t)dscDb_[0]);
    dmaSetInLinkAddr(kInCh(0), (uint32_t)(uintptr_t)dscDb_[1]);
    dmaSetOutLinkAddr(kOutCh(4), (uint32_t)(uintptr_t)dscDb_[2]);
    dmaSetInLinkAddr(kInCh(1), (uint32_t)(uintptr_t)dscDb_[3]);

    uint32_t sizeDbtmp = kDbtmp4LinesRowLength * (uint32_t)mbWidth_;
    uint8_t* dbTmpAligned = alignUp8(dbTmpBuf_);
    configureDesc(dscDbtmp_[0], false, 0, (uint16_t)(sizeDbtmp & 0x3fff),
                  (uint16_t)(sizeDbtmp & 0x3fff), 1, 1,
                  (uint16_t)(sizeDbtmp >> 14), (uint16_t)(sizeDbtmp >> 14),
                  dbTmpAligned, dscDbtmp_[0]);
    configureDesc(dscDbtmp_[1], false, 0, (uint16_t)(sizeDbtmp & 0x3fff),
                  (uint16_t)(sizeDbtmp & 0x3fff), 1, 1,
                  (uint16_t)(sizeDbtmp >> 14), (uint16_t)(sizeDbtmp >> 14),
                  dbTmpAligned, dscDbtmp_[1]);
    dmaSetOutLinkAddr(kOutCh(2), (uint32_t)(uintptr_t)dscDbtmp_[0]);
    dmaSetInLinkAddr(kInCh(2), (uint32_t)(uintptr_t)dscDbtmp_[1]);
  }

  /// Ported from `h264_start_gop_mode_enc()` - the I-frame-vs-P-frame
  /// DMA channel start choreography (see file header's risk note: this
  /// exact branching, including which channels get reset/(re)started on
  /// which frame type, is copied faithfully from Espressif's source
  /// rather than reasoned out independently, since there's no
  /// documentation explaining *why* the two paths differ).
  ///
  /// Channel start order and the I-frame-only `h264Reset()` call are now
  /// copied EXACTLY from `codec-h264-ESP32P4`'s vendored, confirmed-
  /// working Espressif driver source (`esp_h264_enc_single_hw.c`'s
  /// `h264_start_gop_mode_enc()`) - including TX channel 2 (`kOutCh(2)`,
  /// the deblocking-intermediate-data feedback channel into ENC_CORE)
  /// being started here, up front, alongside every other channel. An
  /// earlier version of this port moved that start into the
  /// `DB_TMP_READY_INT` handler instead, reasoning from the official
  /// TRM's prose programming procedure (37.7.1, step (h)) - that
  /// reasoning was plausible but wrong: the real driver starts it here
  /// unconditionally, contradicting the TRM's stated sequence. Real,
  /// working source beats documentation prose.
  bool startFrameDma(bool isIframe) {
    using namespace hw_p4_detail;
    dmaClearAllInterrupts();
    h264ClearInterrupts(0xffffffffu);
    dmaResetCounterDbtmp();
    bool ok = true;
    if (isIframe) {
      dmaResetCounterDb();
      dmaResetCounterRef();
      ok &= dmaStartOutChannel(kOutCh(0));  // yuv
      ok &= dmaStartInChannel(kInCh(0));    // rx_db_12line
      ok &= dmaStartInChannel(kInCh(1));    // rx_db_4line
      ok &= dmaStartInChannel(kInCh(4));    // rx_bs
      ok &= dmaStartOutChannel(kOutCh(2));  // tx_dbtmp
      ok &= dmaStartInChannel(kInCh(2));    // rx_dbtmp
      h264Reset();
    } else {
      ok &= dmaStartOutChannel(kOutCh(0));  // yuv
      ok &= dmaStartInChannel(kInCh(4));    // rx_bs
      ok &= dmaStartInChannel(kInCh(0));    // rx_db_12line
      ok &= dmaStartInChannel(kInCh(1));    // rx_db_4line
      ok &= dmaStartOutChannel(kOutCh(2));  // tx_dbtmp
      ok &= dmaStartInChannel(kInCh(2));    // rx_dbtmp
      // Motion-vector-telemetry (MVM) RX channel deliberately not
      // started here - see file header's "Scope" note.
    }
    if (!ok) return false;
    if (h264_debug_before_frame_start_hook) h264_debug_before_frame_start_hook();
    // Tested and reverted: explicit settling delays here (10us, 2ms, and
    // 200us between every single DMA channel start) were all tried,
    // hoping to explain a one-off successful hybrid-bisection run (see
    // codec-h264-ESP32P4's examples/HybridBisection3) as a hardware
    // settling-time race. None of them changed this class's own
    // encodeDiagnostic() outcome at all - it fails identically,
    // deterministically, regardless of delay. This rules out timing/
    // settling as the explanation; the one successful standalone run
    // was most likely a non-reproducible fluke (e.g. leftover register
    // state from a prior JTAG debugging session), not a real clue.
    h264SetFrameStart();
    return true;
  }

  static void IRAM_ATTR isrThunk(void* arg) {
    static_cast<HwEncoderP4*>(arg)->isr();
  }

  /// Ported from `h264_gop_isr()` - see file header's highest-risk note:
  /// the I-frame-only mid-frame DMA kicks below (on `REC_READY` and
  /// `2MB_LINE_DONE`) are load-bearing, undocumented hardware
  /// choreography, not just interrupt bookkeeping. Factored out from the
  /// ISR itself so `encodeDiagnostic()` can run the exact same logic
  /// from a normal (non-interrupt) polling loop - see that method's own
  /// comment for why. Handles exactly one matched condition per call
  /// (matching the original inline `if`/`else if` chain's own behavior -
  /// the caller's own loop re-reads status and calls again). Returns
  /// true only when `FRAME_DONE` was the condition handled (encode
  /// complete) - `fromIsr`/`outHigherPriorityTaskWoken` are only used in
  /// that branch, to give the completion semaphore ISR-safely when
  /// called from `isr()` (from `encodeDiagnostic()`'s polling loop,
  /// `fromIsr` is false and no semaphore is touched at all - the caller
  /// just observes the true return value directly).
  bool pumpInterruptStatus(uint32_t status, bool isIframe, bool fromIsr,
                            BaseType_t* outHigherPriorityTaskWoken) {
    using namespace hw_p4_detail;
    if (status & kIntrDbTmpReady) {
      h264ClearInterrupts(kIntrDbTmpReady);
    } else if ((status & kIntrRecReady) && isIframe) {
      h264ClearInterrupts(kIntrRecReady);
      // Ported from `h264_dma_hal_start_tx_db12_4_dma()`: stop both TX
      // channels, reset ch5 (NOT channels 3/4 themselves - a real,
      // asymmetric sequence in Espressif's own source, not a typo
      // here), then start both TX channels.
      *dmaChnLinkConf(kOutCh(3)) |= (1u << 20);  // outlink_stop
      *dmaChnLinkConf(kOutCh(4)) |= (1u << 20);
      dmaResetIn5Channel();
      *dmaChnLinkConf(kOutCh(3)) |= (1u << 21);  // outlink_start
      *dmaChnLinkConf(kOutCh(4)) |= (1u << 21);
      h264DmaMoveStart();
    } else if (status & kIntrRecReady) {
      h264ClearInterrupts(kIntrRecReady);
    } else if ((status & kIntr2mbLineDone) && isIframe) {
      h264ClearInterrupts(kIntr2mbLineDone);
      dmaStartOutChannel(kOutCh(1));
    } else if (status & kIntr2mbLineDone) {
      h264ClearInterrupts(kIntr2mbLineDone);
    } else if (status & kIntrFrameDone) {
      h264ClearInterrupts(kIntrFrameDone);
      if (fromIsr) {
        xSemaphoreGiveFromISR(frameDoneSem_, outHigherPriorityTaskWoken);
      }
      return true;
    }
    return false;
  }

  void IRAM_ATTR isr() {
    bool isIframe = (frameNum_ == 0);
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    for (;;) {
      uint32_t status = hw_p4_detail::h264GetInterruptStatus();
      if (!status) break;
      pumpInterruptStatus(status, isIframe, /*fromIsr=*/true,
                          &higherPriorityTaskWoken);
    }
    if (higherPriorityTaskWoken) portYIELD_FROM_ISR();
  }
};

}  // namespace tinyh264

#endif  // TINYH264_HW_ENCODER_P4_AVAILABLE
