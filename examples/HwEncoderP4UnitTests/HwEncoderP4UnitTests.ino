/*
 * Unit tests for the individual building blocks of
 * src/encoder/h264_hw_encoder_p4.h's ESP32-P4 hardware H.264 encoder
 * driver - NOT an end-to-end encode test (that's
 * examples/EncodeDecodeRoundTrip and examples/EncodeSyntheticFrame,
 * which exercise TinyH264Encoder::setUseHardware() against real silicon
 * and confirm both correctness - PSNR ~48-52dB self-decode - and
 * performance - ~1037fps at QCIF; see README.md's "ESP32-P4 hardware
 * encoder" section for the full numbers and the investigation history
 * that led there).
 *
 * The point of this sketch is different: isolate and check each
 * low-level piece *independently* - clock/reset, individual register
 * write/readback roundtrips, DMA channel reset-and-start in isolation,
 * and the public open()/close() lifecycle - so a *future* regression
 * (like end-to-end encode() failing again) can be narrowed down to
 * "this specific register write doesn't do what the code assumes" or
 * "this specific DMA channel never asserts reset_avail" instead of just
 * "something in a ~1000-line pipeline is wrong".
 *
 * This deliberately reaches into `tinyh264::hw_p4_detail` - the header's
 * own internal register/DMA primitives, not the public
 * TinyH264Encoder/HwEncoderP4 API - since that's exactly the level this
 * sketch needs to test individually. Not part of this library's
 * documented public API surface; only exists for this kind of testing.
 *
 * IMPORTANT: this pokes real hardware registers and starts/resets real
 * DMA channels, on real ESP32-P4 silicon. It's designed to be safe (every
 * DMA channel touched here is first pointed at a small, valid, harmless
 * dummy descriptor+buffer, matching how they'd be configured before a
 * real start in production use - never left pointing at an unconfigured
 * address), but this is still genuinely unvalidated hardware-facing code
 * - see h264_hw_encoder_p4.h's own file header for the full disclaimer.
 */

#include <stdio.h>
#include <string.h>

#include <TinyH264Encoder.h>

using namespace tinyh264;

#ifdef TINYH264_HW_ENCODER_P4_AVAILABLE

static int testsRun = 0;
static int testsPassed = 0;

static void check(const char* name, bool condition) {
  testsRun++;
  Serial.print(name);
  Serial.print(": ");
  if (condition) {
    testsPassed++;
    Serial.println("PASS");
  } else {
    Serial.println("FAIL");
  }
}

// A single small, 16-byte-aligned, harmless dummy descriptor + 16-byte
// buffer - used to point every DMA channel at *something valid* before
// testing its reset/start sequence in isolation, rather than leaving
// link_addr pointing at whatever happened to be there before (0, most
// likely) when we ask the channel to start fetching from it. `eof=1`
// (a complete, self-contained "transfer") and `dma2dEn=0` (flat linear,
// not 2D-picture-addressed) keep this as inert as a real descriptor can
// be - just enough for the reset/start register dance itself to be
// tested safely, not a real data transfer.
alignas(16) static uint8_t dummyBuf[16];
alignas(16) static hw_p4_detail::H264DmaDesc dummyDesc;

static void initDummyDescriptor() {
  memset(&dummyDesc, 0, sizeof(dummyDesc));
  dummyDesc.dma2dEn = 0;
  dummyDesc.mode = 0;
  dummyDesc.vb = 0;
  dummyDesc.hb = 0;
  dummyDesc.eof = 1;
  dummyDesc.owner = 1;
  dummyDesc.va = 0;
  dummyDesc.ha = 0;
  dummyDesc.buf = dummyBuf;
  dummyDesc.nextDesc = nullptr;
}

static void testClockAndReset() {
  using namespace hw_p4_detail;
  Serial.println("-- Clock/reset --");
  h264ClockAndResetInit();
  // sys_rst_pulse and the reset-enable bit are self-clearing/pulsed, so
  // only the two plain R/W clock-enable bits are meaningful to read back.
  volatile uint32_t* clkCtrl1 = reg<uint32_t>(kHpSysClkrstBase, kOffSocClkCtrl1);
  volatile uint32_t* periClkCtrl26 = reg<uint32_t>(kHpSysClkrstBase, kOffPeriClkCtrl26);
  check("reg_h264_sys_clk_en readback == 1", (*clkCtrl1 & kBitH264SysClkEn) != 0);
  check("reg_h264_clk_en readback == 1", (*periClkCtrl26 & kBitH264ClkEn) != 0);
  check("reg_h264_clk_src_sel readback == 1", (*periClkCtrl26 & kBitH264ClkSrcSel) != 0);
}

static void testCoreRegisterRoundtrips() {
  using namespace hw_p4_detail;
  Serial.println("-- Core register write/readback roundtrips --");

  h264SetSys();

  h264SetGop(4, true);
  volatile uint32_t* gopConf = reg<uint32_t>(kH264Base, kOffGopConf);
  check("h264SetGop(4,...): gop_num readback == 4",
        ((*gopConf >> 1) & 0xffu) == 4);
  volatile uint32_t* sysCtrl = reg<uint32_t>(kH264Base, kOffSysCtrl);
  check("h264SetGop(...,true): frame_mode == 0 (GOP mode)",
        ((*sysCtrl >> 2) & 1u) == 0);

  h264SetMb(11, 9);
  volatile uint32_t* mbRes = reg<uint32_t>(kH264Base, kOffCtrl0SysMbRes);
  check("h264SetMb(11,9): sys_total_mb_x readback == 11",
        ((*mbRes >> 7) & 0x7fu) == 11);
  check("h264SetMb(11,9): sys_total_mb_y readback == 9",
        (*mbRes & 0x7fu) == 9);

  h264SetQp(26);
  volatile uint32_t* rcConf0 = reg<uint32_t>(kH264Base, kOffCtrl0RcConf0);
  check("h264SetQp(26): qp readback == 26", (*rcConf0 & 0x3fu) == 26);

  h264SetDbBypass(true);
  volatile uint32_t* dbBypass = reg<uint32_t>(kH264Base, kOffCtrl0DbBypass);
  check("h264SetDbBypass(true): bypass_db_filter == 1", (*dbBypass & 1u) != 0);
  h264SetDbBypass(false);
  check("h264SetDbBypass(false): bypass_db_filter == 0", (*dbBypass & 1u) == 0);

  h264EnableInterrupts(kIntrDbTmpReady | kIntrRecReady | kIntr2mbLineDone |
                        kIntrFrameDone);
  volatile uint32_t* intEna = reg<uint32_t>(kH264Base, kOffIntEna);
  check("h264EnableInterrupts(all): int_ena readback == mask",
        (*intEna & kIntrMask) == kIntrMask);
  h264ClearInterrupts(0xffffffffu);
  check("h264GetInterruptStatus() == 0 right after clear",
        h264GetInterruptStatus() == 0);
}

static void testDmaDescriptorLayout() {
  Serial.println("-- DMA descriptor layout --");
  // Mirrors the compile-time static_assert in h264_hw_encoder_p4.h -
  // this is a real on-device confirmation of the same fact, not just a
  // repeat of the same compile-time check.
#if UINTPTR_MAX == 0xffffffffu
  check("sizeof(H264DmaDesc) == 16 (real hardware descriptor size)",
        sizeof(hw_p4_detail::H264DmaDesc) == 16);
#else
  Serial.println("(skipped - not a 32-bit target)");
#endif
}

static void testDmaChannelResetInIsolation() {
  using namespace hw_p4_detail;
  Serial.println("-- DMA channel reset/start, each in isolation --");

  initDummyDescriptor();
  dmaSetUnrestrictedMemRange();
  for (int i = 0; i < 5; i++) {
    dmaSetChnConf0(kOutCh(i), 1u << 1);  // EOF_EN only, no reorder
    dmaSetChnConf0(kInCh(i), 0);
  }
  dmaSetCh5Conf0(0);
  dmaSetAllBurstSize(4);

  // out_ch[0..2] use the normal self-reset (stop -> wait for
  // out_reset_avail -> pulse out_rst -> start), same as every in_ch.
  // out_ch[3]/[4] are a real, deliberate exception: production code
  // (HwEncoderP4's ISR, mirroring Espressif's own
  // h264_dma_hal_start_tx_db12_4_dma()) never self-resets them - it
  // stops both, resets ch5 *instead*, then starts both, with no
  // out_reset_avail wait on channels 3/4 themselves at all. An earlier
  // version of this test called the generic self-reset helper on
  // channels 3/4 too and got two clean FAILs (timeouts) - not a
  // hardware bug, just this test asking those two channels to do
  // something production code never asks of them. Testing the *actual*
  // asymmetric sequence here instead, for real coverage of what
  // HwEncoderP4 actually does.
  for (int i = 0; i < 3; i++) {
    dmaSetOutLinkAddr(kOutCh(i), (uint32_t)(uintptr_t)&dummyDesc);
    char name[40];
    snprintf(name, sizeof(name), "dmaStartOutChannel(out_ch[%d])", i);
    check(name, dmaStartOutChannel(kOutCh(i)));
  }
  {
    dmaSetOutLinkAddr(kOutCh(3), (uint32_t)(uintptr_t)&dummyDesc);
    dmaSetOutLinkAddr(kOutCh(4), (uint32_t)(uintptr_t)&dummyDesc);
    *dmaChnLinkConf(kOutCh(3)) |= (1u << 20);  // outlink_stop
    *dmaChnLinkConf(kOutCh(4)) |= (1u << 20);
    bool ch5ResetOk = dmaResetIn5Channel();
    *dmaChnLinkConf(kOutCh(3)) |= (1u << 21);  // outlink_start
    *dmaChnLinkConf(kOutCh(4)) |= (1u << 21);
    check("out_ch[3]/[4] asymmetric start (via in_ch5 reset, matching production ISR path)",
          ch5ResetOk);
  }
  for (int i = 0; i < 5; i++) {
    dmaSetInLinkAddr(kInCh(i), (uint32_t)(uintptr_t)&dummyDesc);
    char name[40];
    snprintf(name, sizeof(name), "dmaStartInChannel(in_ch[%d])", i);
    check(name, dmaStartInChannel(kInCh(i)));
  }
  dmaSetIn5Block((uint32_t)(uintptr_t)dummyBuf, /*mbWidth=*/11);
  check("dmaResetIn5Channel()", dmaResetIn5Channel());
}

static void testOpenCloseLifecycle() {
  Serial.println("-- HwEncoderP4::open()/close() lifecycle --");
  AllocatorMemoryResource<StdAllocator<uint8_t>> mr;
  HwEncoderP4 hw(mr);
  check("isOpen() == false before open()", !hw.isOpen());
  bool opened = hw.open(176, 144, /*qp=*/26, /*gop=*/0);
  check("open(176, 144, 26, 0)", opened);
  if (opened) {
    check("isOpen() == true after open()", hw.isOpen());
    check("width() == 176", hw.width() == 176);
    check("height() == 144", hw.height() == 144);
  } else {
    Serial.println("(open() failed - skipping the checks that depend on it)");
  }
  hw.close();
  check("isOpen() == false after close()", !hw.isOpen());
  // Safe to open() again after close() - a real lifecycle property, not
  // just "doesn't crash".
  check("open() again after close()", hw.open(176, 144, 26, 0));
  hw.close();
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("HwEncoderP4 unit tests - individual register/DMA primitives");
  Serial.print("hardwareAvailable() = ");
  Serial.println(TinyH264Encoder<>::hardwareAvailable() ? "true" : "false");
  Serial.println();

  testClockAndReset();
  Serial.println();
  testCoreRegisterRoundtrips();
  Serial.println();
  testDmaDescriptorLayout();
  Serial.println();
  testDmaChannelResetInIsolation();
  Serial.println();
  testOpenCloseLifecycle();

  Serial.println();
  Serial.print(testsPassed);
  Serial.print(" / ");
  Serial.print(testsRun);
  Serial.println(" checks passed.");
  Serial.println(testsPassed == testsRun ? "RESULT: PASS" : "RESULT: FAIL");
}

void loop() {}

#else  // !TINYH264_HW_ENCODER_P4_AVAILABLE

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println(
      "HwEncoderP4 not available on this build (needs a real ESP-IDF "
      "esp32p4 build, chip revision < 3.0) - nothing to test.");
}
void loop() {}

#endif
