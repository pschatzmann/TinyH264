/*
 * HwEncoderP4RegisterDump: diagnostic-only sketch, NOT part of this
 * library's supported API. Dumps the ENTIRE H.264 core (0x50084000) and
 * DMA (0x500A7000) register banks as raw hex, at the exact moment right
 * before `frame_start` is written (via a weak debug hook added to
 * `startFrameDma()` in `h264_hw_encoder_p4.h` for this purpose).
 *
 * Purpose: direct comparison against codec-h264-ESP32P4's own matching
 * `examples/RegisterDump` sketch (which wraps Espressif's real, working
 * hardware driver) - both dump the complete register bank at the same
 * logical "configured and armed, not yet started" point, on the same
 * board/silicon, so any remaining configuration difference neither
 * project's author has thought to check individually by hand shows up
 * directly in a diff between the two captures. See this project's own
 * README.md for the full investigation history.
 */
#include <cstring>

#include <TinyH264Encoder.h>

using namespace tinyh264;

static const uintptr_t kH264Base = 0x50084000;
static const uintptr_t kH264DmaBase = 0x500A7000;

static void dumpRegs(const char* label, uintptr_t base, uint32_t startOff,
                      uint32_t endOff) {
  Serial.printf("=== %s  base=0x%08X  [0x%03X..0x%03X) ===\n", label,
                (unsigned)base, (unsigned)startOff, (unsigned)endOff);
  for (uint32_t off = startOff; off < endOff; off += 4) {
    if ((off - startOff) % 32 == 0) {
      Serial.printf("\n0x%03X:", (unsigned)off);
    }
    uint32_t v = *(volatile uint32_t*)(base + off);
    Serial.printf(" %08X", (unsigned)v);
  }
  Serial.println("\n");
}

static void dumpAll(const char* label) {
  dumpRegs(label, kH264Base, 0x000, 0x0F4);
  dumpRegs(label, kH264DmaBase, 0x000, 0xB64);
}

static void dumpDesc(const char* label, uintptr_t addr) {
  if (!addr) {
    Serial.printf("%s: (null)\n", label);
    return;
  }
  const uint8_t* p = (const uint8_t*)addr;
  uint32_t dw0, dw1;
  uint32_t bufPtr, nextPtr;
  memcpy(&dw0, p + 0, 4);
  memcpy(&dw1, p + 4, 4);
  memcpy(&bufPtr, p + 8, 4);
  memcpy(&nextPtr, p + 12, 4);
  uint32_t vb = dw0 & 0x3fff;
  uint32_t hb = (dw0 >> 14) & 0x3fff;
  uint32_t errEof = (dw0 >> 28) & 1;
  uint32_t dma2dEn = (dw0 >> 29) & 1;
  uint32_t eof = (dw0 >> 30) & 1;
  uint32_t owner = (dw0 >> 31) & 1;
  uint32_t va = dw1 & 0x3fff;
  uint32_t ha = (dw1 >> 14) & 0x7fff;
  uint32_t mode = (dw1 >> 29) & 1;
  Serial.printf(
      "%s @0x%08X: raw=[%08X %08X %08X %08X] vb=%u hb=%u errEof=%u "
      "dma2dEn=%u eof=%u owner=%u va=%u ha=%u mode=%u buf=0x%08X next=0x%08X\n",
      label, (unsigned)addr, (unsigned)dw0, (unsigned)dw1, (unsigned)bufPtr,
      (unsigned)nextPtr, (unsigned)vb, (unsigned)hb, (unsigned)errEof,
      (unsigned)dma2dEn, (unsigned)eof, (unsigned)owner, (unsigned)va,
      (unsigned)ha, (unsigned)mode, (unsigned)bufPtr, (unsigned)nextPtr);
}

static HwEncoderP4* g_hw = nullptr;

extern "C" void h264_debug_before_frame_start_hook(void) {
  dumpAll("BEFORE_FRAME_START (armed, configured, not yet started)");
  if (g_hw) {
    HwEncoderP4::BufferAddresses a = g_hw->debugBufferAddresses();
    Serial.println("--- descriptor contents (raw bytes + decoded fields) ---");
    dumpDesc("dscYuv", a.dscYuv);
    dumpDesc("dscBs ", a.dscBs);
  }
}

static const int kWidth = 176;
static const int kHeight = 144;
static uint8_t srcY[kWidth * kHeight];
static uint8_t srcU[(kWidth / 2) * (kHeight / 2)];
static uint8_t srcV[(kWidth / 2) * (kHeight / 2)];
static uint8_t bitstream[8192];

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("TinyH264 HwEncoderP4RegisterDump (full register-bank capture)");

  for (int y = 0; y < kHeight; y++) {
    for (int x = 0; x < kWidth; x++) {
      srcY[y * kWidth + x] = (uint8_t)(((x + y) * 255) / (kWidth + kHeight));
    }
  }
  for (int i = 0; i < (kWidth / 2) * (kHeight / 2); i++) {
    srcU[i] = 128;
    srcV[i] = 128;
  }

  HwEncoderP4 hw;
  g_hw = &hw;
  bool opened = hw.open(kWidth, kHeight, /*qp=*/26, /*gop=*/30);
  Serial.printf("open() = %d\n", (int)opened);
  if (!opened) return;

  Serial.println("Encoding frame 0 (IDR) - h264_debug_before_frame_start_hook() will fire and dump registers...");
  size_t n = hw.encodeDiagnostic(srcY, kWidth, srcU, srcV, kWidth / 2,
                                  bitstream, sizeof(bitstream), nullptr,
                                  /*timeoutUs=*/500000, /*sampleIntervalUs=*/500000);
  Serial.printf("encodeDiagnostic() result: %u bytes\n", (unsigned)n);

  dumpAll("AFTER_TIMEOUT (post-stall idle state)");

  hw.close();
  Serial.println("DONE");
}

void loop() {}
