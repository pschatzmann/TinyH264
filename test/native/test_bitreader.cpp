// Desktop-only unit test for BitReader. Build/run via test/native/run.sh.
#include <cassert>
#include <cstdio>
#include "../../src/decoder/h264_bitreader.h"

using tinyh264::BitReader;

static void test_u() {
  // 1011 0110 -> u(1)=1 u(3)=011=3 u(4)=0110=6
  uint8_t data[] = {0xB6};
  BitReader br(data, 1);
  assert(br.u(1) == 1);
  assert(br.u(3) == 3);
  assert(br.u(4) == 6);
  assert(!br.error());
}

static void test_ue() {
  /*
   * Exp-Golomb codes for 0,1,2,3,4 concatenated:
   * 0 -> 1
   * 1 -> 010
   * 2 -> 011
   * 3 -> 00100
   * 4 -> 00101
   * bitstream: 1 010 011 00100 00101  (17 bits) padded to 3 bytes with zeros
   * 1 010011 00100001 01000000
   */
  uint8_t data[] = {0b10100110, 0b01000010, 0b10000000};
  BitReader br(data, 3);
  assert(br.ue() == 0);
  assert(br.ue() == 1);
  assert(br.ue() == 2);
  assert(br.ue() == 3);
  assert(br.ue() == 4);
  assert(!br.error());
}

static void test_se() {
  // se(v) codeNum->value: 0->0,1->1,2->-1,3->2,4->-2
  uint8_t data[] = {0b10100110, 0b01000010, 0b10000000};
  BitReader br(data, 3);
  assert(br.se() == 0);
  assert(br.se() == 1);
  assert(br.se() == -1);
  assert(br.se() == 2);
  assert(br.se() == -2);
}

static void test_more_rbsp_data() {
  /*
   * RBSP: one byte 0xA0 = 1010 0000, then stop bit is bit index 1 (the last
   * '1'). After consuming 2 bits, more_rbsp_data() should be false.
   */
  uint8_t data[] = {0xA0};
  BitReader br(data, 1);
  assert(br.moreRbspData() == true);
  br.u(2);  // consume "10"
  assert(br.moreRbspData() == false);
}

static void test_error_on_overrun() {
  uint8_t data[] = {0xFF};
  BitReader br(data, 1);
  br.u(8);
  assert(!br.error());
  br.u(1);  // past end
  assert(br.error());
}

int main() {
  test_u();
  test_ue();
  test_se();
  test_more_rbsp_data();
  test_error_on_overrun();
  printf("test_bitreader: all tests passed\n");
  return 0;
}
