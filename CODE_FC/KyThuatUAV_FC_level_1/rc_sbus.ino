// ============================================================================
//  KyThuatUAV FC - firmware bay ESP32 (FreeRTOS), che do Angle + Alt Hold
//
//  Copyright (C) 2026  Nguyễn Văn Quý (Ky Thuat UAV)
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//  Giu nguyen phan ghi cong nay khi dung lai hoac chinh sua ma nguon.
//
//  Nguon tham khao va ghi cong day du: xem file LOI_NHAN_VA_GHI_CONG.md
//  Lien he: Nguyen Van Quy - 0817 550 271 (Zalo)
// ============================================================================
// ============================================================================
//  Giải mã SBUS.
//  SBUS là tín hiệu ĐẢO, 100000 baud, khung 8E2. ESP32 đảo được bằng phần cứng
//  nên chỉ cần bật cờ invert lúc mở UART, không cần mạch đảo bên ngoài.
//
//  Một khung SBUS dài 25 byte: 0x0F, 22 byte dữ liệu (16 kênh x 11 bit),
//  1 byte cờ, 1 byte kết thúc 0x00.
// ============================================================================

#include <HardwareSerial.h>
HardwareSerial sbus_uart(2);   // UART2

#define SBUS_FRAME_LEN     25
#define SBUS_RAW_MIN      173     // giá trị thô ứng với cần gạt hết về một phía
#define SBUS_RAW_MAX     1811
#define SBUS_OUT_MIN      990     // quy đổi ra dải quen thuộc 1000-2000
#define SBUS_OUT_MAX     2010
#define SBUS_TIMEOUT_MS   200     // quá thời gian này không có khung mới = mất sóng

static uint8_t  sbus_frame[SBUS_FRAME_LEN];
static unsigned int sbus_byte, sbus_byte_index;
static unsigned int sbus_raw_ch[17];
static unsigned int sbus_ch_us[17];
static bool     sbus_frame_start;
static unsigned long sbus_last_frame_ms;


void rc_init() {
  sbus_uart.begin(100000, SERIAL_8E2, PIN_SBUS_RX, -1, true);
  sbus_byte_index  = 255;
  sbus_frame_start = false;
}


// Gom byte cho tới khi đủ một khung hợp lệ
static bool sbus_read_frame() {
  while (sbus_uart.available()) {
    sbus_byte = sbus_uart.read();

    if ((sbus_byte == 0x0F) && sbus_frame_start) {
      sbus_frame_start = false;
      sbus_byte_index  = 0;
    } else if (sbus_byte == 0) {
      sbus_frame_start = true;
    }

    if (sbus_byte_index <= SBUS_FRAME_LEN - 1) {
      sbus_frame[sbus_byte_index] = sbus_byte;
      sbus_byte_index++;
      if ((sbus_byte_index == SBUS_FRAME_LEN) && (sbus_byte == 0) &&
          (sbus_frame[0] == 0x0F)) {
        return true;
      }
    }
  }
  return false;
}

// Tách 16 kênh, mỗi kênh 11 bit, xếp liên tục không theo biên byte
static void sbus_decode_channels() {
  int bit_ptr  = 0;
  int byte_ptr = 1;
  sbus_raw_ch[0] = sbus_frame[23];

  for (int chan = 1; chan <= 16; chan++) {
    sbus_raw_ch[chan] = 0;
    for (int bit = 0; bit < 11; bit++) {
      sbus_raw_ch[chan] |= ((sbus_frame[byte_ptr] >> bit_ptr) & 1) << bit;
      if (++bit_ptr > 7) {
        bit_ptr = 0;
        byte_ptr++;
      }
    }
  }
}


// Trả về true khi vẫn còn sóng, false khi mất sóng.
bool rc_update() {
  if (sbus_read_frame()) {
    sbus_decode_channels();
    for (int i = 1; i <= 16; i++) {
      sbus_ch_us[i] = map(sbus_raw_ch[i], SBUS_RAW_MIN, SBUS_RAW_MAX,
                          SBUS_OUT_MIN, SBUS_OUT_MAX);
    }
    sbus_last_frame_ms = millis();
  }
  return (millis() - sbus_last_frame_ms <= SBUS_TIMEOUT_MS);
}

int rc_get_channel(int index) { return sbus_ch_us[index]; }
