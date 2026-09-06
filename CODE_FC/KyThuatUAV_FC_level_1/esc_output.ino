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
//  Xuất xung PWM cho 4 ESC.
//  Tần số 391Hz với độ phân giải 11 bit: giá trị 800-1600 ứng với xung
//  1000-2000 us, đúng chuẩn PWM của ESC thường.
// ============================================================================

#define ESC_PWM_FREQ_HZ    391
#define ESC_PWM_BITS        11

const int ESC_LEDC_CH_1 = 0;
const int ESC_LEDC_CH_2 = 1;
const int ESC_LEDC_CH_3 = 2;
const int ESC_LEDC_CH_4 = 3;


void esc_init() {
  ledcSetup(ESC_LEDC_CH_1, ESC_PWM_FREQ_HZ, ESC_PWM_BITS);  ledcAttachPin(PIN_ESC_1, ESC_LEDC_CH_1);
  ledcSetup(ESC_LEDC_CH_2, ESC_PWM_FREQ_HZ, ESC_PWM_BITS);  ledcAttachPin(PIN_ESC_2, ESC_LEDC_CH_2);
  ledcSetup(ESC_LEDC_CH_3, ESC_PWM_FREQ_HZ, ESC_PWM_BITS);  ledcAttachPin(PIN_ESC_3, ESC_LEDC_CH_3);
  ledcSetup(ESC_LEDC_CH_4, ESC_PWM_FREQ_HZ, ESC_PWM_BITS);  ledcAttachPin(PIN_ESC_4, ESC_LEDC_CH_4);

  esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);
  delay(100);
  esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);
  delay(100);

  // THÁO CÁNH rồi mở khối này để dò chân nào ra motor nào.
  // Mỗi motor sẽ quay nhẹ lần lượt theo đúng thứ tự 1 -> 2 -> 3 -> 4.
  //   esc_write(900, 800, 800, 800);  delay(200);
  //   esc_write(800, 900, 800, 800);  delay(200);
  //   esc_write(800, 800, 900, 800);  delay(200);
  //   esc_write(800, 800, 800, 900);  delay(200);
  //   esc_write(800, 800, 800, 800);  delay(1000);
}

void esc_write(int m1, int m2, int m3, int m4) {
  ledcWrite(ESC_LEDC_CH_1, m1);
  ledcWrite(ESC_LEDC_CH_2, m2);
  ledcWrite(ESC_LEDC_CH_3, m3);
  ledcWrite(ESC_LEDC_CH_4, m4);
}
