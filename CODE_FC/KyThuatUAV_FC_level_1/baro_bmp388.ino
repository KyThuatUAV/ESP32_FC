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
//  BMP388 - driver thanh ghi, bus I2C, không dùng thư viện ngoài.
//  Chuyển từ bản SPI của FC_teensy41_v24 sang I2C.
//
//  KHÁC BIỆT SO VỚI BẢN SPI: khi đọc, SPI của BMP388 trả về một byte rác ngay
//  sau byte địa chỉ rồi mới tới dữ liệu thật. I2C KHÔNG có byte rác đó. Chép
//  nguyên hàm đọc SPI sang I2C là lệch một byte, mà lệch kiểu này không báo
//  lỗi gì, chỉ ra số sai.
//
//  API công khai:
//     bool  baro_init();              dò chip, cấu hình, đo cao độ gốc
//     bool  baro_update();            true khi có mẫu MỚI
//     float baro_get_altitude_m();    mét, đã trừ cao độ gốc lúc khởi động
//     bool  baro_is_present();
// ============================================================================

#include <Wire.h>

#define BMP388_CHIP_ID_VALUE   0x50

// Thanh ghi
#define BMP388_REG_CHIP_ID     0x00
#define BMP388_REG_ERR         0x02
#define BMP388_REG_STATUS      0x03
#define BMP388_REG_DATA        0x04
#define BMP388_REG_PWR_CTRL    0x1B
#define BMP388_REG_OSR         0x1C
#define BMP388_REG_ODR         0x1D
#define BMP388_REG_CONFIG      0x1F
#define BMP388_REG_CALIB       0x31
#define BMP388_REG_CMD         0x7E

// Bit của thanh ghi STATUS (0x03)
#define BMP388_STATUS_CMD_RDY    (1 << 4)
#define BMP388_STATUS_DRDY_PRESS (1 << 5)

#define BMP388_CMD_SOFT_RESET  0xB6

// Thanh ghi PWR_CTRL
#define BMP388_PRESS_EN        (1U << 0)
#define BMP388_TEMP_EN         (1U << 1)
#define BMP388_MODE_NORMAL     (3U << 4)

// Hệ số lấy mẫu bội (oversampling)
#define BMP388_OSR_X1          0x00
#define BMP388_OSR_X2          0x01
#define BMP388_OSR_X4          0x02
#define BMP388_OSR_X8          0x03
#define BMP388_OSR_X16         0x04
#define BMP388_OSR_X32         0x05

// Tần số lấy mẫu
#define BMP388_ODR_200HZ       0x00
#define BMP388_ODR_100HZ       0x01
#define BMP388_ODR_50HZ        0x02
#define BMP388_ODR_25HZ        0x03

// Bộ lọc IIR, nằm ở bit [3:1] của thanh ghi CONFIG
#define BMP388_IIR_OFF         0x00
#define BMP388_IIR_1           0x01
#define BMP388_IIR_3           0x02
#define BMP388_IIR_7           0x03

/* Cấu hình đang dùng: ODR 50 Hz, OSR áp suất x8.
 *
 * OSR phải ăn khớp với ODR, không chọn bừa được. Công thức thời gian đo của
 * datasheet (mục 3.9.2):
 *
 *     t = 234 + (392 + 2^osr_p * 2020) + (163 + 2^osr_t * 2020)   [us]
 *
 *     OSR_P x8, OSR_T x1 :  234 + 16552 + 2183 = 18969 us = 19.0 ms
 *
 * Chu kì ở 50 Hz là 20 ms nên vừa lọt. Đặt ODR nhanh hơn mà vẫn để OSR x8 thì
 * cảm biến không kịp đo và tự báo lỗi conf_err.
 *
 * Vì sao 50 Hz chứ không phải 200 Hz như bản teensy: ở đây chỉ cần giữ độ cao,
 * mà baro_task chạy 100 Hz nên 50 Hz đã dư. Đổi lại được OSR x8 thay vì x1,
 * nhiễu áp suất thấp hơn hẳn - với alt hold thì điều đó quan trọng hơn tốc độ.
 */
#define BARO_ODR_SETTING       BMP388_ODR_50HZ
#define BARO_OSR_PRESS         BMP388_OSR_X8
#define BARO_OSR_TEMP          BMP388_OSR_X1
#define BARO_IIR_SETTING       BMP388_IIR_3
#define BARO_SEA_LEVEL_PA      101325.0

static float baro_temp_c    = 0.0f;
static float baro_press_pa  = 0.0f;
static float baro_alt_abs_m = 0.0f;   // cao độ tuyệt đối so với mực nước biển
static float baro_alt_ref_m = 0.0f;   // cao độ đo được lúc khởi động
static bool  baro_ready     = false;

// Hệ số hiệu chỉnh riêng của từng con chip, đọc từ thanh ghi CALIB
struct BaroCalib {
  double t1, t2, t3;
  double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
};
static BaroCalib calib;


// ============================================================================
//  I2C mức thấp
// ============================================================================
static bool baro_write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BARO_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool baro_read_regs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(BARO_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start

  if (Wire.requestFrom((int)BARO_I2C_ADDR, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool baro_read_reg(uint8_t reg, uint8_t &val) {
  return baro_read_regs(reg, &val, 1);
}

static inline uint16_t u16le(const uint8_t *b) {
  return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}
static inline int16_t s16le(const uint8_t *b) {
  return (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}
static inline uint32_t u24le(const uint8_t *b) {
  return ((uint32_t)b[0]) | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
}

static uint8_t baro_sample_period_ms() {
  switch (BARO_ODR_SETTING) {
    case BMP388_ODR_200HZ: return 5;
    case BMP388_ODR_100HZ: return 10;
    case BMP388_ODR_50HZ:  return 20;
    case BMP388_ODR_25HZ:  return 40;
    default:               return 20;
  }
}


// ============================================================================
//  Đọc và giải mã hệ số hiệu chỉnh
// ============================================================================
static bool baro_read_calibration() {
  uint8_t buf[21];
  if (!baro_read_regs(BMP388_REG_CALIB, buf, sizeof(buf))) return false;

  uint16_t t1  = u16le(&buf[0]);
  uint16_t t2  = u16le(&buf[2]);
  int8_t   t3  = (int8_t)buf[4];
  int16_t  p1  = s16le(&buf[5]);
  int16_t  p2  = s16le(&buf[7]);
  int8_t   p3  = (int8_t)buf[9];
  int8_t   p4  = (int8_t)buf[10];
  uint16_t p5  = u16le(&buf[11]);
  uint16_t p6  = u16le(&buf[13]);
  int8_t   p7  = (int8_t)buf[15];
  int8_t   p8  = (int8_t)buf[16];
  int16_t  p9  = s16le(&buf[17]);
  int8_t   p10 = (int8_t)buf[19];
  int8_t   p11 = (int8_t)buf[20];

  calib.t1  = ((double)t1 / 0.00390625);
  calib.t2  = ((double)t2 / 1073741824.0);
  calib.t3  = ((double)t3 / 281474976710656.0);

  calib.p1  = ((double)(p1 - 16384) / 1048576.0);
  calib.p2  = ((double)(p2 - 16384) / 536870912.0);
  calib.p3  = ((double)p3 / 4294967296.0);
  calib.p4  = ((double)p4 / 137438953472.0);
  calib.p5  = ((double)p5 / 0.125);
  calib.p6  = ((double)p6 / 64.0);
  calib.p7  = ((double)p7 / 256.0);
  calib.p8  = ((double)p8 / 32768.0);
  calib.p9  = ((double)p9 / 281474976710656.0);
  calib.p10 = ((double)p10 / 281474976710656.0);
  calib.p11 = ((double)p11 / 36893488147419103232.0);

  return true;
}


// ============================================================================
//  Bù nhiệt và bù áp suất, theo đúng datasheet Bosch
// ============================================================================
static double baro_compensate_temp(uint32_t raw_temp) {
  double d1 = (double)raw_temp - calib.t1;
  double d2 = d1 * calib.t2;
  return d2 + (d1 * d1) * calib.t3;
}

static double baro_compensate_press(uint32_t raw_press, double temp_c) {
  double d1 = calib.p6 * temp_c;
  double d2 = calib.p7 * temp_c * temp_c;
  double d3 = calib.p8 * temp_c * temp_c * temp_c;
  double out1 = calib.p5 + d1 + d2 + d3;

  d1 = calib.p2 * temp_c;
  d2 = calib.p3 * temp_c * temp_c;
  d3 = calib.p4 * temp_c * temp_c * temp_c;
  double out2 = (double)raw_press * (calib.p1 + d1 + d2 + d3);

  d1 = (double)raw_press * (double)raw_press;
  d2 = calib.p9 + calib.p10 * temp_c;
  d3 = d1 * d2;
  double out3 = d3 + ((double)raw_press * (double)raw_press * (double)raw_press) * calib.p11;

  return out1 + out2 + out3;
}

static double baro_press_to_altitude_m(double press_pa) {
  return 44330.0 * (1.0 - pow(press_pa / BARO_SEA_LEVEL_PA, 0.1902949571836346));
}


// ============================================================================
//  Cấu hình cảm biến
// ============================================================================
static bool baro_wait_cmd_ready(uint32_t timeout_ms = 30) {
  uint32_t t0 = millis();
  uint8_t status = 0;
  while ((millis() - t0) < timeout_ms) {
    if (baro_read_reg(BMP388_REG_STATUS, status)) {
      if (status & BMP388_STATUS_CMD_RDY) return true;
    }
    delay(1);
  }
  return false;
}

static bool baro_configure() {
  if (!baro_write_reg(BMP388_REG_PWR_CTRL, 0x00)) return false;   // về sleep
  delay(2);

  uint8_t osr = (uint8_t)((BARO_OSR_TEMP << 3) | BARO_OSR_PRESS);
  if (!baro_write_reg(BMP388_REG_OSR, osr))                             return false;
  if (!baro_write_reg(BMP388_REG_ODR, BARO_ODR_SETTING))                return false;
  if (!baro_write_reg(BMP388_REG_CONFIG, (uint8_t)(BARO_IIR_SETTING << 1))) return false;

  uint8_t pwr = (uint8_t)(BMP388_PRESS_EN | BMP388_TEMP_EN | BMP388_MODE_NORMAL);
  if (!baro_write_reg(BMP388_REG_PWR_CTRL, pwr)) return false;
  delay(5);

  uint8_t err = 0;
  if (!baro_read_reg(BMP388_REG_ERR, err)) return false;
  if (err & 0x01) { Serial.println("BMP388 fatal_err"); return false; }
  if (err & 0x02) { Serial.println("BMP388 cmd_err");   return false; }
  if (err & 0x04) { Serial.println("BMP388 conf_err - OSR khong khop ODR"); return false; }

  return true;
}


// ============================================================================
//  Đọc một mẫu. Trả về false khi cảm biến CHƯA có mẫu mới.
//
//  Bắt buộc phải xét cờ drdy: baro_task chạy 100 Hz còn cảm biến chỉ 50 Hz,
//  đọc mù sẽ lấy trùng một mẫu hai lần. Mẫu trùng đi vào KF độ cao thì bộ lọc
//  tưởng đó là phép đo mới độc lập rồi thu hẹp sai số ước lượng sai cách.
//
//  STATUS (0x03) nằm ngay trước DATA (0x04..0x09) nên đọc gộp 7 byte trong
//  MỘT giao dịch I2C, không tốn thêm lần nào.
// ============================================================================
bool baro_update() {
  if (!baro_ready) return false;

  uint8_t buf[7];
  if (!baro_read_regs(BMP388_REG_STATUS, buf, 7)) return false;
  if (!(buf[0] & BMP388_STATUS_DRDY_PRESS))       return false;

  const uint8_t *data = &buf[1];
  uint32_t raw_press = u24le(&data[0]);
  uint32_t raw_temp  = u24le(&data[3]);
  if (raw_press == 0 && raw_temp == 0) return false;

  double temp_c   = baro_compensate_temp(raw_temp);
  double press_pa = baro_compensate_press(raw_press, temp_c);

  baro_temp_c    = (float)temp_c;
  baro_press_pa  = (float)press_pa;
  baro_alt_abs_m = (float)baro_press_to_altitude_m(press_pa);
  return true;
}


// Đo cao độ gốc lúc khởi động để sau này quy về "cao hơn điểm cất cánh"
static void baro_measure_reference(uint16_t samples) {
  if (!baro_ready) return;

  double sum = 0.0;
  uint8_t period = baro_sample_period_ms();

  for (uint8_t i = 0; i < 10; i++) { baro_update(); delay(period + 2); }

  uint16_t got = 0;
  for (uint16_t i = 0; i < samples * 3 && got < samples; i++) {
    if (baro_update()) { sum += baro_alt_abs_m; got++; }
    delay(period / 2 + 1);
  }
  if (got == 0) return;

  baro_alt_ref_m = (float)(sum / got);
  Serial.print("Cao do goc = ");
  Serial.println(baro_alt_ref_m, 3);
}


static bool baro_detect() {
  uint8_t chip_id = 0;
  // Đọc lại ba lần, đòi cả ba đều đúng 0x50 để chắc chắn đúng con chip
  for (uint8_t i = 0; i < 3; i++) {
    if (!baro_read_reg(BMP388_REG_CHIP_ID, chip_id)) return false;
    if (chip_id != BMP388_CHIP_ID_VALUE)             return false;
    delay(1);
  }
  return true;
}


bool baro_init() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  delay(10);

  baro_ready = false;

  if (!baro_detect()) {
    Serial.println("BMP388 khong tim thay - kiem tra day I2C va dia chi");
    return false;
  }

  if (!baro_write_reg(BMP388_REG_CMD, BMP388_CMD_SOFT_RESET)) {
    Serial.println("BMP388 reset that bai");
    return false;
  }
  delay(10);

  if (!baro_wait_cmd_ready(30)) { Serial.println("BMP388 chua san sang nhan lenh"); return false; }
  if (!baro_read_calibration()) { Serial.println("BMP388 doc he so hieu chinh that bai"); return false; }
  if (!baro_configure())        { Serial.println("BMP388 cau hinh that bai"); return false; }

  baro_ready = true;

  for (uint8_t i = 0; i < 5; i++) { baro_update(); delay(baro_sample_period_ms() + 2); }
  baro_measure_reference(50);
  return true;
}


float baro_get_altitude_m() { return baro_alt_abs_m - baro_alt_ref_m; }

bool baro_is_present() {
  uint8_t chip_id = 0;
  return (baro_read_reg(BMP388_REG_CHIP_ID, chip_id) && chip_id == BMP388_CHIP_ID_VALUE);
}
