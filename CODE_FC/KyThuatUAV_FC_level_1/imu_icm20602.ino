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
//  ICM20602 - driver thanh ghi qua SPI, kèm bộ lọc Kalman 1D ước lượng góc.
//
//  Góc roll/pitch lấy từ Kalman 1D: gyro cho phần biến thiên nhanh, gia tốc kế
//  kéo về cho khỏi trôi. Bản đầy đủ dùng thêm Madgwick/Mahony để có góc yaw từ
//  la bàn, ở đây bỏ vì angle và alt hold không cần giữ hướng - yaw chỉ điều
//  khiển theo tốc độ xoay.
//
//  GHI CÔNG: cách ước lượng góc bằng bộ lọc Kalman một chiều ở hàm
//  imu_kalman_1d() học từ tài liệu của Carbon Aeronautics
//  (https://github.com/CarbonAeronautics). Chi tiết xem file LOI_NHAN_VA_GHI_CONG.md.
// ============================================================================

#include <SPI.h>

#define IMU_SPI_HZ   10000000     // 10 MHz

// Địa chỉ thanh ghi
#define REG_WHO_AM_I       0x75
#define REG_PWR_MGMT_1     0x6B
#define REG_ACCEL_XOUT_H   0x3B
#define REG_GYRO_XOUT_H    0x43
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_CONFIG         0x1A
#define REG_ACCEL_CONFIG2  0x1D

#define ICM20602_WHO_AM_I_VALUE  0x12

// Hệ số quy đổi số thô sang đơn vị vật lý
#define GYRO_LSB_PER_DPS   16.4f     // thang do +-2000 do/giay
#define ACC_LSB_PER_G      2048.0f   // thang do +-16g

static SPISettings imu_spi(IMU_SPI_HZ, MSBFIRST, SPI_MODE0);

// Số đo thô đã quy đổi, CHƯA trừ bias
static float imu_acc_x_g, imu_acc_y_g, imu_acc_z_g;
static float imu_rate_roll_dps, imu_rate_pitch_dps, imu_rate_yaw_dps;

// Bias gyro đo được lúc khởi động
static float gyro_bias_roll_dps, gyro_bias_pitch_dps, gyro_bias_yaw_dps;

// Góc suy ra từ riêng gia tốc kế, dùng làm phép đo cho Kalman
static float acc_roll_deg, acc_pitch_deg;

// Kết quả ước lượng góc
static float att_roll_deg  = 0.0f, att_roll_var  = 2 * 2;
static float att_pitch_deg = 0.0f, att_pitch_var = 2 * 2;


// Chỉ mở bus, chưa đụng gì tới cảm biến. Tách riêng để setup() kiểm tra
// WHO_AM_I trước khi bước vào vòng hiệu chỉnh gyro.
void imu_init_bus() {
  SPI.begin();
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);
  pinMode(PIN_IMU_LED, OUTPUT);
  digitalWrite(PIN_IMU_LED, LOW);
}

bool imu_is_present() {
  return (imu_read_reg(REG_WHO_AM_I) == ICM20602_WHO_AM_I_VALUE);
}


void imu_init() {
  imu_write_reg(REG_PWR_MGMT_1, 0x00);          // thoát chế độ ngủ
  delay(100);
  imu_write_reg(REG_ACCEL_CONFIG,  0x18);       // thang đo gia tốc +-16g
  imu_write_reg(REG_GYRO_CONFIG,   0x18);       // thang đo gyro +-2000 độ/giây
  imu_write_reg(REG_CONFIG,        IMU_DLPF_GYRO);
  imu_write_reg(REG_ACCEL_CONFIG2, IMU_DLPF_ACC);
  delay(100);

  Serial.println("[ICM20602 khoi tao xong]");
  Serial.print("CONFIG 0x");        Serial.println(imu_read_reg(REG_CONFIG), HEX);
  Serial.print("ACCEL_CONFIG2 0x"); Serial.println(imu_read_reg(REG_ACCEL_CONFIG2), HEX);

#ifdef CALIBRATE_ACCEL
  imu_calibrate_accel();      // hàm này không bao giờ trả về
#endif

  imu_calibrate_gyro();
}


#ifdef CALIBRATE_ACCEL
// ===== Hiệu chỉnh gia tốc kế =====
// Chỉ chạy khi bật #define CALIBRATE_ACCEL. Đo xong thì in ra đúng ba dòng cần
// dán vào khối CẤU HÌNH rồi dừng hẳn - cố tình không cho bay ở chế độ này.
//
// Nguyên lý: đặt drone nằm phẳng thì đúng ra gia tốc kế phải đọc
// X = 0, Y = 0, Z = +1g. Lệch bao nhiêu so với ba số đó chính là sai số cần bù.
void imu_calibrate_accel() {
  const int SAMPLES = 2000;          // 2000 mẫu x 2ms = 4 giây

  Serial.println();
  Serial.println("=========================================");
  Serial.println("     HIEU CHINH GIA TOC KE");
  Serial.println("=========================================");
  Serial.println("Dat drone nam phang tren mat ban phang.");
  Serial.println("Bo tay ra, dung cham vao ban trong luc do.");

  for (int s = 5; s > 0; s--) {
    Serial.printf("Bat dau sau %d giay...\n", s);
    delay(1000);
  }
  Serial.println("Dang do, giu yen...");

  double sum_x = 0, sum_y = 0, sum_z = 0;
  for (int i = 0; i < SAMPLES; i++) {
    imu_read_raw();
    sum_x += imu_acc_x_g;            // lấy số THÔ, chưa cộng offset cũ
    sum_y += imu_acc_y_g;
    sum_z += imu_acc_z_g;
    if ((i % 500) == 0) digitalWrite(PIN_IMU_LED, !digitalRead(PIN_IMU_LED));
    delay(2);
  }
  digitalWrite(PIN_IMU_LED, LOW);

  float mean_x = sum_x / SAMPLES;
  float mean_y = sum_y / SAMPLES;
  float mean_z = sum_z / SAMPLES;

  Serial.println();
  Serial.printf("Do duoc:  X = %+.4f   Y = %+.4f   Z = %+.4f  (g)\n",
                mean_x, mean_y, mean_z);
  Serial.println();

  // Kiểm tra tư thế trước khi tin kết quả
  if (mean_z < 0.80f) {
    Serial.println("!! LOI: Z phai gan +1.00 g khi drone nam phang.");
    Serial.println("   Z am  -> drone dang bi lat nguoc.");
    Serial.println("   Z nho -> drone dang dung nghieng.");
    Serial.println("   Dat lai cho dung roi khoi dong lai mach.");
  } else if (fabsf(mean_x) > 0.30f || fabsf(mean_y) > 0.30f) {
    Serial.println("!! LOI: X hoac Y lech qua nhieu (>0.30 g).");
    Serial.println("   Mat ban khong phang, hoac cam bien gan lech tren mach.");
    Serial.println("   Kiem tra lai roi khoi dong lai mach.");
  } else {
    Serial.println("Chep DUNG ba dong duoi day, thay vao khoi CAU HINH");
    Serial.println("trong file KyThuatUAV_FC_level_1.ino:");
    Serial.println();
    Serial.printf("#define ACC_OFFSET_X_G   %+.4ff\n", -mean_x);
    Serial.printf("#define ACC_OFFSET_Y_G   %+.4ff\n", -mean_y);
    Serial.printf("#define ACC_OFFSET_Z_G   %+.4ff\n", 1.0f - mean_z);
    Serial.println();
    Serial.println("Xong thi TAT #define CALIBRATE_ACCEL di roi nap lai.");
    Serial.println("Muon kiem tra: bat DEBUG_ACC_OFFSET, phai thay X~0 Y~0 Z~1.");
  }

  Serial.println("=========================================");
  while (1) {                        // dừng hẳn, không cho bay ở chế độ cali
    digitalWrite(PIN_IMU_LED, !digitalRead(PIN_IMU_LED));
    delay(500);
  }
}
#endif


// ===== Hiệu chỉnh gyro: lặp lại tới khi drone thật sự đứng yên =====
// So tổng của đợt này với đợt trước, lệch còn nhỏ mới nhận. Nhờ vậy cắm nguồn
// lúc tay còn cầm drone thì nó tự đo lại chứ không nhận bias sai.
void imu_calibrate_gyro() {
  const int   SAMPLES        = 500;
  const float STABLE_LIMIT   = 40.0f;   // ngưỡng lệch giữa hai đợt liên tiếp

  float sum_roll = 0, sum_pitch = 0, sum_yaw = 0;
  float prev_roll = 0, prev_pitch = 0, prev_yaw = 0;

  Serial.println("bat dau hieu chinh gyro");

  while (1) {
    prev_roll = sum_roll;  prev_pitch = sum_pitch;  prev_yaw = sum_yaw;
    sum_roll = 0;  sum_pitch = 0;  sum_yaw = 0;

    digitalWrite(PIN_IMU_LED, HIGH);
    for (int i = 0; i < SAMPLES; i++) {
      imu_read_raw();
      sum_roll  += imu_rate_roll_dps;
      sum_pitch += imu_rate_pitch_dps;
      sum_yaw   += imu_rate_yaw_dps;
      delayMicroseconds(10);
    }

    Serial.println("Dang hieu chinh gyro - Khong di chuyen Drone!");

    if (fabsf(sum_roll  - prev_roll)  < STABLE_LIMIT &&
        fabsf(sum_pitch - prev_pitch) < STABLE_LIMIT &&
        fabsf(sum_yaw   - prev_yaw)   < STABLE_LIMIT) {
      Serial.println("cali gyro thanh cong");
      digitalWrite(PIN_IMU_LED, LOW);
      break;
    }
  }

  gyro_bias_roll_dps  = sum_roll  / SAMPLES;
  gyro_bias_pitch_dps = sum_pitch / SAMPLES;
  gyro_bias_yaw_dps   = sum_yaw   / SAMPLES;

  Serial.print("bias gyro (do/giay) ");
  Serial.print(gyro_bias_roll_dps);  Serial.print(" | ");
  Serial.print(gyro_bias_pitch_dps); Serial.print(" | ");
  Serial.println(gyro_bias_yaw_dps);
}


// Gọi mỗi vòng điều khiển: đọc cảm biến rồi cập nhật ước lượng góc.
void imu_update() {
  imu_read_raw();
  imu_kalman_1d(att_roll_deg,  att_roll_var,
                imu_rate_roll_dps  - gyro_bias_roll_dps,  acc_roll_deg);
  imu_kalman_1d(att_pitch_deg, att_pitch_var,
                imu_rate_pitch_dps - gyro_bias_pitch_dps, acc_pitch_deg);
}


void imu_read_raw() {
  int16_t ax_lsb, ay_lsb, az_lsb;
  imu_read_accel_raw(ax_lsb, ay_lsb, az_lsb);

  int16_t gx_lsb, gy_lsb, gz_lsb;
  imu_read_gyro_raw(gx_lsb, gy_lsb, gz_lsb);

  imu_rate_roll_dps  = (float)gx_lsb / GYRO_LSB_PER_DPS;
  imu_rate_pitch_dps = (float)gy_lsb / GYRO_LSB_PER_DPS;
  imu_rate_yaw_dps   = (float)gz_lsb / GYRO_LSB_PER_DPS;

  imu_acc_x_g = (float)ax_lsb / ACC_LSB_PER_G;
  imu_acc_y_g = (float)ay_lsb / ACC_LSB_PER_G;
  imu_acc_z_g = (float)az_lsb / ACC_LSB_PER_G;

  // Góc từ gia tốc kế, tính trên giá trị đã bù lệch.
  // Roll chủ yếu từ acc_Y và acc_Z, pitch chủ yếu từ acc_X và acc_Z.
  float ax = imu_acc_x_g + ACC_OFFSET_X_G;
  float ay = imu_acc_y_g + ACC_OFFSET_Y_G;
  float az = imu_acc_z_g + ACC_OFFSET_Z_G;

  acc_roll_deg  =  atanf(ay / sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
  acc_pitch_deg = -atanf(ax / sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}


// Kalman 1 chiều: gyro là đầu vào dự đoán, góc từ gia tốc kế là phép đo.
// var_process = 1 (độ/giây)^2, var_measure = 3^2 độ^2.
void imu_kalman_1d(float &state_deg, float &variance,
                   float rate_dps, float measure_deg) {
  state_deg += DT_CTRL * rate_dps;
  variance  += DT_CTRL * DT_CTRL * 1.0f * 1.0f;

  float gain = variance / (variance + 3.0f * 3.0f);
  state_deg += gain * (measure_deg - state_deg);
  variance   = (1.0f - gain) * variance;
}


// ---- SPI mức thấp -----------------------------------------------------------
void imu_write_reg(uint8_t reg, uint8_t value) {
  digitalWrite(PIN_IMU_CS, LOW);
  SPI.beginTransaction(imu_spi);
  SPI.transfer(reg & 0x7F);          // bit7 = 0 -> ghi
  SPI.transfer(value);
  SPI.endTransaction();
  digitalWrite(PIN_IMU_CS, HIGH);
}

uint8_t imu_read_reg(uint8_t reg) {
  digitalWrite(PIN_IMU_CS, LOW);
  SPI.beginTransaction(imu_spi);
  SPI.transfer(reg | 0x80);          // bit7 = 1 -> đọc
  uint8_t value = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(PIN_IMU_CS, HIGH);
  return value;
}

void imu_read_accel_raw(int16_t &x, int16_t &y, int16_t &z) {
  digitalWrite(PIN_IMU_CS, LOW);
  SPI.beginTransaction(imu_spi);
  SPI.transfer(REG_ACCEL_XOUT_H | 0x80);
  x = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  y = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  z = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(PIN_IMU_CS, HIGH);
}

void imu_read_gyro_raw(int16_t &x, int16_t &y, int16_t &z) {
  digitalWrite(PIN_IMU_CS, LOW);
  SPI.beginTransaction(imu_spi);
  SPI.transfer(REG_GYRO_XOUT_H | 0x80);
  x = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  y = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  z = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(PIN_IMU_CS, HIGH);
}

// ---- Các hàm lấy dữ liệu ----------------------------------------------------
float imu_get_acc_x_g() { return imu_acc_x_g + ACC_OFFSET_X_G; }
float imu_get_acc_y_g() { return imu_acc_y_g + ACC_OFFSET_Y_G; }
float imu_get_acc_z_g() { return imu_acc_z_g + ACC_OFFSET_Z_G; }

float imu_get_rate_roll_dps()  { return imu_rate_roll_dps  - gyro_bias_roll_dps;  }
float imu_get_rate_pitch_dps() { return imu_rate_pitch_dps - gyro_bias_pitch_dps; }
float imu_get_rate_yaw_dps()   { return imu_rate_yaw_dps   - gyro_bias_yaw_dps;   }

float imu_get_roll_deg()  { return att_roll_deg;  }
float imu_get_pitch_deg() { return att_pitch_deg; }
