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
//  Kalman 2 trạng thái ước lượng độ cao và tốc độ thẳng đứng.
//
//     trạng thái   S = [ z (cm) , vz (cm/s) ]'
//     đầu vào      u = gia tốc thẳng đứng (cm/s^2), từ IMU
//     phép đo      M = độ cao từ baro (cm)
//
//  Giữ nguyên dạng ma trận và bộ tham số của bản virex v14, chạy đúng nhịp
//  2ms của vòng điều khiển.
//
//  ------------------------------------------------------------------------
//  QUAN TRỌNG - vì sao gọi CẢ predict LẪN correct ở MỖI vòng 500Hz,
//  kể cả khi baro chưa có mẫu mới:
//
//  Bộ tham số này (sigma gia tốc = 4 cm/s^2, R = 30^2) cho ma trận P mọc rất
//  chậm, nên độ lợi Kalman K rất nhỏ. Nó chỉ kéo được ước lượng bám theo baro
//  nhờ ĐƯỢC ÁP DỤNG 500 LẦN MỖI GIÂY - cộng dồn lại mới đủ lực.
//
//  Từng thử đổi sang "chỉ correct khi baro có mẫu mới" (~50Hz) cho đúng lý
//  thuyết hơn. Kết quả: mất 9/10 số lần hiệu chỉnh mà K vẫn nhỏ như cũ, phần
//  kéo về gần như biến mất, vz thành tích phân gia tốc thuần và TRÔI VÔ HẠN.
//
//  Muốn làm bản đúng lý thuyết thì phải tăng sigma gia tốc lên cho P mọc
//  nhanh tương ứng (cỡ 3 lần khi giảm 10 lần số nhịp correct), rồi bay thử
//  lại từ đầu. Chưa kiểm chứng trên phần cứng thì ĐỪNG đổi.
//  ------------------------------------------------------------------------
// ============================================================================

#include <BasicLinearAlgebra.h>
using namespace BLA;

// sigma của nhiễu gia tốc (cm/s^2) và của phép đo baro (cm)
#define KF_SIGMA_ACC_CMS2    4.0f
#define KF_SIGMA_BARO_CM    30.0f

BLA::Matrix<2,2> Fz;      // ma trận chuyển trạng thái
BLA::Matrix<2,1> Gz;      // ma trận đầu vào
BLA::Matrix<2,2> Pz;      // hiệp phương sai sai số ước lượng
BLA::Matrix<2,2> Qz;      // hiệp phương sai nhiễu quá trình
BLA::Matrix<2,1> Sz;      // vector trạng thái
BLA::Matrix<1,2> Hz;      // ma trận quan sát
BLA::Matrix<2,2> Iz;      // ma trận đơn vị
BLA::Matrix<1,1> Acczz;   // đầu vào: gia tốc thẳng đứng
BLA::Matrix<2,1> Kz;      // độ lợi Kalman
BLA::Matrix<1,1> Rz;      // hiệp phương sai nhiễu đo
BLA::Matrix<1,1> Lz;      // hiệp phương sai phần dư
BLA::Matrix<1,1> Mz;      // phép đo

static float kf_alt_cm   = 0.0f;
static float kf_climb_cms = 0.0f;


void alt_kf_init() {
  Fz = {1, DT_CTRL,
        0, 1};
  Gz = {0.5f * DT_CTRL * DT_CTRL,
        DT_CTRL};
  Hz = {1, 0};
  Iz = {1, 0,
        0, 1};
  Qz = Gz * ~Gz * KF_SIGMA_ACC_CMS2 * KF_SIGMA_ACC_CMS2;
  Rz = {KF_SIGMA_BARO_CM * KF_SIGMA_BARO_CM};
  Pz = {0, 0,
        0, 0};
  Sz = {0,
        0};

  kf_alt_cm = 0.0f;
  kf_climb_cms = 0.0f;
}


// Gọi MỖI vòng điều khiển.
//   alt_meas_m     : độ cao baro, đơn vị MÉT (hàm tự đổi sang cm)
//   acc_earth_z_g  : gia tốc thẳng đứng đã trừ trọng lực, đơn vị g
void alt_kf_update(float alt_meas_m, float acc_earth_z_g) {
  Acczz = {acc_earth_z_g * 981.0f};        // g -> cm/s^2

  Sz = Fz * Sz + Gz * Acczz;               // dự đoán trạng thái
  Pz = Fz * Pz * ~Fz + Qz;                 // dự đoán hiệp phương sai

  Lz = Hz * Pz * ~Hz + Rz;                 // hiệp phương sai phần dư

  // Độ lợi Kalman: K = P·H'·L⁻¹
  // Bản virex v14 viết Invert(Lz), nhưng đó là cú pháp của BasicLinearAlgebra
  // 3.x. Từ bản 4.x, Invert() đảo ma trận TẠI CHỖ và trả về bool, nên viết như
  // cũ sẽ không biên dịch được. Lz là ma trận 1x1 nên nghịch đảo của nó chính
  // là nghịch đảo số học - viết thẳng vừa đúng, vừa không phụ thuộc phiên bản
  // thư viện, lại nhanh hơn.
  float Lz_scalar = Lz(0, 0);
  if (Lz_scalar < 1e-6f) return;           // chặn chia cho 0
  Kz = Pz * ~Hz * (1.0f / Lz_scalar);

  Mz = {alt_meas_m * 100.0f};              // m -> cm
  Sz = Sz + Kz * (Mz - Hz * Sz);           // hiệu chỉnh theo phép đo
  Pz = (Iz - Kz * Hz) * Pz;

  kf_alt_cm    = Sz(0, 0);
  kf_climb_cms = Sz(1, 0);
}


float alt_kf_get_altitude_cm() { return kf_alt_cm;    }
float alt_kf_get_climb_cms()   { return kf_climb_cms; }
