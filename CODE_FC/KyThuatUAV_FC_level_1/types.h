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
#ifndef KYTHUATUAV_TYPES_H
#define KYTHUATUAV_TYPES_H

// Các kiểu dữ liệu dùng chung phải nằm trong header, không để trong file .ino.
// Lý do: Arduino tự sinh prototype cho mọi hàm rồi chèn lên đầu sketch. Nếu
// một hàm có tham số kiểu struct mà struct đó khai báo trong .ino, prototype
// sẽ xuất hiện trước phần khai báo struct và trình biên dịch báo lỗi
// "was not declared in this scope". Đặt trong .h thì #include chạy trước,
// nên kiểu luôn được biết trước prototype.

// Trạng thái nội bộ của một bộ điều khiển PID
struct PidState {
  float prev_error;
  float integral;
};

// Cảm biến nào có mặt lúc khởi động
struct SensorPresent {
  bool imu;
  bool baro;
};

#endif
