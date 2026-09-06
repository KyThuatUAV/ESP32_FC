# Mạch bay ESP32 — Kỹ Thuật UAV

Mã nguồn mở của một bộ điều khiển bay (flight controller) cho drone bốn cánh,
chạy trên ESP32. Kèm tài liệu hướng dẫn từ lúc cài phần mềm tới lúc bay thử.

Dành cho sinh viên và người tự học. Miễn phí, không điều kiện.

---

## Phần cứng

| Thành phần | Linh kiện | Giao tiếp |
|---|---|---|
| Vi điều khiển | ESP32-WROOM-DA | — |
| Cảm biến quán tính | ICM-20602 | SPI, 10 MHz |
| Cảm biến áp suất | BMP388 | I2C |
| Bộ thu điều khiển | Bất kỳ loại nào ra SBUS | UART2, tín hiệu đảo |
| Động cơ | 4 ESC, xung PWM chuẩn | LEDC 391 Hz |

## Firmware làm được gì

- **Chế độ ANGLE** — cần gạt quy định góc nghiêng, thả cần thì drone tự về bằng.
- **Chế độ ALT HOLD** — cần ga quy định tốc độ lên xuống, để giữa thì giữ độ cao.
- **Tự hạ cánh khi mất sóng** — giữ thăng bằng, hạ đều 60 cm/s, chạm đất tự tắt máy.
- **Hiệu chỉnh gyro tự kiểm tra** — lặp lại tới khi drone thật sự đứng yên mới nhận.
- **Chế độ hiệu chỉnh gia tốc kế** — đo xong in ra sẵn ba dòng để dán vào code.

Chạy trên FreeRTOS, chia hai nhân của ESP32:

| Tác vụ | Nhân | Nhịp | Việc |
|---|---|---|---|
| `control_task` | 1 | 500 Hz | IMU, ước lượng góc, Kalman độ cao, PID, xuất ESC |
| `rc_task` | 0 | 500 Hz | Giải mã SBUS |
| `baro_task` | 0 | 100 Hz | Đọc BMP388, in dữ liệu gỡ lỗi |

Nhân 1 chỉ chạy vòng điều khiển, không chia sẻ với việc gì khác.

---

## Bắt đầu từ đâu

Đọc **[Tài liệu kèm theo/HUONG_DAN_CAI_DAT.md](Tài%20liệu%20kèm%20theo/HUONG_DAN_CAI_DAT.md)**.

Tóm tắt cho người đã quen Arduino:

1. Arduino IDE 2.x
2. Board Manager URL: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. Cài gói **esp32 phiên bản 2.0.17** — **không phải bản mới nhất**
4. Cài thư viện `BasicLinearAlgebra`
5. Board: **ESP32-WROOM-DA Module**
6. Mở `CODE_FC/KyThuatUAV_FC_level_1/KyThuatUAV_FC_level_1.ino`

> ⚠️ **Phải đúng core 2.0.17.** Từ bản 3.x, Espressif đổi hoàn toàn cách gọi hàm
> xuất xung PWM, cài nhầm là biên dịch báo `'ledcSetup' was not declared in this scope`.

Toàn bộ thiết lập — chân cắm, kênh điều khiển, hệ số PID, bù gia tốc kế — nằm
gọn trong **một khối duy nhất** ở đầu file `.ino` chính.

---

## Cấu trúc thư mục

```
CODE_FC/KyThuatUAV_FC_level_1/   firmware
├── KyThuatUAV_FC_level_1.ino    khối cấu hình + setup + các tác vụ RTOS
├── control_task.ino             vòng điều khiển chính, các chế độ bay
├── imu_icm20602.ino             driver IMU + ước lượng góc + hiệu chỉnh
├── baro_bmp388.ino              driver áp suất, viết bằng thanh ghi
├── altitude_kf.ino              Kalman 2 trạng thái cho độ cao
├── pid.ino                      PID xếp tầng
├── rc_sbus.ino                  giải mã SBUS
├── esc_output.ino               xuất xung 4 ESC
└── types.h                      kiểu dữ liệu dùng chung

Tài liệu kèm theo/               hướng dẫn cài đặt, sơ đồ lắp, danh sách linh kiện
driver esp32/                    driver USB CP210x cho Windows
```

---

## An toàn

Firmware biên dịch được **không có nghĩa là bay được**. Trước khi lắp cánh, làm
đủ ba bước kiểm tra trong phần cuối tài liệu hướng dẫn:

1. Kiểm tra các kênh tay điều khiển đúng thứ tự
2. Kiểm tra thứ tự motor 1 → 2 → 3 → 4
3. Kiểm tra dấu của cảm biến và cần gạt **cùng chiều**

**Luôn tháo cánh quạt** khi thử nghiệm trên bàn.

---

## Đã bay nghiệm thu

Bộ hệ số PID mặc định trong code đã bay thật trên cấu hình:

| | |
|---|---|
| Khung | 5 inch |
| Động cơ | 2204, 2300KV |
| Cánh | 5 inch |
| Pin | 3S 2200mAh |

Khung của bạn khác nhiều so với cấu hình này — nặng hơn, cánh to hơn, pin nhiều
cell hơn — thì nhiều khả năng phải chỉnh lại PID. Đó là chuyện bình thường, không
phải code sai.

Hướng chỉnh khi khung khác:

- **Khung to và nặng hơn** → giảm `KP_RATE_ROLL` và `KP_RATE_PITCH`. Khung càng
  nặng thì phản ứng càng chậm, giữ hệ số cao sẽ dao động.
- **Nghe motor rít tần số cao** → giảm `KD_RATE_ROLL` và `KD_RATE_PITCH`.
- **Lắc chậm biên độ lớn** → giảm `KP_RATE_*` trước, chưa hết thì giảm `KI_RATE_*`.

Lần bay đầu tiên luôn buộc dây và bay thấp, dù cấu hình có giống hệt.

[GPL-3.0](LICENSE).

Bạn được tự do dùng, sửa, và chia sẻ lại. Điều kiện: bản phát hành lại phải mở
mã nguồn và giữ nguyên phần ghi công.

## Ghi công

Bộ mã nguồn này học từ những người đi trước, và có ghi rõ học cái gì từ ai.
Xem **[Tài liệu kèm theo/LOI_NHAN_VA_GHI_CONG.md](Tài%20liệu%20kèm%20theo/LOI_NHAN_VA_GHI_CONG.md)**.

Trong đó cũng nói vì sao dự án này được công khai, và ba điều mong bạn giữ khi
dùng lại.

---

## Khoá học

Bộ mã nguồn ở đây là **bậc đầu tiên**: bay cân bằng. Miễn phí, đầy đủ, và tài
liệu viết để bạn tự đi hết được một mình. **Không cần học khoá nào cũng dùng
được trọn bộ này.**

Ai muốn đi nhanh hơn thì có khoá học, đi từ số 0, kèm 1-1:

| Bậc | Nội dung |
|---|---|
| 1 | **Lập trình bay cân bằng** — ước lượng góc, PID, mixer |
| 2 | **Lập trình giữ vị trí** — baro, lidar, optical flow, hợp nhất cảm biến |
| 3 | **Lập trình đa luồng và xử lý nhiễu** — làm drone hết rung, motor hết nóng, chỉnh PID sâu hơn mà không bị rít |
| 4 | **Lập trình hệ định vị và tự hành** |

Học trực tiếp tại TP.HCM, hoặc online cho các tỉnh thành khác.

Khoá học là để đi nhanh, an toàn và xa hơn — không phải để mở khoá thứ gì.

## Liên hệ

**Nguyễn Văn Quý** — Kỹ Thuật UAV
📞 0817 550 271 (gọi hoặc Zalo)

Lỗi trong mã nguồn thì mở [Issue](../../issues) trên GitHub — để người sau gặp
cùng lỗi còn đọc được câu trả lời.
