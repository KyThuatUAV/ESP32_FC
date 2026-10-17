# LacWing — mạch bay ESP32 mã nguồn mở

Mã nguồn mở của một bộ điều khiển bay (flight controller) cho drone bốn cánh,
chạy trên ESP32. Kèm tài liệu hướng dẫn từ lúc cài phần mềm tới lúc bay thử.

Dành cho sinh viên và người tự học. Miễn phí, không điều kiện.

LacWing là dòng bộ điều khiển bay của Kỹ Thuật UAV.

## 💬 Nhóm cộng đồng — Kỹ thuật UAV & LacWing

Chỗ để hỏi khi bị kẹt, khoe mạch đã bay được, và hỗ trợ lẫn nhau. Vào tự do.

**[Bấm vào đây để tham gia](https://zalo.me/g/82dzphstzd1gbarwtckt)** — hoặc mở Zalo quét mã:

<img src="Tài%20liệu%20kèm%20theo/nhom_zalo.png" alt="Mã QR nhóm Zalo Kỹ thuật UAV và LacWing" width="250">

Mở từ TikTok thì bấm link sẽ không vào được app Zalo — quét mã QR hoặc mở link
bằng trình duyệt.

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
6. Mở `CODE_FC/LacWing_FC_level_1/LacWing_FC_level_1.ino`

> ⚠️ **Phải đúng core 2.0.17.** Từ bản 3.x, Espressif đổi hoàn toàn cách gọi hàm
> xuất xung PWM, cài nhầm là biên dịch báo `'ledcSetup' was not declared in this scope`.

Toàn bộ thiết lập — chân cắm, kênh điều khiển, hệ số PID, bù gia tốc kế — nằm
gọn trong **một khối duy nhất** ở đầu file `.ino` chính.

---

## Cấu trúc thư mục

```
CODE_FC/LacWing_FC_level_1/   firmware
├── LacWing_FC_level_1.ino    khối cấu hình + setup + các tác vụ RTOS
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

Ai muốn đi nhanh hơn thì có khoá học, đi từ số 0, học **1 kèm 1**:

| Bậc | Đầu ra | Học phí |
|---|---|---|
| **1** | Drone hover ổn định, firmware do học viên tự viết | **4.000.000 đ**<br>8 buổi |
| **2** | Giữ độ cao và giữ vị trí, buông tay không trôi | **6.000.000 đ**<br>12 buổi |
| **3** | Mạch ổn định, đủ làm nền cho nghiên cứu khoa học | Liên hệ |
| **4** | Bay tự hành không GPS, tự về điểm xuất phát | Liên hệ |

Nghiệm thu xong bậc trước mới lên bậc sau. Dừng ở bậc nào cũng được — máy vẫn bay.

Chi phí linh kiện, học viên tự mua: bậc 1 khoảng **6.095.000 đ**, bậc 2 thêm
**880.000 đ**, bậc 3 thêm **4.200.000 đ**.

Firmware chạy trong mạch là thứ **học viên tự viết ra trong khoá**, không phải
phần mềm mua sẵn rồi học cách bấm nút. Linh kiện học viên tự mua theo danh sách
công khai, không mua qua mình; PCB thì được tặng.

Học trực tiếp tại TP.HCM, hoặc online cho các tỉnh thành khác. Hơn 50 học viên
từ nhiều trường đại học trên toàn quốc.

Khoá học là để đi nhanh, an toàn và xa hơn — không phải để mở khoá thứ gì.

## Thành tích của người đứng lớp

Đây là thành tích cá nhân của người hướng dẫn, không phải của riêng bộ mã nguồn
này. Mạch LacWing chỉ được dùng ở hai cuộc có dấu ★; các giải còn lại bay bằng
nền tảng khác.

| Năm | Giải |
|---|---|
| 2026 | ★ **Vô địch** cuộc thi UAV, Học viện Hàng không Việt Nam — đội do mình huấn luyện, bay bằng mạch và mã nguồn tự phát triển |
| 2026 | ★ **Đội duy nhất tự phát triển flight controller** — bay tự hành bằng UWB, không dùng GPS. UAV Contest lần 3, ĐH Công nghệ Kỹ thuật TP.HCM |
| 2025 | **Á quân UAV Contest lần 2** — cuộc thi quốc tế tại Ostrava, Cộng hoà Séc |
| 2025 | **Vô địch nội dung bay tự động** — cuộc thi UAV, Học viện Hàng không Việt Nam |
| 2025 | **Top 8 toàn quốc** — trưởng nhóm đội AERO-V, UAV Cup PV GAS |
| 2024 | **Giải Sáng tạo** — đội duy nhất trong 9 đội bay tự động thành công, UAV Contest lần 1, ĐH Tôn Đức Thắng |

## Liên hệ

**Nhóm cộng đồng Zalo — Kỹ thuật UAV & LacWing:** https://zalo.me/g/82dzphstzd1gbarwtckt
Hỏi đáp, gỡ lỗi, chia sẻ kinh nghiệm giữa những người cùng làm. Đây là nơi nên
hỏi trước, vì nhiều người thấy câu hỏi thì khả năng được trả lời nhanh hơn.

**Nguyễn Văn Quý** — Kỹ Thuật UAV
📞 0817 550 271 (gọi hoặc Zalo)

Lỗi trong mã nguồn thì mở [Issue](../../issues) trên GitHub — để người sau gặp
cùng lỗi còn đọc được câu trả lời.
