# Hướng dẫn cài đặt môi trường

Tài liệu này hướng dẫn cài đặt để biên dịch và nạp firmware cho mạch bay
ESP32 + ICM20602 + BMP388.

Trong bộ mã nguồn có hai sketch:

| Thư mục | Nội dung | Tốc độ Serial |
|---|---|---|
| `Fc_basic_for_KyThuatUAV` | Bản cơ bản, một vòng lặp, chỉ bay chế độ Angle | 115200 |
| `KyThuatUAV_FC_level_1` | Bản FreeRTOS, hai chế độ Angle và Alt Hold | 500000 |

Cả hai đều nạp bằng cùng một môi trường mô tả dưới đây.

---

## Tóm tắt nhanh

Nếu bạn đã quen Arduino thì chỉ cần 4 dòng này:

1. Arduino IDE 2.x
2. Board Manager URL: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. Cài gói **esp32 by Espressif Systems, phiên bản 2.0.17** — không phải bản mới nhất
4. Chọn board **ESP32-WROOM-DA Module**. Không cần cài thư viện nào cả.

Phần còn lại của tài liệu giải thích từng bước và cách xử lý lỗi.

---

## Bước 1 — Cài Arduino IDE

Tải tại <https://www.arduino.cc/en/software>, chọn bản **2.x** cho Windows.

Bản 1.8.x cũng dùng được nhưng giao diện khác, các bước bên dưới mô tả theo 2.x.

---

## Bước 2 — Cài driver USB

Cắm mạch vào máy tính, mở **Device Manager** (nhấn `Win + X`, chọn Device Manager)
rồi xem mục **Ports (COM & LPT)**.

- Thấy `Silicon Labs CP210x` hoặc `USB-SERIAL CH340` kèm số COM → xong, sang bước 3.
- Không thấy gì, hoặc thấy thiết bị lạ có dấu chấm than vàng → cần cài driver.

Nhìn con chip vuông nhỏ nằm cạnh cổng USB trên mạch để biết cài driver nào:

| Chữ trên chip | Driver cần cài |
|---|---|
| CP2102 / CP2104 | CP210x VCP Driver của Silicon Labs |
| CH340 / CH9102 | CH341SER của WCH |

Cài xong rút mạch ra cắm lại, kiểm tra Device Manager lần nữa.

> **Không thấy COM port dù đã cài driver?** Rất hay gặp: dùng nhầm **cáp chỉ để
> sạc**. Loại cáp này chỉ có hai dây nguồn, không có dây dữ liệu. Đổi sang cáp
> khác đã từng dùng để chép dữ liệu điện thoại.

---

## Bước 3 — Cài gói board ESP32

**Đây là bước hay sai nhất. Đọc kỹ phần phiên bản.**

1. Mở Arduino IDE, vào **File → Preferences**.
2. Ở ô **Additional boards manager URLs**, dán đường dẫn sau:

   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```

3. Nhấn OK.
4. Mở **Tools → Board → Boards Manager** (hoặc bấm biểu tượng con chip ở thanh trái).
5. Gõ `esp32` vào ô tìm kiếm, tìm mục **esp32 by Espressif Systems**.
6. Ở ô chọn phiên bản, **chọn đúng `2.0.17`**, rồi nhấn Install.

Gói này khoảng vài trăm MB, tải lần đầu hơi lâu.

### Vì sao phải là 2.0.17 mà không phải bản mới nhất

Từ phiên bản **3.0.0**, Espressif thay đổi hoàn toàn cách gọi hàm xuất xung PWM
cho ESC. Bản 2.x dùng cơ chế "kênh":

```cpp
ledcSetup(kenh, tan_so, do_phan_giai);
ledcAttachPin(chan, kenh);
ledcWrite(kenh, gia_tri);
```

Bản 3.x bỏ khái niệm kênh, gộp thành `ledcAttach(chan, tan_so, do_phan_giai)`.
Hai cách này không tương thích với nhau.

Firmware trong bộ mã nguồn viết theo API 2.x. Cài nhầm bản 3.x thì khi biên dịch
sẽ báo:

```
'ledcSetup' was not declared in this scope
'ledcAttachPin' was not declared in this scope
```

Gặp lỗi này thì quay lại Boards Manager, hạ về 2.0.17. Trong Boards Manager có
thể chọn phiên bản cũ rồi nhấn Install để thay thế, không cần gỡ bản mới trước.

---

## Bước 4 — Chọn board và cấu hình

Vào **Tools** rồi đặt như sau:

| Mục | Giá trị |
|---|---|
| Board | **ESP32-WROOM-DA Module** |
| Port | COM thấy được ở Bước 2 |
| Upload Speed | 921600 (nếu nạp hay lỗi thì hạ xuống 115200) |
| Flash Frequency | 80MHz |
| Flash Mode | QIO |
| Partition Scheme | Default 4MB with spiffs |
| Core Debug Level | None |

Các mục không nhắc tới thì để nguyên mặc định.

---

## Bước 5 — Thư viện

Chỉ cần cài **một** thư viện, và chỉ cho sketch `KyThuatUAV_FC_level_1`:

1. Mở **Tools → Manage Libraries** (hoặc biểu tượng sách ở thanh trái).
2. Gõ `BasicLinearAlgebra`, tìm thư viện của **Tom Stewart**.
3. Nhấn Install.

Thư viện này dùng cho phép toán ma trận của bộ lọc Kalman ước lượng độ cao và
tốc độ thẳng đứng.

Sketch `Fc_basic_for_KyThuatUAV` không cần thư viện nào.

Ba thư viện còn lại đã nằm sẵn trong gói board ESP32 cài ở Bước 3, không phải
làm gì thêm:

| Thư viện | Dùng để làm gì |
|---|---|
| `SPI.h` | nói chuyện với cảm biến IMU ICM20602 |
| `Wire.h` | nói chuyện với cảm biến áp suất BMP388 qua I2C |
| `HardwareSerial.h` | đọc tín hiệu SBUS từ bộ thu |

File `types.h` nằm cùng thư mục sketch, không phải thư viện, không cần cài gì.

> Driver BMP388 trong bộ mã nguồn này viết thẳng bằng thanh ghi, **không** dùng
> `BMP388_DEV` hay `Adafruit_BMP3XX`. Nếu tài liệu nào bảo bạn cài hai thư viện
> đó thì là hướng dẫn của bản firmware cũ.

### Nếu thư mục Arduino của bạn nằm trong OneDrive

Windows có thể để file thư viện ở dạng "chỉ có trên mây" — nhìn thấy tên file
nhưng nội dung chưa tải về máy. Lúc đó biên dịch sẽ báo:

```
fatal error: BasicLinearAlgebra.h: No such file or directory
```

dù trong Manage Libraries vẫn hiện là đã cài.

Cách xử lý: mở File Explorer, vào `Documents\Arduino\libraries`, chuột phải vào
thư mục `BasicLinearAlgebra` rồi chọn **"Always keep on this device"**. Chờ biểu
tượng chuyển thành dấu tích xanh đặc rồi biên dịch lại.

---

## Bước 6 — Mở và nạp firmware

1. Mở thư mục sketch, **nhấn đúp vào file `.ino` trùng tên với thư mục**:
   - `KyThuatUAV_FC_level_1/KyThuatUAV_FC_level_1.ino`
   - hoặc `Fc_basic_for_KyThuatUAV/Fc_basic_for_KyThuatUAV.ino`

   Arduino IDE sẽ tự mở tất cả các file còn lại thành các tab bên cạnh. Nếu chỉ
   thấy một tab thì bạn mở sai file.

2. Nhấn nút **Verify** (dấu tích) để thử biên dịch trước. Thành công sẽ hiện
   dòng dạng `Sketch uses ... bytes (23%) of program storage space.`

3. **Tháo cánh quạt ra khỏi động cơ** trước khi nạp.

4. Nhấn **Upload** (mũi tên).

> **Nạp báo lỗi `Failed to connect to ESP32`?** Giữ nút **BOOT** trên mạch, nhấn
> Upload, tới khi dòng `Connecting....` hiện ra thì thả nút BOOT.

---

## Bước 7 — Kiểm tra bằng Serial Monitor

Mở **Tools → Serial Monitor**, chọn đúng tốc độ ở góc phải:

- `KyThuatUAV_FC_level_1` → **500000**
- `Fc_basic_for_KyThuatUAV` → **115200**

Chọn sai tốc độ thì màn hình ra toàn ký tự rác. Đây là lỗi hiển thị, không phải
mạch hỏng.

Với `KyThuatUAV_FC_level_1`, nếu mọi thứ đúng bạn sẽ thấy:

```
ICM20602 = 1
[ICM20602 khoi tao xong]
CONFIG 0x6
ACCEL_CONFIG2 0x5
bat dau hieu chinh gyro
Dang hieu chinh gyro - Khong di chuyen Drone!
cali gyro thanh cong
bias gyro (do/giay) -1.24 | 0.87 | 0.31
BMP388   = 1
Cao do goc = 12.437
KyThuatUAV FC - angle + alt hold
```

Ý nghĩa:

- `ICM20602 = 1` và `BMP388 = 1` — cả hai cảm biến đã nhận. Nếu ra `0` xem bảng lỗi bên dưới.
- Dòng `Dang hieu chinh gyro` lặp lại vài lần là **bình thường**. Mạch đang đo
  lại cho tới khi drone thật sự đứng yên. Đặt drone xuống mặt phẳng, bỏ tay ra,
  đừng chạm vào bàn.

---

## Lỗi thường gặp

| Thông báo / hiện tượng | Nguyên nhân | Cách xử lý |
|---|---|---|
| `'ledcSetup' was not declared in this scope` | Cài nhầm gói ESP32 bản 3.x | Boards Manager, hạ về 2.0.17 (Bước 3) |
| Không có COM port nào trong Tools → Port | Thiếu driver USB, hoặc dùng cáp chỉ sạc | Bước 2 |
| `A fatal error occurred: Failed to connect to ESP32` | Mạch chưa vào chế độ nạp | Giữ nút BOOT khi bấm Upload |
| Chỉ mở được 1 tab, biên dịch báo thiếu hàm | Mở nhầm file `.ino` con | Mở file `.ino` trùng tên thư mục |
| Serial Monitor ra ký tự rác | Sai tốc độ baud | Đặt 500000 hoặc 115200 tùy sketch |
| `KHONG TIM THAY ICM20602 - dung lai` | Sai dây SPI hoặc sai chân CS | Kiểm tra 4 dây SCK, MISO, MOSI, CS. Chân CS khai báo ở `PIN_IMU_CS` |
| `BMP388 khong tim thay - kiem tra day I2C va dia chi` | Sai dây I2C hoặc sai địa chỉ | Kiểm tra SDA/SCL. Một số module dùng địa chỉ `0x77`, đổi `BARO_I2C_ADDR` trong file `.ino` chính |
| `BMP388 conf_err - OSR khong khop ODR` | Đã sửa cấu hình cảm biến sai | Trả `BARO_ODR_SETTING` và `BARO_OSR_PRESS` về giá trị gốc |
| Kẹt mãi ở `Dang hieu chinh gyro` | Drone bị rung hoặc bị chạm vào | Đặt xuống mặt phẳng cứng, bỏ tay ra, chờ. Vẫn kẹt thì có thể IMU nhiễu bất thường |
| Biên dịch xong nhưng Serial không ra gì | Board reset liên tục, hoặc sai baud | Kiểm tra nguồn cấp có đủ không, thử baud khác |

---

## Sửa cấu hình cho mạch của bạn

Toàn bộ thiết lập nằm trong khối **CẤU HÌNH** ở đầu file `.ino` chính, không rải
rác trong các file khác. Mở
`KyThuatUAV_FC_level_1/KyThuatUAV_FC_level_1.ino` là thấy ngay.

Những thứ hay phải sửa:

| Tên | Ý nghĩa |
|---|---|
| `PIN_ESC_1` … `PIN_ESC_4` | chân xuất tín hiệu cho 4 ESC |
| `PIN_IMU_CS` | chân CS của ICM20602 |
| `PIN_SBUS_RX` | chân nhận SBUS |
| `BARO_I2C_ADDR` | địa chỉ I2C của BMP388, `0x76` hoặc `0x77` |
| `CH_ROLL` … `CH_MODE` | kênh nào của tay điều khiển ứng với chức năng gì |
| `ACC_OFFSET_X_G` … | bù lệch gia tốc kế, cách đo ghi ngay trong file |
| `KP_RATE_ROLL` … | hệ số PID |

---

## Hiệu chỉnh gia tốc kế

Làm một lần cho mỗi mạch, sau khi đã hàn xong và bắt cảm biến cố định vào khung.

### Vì sao phải làm

Đặt drone nằm phẳng thì đúng ra gia tốc kế phải đọc `X = 0`, `Y = 0`, `Z = +1g`.
Thực tế mỗi con chip lệch một kiểu, cộng thêm sai số khi hàn và khi bắt mạch vào
khung. Ba số `ACC_OFFSET_*` là để bù đúng phần lệch đó.

Bỏ qua bước này thì hỏng hai thứ:

- **Góc bị lệch.** Drone tưởng mình đang nghiêng trong khi thực ra đang bằng, nên
  tự nghiêng đi để "sửa" — bay bị trôi về một phía.
- **Giữ độ cao kém.** Trục Z nuôi thẳng vào bộ lọc Kalman ước lượng tốc độ lên
  xuống. Lệch `0.01 g` thôi là đã thành sai số gia tốc `9,8 cm/s²` nạp liên tục
  vào bộ lọc.

### Các bước

1. Mở file `KyThuatUAV_FC_level_1.ino`, tìm trong khối CẤU HÌNH dòng:

   ```cpp
   // #define CALIBRATE_ACCEL
   ```

   Bỏ hai dấu `//` ở đầu để bật lên.

2. Nạp firmware. **Tháo cánh quạt.**

3. Đặt drone nằm phẳng trên mặt bàn phẳng, **bỏ tay ra**, không chạm vào bàn.

4. Mở Serial Monitor ở tốc độ **500000**. Mạch đếm ngược 5 giây rồi đo trong
   khoảng 4 giây, LED chân 14 nháy trong lúc đo.

5. Đo xong, màn hình in ra dạng:

   ```
   Do duoc:  X = -0.0213   Y = +0.0087   Z = +0.9931  (g)

   Chep DUNG ba dong duoi day, thay vao khoi CAU HINH
   trong file KyThuatUAV_FC_level_1.ino:

   #define ACC_OFFSET_X_G   +0.0213f
   #define ACC_OFFSET_Y_G   -0.0087f
   #define ACC_OFFSET_Z_G   +0.0069f
   ```

6. Chép ba dòng đó dán đè lên ba dòng `ACC_OFFSET_*` cũ trong file.

7. **Tắt lại** `#define CALIBRATE_ACCEL` (thêm `//` vào đầu dòng), rồi nạp lại.

8. Kiểm tra: bật `DEBUG_ACC_OFFSET`, phải thấy `X` và `Y` quanh 0, `Z` quanh 1.
   Đúng rồi thì tắt debug đi.

> Ở chế độ hiệu chỉnh, mạch đo xong là **dừng hẳn**, không tạo các task điều
> khiển, nên chắc chắn không thể bay. Nhớ tắt `#define` đi sau khi xong.

### Lỗi có thể gặp khi hiệu chỉnh

| Thông báo | Nghĩa là gì |
|---|---|
| `LOI: Z phai gan +1.00 g` | Drone đang lật ngược hoặc dựng nghiêng. Đặt lại cho nằm phẳng, ngửa lên. |
| `LOI: X hoac Y lech qua nhieu` | Mặt bàn không phẳng, hoặc cảm biến gắn lệch trên mạch. Kiểm tra lại phần cơ khí trước khi bù bằng phần mềm. |

---

## Trước khi lắp cánh

Firmware nạp được không có nghĩa là bay được. **Luôn tháo cánh** khi làm ba bước
kiểm tra sau, mở lần lượt từng nhóm `#define DEBUG_...` ở cuối khối CẤU HÌNH:

1. `DEBUG_RC` — kiểm tra các kênh tay điều khiển đúng thứ tự, công tắc arm và
   công tắc chọn chế độ nhảy đúng nấc 0 / 1 / 2.
2. Mở khối test motor trong `esc_output.ino` — xác nhận thứ tự motor 1 → 2 → 3 → 4
   khớp với sơ đồ khung.
3. `DEBUG_ATTITUDE` — nghiêng drone sang phải thì `roll` phải dương, gạt cần roll
   sang phải thì giá trị mong muốn cũng phải dương. **Hai dấu phải cùng chiều.**

Mỗi lần chỉ bật **một** nhóm debug. Bật nhiều nhóm cùng lúc sẽ tràn cổng Serial.
