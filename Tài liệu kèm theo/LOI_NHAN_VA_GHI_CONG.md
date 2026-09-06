# Lời nhắn và ghi công

## Vì sao có bộ mã nguồn này

Tôi làm mạch bay này và viết firmware cho nó. Bây giờ tôi công khai toàn bộ:
thiết kế mạch, mã nguồn, tài liệu. Kèm theo đó là 100 bo mạch tặng miễn phí cho
sinh viên trên cả nước.

Không có điều kiện gì. Không cần xin phép. Không cần nhắn tin hỏi tôi.
Tải về, hàn lấy, nạp code, bay. Hỏng thì sửa. Sửa được thì làm tiếp cái tốt hơn.

Tôi làm điều này vì hồi tôi bắt đầu, thứ khó nhất không phải là linh kiện đắt,
mà là không có ai chỉ cho biết vì sao con drone của mình không bay được. Tôi
muốn người đi sau đỡ mất mấy năm mò mẫm như tôi.

## Ba điều tôi mong

Bộ mã nguồn này là của chung. Tôi chỉ mong ba điều, và cả ba đều không tốn của
bạn đồng nào:

1. **Giữ lại tên tác giả trong mã nguồn.** Bạn sửa bao nhiêu cũng được, thêm
   tính năng gì cũng được, nhưng đừng xoá dòng ghi công của người viết ra nó.
2. **Nói rõ bạn lấy từ đâu.** Khi bạn chia sẻ lại cho người khác, một dòng
   "dựa trên dự án X" là đủ.
3. **Nếu bạn làm nó tốt hơn, hãy chia sẻ lại.** Bạn không nợ tôi điều đó,
   nhưng cộng đồng sẽ lớn lên nhờ vậy.

Điều kiện pháp lý cụ thể nằm trong file `LICENSE`. Nhưng thật ra ba dòng trên
mới là thứ tôi quan tâm.

---

# Phần ghi công

Tôi vừa xin bạn ba điều ở trên. Phần này là tôi làm đúng ba điều đó với những
người đi trước tôi.

Không có dự án nào bắt đầu từ con số không.

## Tôi học từ đâu — Carbon Aeronautics

- Kênh YouTube và bộ tài liệu: <https://github.com/CarbonAeronautics>
- Tác giả ở Antwerp, Bỉ. Làm dự án mã nguồn mở và tài liệu dạy học về drone.

Đây là nguồn ảnh hưởng lớn nhất tới cách tổ chức firmware này. Cụ thể những
điều học được từ đó:

- **Cách ước lượng góc bằng bộ lọc Kalman một chiều.** Dùng gyro cho phần biến
  thiên nhanh, lấy góc từ gia tốc kế làm phép đo kéo về chống trôi. Đây là ý
  tưởng nền của hàm `imu_kalman_1d()`.
- **Cấu trúc PID xếp tầng.** Tầng ngoài nhận góc mong muốn và sinh ra tốc độ góc
  mong muốn, tầng trong bám theo tốc độ góc đó. Toàn bộ `pid_tilt_*` và
  `pid_rate_*` đi theo mô hình này.
- **Cách nghĩ về một firmware bay tối giản.** Vòng lặp chu kì cố định, đọc cảm
  biến rồi ước lượng trạng thái rồi điều khiển rồi trộn tín hiệu ra bốn động cơ,
  theo đúng thứ tự đó.
- **Trình tự dạy từ dễ tới khó.** Bay rate trước, rồi angle, rồi mới tới giữ độ
  cao. Cấu trúc hai bản `Fc_basic_for_KyThuatUAV` và `KyThuatUAV_FC_level_1`
  chính là đi theo lối đó.

Nếu bạn đang học làm flight controller, hãy xem trực tiếp tài liệu gốc của họ.
Nó giải thích phần lý thuyết kỹ hơn nhiều so với những gì nhét vừa vào phần chú
thích của một bộ mã nguồn.

> **Về giấy phép:** tại thời điểm viết tài liệu này, các repo của Carbon
> Aeronautics **không kèm file LICENSE**. Theo luật bản quyền mặc định, không
> ghi giấy phép nghĩa là tác giả giữ toàn bộ quyền. Vì vậy bộ mã nguồn ở đây
> tiếp thu **phương pháp và ý tưởng** — những thứ không thuộc phạm vi bảo hộ bản
> quyền — chứ không phát hành lại mã nguồn của họ. Muốn dùng lại mã nguồn gốc
> của Carbon Aeronautics, bạn nên hỏi trực tiếp tác giả.

---

## Bộ mã nguồn này làm thêm những gì

Phần trên là những gì học được. Phần này là những gì tự làm, không có trong bản
mẫu cơ bản. Ghi ra để người đọc biết ranh giới nằm ở đâu.

### Nâng cấp cảm biến

Thay đổi này nằm ở phần cứng chứ không chỉ ở mã nguồn, và nó kéo theo phải viết
lại toàn bộ driver.

**IMU: MPU-6050 → ICM-20602**

- MPU-6050 chỉ nói chuyện được qua I2C, nhanh nhất 400 kHz. ICM-20602 chạy được
  SPI tới 10 MHz. Với vòng điều khiển 500Hz thì đây là khác biệt quyết định:
  thời gian đọc cảm biến rút xuống đủ để phần còn lại của vòng lặp có chỗ thở.
- ICM-20602 là đời mới hơn nhiều, nhiễu gyro thấp hơn và ổn định theo nhiệt độ
  tốt hơn. Nhiễu thấp cho phép để hệ số D của PID cao hơn mà motor không rít.
- MPU-6050 là linh kiện từ 2011, nhà sản xuất đã ngừng khuyến nghị dùng cho
  thiết kế mới. Chọn ICM-20602 để bo mạch còn mua được linh kiện lâu dài.

**Cảm biến áp suất: BMP280 → BMP388**

- Nhiễu áp suất của BMP388 thấp hơn rõ rệt. Với giữ độ cao thì nhiễu áp suất
  chính là thứ quyết định drone đứng yên hay bồng bềnh, vì độ cao nuôi thẳng vào
  bộ lọc Kalman.
- BMP388 đo được tới 200Hz, cao hơn BMP280. Số liệu tươi hơn nghĩa là bộ lọc
  Kalman bớt phải ngoại suy từ gia tốc kế.
- BMP388 có cờ báo "đã có mẫu mới" đọc được qua thanh ghi trạng thái. Bản này
  dùng đúng cờ đó để không nạp trùng một mẫu vào bộ lọc.

### Nhận tín hiệu điều khiển bằng SBUS

Bản này đọc bộ thu bằng giao thức SBUS, thay cho kiểu nhận từng kênh một dây
hoặc kiểu đo độ rộng xung.

- **16 kênh trên đúng một sợi dây.** Kiểu cũ mỗi kênh một dây tín hiệu, sáu kênh
  là sáu sợi. Ít dây thì bớt chỗ hỏng, bớt nhiễu, và bo mạch gọn hơn.
- **Dữ liệu số, không phải đo thời gian.** SBUS gửi thẳng con số 11 bit cho mỗi
  kênh. Không phải đếm độ rộng xung nên không bị sai vì ngắt tới trễ hay vì vòng
  lặp bận — thứ vốn làm giá trị cần gạt bị rung ở kiểu cũ.
- **Kiểm tra được khung có nguyên vẹn không.** Một khung SBUS dài đúng 25 byte,
  mở đầu bằng `0x0F` và kết thúc bằng `0x00`. Sai một trong ba điều kiện đó là
  bỏ nguyên khung, không đưa giá trị rác vào bộ điều khiển.
- **Phát hiện mất sóng một cách chắc chắn.** Vì khung tới đều đặn, chỉ cần đếm
  thời gian kể từ khung hợp lệ cuối cùng: quá 200 mili giây là coi như mất sóng.
  Đây chính là thứ kích hoạt chế độ tự hạ cánh — không có tín hiệu sạch như vậy
  thì không thể làm fail-safe đáng tin.
- **ESP32 đảo tín hiệu ngay trong chip.** SBUS là tín hiệu đảo, nhiều nền tảng
  phải lắp thêm transistor để đảo lại. UART của ESP32 làm được bằng phần cứng,
  chỉ cần bật cờ `invert` lúc mở cổng, nên bo mạch không cần linh kiện phụ nào.

### Fail-safe tự hạ cánh khi mất sóng

Bản mẫu dạy học thường xử lý mất sóng bằng cách cắt động cơ. Cắt động cơ ở độ
cao vài mét nghĩa là drone rơi tự do — hỏng máy, và nguy hiểm cho người xung
quanh.

Bản này hạ cánh có điều khiển:

1. **Phát hiện.** Quá 200 mili giây không có khung SBUS hợp lệ thì coi như mất
   sóng.
2. **Giữ thăng bằng.** Góc mong muốn của cả roll lẫn pitch được đặt về 0, tốc độ
   xoay yaw cũng về 0. Drone tự cân bằng chứ không lật.
3. **Hạ đều, có điều khiển.** Không phải giảm ga rồi mặc kệ. Bộ điều khiển tốc
   độ thẳng đứng nhận lệnh "đi xuống 60 cm mỗi giây" và bám theo đúng tốc độ đó,
   dùng vận tốc ước lượng từ bộ lọc Kalman. Gió hay pin yếu thì nó tự bù ga.
   Tốc độ này sửa được ở `FAILSAFE_DESCENT_CMS`.
4. **Tự nhận biết đã chạm đất.** Khi ga đã xuống đáy, tốc độ thẳng đứng gần như
   bằng không, và trạng thái đó giữ quá 2 giây, thì drone tự disarm. Ba điều
   kiện cùng lúc để tránh disarm nhầm giữa không trung lúc đang rơi đều.

**Điểm đáng nói nhất:** toàn bộ quy trình trên chỉ chạy khi cảm biến áp suất
hoạt động. Không có nó thì không biết đang lên hay đang xuống, nên "hạ cánh có
điều khiển" thành đoán mò. Trường hợp đó firmware cắt động cơ ngay thay vì giả
vờ hạ cánh — thà rơi từ chỗ biết chắc còn hơn rơi vì tin vào số liệu không có.

### Chuyển sang nền tảng ESP32

Tài liệu gốc viết cho Teensy và Arduino. Chuyển sang ESP32 không phải chỉ đổi
tên chân — gần như mọi thứ chạm tới phần cứng đều phải làm lại:

- **Xuất xung ESC bằng ngoại vi LEDC.** ESP32 không dùng `analogWrite` như
  Arduino mà có bộ LEDC riêng theo cơ chế "kênh": `ledcSetup` đặt tần số và độ
  phân giải, `ledcAttachPin` gắn kênh vào chân, `ledcWrite` xuất giá trị. Chọn
  391Hz với 11 bit để dải 800-1600 rơi đúng vào xung 1000-2000 micro giây.
- **Đảo tín hiệu SBUS bằng phần cứng.** UART của ESP32 đảo được tín hiệu ngay
  trong chip, chỉ cần bật cờ `invert` lúc mở cổng. Không cần mạch đảo bên ngoài
  như nhiều nền tảng khác.
- **Né các chân không dùng được.** GPIO34 và GPIO35 của ESP32 là chân chỉ vào,
  không xuất được xung, nên không thể nối ESC vào đó. Trong bản này GPIO35 dành
  cho SBUS RX. Chỗ này ghi rõ trong chú thích vì rất dễ mất buổi để tìm ra.
- **Khoá phiên bản core 2.0.17.** Từ core 3.x, Espressif bỏ hẳn khái niệm kênh
  của LEDC và gộp thành `ledcAttach`, không tương thích ngược. Tài liệu cài đặt
  ghi rõ phải cài đúng 2.0.17 và giải thích vì sao, kèm cách nhận ra lỗi.
- **Giải phóng tác vụ thừa.** Sau khi tạo xong các tác vụ, `setup()` gọi
  `vTaskDelete(NULL)` để xoá luôn tác vụ `loop()` mặc định của Arduino, trả lại
  phần bộ nhớ ngăn xếp của nó.
- **Xử lý cách Arduino sinh nguyên mẫu hàm.** Arduino tự sinh prototype rồi chèn
  lên đầu sketch, nên kiểu `struct` khai báo trong file `.ino` sẽ bị dùng trước
  khi được định nghĩa. Vì vậy các kiểu dùng chung nằm trong `types.h`, có ghi
  luôn lý do trong file đó.

### Kiến trúc chạy đa nhân

Bản mẫu cơ bản chạy một vòng lặp duy nhất trong `loop()`. Bản này chạy trên
FreeRTOS với ba tác vụ chia trên hai nhân của ESP32:

- Nhân 1 dành riêng cho vòng điều khiển 500Hz, không chia sẻ với việc gì khác.
- Nhân 0 lo đọc SBUS và đọc cảm biến áp suất.
- Mỗi nhóm dữ liệu dùng chung có mutex riêng, không dùng một khoá chung.
- Dữ liệu chuyển giữa các tác vụ qua bộ đệm kép, lấy mutex không được thì bỏ
  qua chứ **không bao giờ chặn vòng điều khiển**.

### Cảm biến viết bằng thanh ghi

Không gọi thư viện cảm biến nào. Cả ICM-20602 (SPI) lẫn BMP388 (I2C) đều nói
chuyện trực tiếp với thanh ghi theo datasheet. Nhờ vậy người tải về không phải
đi tìm đúng phiên bản thư viện, và khi có lỗi thì đọc được tới tận nơi.

Riêng BMP388 có hai chỗ đáng nói:

- **Xét cờ drdy trước khi lấy dữ liệu.** Không có nó thì mẫu cũ bị đọc lại nhiều
  lần, và bộ lọc Kalman tưởng đó là phép đo mới độc lập rồi thu hẹp sai số sai
  cách.
- **Chọn cặp ODR/OSR theo công thức thời gian đo của datasheet**, có ghi rõ phép
  tính trong chú thích. Đặt sai cặp này thì cảm biến không kịp đo và tự báo lỗi
  `conf_err`.

### Ước lượng độ cao và tốc độ thẳng đứng

Bộ lọc Kalman hai trạng thái dạng ma trận, hợp nhất độ cao từ baro với gia tốc
thẳng đứng từ IMU. Gia tốc thẳng đứng tính thẳng từ góc roll/pitch nên không cần
tới bộ lọc quaternion nào.

### Những thứ về an toàn

Đây là phần khác bản mẫu nhiều nhất, vì bản mẫu là để dạy học còn bản này là để
bay thật:

- **Hiệu chỉnh gyro tự kiểm tra.** Cách thông thường là lấy trung bình một số
  mẫu cố định rồi tin luôn kết quả. Cách đó có một lỗ hổng: nếu lúc cắm nguồn
  drone đang bị cầm trên tay, đang rung, hoặc đặt trên bàn có người chạm vào,
  thì cái "bias" đo được sẽ dính luôn chuyển động đó. Bias sai thì drone tưởng
  mình đang xoay trong khi đứng yên, và tự nghiêng đi để "sửa" — trôi ngay từ
  giây đầu cất cánh, mà nhìn code thì không thấy gì sai.

  Bản này đo thành từng đợt và so tổng đợt này với đợt trước. Chênh lệch còn
  nhỏ hơn ngưỡng mới chấp nhận, chưa đạt thì đo lại từ đầu, lặp bao lâu cũng
  được. LED báo trạng thái sáng suốt lúc đang đo. Nhờ vậy không có cách nào
  nhận nhầm bias sai — cùng lắm là phải chờ lâu hơn vài giây.
- **Chế độ hiệu chỉnh gia tốc kế riêng.** Đo xong in ra đúng ba dòng để dán, có
  kiểm tra tư thế trước khi tin kết quả, và dừng hẳn không cho bay ở chế độ đó.
- **Kiểm tra IMU trước khi cho bay.** Không đọc được cảm biến thì dừng hẳn và
  nháy LED, chứ không in cảnh báo rồi vẫn chạy tiếp.
- **Điều kiện arm chặt.** Phải gạt công tắc VÀO nấc arm và cần ga phải ở đáy.
  Cắm nguồn lúc công tắc arm đang bật thì không tự arm.
- **Tự hạ cánh khi mất sóng** — xem mục riêng ở trên.

### Cách gỡ lỗi

- **Không bao giờ in Serial từ vòng điều khiển.** Vòng điều khiển chỉ chụp lại
  trạng thái, việc in do tác vụ ưu tiên thấp ở nhân còn lại lo. Bật gỡ lỗi không
  làm lệch nhịp PID.
- Sáu nhóm gỡ lỗi bật tắt bằng `#define`, mỗi lần một nhóm.

### Cách viết mã

- Toàn bộ thiết lập gom vào **một khối duy nhất** ở đầu file chính. Không phải
  đi lục từng file để đổi chân cắm hay hệ số PID.
- **Đơn vị nằm luôn trong tên biến** (`_deg`, `_dps`, `_cm`, `_cms`, `_m`, `_g`).
  Chỗ nối baro với bộ lọc Kalman là mét sang xen-ti-mét, có đơn vị trong tên thì
  nhìn ra ngay chỗ nhân 100.
- Quy ước đặt tên ghi ở đầu file chính, ai đọc cũng theo được.

---

## Thư viện dùng trong mã nguồn

| Thư viện | Tác giả | Giấy phép | Dùng ở đâu |
|---|---|---|---|
| [BasicLinearAlgebra](https://github.com/tomstewart89/BasicLinearAlgebra) | Tom Stewart | MIT, © 2019 | Phép toán ma trận của bộ lọc Kalman độ cao (`altitude_kf.ino`) |
| Arduino core for ESP32 | Espressif Systems | LGPL-2.1 | Toàn bộ nền tảng, gồm `SPI`, `Wire`, `HardwareSerial`, FreeRTOS |

Cả hai giấy phép trên đều tương thích với GPL-3.0 của dự án này.

## Tài liệu kỹ thuật của nhà sản xuất

Phần driver cảm biến viết bằng thanh ghi, dựa thẳng vào tài liệu chính thức:

- **Bosch Sensortec BMP388 datasheet** — sơ đồ thanh ghi, công thức bù nhiệt và
  bù áp suất, và công thức tính thời gian đo dùng để chọn cặp ODR/OSR trong
  `baro_bmp388.ino`.
- **InvenSense (TDK) ICM-20602 datasheet** — sơ đồ thanh ghi, thang đo, và các
  mức bộ lọc thông thấp trong `imu_icm20602.ino`.

---

# Gửi các bạn sinh viên

Đừng học cách đi tắt.

Con drone không quan tâm bạn quen ai, bạn có bao nhiêu người theo dõi, hay bạn
nói hay tới mức nào. Nó chỉ phản ứng với việc bạn có hiểu đúng vật lý và viết
đúng code hay không. Nó rơi hay không rơi, vậy thôi. Đó là thứ trung thực nhất
mà nghề này dạy được cho bạn.

Cứ đọc từng dòng trong bộ mã nguồn này. Chỗ nào thấy sai thì sửa, rồi nói cho
tôi biết là tôi sai ở đâu — tôi cảm ơn thật lòng, vì mỗi lỗi được chỉ ra là một
con drone không rơi.

Nói trước cho rõ: tôi không kèm riêng được từng người, vì tôi còn lớp phải dạy.
Nhưng những gì tôi giải thích được thì đã viết hết vào chú thích trong mã nguồn
và vào file hướng dẫn cài đặt — không phải viết cho có, mà viết đúng chỗ tôi
từng mất thời gian nhất. Đọc kỹ hai thứ đó là đủ để tự đi tiếp.

Và khi nào bạn đủ giỏi để người khác muốn chép lại của bạn, tôi mong bạn cũng sẽ
mở ra cho họ như thế này.

Bay an toàn. Nhớ tháo cánh khi thử nghiệm.

---

## Lời cảm ơn

Cảm ơn Carbon Aeronautics vì đã công khai kiến thức thay vì giữ cho riêng mình.
Bộ mã nguồn này được phát hành theo tinh thần đó.

Nếu bạn dùng lại mã nguồn ở đây, xin giữ nguyên phần ghi công này — cho cả tôi
lẫn những người đi trước tôi.
