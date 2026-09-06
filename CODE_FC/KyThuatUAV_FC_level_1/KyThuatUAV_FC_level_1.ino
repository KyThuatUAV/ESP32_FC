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
//   KyThuatUAV FC  -  ESP32 + ICM20602 (SPI) + BMP388 (I2C) + SBUS
//
//   Hai chế độ bay: ANGLE và ALT HOLD.
//
//   Khung chạy FreeRTOS:
//       core 1 : control_task - IMU, ước lượng góc, KF độ cao, PID, ESC (500Hz)
//       core 0 : rc_task      - giải mã SBUS                          (500Hz)
//       core 0 : baro_task    - đọc BMP388 và in debug                (100Hz)
//
//   QUY ƯỚC ĐẶT TÊN
//     HẰNG SỐ CẤU HÌNH  viết HOA, có tiền tố miền:  PIN_ , CH_ , MODE_ , LIM_ , KP_
//     Hàm của một khối  có tiền tố tên khối:        imu_ , baro_ , rc_ , esc_ , pid_ , alt_
//     Biến dùng chung giữa các task có tiền tố      shared_
//     Bản chụp để in debug có tiền tố               tlm_
//     Giá trị mong muốn (setpoint) có hậu tố        _sp
//     Đơn vị nằm luôn trong tên khi dễ nhầm:        _deg _dps _cm _cms _m _g _us
//     Đầu ra bộ điều khiển đặt là                   u_  (ký hiệu chuẩn của
//                                                   tín hiệu điều khiển)
// ============================================================================

#include "types.h"

// ============================================================================
//                      CẤU HÌNH  -  SỬA TRỰC TIẾP TẠI ĐÂY
// ============================================================================

// ---- Chân kết nối -----------------------------------------------------------
#define PIN_ESC_1        27
#define PIN_ESC_2        26
#define PIN_ESC_3        25
#define PIN_ESC_4        33

#define PIN_IMU_CS        5     // ICM20602 chip select (SPI)
#define PIN_IMU_LED      14     // LED báo trạng thái hiệu chỉnh gyro

#define PIN_SBUS_RX      35     // SBUS là tín hiệu đảo, dùng UART2

#define PIN_I2C_SDA      21
#define PIN_I2C_SCL      22
#define BARO_I2C_ADDR  0x76     // BMP388, nối kiểu 0x77 thì đổi ở đây

// ---- Kênh tay điều khiển ----------------------------------------------------
#define CH_ROLL           1
#define CH_PITCH          2
#define CH_THROTTLE       3
#define CH_YAW            4
#define CH_ARM            5     // công tắc arm, nấc trên cùng = arm
#define CH_MODE           6     // công tắc 3 nấc chọn chế độ bay

// ---- Chế độ bay và cách gán cho từng nấc công tắc --------------------------
#define MODE_ANGLE        0
#define MODE_ALT_HOLD     1
#define MODE_FAILSAFE     2     // dùng nội bộ, không gán cho công tắc

#define MODE_FOR_SW_0     MODE_ANGLE
#define MODE_FOR_SW_1     MODE_ANGLE
#define MODE_FOR_SW_2     MODE_ALT_HOLD

// ---- Giới hạn lệnh từ cần gạt ----------------------------------------------
#define LIM_TILT_DEG           20.0f   // độ nghiêng tối đa ở chế độ angle
#define LIM_YAW_RATE_DPS      100.0f   // tốc độ xoay tối đa, độ/giây
#define LIM_CLIMB_RATE_CMS    100.0f   // tốc độ lên xuống tối đa, cm/giây
#define FAILSAFE_DESCENT_CMS  -60.0f   // tốc độ hạ khi mất sóng, cm/giây

// ---- Giới hạn xung ra ESC ---------------------------------------------------
#define ESC_IDLE        800     // giá trị khi chưa arm
#define ESC_MIN_ARMED   900     // đã arm thì không cho tụt dưới mức này
#define ESC_MAX        1600

// ---- Hệ số PID --------------------------------------------------------------
// KP / KI / KD là ký hiệu chuẩn của khâu tỉ lệ, tích phân, vi phân.
// Đây là bộ số của chính khung này. fc_basic_loiter dùng KP_RATE_ROLL 0.4,
// KI 0.4, KD 0.03 vì khác khung và khác động cơ - không chép chéo qua lại.
const float KP_RATE_ROLL  = 0.5f,  KI_RATE_ROLL  = 1.4f,  KD_RATE_ROLL  = 0.03f;
const float KP_RATE_PITCH = 0.5f,  KI_RATE_PITCH = 1.4f,  KD_RATE_PITCH = 0.03f;
const float KP_RATE_YAW   = 1.0f,  KI_RATE_YAW   = 10.0f, KD_RATE_YAW   = 0.0f;

const float KP_TILT_ROLL  = 10.0f, KI_TILT_ROLL  = 0.0f,  KD_TILT_ROLL  = 0.0f;
const float KP_TILT_PITCH = 10.0f, KI_TILT_PITCH = 0.0f,  KD_TILT_PITCH = 0.0f;

const float KP_CLIMB      = 3.0f,  KI_CLIMB      = 15.0f, KD_CLIMB      = 0.0f;

// Chặn khâu tích phân và chặn đầu ra của từng tầng
const float I_LIM_RATE   = 200.0f,  U_LIM_RATE   = 400.0f;
const float I_LIM_TILT   = 200.0f,  U_LIM_TILT   = 400.0f;
const float I_LIM_YAW    = 100.0f,  U_LIM_YAW    = 100.0f;
const float I_LIM_CLIMB  = 700.0f,  P_LIM_CLIMB  = 200.0f;

// ---- Bù lệch gia tốc kế (đơn vị g) -----------------------------------------
// Cách đo: bật DEBUG_ACC_OFFSET bên dưới, đặt drone nằm phẳng, chờ số ổn định,
// rồi điền:  X = -giá_trị_X,  Y = -giá_trị_Y,  Z = 1 - giá_trị_Z
#define ACC_OFFSET_X_G   0.00f
#define ACC_OFFSET_Y_G   0.00f
#define ACC_OFFSET_Z_G   0.00f

// ---- Bộ lọc thông thấp bên trong ICM20602 ----------------------------------
// 0x04=20Hz  0x05=10Hz  0x06=5.1Hz  0x07=2.5Hz  (số nhỏ = ít trễ, nhiều nhiễu)
#define IMU_DLPF_GYRO    0x06
#define IMU_DLPF_ACC     0x05

// ---- Debug ------------------------------------------------------------------
// Mỗi lần chỉ bật MỘT dòng. Việc in do baro_task lo, không bao giờ in từ
// vòng điều khiển, nên bật debug không làm lệch nhịp PID.
// #define DEBUG_ATTITUDE      // góc hiện tại so với góc mong muốn
// #define DEBUG_RATE          // tốc độ góc mong muốn so với thực tế
// #define DEBUG_ALTITUDE      // độ cao và tốc độ lên xuống sau KF
// #define DEBUG_RC            // giá trị các kênh tay điều khiển
// #define DEBUG_ACC_OFFSET    // để đo ACC_OFFSET_* bên trên
// #define DEBUG_LOOP_TIME     // chu kì vòng điều khiển, phải luôn ~2000 us

// ============================================================================
//                        HẾT PHẦN CẤU HÌNH
// ============================================================================

#define PERIOD_CTRL_MS    2       // 500 Hz
#define PERIOD_RC_MS      2       // 500 Hz
#define PERIOD_BARO_MS   10       // 100 Hz
#define DT_CTRL      0.002f       // chu kì vòng điều khiển, tính bằng giây

// Trạng thái cảm biến lúc khởi động (kiểu khai báo trong types.h)
SensorPresent sensor_present;

// Mutex cho từng nhóm dữ liệu dùng chung
SemaphoreHandle_t mtx_rc;
SemaphoreHandle_t mtx_baro;
SemaphoreHandle_t mtx_tlm;

// rc_task ghi, control_task đọc
int  shared_rc_ch[17];
bool shared_rc_ok = false;

// baro_task ghi, control_task đọc. Cờ _new để KF chỉ hiệu chỉnh khi thật sự
// có phép đo mới, tránh nạp trùng một mẫu nhiều lần.
float shared_baro_alt_m = 0.0f;
bool  shared_baro_new   = false;

// control_task ghi, baro_task đọc để in debug
float tlm_roll_deg, tlm_pitch_deg;
float tlm_roll_sp_deg, tlm_pitch_sp_deg;
float tlm_rate_roll_dps, tlm_rate_pitch_dps, tlm_rate_yaw_dps;
float tlm_rate_roll_sp_dps, tlm_rate_pitch_sp_dps;
float tlm_alt_cm, tlm_climb_cms, tlm_climb_sp_cms;
float tlm_acc_x_g, tlm_acc_y_g, tlm_acc_z_g;
int   tlm_rc_ch[17];
int   tlm_esc[5];
int   tlm_flight_mode;
bool  tlm_armed, tlm_rc_ok;
uint32_t tlm_loop_us;


void setup() {
  Serial.begin(500000);

  esc_init();
  esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);

  rc_init();

  // Kiểm tra IMU TRƯỚC khi hiệu chỉnh gyro. Bay với IMU chết là rơi ngay,
  // nên ở đây dừng hẳn chứ không chỉ in cảnh báo rồi chạy tiếp.
  imu_init_bus();
  sensor_present.imu = imu_is_present();
  Serial.printf("ICM20602 = %d\n", sensor_present.imu);
  if (!sensor_present.imu) {
    while (1) {
      Serial.println("KHONG TIM THAY ICM20602 - dung lai, khong cho bay");
      digitalWrite(PIN_IMU_LED, !digitalRead(PIN_IMU_LED));
      delay(300);
    }
  }
  imu_init();

  sensor_present.baro = baro_init();
  Serial.printf("BMP388   = %d\n", sensor_present.baro);

  alt_kf_init();

  Serial.println("KyThuatUAV FC - angle + alt hold");

  mtx_rc   = xSemaphoreCreateMutex();
  mtx_baro = xSemaphoreCreateMutex();
  mtx_tlm  = xSemaphoreCreateMutex();

  if (mtx_rc == NULL || mtx_baro == NULL || mtx_tlm == NULL) {
    Serial.println("Loi: khong tao duoc Mutex!");
    while (1) delay(1000);
  }

  xTaskCreatePinnedToCore(rc_task,      "RC",   3072, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(baro_task,    "BARO", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(control_task, "CTRL", 8192, NULL, 5, NULL, 1);

  vTaskDelete(NULL);   // xoá loopTask của Arduino, giải phóng stack của nó
}

void loop() {
  // Không dùng, mọi thứ chạy trong các task
}


// ============================================================================
//  rc_task  -  core 0, 500Hz
// ============================================================================
void rc_task(void *parameter) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    bool ok = rc_update();

    if (xSemaphoreTake(mtx_rc, 0) == pdTRUE) {
      shared_rc_ok = ok;
      for (int i = 1; i <= 16; i++) shared_rc_ch[i] = rc_get_channel(i);
      xSemaphoreGive(mtx_rc);
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_RC_MS));
  }
}


// ============================================================================
//  baro_task  -  core 0, 100Hz. Kiêm luôn việc in debug.
// ============================================================================
void baro_task(void *parameter) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    if (sensor_present.baro) {
      // baro_update() chỉ trả true khi cảm biến có mẫu MỚI
      if (baro_update()) {
        float alt_m = baro_get_altitude_m();
        if (xSemaphoreTake(mtx_baro, 0) == pdTRUE) {
          shared_baro_alt_m = alt_m;
          shared_baro_new   = true;
          xSemaphoreGive(mtx_baro);
        }
      }
    }

    debug_print();

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_BARO_MS));
  }
}


// ============================================================================
//  In debug. Chạy ở core 0 priority thấp nên không ảnh hưởng vòng điều khiển.
// ============================================================================
void debug_print() {
#if defined(DEBUG_ATTITUDE) || defined(DEBUG_RATE) || defined(DEBUG_ALTITUDE) || \
    defined(DEBUG_RC) || defined(DEBUG_ACC_OFFSET) || defined(DEBUG_LOOP_TIME)

  static uint32_t tick = 0;
  if (++tick % 5) return;              // 20Hz là đủ nhìn

  if (xSemaphoreTake(mtx_tlm, 0) != pdTRUE) return;

  #ifdef DEBUG_ATTITUDE
    Serial.printf("%.1f,%.1f,%.1f,%.1f\n",
                  tlm_roll_sp_deg, tlm_roll_deg, tlm_pitch_sp_deg, tlm_pitch_deg);
  #endif

  #ifdef DEBUG_RATE
    Serial.printf("%.0f,%.0f,%.0f,%.0f\n",
                  tlm_rate_roll_sp_dps,  tlm_rate_roll_dps,
                  tlm_rate_pitch_sp_dps, tlm_rate_pitch_dps);
  #endif

  #ifdef DEBUG_ALTITUDE
    Serial.printf("%.1f,%.1f,%.1f\n", tlm_alt_cm, tlm_climb_cms, tlm_climb_sp_cms);
  #endif

  #ifdef DEBUG_RC
    Serial.printf("ch1:%d ch2:%d ch3:%d ch4:%d arm:%d mode:%d | rc_ok:%d armed:%d fm:%d\n",
                  tlm_rc_ch[CH_ROLL], tlm_rc_ch[CH_PITCH],
                  tlm_rc_ch[CH_THROTTLE], tlm_rc_ch[CH_YAW],
                  tlm_rc_ch[CH_ARM], tlm_rc_ch[CH_MODE],
                  tlm_rc_ok, tlm_armed, tlm_flight_mode);
  #endif

  #ifdef DEBUG_ACC_OFFSET
    static float ax = 0, ay = 0, az = 1;
    ax = ax * 0.98f + tlm_acc_x_g * 0.02f;
    ay = ay * 0.98f + tlm_acc_y_g * 0.02f;
    az = az * 0.98f + tlm_acc_z_g * 0.02f;
    Serial.printf("X:%.4f Y:%.4f Z:%.4f\n", ax, ay, az);
  #endif

  #ifdef DEBUG_LOOP_TIME
    Serial.printf("loop_us:%u\n", tlm_loop_us);
  #endif

  xSemaphoreGive(mtx_tlm);
#endif
}
