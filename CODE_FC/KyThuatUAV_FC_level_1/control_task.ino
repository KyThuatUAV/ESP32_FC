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
//  control_task  -  core 1, 500Hz. Đây là vòng điều khiển chính.
//
//  Thứ tự mỗi vòng:
//     đọc IMU -> ước lượng góc -> gia tốc thẳng đứng -> KF độ cao
//     -> lệnh tay điều khiển -> arm/mode -> PID -> mixer -> xuất ESC
// ============================================================================

// ---- Trạng thái đo được -----------------------------------------------------
float acc_x_g,  acc_y_g,  acc_z_g;          // gia tốc thân máy, đơn vị g
float rate_roll_dps, rate_pitch_dps, rate_yaw_dps;   // tốc độ góc, độ/giây
float roll_deg, pitch_deg;                  // góc nghiêng đã lọc
float acc_earth_z_g;                        // gia tốc thẳng đứng đã trừ trọng lực

// ---- Giá trị mong muốn ------------------------------------------------------
float roll_sp_deg, pitch_sp_deg;
float yaw_rate_sp_dps;
float climb_sp_cms;

// ---- Đầu ra bộ điều khiển ---------------------------------------------------
// u = tín hiệu điều khiển, ký hiệu chuẩn trong lý thuyết điều khiển
float u_roll, u_pitch, u_yaw;
int   u_throttle;

// ---- Đầu ra cuối cùng cho 4 ESC ---------------------------------------------
int esc_1, esc_2, esc_3, esc_4;
int esc_lim_lo = ESC_IDLE, esc_lim_hi = ESC_IDLE;

// ---- Trạng thái bay ---------------------------------------------------------
int  rc_ch[17];
bool rc_ok, rc_ok_prev;
bool armed = false;
int  flight_mode = MODE_ANGLE;

// Khởi tạo bằng 2 chứ không phải -1: nếu cắm nguồn lúc công tắc arm ĐANG bật,
// điều kiện "vừa vào nấc 2" sẽ không thoả, nên drone không tự arm ngay khi
// khởi động. Muốn arm phải gạt công tắc xuống rồi gạt lên lại.
int  arm_sw_prev = 2;

float baro_alt_m;      // độ cao baro mới nhất, đơn vị mét

unsigned long failsafe_start_ms;
uint32_t loop_period_us;


void control_task(void *parameter) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    static uint32_t t_prev_us = 0;
    uint32_t t_now_us = micros();
    loop_period_us = t_now_us - t_prev_us;
    t_prev_us = t_now_us;

    // ---------- 1. Cảm biến và ước lượng trạng thái ----------
    imu_update();
    acc_x_g        = imu_get_acc_x_g();
    acc_y_g        = imu_get_acc_y_g();
    acc_z_g        = imu_get_acc_z_g();
    rate_roll_dps  = imu_get_rate_roll_dps();
    rate_pitch_dps = imu_get_rate_pitch_dps();
    rate_yaw_dps   = imu_get_rate_yaw_dps();
    roll_deg       = imu_get_roll_deg();
    pitch_deg      = imu_get_pitch_deg();

    // Gia tốc theo phương thẳng đứng của mặt đất, trừ đi 1g.
    // Chỉ cần góc roll/pitch nên không phải dùng tới Madgwick hay Mahony.
    float sr = sinf(roll_deg  * DEG_TO_RAD), cr = cosf(roll_deg  * DEG_TO_RAD);
    float sp = sinf(pitch_deg * DEG_TO_RAD), cp = cosf(pitch_deg * DEG_TO_RAD);
    acc_earth_z_g = -sp * acc_x_g + cp * sr * acc_y_g + cp * cr * acc_z_g - 1.0f;

    // KF độ cao: chạy đủ cả predict lẫn correct MỖI vòng, kể cả khi baro chưa
    // có mẫu mới. Lý do nằm trong phần chú thích đầu file altitude_kf.ino -
    // bỏ bớt nhịp correct thì vận tốc thẳng đứng trôi.
    baro_fetch_shared();
    alt_kf_update(baro_alt_m, acc_earth_z_g);

    // ---------- 2. Lệnh từ tay điều khiển ----------
    rc_ok_prev = rc_ok;
    rc_fetch_and_map();

    // ---------- 3. Arm / disarm ----------
    // Arm khi công tắc VÀO nấc 2 và cần ga đang ở dưới cùng. Dùng điều kiện
    // "vào nấc 2 từ nấc bất kỳ" nên chạy đúng với cả công tắc 2 nấc lẫn 3 nấc.
    if (rc_ok) {
      if (rc_ch[CH_ARM] != 2) {
        armed = false;
      } else if (arm_sw_prev != 2 && rc_ch[CH_THROTTLE] < 1050) {
        armed = true;
      }
      arm_sw_prev = rc_ch[CH_ARM];

      if      (rc_ch[CH_MODE] == 0) flight_mode = MODE_FOR_SW_0;
      else if (rc_ch[CH_MODE] == 1) flight_mode = MODE_FOR_SW_1;
      else                          flight_mode = MODE_FOR_SW_2;

    } else if (armed && sensor_present.baro) {
      // Mất sóng khi đang bay: tự hạ độ cao
      if (rc_ok_prev) failsafe_start_ms = millis();
      flight_mode = MODE_FAILSAFE;
    } else {
      armed = false;              // mất sóng mà không có baro thì tắt máy
    }

    if (armed) { esc_lim_lo = ESC_MIN_ARMED; esc_lim_hi = ESC_MAX; }
    else       { esc_lim_lo = ESC_IDLE;      esc_lim_hi = ESC_IDLE; pid_reset_all(); }

    // Hạ xong thì tự disarm: ga đã đáy, gần như hết tốc độ rơi, quá 2 giây
    if (!rc_ok && armed) {
      if (u_throttle < ESC_MIN_ARMED + 1 &&
          fabsf(alt_kf_get_climb_cms()) < 5.0f &&
          (millis() - failsafe_start_ms) > 2000) {
        armed = false;
      }
    }

    // ---------- 4. Bộ điều khiển ----------
    switch (flight_mode) {
      case MODE_ANGLE:
        if (u_throttle < 830) pid_bleed_integrators();
        mode_angle();
        break;
      case MODE_ALT_HOLD:
        mode_alt_hold();
        break;
      case MODE_FAILSAFE:
        mode_failsafe();
        break;
      default:
        mode_angle();
        break;
    }

    // ---------- 5. Mixer quad X và xuất ESC ----------
    esc_1 = u_throttle - u_roll - u_pitch - u_yaw;
    esc_2 = u_throttle + u_roll + u_pitch - u_yaw;
    esc_3 = u_throttle + u_roll - u_pitch + u_yaw;
    esc_4 = u_throttle - u_roll + u_pitch + u_yaw;
    esc_clamp(esc_lim_lo, esc_lim_hi);

    if (!armed) { esc_1 = esc_2 = esc_3 = esc_4 = ESC_IDLE; }
    esc_write(esc_1, esc_2, esc_3, esc_4);

    telemetry_snapshot();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_CTRL_MS));
  }
}


// ============================================================================
//  Các chế độ bay
// ============================================================================

// ANGLE: cần gạt quy định góc nghiêng, thả cần thì drone tự về bằng.
void mode_angle() {
  u_throttle = rc_ch[CH_THROTTLE] * 0.8f;
  u_roll  = pid_rate_roll (rate_roll_dps,  pid_tilt_roll (roll_deg,  roll_sp_deg));
  u_pitch = pid_rate_pitch(rate_pitch_dps, pid_tilt_pitch(pitch_deg, pitch_sp_deg));
  u_yaw   = pid_rate_yaw  (rate_yaw_dps,   yaw_rate_sp_dps);
}

// ALT HOLD: cần ga quy định TỐC ĐỘ lên xuống. Cần ga ở giữa = giữ độ cao.
void mode_alt_hold() {
  u_throttle = ESC_IDLE + pid_climb(alt_kf_get_climb_cms(), climb_sp_cms);
  u_roll  = pid_rate_roll (rate_roll_dps,  pid_tilt_roll (roll_deg,  roll_sp_deg));
  u_pitch = pid_rate_pitch(rate_pitch_dps, pid_tilt_pitch(pitch_deg, pitch_sp_deg));
  u_yaw   = pid_rate_yaw  (rate_yaw_dps,   yaw_rate_sp_dps);
}

// FAILSAFE: giữ thăng bằng và hạ đều cho tới khi chạm đất rồi tự tắt máy.
void mode_failsafe() {
  u_throttle = ESC_IDLE + pid_climb(alt_kf_get_climb_cms(), FAILSAFE_DESCENT_CMS);
  u_roll  = pid_rate_roll (rate_roll_dps,  pid_tilt_roll (roll_deg,  0));
  u_pitch = pid_rate_pitch(rate_pitch_dps, pid_tilt_pitch(pitch_deg, 0));
  u_yaw   = pid_rate_yaw  (rate_yaw_dps,   0);
}


// ============================================================================
//  Lấy dữ liệu dùng chung từ các task khác
// ============================================================================

// Luôn lấy giá trị baro mới nhất. Lấy mutex không được thì giữ giá trị vòng
// trước, không sao - bộ lọc vẫn chạy đúng nhịp.
void baro_fetch_shared() {
  if (xSemaphoreTake(mtx_baro, 0) == pdTRUE) {
    baro_alt_m = shared_baro_alt_m;
    xSemaphoreGive(mtx_baro);
  }
}

// Đổi giá trị 1000-2000 của công tắc thành nấc 0 / 1 / 2
int rc_switch_position(int ch_value) {
  if      (ch_value > 900  && ch_value < 1100) return 0;
  else if (ch_value > 1400 && ch_value < 1600) return 1;
  else if (ch_value > 1900 && ch_value < 2100) return 2;
  else                                          return 0;
}

void rc_fetch_and_map() {
  if (xSemaphoreTake(mtx_rc, 0) == pdTRUE) {
    rc_ok = shared_rc_ok;
    for (int i = 1; i <= 16; i++) rc_ch[i] = shared_rc_ch[i];
    xSemaphoreGive(mtx_rc);
  }
  for (int i = 5; i <= 8; i++) rc_ch[i] = rc_switch_position(rc_ch[i]);

  // Lọc mượt lệnh cần gạt. Hệ số 0.05 ở 500Hz tương đương hằng số thời gian ~40ms
  const float a = 0.05f;
  roll_sp_deg     = roll_sp_deg     * (1 - a) + ( float(rc_ch[CH_ROLL]     - 1500) / (500.0f / LIM_TILT_DEG))        * a;
  pitch_sp_deg    = pitch_sp_deg    * (1 - a) + ( float(rc_ch[CH_PITCH]    - 1500) / (500.0f / LIM_TILT_DEG))        * a;
  yaw_rate_sp_dps = yaw_rate_sp_dps * (1 - a) + (-float(rc_ch[CH_YAW]      - 1500) / (500.0f / LIM_YAW_RATE_DPS))    * a;
  climb_sp_cms    = climb_sp_cms    * (1 - a) + ( float(rc_ch[CH_THROTTLE] - 1500) / (500.0f / LIM_CLIMB_RATE_CMS))  * a;
}

void esc_clamp(int lo, int hi) {
  if (esc_1 < lo) esc_1 = lo;
  if (esc_2 < lo) esc_2 = lo;
  if (esc_3 < lo) esc_3 = lo;
  if (esc_4 < lo) esc_4 = lo;

  if (esc_1 > hi) esc_1 = hi;
  if (esc_2 > hi) esc_2 = hi;
  if (esc_3 > hi) esc_3 = hi;
  if (esc_4 > hi) esc_4 = hi;
}

// Chụp lại trạng thái cho baro_task in debug. Lấy mutex không được thì bỏ qua,
// vì đây chỉ là dữ liệu hiển thị, không được phép làm chậm vòng điều khiển.
void telemetry_snapshot() {
  if (xSemaphoreTake(mtx_tlm, 0) != pdTRUE) return;

  tlm_roll_deg     = roll_deg;      tlm_pitch_deg    = pitch_deg;
  tlm_roll_sp_deg  = roll_sp_deg;   tlm_pitch_sp_deg = pitch_sp_deg;

  tlm_rate_roll_dps  = rate_roll_dps;
  tlm_rate_pitch_dps = rate_pitch_dps;
  tlm_rate_yaw_dps   = rate_yaw_dps;
  tlm_rate_roll_sp_dps  = pid_tilt_roll_output();
  tlm_rate_pitch_sp_dps = pid_tilt_pitch_output();

  tlm_alt_cm       = alt_kf_get_altitude_cm();
  tlm_climb_cms    = alt_kf_get_climb_cms();
  tlm_climb_sp_cms = climb_sp_cms;

  tlm_acc_x_g = acc_x_g;  tlm_acc_y_g = acc_y_g;  tlm_acc_z_g = acc_z_g;

  for (int i = 1; i <= 16; i++) tlm_rc_ch[i] = rc_ch[i];
  tlm_esc[1] = esc_1;  tlm_esc[2] = esc_2;  tlm_esc[3] = esc_3;  tlm_esc[4] = esc_4;

  tlm_flight_mode = flight_mode;
  tlm_armed       = armed;
  tlm_rc_ok       = rc_ok;
  tlm_loop_us     = loop_period_us;

  xSemaphoreGive(mtx_tlm);
}
