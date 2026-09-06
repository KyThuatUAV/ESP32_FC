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
//  PID xếp tầng
//
//     góc mong muốn --[pid_tilt_*]--> tốc độ góc mong muốn --[pid_rate_*]--> u
//     tốc độ lên xuống mong muốn --[pid_climb]--> ga
//
//  Hệ số KP/KI/KD nằm hết trong khối cấu hình ở file .ino chính.
//  Mỗi bộ điều khiển giữ riêng trạng thái trong một struct PidState, thay vì
//  rải ra thành ba biến toàn cục rời như trước - thêm bộ mới không phải nhớ
//  khai báo đủ ba biến nữa.
//
//  GHI CÔNG: cấu trúc PID xếp tầng (tầng ngoài sinh tốc độ góc mong muốn cho
//  tầng trong bám theo) học từ tài liệu của Carbon Aeronautics
//  (https://github.com/CarbonAeronautics). Chi tiết xem file LOI_NHAN_VA_GHI_CONG.md.
// ============================================================================

#include "types.h"   // PidState

PidState pid_rate_roll_st, pid_rate_pitch_st, pid_rate_yaw_st;
PidState pid_tilt_roll_st, pid_tilt_pitch_st;
PidState pid_climb_st;

// Giữ lại đầu ra tầng góc để in debug
static float tilt_roll_out_dps, tilt_pitch_out_dps;


// PID rời rạc, tích phân theo quy tắc hình thang, vi phân theo sai phân lùi.
float pid_step(PidState &st, float error, float kp, float ki, float kd,
               float i_limit, float u_limit) {
  float p_term = kp * error;

  st.integral += ki * (error + st.prev_error) * DT_CTRL / 2.0f;
  if      (st.integral >  i_limit) st.integral =  i_limit;
  else if (st.integral < -i_limit) st.integral = -i_limit;

  float d_term = kd * (error - st.prev_error) / DT_CTRL;

  float u = p_term + st.integral + d_term;
  if      (u >  u_limit) u =  u_limit;
  else if (u < -u_limit) u = -u_limit;

  st.prev_error = error;
  return u;
}

// Riêng trục thẳng đứng: khâu tích phân không cho âm vì nó gánh phần ga treo
// máy, và khâu tỉ lệ bị chặn riêng để một cú nhiễu baro không đẩy ga vọt lên.
float pid_step_climb(PidState &st, float error, float kp, float ki, float kd,
                     float i_limit, float p_limit) {
  float p_term = kp * error;
  if      (p_term >  p_limit) p_term =  p_limit;
  else if (p_term < -p_limit) p_term = -p_limit;

  st.integral += ki * (error + st.prev_error) * DT_CTRL / 2.0f;
  if      (st.integral > i_limit) st.integral = i_limit;
  else if (st.integral < 0)       st.integral = 0;

  float d_term = kd * (error - st.prev_error) / DT_CTRL;

  st.prev_error = error;
  return p_term + st.integral + d_term;
}


// ---- Tầng trong: tốc độ góc ------------------------------------------------
float pid_rate_roll(float rate_dps, float rate_sp_dps) {
  return pid_step(pid_rate_roll_st, rate_sp_dps - rate_dps,
                  KP_RATE_ROLL, KI_RATE_ROLL, KD_RATE_ROLL,
                  I_LIM_RATE, U_LIM_RATE);
}

float pid_rate_pitch(float rate_dps, float rate_sp_dps) {
  return pid_step(pid_rate_pitch_st, rate_sp_dps - rate_dps,
                  KP_RATE_PITCH, KI_RATE_PITCH, KD_RATE_PITCH,
                  I_LIM_RATE, U_LIM_RATE);
}

float pid_rate_yaw(float rate_dps, float rate_sp_dps) {
  return pid_step(pid_rate_yaw_st, rate_sp_dps - rate_dps,
                  KP_RATE_YAW, KI_RATE_YAW, KD_RATE_YAW,
                  I_LIM_YAW, U_LIM_YAW);
}

// ---- Tầng ngoài: góc nghiêng -----------------------------------------------
float pid_tilt_roll(float angle_deg, float angle_sp_deg) {
  tilt_roll_out_dps = pid_step(pid_tilt_roll_st, angle_sp_deg - angle_deg,
                               KP_TILT_ROLL, KI_TILT_ROLL, KD_TILT_ROLL,
                               I_LIM_TILT, U_LIM_TILT);
  return tilt_roll_out_dps;
}

float pid_tilt_pitch(float angle_deg, float angle_sp_deg) {
  tilt_pitch_out_dps = pid_step(pid_tilt_pitch_st, angle_sp_deg - angle_deg,
                                KP_TILT_PITCH, KI_TILT_PITCH, KD_TILT_PITCH,
                                I_LIM_TILT, U_LIM_TILT);
  return tilt_pitch_out_dps;
}

float pid_tilt_roll_output()  { return tilt_roll_out_dps;  }
float pid_tilt_pitch_output() { return tilt_pitch_out_dps; }

// ---- Tốc độ lên xuống -------------------------------------------------------
float pid_climb(float climb_cms, float climb_sp_cms_in) {
  return pid_step_climb(pid_climb_st, climb_sp_cms_in - climb_cms,
                        KP_CLIMB, KI_CLIMB, KD_CLIMB,
                        I_LIM_CLIMB, P_LIM_CLIMB);
}


// Xoá sạch khâu tích phân, dùng khi disarm
void pid_reset_all() {
  pid_rate_roll_st.integral  = 0;
  pid_rate_pitch_st.integral = 0;
  pid_rate_yaw_st.integral   = 0;
  pid_tilt_roll_st.integral  = 0;
  pid_tilt_pitch_st.integral = 0;
  pid_climb_st.integral      = 0;
}

// Xả dần khâu tích phân, dùng khi hạ hết ga mà vẫn đang arm
void pid_bleed_integrators() {
  pid_rate_roll_st.integral  *= 0.9f;
  pid_rate_pitch_st.integral *= 0.9f;
  pid_rate_yaw_st.integral   *= 0.9f;
  pid_tilt_roll_st.integral  *= 0.9f;
  pid_tilt_pitch_st.integral *= 0.9f;
}
