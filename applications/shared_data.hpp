#ifndef SHARED_DATA_HPP
#define SHARED_DATA_HPP

#include <cstdint>

struct SharedData
{
  // 遥控器
  uint8_t sw_r;
  uint8_t sw_l;
  float ch_rh;  // 右摇杆左右 (云台Yaw)
  float ch_rv;  // 右摇杆上下 (云台Pitch)
  float ch_lh;  // 左摇杆左右 (底盘横向)
  float ch_lv;  // 左摇杆上下 (底盘前后)
  float ch_lu;  // 拨轮 (控制折叠)

  // 红外测距
  uint16_t distance_cm;
  uint8_t infrared_valid;
  uint32_t infrared_last_rx_ms;

  // 自主任务
  uint8_t auto_state;
  uint8_t auto_trigger;
  float auto_vx;
  float auto_vy;
  float auto_omega;

  // 云台与折叠目标（给 gimbal 和 fold 任务读）
  float gimbal_yaw_spd;
  float gimbal_pitch_spd;
  float fold_spd;
};

extern SharedData g_data;

#endif  // SHARED_DATA_HPP