#include <cmath>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "control_constants.hpp"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "power_meter.hpp"
#include "power_model.hpp"
#include "shared_data.hpp"
#include "task.h"
#include "tools/pid/pid.hpp"

extern "C" void rc_control_diag(uint32_t now_ms, float * values);

namespace
{
constexpr float WHEEL_RADIUS_M = 0.077f;

// ?????????????????
constexpr float MAX_LINEAR_SPEED = 1.6f;
constexpr float MAX_ANGULAR_SPEED = 5.0f;
constexpr float GEAR_RATIO = 14.9f;
constexpr float MAX_TORQUE = 0.8f;
constexpr float DEADZONE = 0.05f;

float apply_deadzone(float val)
{
  if (val > -DEADZONE && val < DEADZONE) return 0.0f;
  return val;
}

// ---- Chassis direction signs ----
// 2026-10: the chassis frame is physically mounted 180 deg from the firmware
// convention (front and back swapped). A 180 deg rotation about z inverts x and y
// but leaves the yaw sense unchanged, so vx and vy are inverted and omega is NOT.
// Verified on the car: forward, lateral and rotation all match the sticks.
// If only ONE axis is wrong later, flip just that single value between 1.0f / -1.0f.
constexpr float DIR_VX = -1.0f;
constexpr float DIR_VY = 1.0f;
constexpr float DIR_OMEGA = 1.0f;

constexpr float DIR_M_FL = 1.0f;   // 1
constexpr float DIR_M_FR = -1.0f;  // 2
constexpr float DIR_M_RL = -1.0f;  // 3
constexpr float DIR_M_RR = 1.0f;   // 4

// ---- 速度环整定（2026-10 修改）----
// Kp 由 2.0 降到 1.0：按 τ = J/Kp 估算，2.0 时约 2 ms，只有 2 个执行器周期(1 ms)，
// 属于边缘偏高，典型表现就是"静止时嗡嗡响、跑起来听不出来"。
constexpr float SPEED_KP = 1.0f;
// 零速释放：目标与实际速度都很小时直接输出 0，让轮子自由滑停。
// 原来的做法是松手后目标变 0、PID 继续以满力矩把实际转速往 0 拽，
// 刹过头再反向，于是在零速附近来回抽 → 嗡嗡响且电机发热。
constexpr float HOLD_SPEED_RAD_S = 0.5f;  // 轮子（输出轴）角速度死区，约 3.9 cm/s
// 输出死区：残余力矩小于它就当 0，避免电机轻轻"哼"
constexpr float OUTPUT_DEADBAND_NM = 0.02f;

float diag_target[4];
float diag_actual[4];
float diag_out[4];
float diag_released[4];

// 单轮速度环：带零速释放与输出死区。index 仅用于诊断快照。
float wheel_output(int index, sp::PID & pid, float target, float actual)
{
  const bool idle = std::fabs(target) < HOLD_SPEED_RAD_S && std::fabs(actual) < HOLD_SPEED_RAD_S;
  diag_target[index] = target;
  diag_actual[index] = actual;
  diag_released[index] = idle ? 1.0f : 0.0f;
  float out = 0.0f;
  if (idle) {
    pid.clear();  // 清历史状态，避免下次起步带着旧值
  }
  else {
    pid.calc(target, actual);
    out = std::fabs(pid.out) < OUTPUT_DEADBAND_NM ? 0.0f : pid.out;
  }
  diag_out[index] = out;
  return out;
}

}  // namespace

sp::CAN can1(&hcan1);

// ?????????? ID
sp::RM_Motor motor_fl(1, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor motor_fr(2, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor motor_rl(3, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor motor_rr(4, sp::RM_Motors::M3508, GEAR_RATIO);

sp::PID pid_fl(1e-3f, SPEED_KP, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);
sp::PID pid_fr(1e-3f, SPEED_KP, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);
sp::PID pid_rl(1e-3f, SPEED_KP, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);
sp::PID pid_rr(1e-3f, SPEED_KP, 0.0f, 0.0f, MAX_TORQUE, 0.0f, 1.0f, false, false);

// 供绘图使用的底盘诊断快照（只读）：0=FL 1=FR 2=RL 3=RR
extern "C" void chassis_diag(float * target, float * actual, float * out, float * released)
{
  for (int i = 0; i < 4; ++i) {
    target[i] = diag_target[i];
    actual[i] = diag_actual[i];
    out[i] = diag_out[i];
    released[i] = diag_released[i];
  }
}

// Diagnostic only: does not send commands or relax any protection.
extern "C" void chassis_control_diag(float * values)
{
  if (!values) return;
  const uint32_t now = osKernelSysTick();
  rc_control_diag(now, values);
  taskENTER_CRITICAL();
  const unsigned feedback_mask =
    (motor_fl.is_alive(now) ? 1U : 0U) | (motor_fr.is_alive(now) ? 2U : 0U) |
    (motor_rl.is_alive(now) ? 4U : 0U) | (motor_rr.is_alive(now) ? 8U : 0U);
  values[5] = static_cast<float>(feedback_mask);
  values[6] = diag_out[0];  // FL requested torque before power limiting.
  values[7] = chassis_power::g_limit_result.torque_scale;
  values[8] = static_cast<float>(HAL_CAN_GetError(&hcan1));
  values[9] = static_cast<float>(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
  taskEXIT_CRITICAL();
}

namespace
{
// Additional forward/back measured-power guard. This is independent of the
// existing model/bias limiter and only reduces the final torque scale when the
// power meter is already close to the hard 80 W limit.
constexpr float TRANS_GUARD_NEAR_W = 78.0f;
constexpr float TRANS_GUARD_RECOVER_W = 76.0f;
constexpr float TRANS_GUARD_RECOVER_STEP = 0.02f;
bool translation_guard_active = false;
float translation_guard_scale = 1.0f;
uint32_t translation_guard_last_seq = 0;

void reset_translation_guard()
{
  translation_guard_active = false;
  translation_guard_scale = 1.0f;
  translation_guard_last_seq = 0;
}

void update_translation_guard(const power_meter::RealtimeData & meter, bool meter_ok)
{
  if (!meter_ok) return;
  if (meter.received_frames == translation_guard_last_seq) return;
  translation_guard_last_seq = meter.received_frames;

  if (meter.power_w > TRANS_GUARD_NEAR_W) {
    // Scale relative to the CURRENT guard, so repeated over-limit samples
    // continue reducing torque instead of latching at the first ratio.
    translation_guard_scale *= TRANS_GUARD_NEAR_W / meter.power_w;
    if (translation_guard_scale < 0.0f) translation_guard_scale = 0.0f;
  }
  else if (meter.power_w < TRANS_GUARD_RECOVER_W) {
    translation_guard_scale += TRANS_GUARD_RECOVER_STEP;
    if (translation_guard_scale > 1.0f) translation_guard_scale = 1.0f;
  }
}

float update_power_prediction(float fl, float fr, float rl, float rr)
{
  const uint32_t now = osKernelSysTick();
  taskENTER_CRITICAL();
  const bool valid = motor_fl.is_alive(now) && motor_fr.is_alive(now) && motor_rl.is_alive(now) &&
                     motor_rr.is_alive(now);
  // Torque and speed use the same physical motor-output direction convention.
  const chassis_power::MotorState motors[chassis_power::MOTOR_COUNT] = {
    {fl, motor_fl.speed}, {fr, motor_fr.speed}, {rl, motor_rl.speed}, {rr, motor_rr.speed}};
  taskEXIT_CRITICAL();
  // Measured-power PI + fast guard use only fresh meter samples.
  const auto meter = power_meter::snapshot();
  const bool meter_ok = power_meter::is_alive(now, control_constants::POWER_METER_TIMEOUT_MS) &&
                        std::isfinite(meter.power_w) && meter.power_w >= 0.0f;
  chassis_power::g_limit_result = chassis_power::limit(
    motors, valid, meter.power_w, meter_ok, meter.last_rx_ms, meter.received_frames, now);
  chassis_power::g_prediction = chassis_power::g_limit_result.before;
  float scale = chassis_power::g_limit_result.torque_scale;
  if (translation_guard_active) {
    update_translation_guard(meter, meter_ok);
    scale *= translation_guard_scale;
  }
  return scale;
}

void set_all_zero()
{
  reset_translation_guard();
  update_power_prediction(0.0f, 0.0f, 0.0f, 0.0f);
  // 清 PID 历史，避免下次起步带着旧状态
  pid_fl.clear();
  pid_fr.clear();
  pid_rl.clear();
  pid_rr.clear();
  for (int i = 0; i < 4; ++i) {
    diag_released[i] = 1.0f;
    diag_out[i] = 0.0f;
  }
  motor_fl.cmd(0.0f);
  motor_fr.cmd(0.0f);
  motor_rl.cmd(0.0f);
  motor_rr.cmd(0.0f);
}

void drive_with(float vx, float vy, float omega)
{
  float v_1 = (vx + vy - omega) / WHEEL_RADIUS_M;
  float v_2 = (vx - vy + omega) / WHEEL_RADIUS_M;
  float v_3 = (vx + vy + omega) / WHEEL_RADIUS_M;
  float v_4 = (vx - vy - omega) / WHEEL_RADIUS_M;

  // 带零速释放与输出死区的速度环（见 wheel_output 注释）
  const float fl = wheel_output(0, pid_fl, v_1, motor_fl.speed * DIR_M_FL) * DIR_M_FL;
  const float fr = wheel_output(1, pid_fr, v_2, motor_fr.speed * DIR_M_FR) * DIR_M_FR;
  const float rl = wheel_output(2, pid_rl, v_3, motor_rl.speed * DIR_M_RL) * DIR_M_RL;
  const float rr = wheel_output(3, pid_rr, v_4, motor_rr.speed * DIR_M_RR) * DIR_M_RR;
  const float power_scale = update_power_prediction(fl, fr, rl, rr);
  motor_fl.cmd(power_scale * fl);
  motor_fr.cmd(power_scale * fr);
  motor_rl.cmd(power_scale * rl);
  motor_rr.cmd(power_scale * rr);
}

void chassis_update()
{
  if (g_data.sw_r == 0) {
    set_all_zero();
    return;
  }

  if (g_data.sw_r == 2) {
    if (g_data.auto_trigger != 0) {
      translation_guard_active = std::fabs(g_data.auto_vx) > 0.01f;
      drive_with(g_data.auto_vx * DIR_VX, g_data.auto_vy * DIR_VY, g_data.auto_omega * DIR_OMEGA);
    }
    else {
      set_all_zero();
    }
    return;
  }

  if (g_data.sw_r == 1) {
    // ????????????
    // ??? = ch_lv (vx)
    // ???? = ch_lh (vy)
    // ???? = ch_lu (omega)
    float target_vx = apply_deadzone(g_data.ch_lv) * MAX_LINEAR_SPEED * DIR_VX;
    float target_vy = apply_deadzone(g_data.ch_lh) * MAX_LINEAR_SPEED * DIR_VY;
    float target_omega = apply_deadzone(g_data.ch_lu) * MAX_ANGULAR_SPEED * DIR_OMEGA;

    translation_guard_active = std::fabs(target_vx) > 0.01f;
    drive_with(target_vx, target_vy, target_omega);
    return;
  }

  set_all_zero();
}
}  // namespace

extern "C" void chassis_task()
{
  can1.config();
  can1.start();

  uint32_t last_wake = osKernelSysTick();

  for (;;) {
    osDelayUntil(&last_wake, 1);
    chassis_update();

    motor_fl.write(can1.tx_data);
    motor_fr.write(can1.tx_data);
    motor_rl.write(can1.tx_data);
    motor_rr.write(can1.tx_data);
    can1.send(motor_fl.tx_id);
  }
}

// One dispatcher, one FIFO read per frame. No undefined gimbal objects.
namespace
{
void receive_can_fifo(CAN_HandleTypeDef * hcan, uint32_t fifo)
{
  if (hcan != &hcan1 && hcan != &hcan2) return;
  while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0) {
    CAN_RxHeaderTypeDef header = {};
    uint8_t data[8] = {};
    if (HAL_CAN_GetRxMessage(hcan, fifo, &header, data) != HAL_OK) break;
    // CAN2 has no configured application devices yet; consume and ignore its frames.
    if (hcan != &hcan1) continue;
    can1.frame_type = (header.IDE == CAN_ID_EXT);
    can1.rx_id = can1.frame_type ? header.ExtId : header.StdId;
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8) continue;
    for (unsigned i = 0; i < 8; ++i) can1.rx_data[i] = data[i];
    const uint32_t stamp_ms = osKernelSysTick();
    if (header.StdId == power_meter::FEEDBACK_ID)
      power_meter::receive(header, data, stamp_ms);
    else if (header.StdId == motor_fl.rx_id)
      motor_fl.read(data, stamp_ms);
    else if (header.StdId == motor_fr.rx_id)
      motor_fr.read(data, stamp_ms);
    else if (header.StdId == motor_rl.rx_id)
      motor_rl.read(data, stamp_ms);
    else if (header.StdId == motor_rr.rx_id)
      motor_rr.read(data, stamp_ms);
  }
}
}  // namespace

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  receive_can_fifo(hcan, CAN_RX_FIFO0);
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  receive_can_fifo(hcan, CAN_RX_FIFO1);
}