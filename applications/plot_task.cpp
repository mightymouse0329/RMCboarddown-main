#include "cmsis_os.h"
#include "control_constants.hpp"
#include "io/plotter/plotter.hpp"
#include "power_meter.hpp"
#include "power_model.hpp"
#include "usart.h"

sp::Plotter plotter(&huart1, false);

namespace
{
// 0 = DBUS 接收信号诊断（"收不到遥控器"时看这个）
// 1 = 链路快照：档位 / 摇杆 / 反馈掩码 / 限幅系数（信号通了以后切过来）
constexpr int PLOT_MODE = 0;

float prev_frames = 0.0f;
float prev_events = 0.0f;
float prev_err = 0.0f;
}  // namespace

// 链路快照，实现在 chassis_task.cpp / rc_task.cpp，长度 >= 11：
//   [0] sw_r  [1] ch_lv  [2] ch_lh  [3] ch_lu  [4] alive  [5] fb_mask
//   [6] fl_torque  [7] torque_scale  [8] can_err  [9] can_tx_free  [10] sw_l
extern "C" void chassis_control_diag(float * values);

// 接收链路原始计数，实现在 rc_task.cpp，长度 >= 7：
//   [0] rx_events  [1] rx_frames  [2] rx_last_size  [3] dma_ndtr
//   [4] uart_err_count  [5] uart_err_last  [6] rx_armed
extern "C" void rc_rx_diag(float * values);

extern "C" void plot_task()
{
  uint32_t last_wake = osKernelSysTick();

  for (;;) {
    osDelayUntil(&last_wake, 20);  // 50Hz

    if (!control_constants::CHASSIS_CONTROL_DIAGNOSTICS) {
      // 功率图：通道 1 限幅后的模型预测功率，通道 2 功率计实测功率。
      const power_meter::RealtimeData meter = power_meter::snapshot();
      plotter.plot(chassis_power::g_limit_result.after.total_w, meter.power_w);
    }
    else if (PLOT_MODE == 0) {
      // ── DBUS 接收信号诊断 ───────────────────────────────────────────────
      // 1 整帧/周期   正常 ~1（DR16 约 14 ms 一帧，20 ms 窗口里 1~2 帧）。恒 0 = 没有整帧
      // 2 事件/周期   所有 RxEvent 的增量。比第 1 条大 → 在收半帧碎片
      // 3 最近帧长    18 = 完整 DBUS 帧；1~17 = 半帧；0 = 从来没收到过任何字节
      // 4 DMA 剩余    18 = 刚挂上等第一个字节；回退到 <18 = 线上确实有电平变化
      // 5 错误/周期   0 = 波特率/校验对；>0 = 帧错/噪声（或 RX 悬空）
      // 6 最近错误码  HAL_UART_ERROR_* 位掩码：PE=1 NE=2 FE=4 ORE=8 DMA=16
      // 7 接收已挂载  1 = DMA 正在等数据；长时间 0 = 接收没挂上
      float rx[7] = {0};
      rc_rx_diag(rx);
      const float d_frames = rx[1] - prev_frames;
      const float d_events = rx[0] - prev_events;
      const float d_err = rx[4] - prev_err;
      prev_frames = rx[1];
      prev_events = rx[0];
      prev_err = rx[4];
      plotter.plot(d_frames, d_events, rx[2], rx[3], d_err, rx[5], rx[6]);
    }
    else {
      // ── 链路快照 ────────────────────────────────────────────────────────
      // 1 sw_r | 2 sw_l | 3 ch_lv | 4 ch_lh | 5 ch_lu |
      // 6 alive | 7 fb_mask | 8 torque_scale | 9 fl_torque
      float v[11] = {0};
      chassis_control_diag(v);
      plotter.plot(v[0], v[10], v[1], v[2], v[3], v[4], v[5], v[7], v[6]);
      // 上位机通道数不够时，改用这一行只画 4 条最关键的（sw_r / sw_l / fb_mask / torque_scale）：
      // plotter.plot(v[0], v[10], v[5], v[7]);
    }
  }
}
