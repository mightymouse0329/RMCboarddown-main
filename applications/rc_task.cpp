#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "shared_data.hpp"
#include "task.h"
#include "usart.h"

sp::DBus remote(&huart3);

// ---- DBUS 接收链路诊断计数器（只增不减，只被中断写）----
namespace
{
volatile uint32_t rx_events = 0;     // RxEventCallback 调用次数（整帧 + 半帧）
volatile uint32_t rx_frames = 0;     // Size == 18 的次数，即真正的 DBUS 整帧
volatile uint16_t rx_last_size = 0;  // 最近一次事件的字节数
volatile uint32_t uart_err_count = 0;
volatile uint32_t uart_err_last = 0;
}  // namespace

// Task-context read-only snapshot: switch, vx/vy/rotation sticks, receive age flag.
// 调用方传入的数组长度必须 >= 11。[10] 是新增的 sw_l，见 plot_task.cpp 的通道表。
extern "C" void rc_control_diag(uint32_t now_ms, float * values)
{
  if (!values) return;
  taskENTER_CRITICAL();
  values[0] = static_cast<float>(g_data.sw_r);
  values[1] = g_data.ch_lv;
  values[2] = g_data.ch_lh;
  values[3] = g_data.ch_lu;
  values[4] = remote.is_alive(now_ms) ? 1.0f : 0.0f;
  values[10] = static_cast<float>(g_data.sw_l);
  taskEXIT_CRITICAL();
}

// 接收链路原始快照（只读，任务上下文）。调用方传入的数组长度必须 >= 7。
//   [0] rx_events      RxEvent 总次数
//   [1] rx_frames      整 18 字节帧数
//   [2] rx_last_size   最近一次事件字节数
//   [3] dma_ndtr       DMA 剩余计数 18..0
//   [4] uart_err_count UART 错误次数
//   [5] uart_err_last  最近一次 HAL_UART_ERROR_* 位掩码
//   [6] rx_armed       1 = DMA 正在等数据
extern "C" void rc_rx_diag(float * values)
{
  if (!values) return;
  values[0] = static_cast<float>(rx_events);
  values[1] = static_cast<float>(rx_frames);
  values[2] = static_cast<float>(rx_last_size);
  values[3] =
    (huart3.hdmarx != NULL) ? static_cast<float>(__HAL_DMA_GET_COUNTER(huart3.hdmarx)) : 0.0f;
  values[4] = static_cast<float>(uart_err_count);
  values[5] = static_cast<float>(uart_err_last);
  values[6] = (huart3.RxState == HAL_UART_STATE_BUSY_RX) ? 1.0f : 0.0f;
}

extern "C" void rc_task()
{
  remote.request();

  for (;;) {
    osDelay(10);

    // 接收看门狗：HAL 在错误回调里重启 DMA 时若返回 BUSY，返回值是被丢弃的，
    // 那样就再没人重启接收，DBUS 会一直死到复位。这里兜一层保护——只在
    // "已经 100 ms 没有整帧" 且 "UART 当前没有任何接收在跑" 时才重挂，
    // 不会与回调里的正常重挂争用。只作用于接收，不改变任何控制逻辑。
    const uint32_t now = osKernelSysTick();
    if (!remote.is_alive(now) && huart3.RxState == HAL_UART_STATE_READY) {
      remote.request();
    }
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == &huart3) {
    auto stamp_ms = osKernelSysTick();

    // 先记诊断计数，再解析：任何一个字节都不会漏统计。
    ++rx_events;
    rx_last_size = Size;
    if (Size == sp::DBUS_BUFF_SIZE) ++rx_frames;

    remote.update(Size, stamp_ms);

    g_data.sw_r = static_cast<uint8_t>(remote.sw_r);
    g_data.sw_l = static_cast<uint8_t>(remote.sw_l);
    g_data.ch_lv = remote.ch_lv;
    g_data.ch_lh = remote.ch_lh;
    g_data.ch_lu = remote.ch_lu;

    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    ++uart_err_count;
    uart_err_last = huart->ErrorCode;
    remote.request();
  }
}
