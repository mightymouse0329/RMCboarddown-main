#include "power_meter.hpp"

namespace power_meter
{
volatile RealtimeData g_data = {};

namespace
{
float decode_hundredths(const uint8_t * bytes)
{
  // The supplied protocol specifies high byte first, values scaled by 100.
  // Treat as unsigned: the supplied table defines no signed/negative encoding.
  const uint16_t raw = static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
  return static_cast<float>(raw) / 100.0f;
}
}  // namespace

bool receive(const CAN_RxHeaderTypeDef & header, const uint8_t * bytes, uint32_t now_ms)
{
  if (
    bytes == nullptr || header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
    header.StdId != FEEDBACK_ID || header.DLC != FRAME_LENGTH)
    return false;
  if (bytes[6] != 0 || bytes[7] != 0) return false;

  // Decode before entering the short critical section.
  const float current = decode_hundredths(bytes);
  const float voltage = decode_hundredths(bytes + 2);
  const float power = decode_hundredths(bytes + 4);
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_data.current_a = current;
  g_data.voltage_v = voltage;
  g_data.power_w = power;
  g_data.last_rx_ms = now_ms;
  g_data.received_frames = g_data.received_frames + 1U;
  g_data.has_received = true;
  __set_PRIMASK(primask);
  return true;
}

RealtimeData snapshot()
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const RealtimeData result = {g_data.current_a,  g_data.voltage_v,       g_data.power_w,
                               g_data.last_rx_ms, g_data.received_frames, g_data.has_received};
  __set_PRIMASK(primask);
  return result;
}

bool is_alive(uint32_t now_ms, uint32_t timeout_ms)
{
  const RealtimeData data = snapshot();
  return data.has_received && (now_ms - data.last_rx_ms) < timeout_ms;
}
}  // namespace power_meter
