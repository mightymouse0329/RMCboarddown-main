#ifndef POWER_METER_HPP
#define POWER_METER_HPP

#include <cstdint>

#include "can.h"

namespace power_meter
{
constexpr uint32_t FEEDBACK_ID = 0x301;
constexpr uint32_t FRAME_LENGTH = 8;

struct RealtimeData
{
  float current_a;
  float voltage_v;
  float power_w;  // Measured power from bytes 4/5, NOT current_a * voltage_v.
  uint32_t last_rx_ms;
  uint32_t received_frames;
  bool has_received;  // Does not imply that the last measurement is still fresh.
};

// Live decoded measurements. Read snapshot() for consistent fields across an ISR update.
extern volatile RealtimeData g_data;

// Call ONCE from the existing CAN receive dispatcher, AFTER HAL_CAN_GetRxMessage.
// No CAN initialization, FIFO polling, or HAL receive callback is defined here.
// Caller selects the CAN bus to which the power meter is physically connected.
// Returns false for wrong ID/type/length, null data, or nonzero reserved bytes.
bool receive(const CAN_RxHeaderTypeDef & header, const uint8_t * bytes, uint32_t now_ms);

RealtimeData snapshot();
// timeout_ms must be chosen according to the actual meter's reporting period.
bool is_alive(uint32_t now_ms, uint32_t timeout_ms);
}  // namespace power_meter

#endif  // POWER_METER_HPP
