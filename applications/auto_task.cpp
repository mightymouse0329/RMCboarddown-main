#include "cmsis_os.h"
#include "shared_data.hpp"

namespace
{
// Auto mode: RC right switch UP starts a one-shot 5 s forward run.
constexpr float AUTO_FORWARD_SPEED_MPS = 0.15f;  // 30 cm/s
constexpr uint32_t AUTO_RUN_MS = 10000U;

enum class AutoState : uint8_t
{
  IDLE = 0,
  RUNNING = 1,
  DONE = 2,
};

AutoState state = AutoState::IDLE;
uint32_t start_ms = 0;

void stop_auto()
{
  // Clear the trigger first so the chassis task stops before the speed changes.
  g_data.auto_trigger = 0;
  g_data.auto_vx = 0.0f;
  g_data.auto_vy = 0.0f;
  g_data.auto_omega = 0.0f;
  g_data.fold_spd = 0.0f;
}

void set_forward()
{
  g_data.auto_vx = AUTO_FORWARD_SPEED_MPS;
  g_data.auto_vy = 0.0f;
  g_data.auto_omega = 0.0f;
  g_data.auto_trigger = 1;
}
}  // namespace

extern "C" void auto_task()
{
  uint32_t last_wake = osKernelSysTick();

  for (;;) {
    osDelayUntil(&last_wake, 20);  // 50 Hz

    const uint32_t now = osKernelSysTick();

    // Auto is only active in the UP position (sw_r == 2). MID/DOWN reset the
    // sequence so the next UP edge starts a fresh 5 s run.
    if (g_data.sw_r != 2) {
      state = AutoState::IDLE;
      stop_auto();
      g_data.auto_state = static_cast<uint8_t>(AutoState::IDLE);
      continue;
    }

    switch (state) {
      case AutoState::IDLE:
        state = AutoState::RUNNING;
        start_ms = now;
        set_forward();
        g_data.auto_state = static_cast<uint8_t>(AutoState::RUNNING);
        break;

      case AutoState::RUNNING:
        if ((now - start_ms) >= AUTO_RUN_MS) {
          state = AutoState::DONE;
          stop_auto();
          g_data.auto_state = static_cast<uint8_t>(AutoState::DONE);
        }
        else {
          set_forward();
          g_data.auto_state = static_cast<uint8_t>(AutoState::RUNNING);
        }
        break;

      case AutoState::DONE:
        // One-shot: stay stopped while the switch remains UP.
        stop_auto();
        g_data.auto_state = static_cast<uint8_t>(AutoState::DONE);
        break;
    }
  }
}
