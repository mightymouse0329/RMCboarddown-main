#include "power_model.hpp"

#include <cmath>

#include "control_constants.hpp"

namespace chassis_power
{
Prediction g_prediction = {};
LimitResult g_limit_result = {};

namespace
{
float bounded(float x, float low, float high) { return x < low ? low : (x > high ? high : x); }

// ---- Bias model state (restored to the 2026-10-19 17:11 version) ----
float meter_bias = 1.0f;
float filtered_measured_w = 0.0f;
bool bias_primed = false;

// Slow multiplicative correction driven by the low-pass filtered measured power.
// Called once per control cycle (1 kHz). Never modifies the measured value itself.
void update_meter_bias(float measured_w, float limit_w)
{
  constexpr float dt = 0.001f;
  const float tau = control_constants::POWER_BIAS_FILTER_S;
  if (!bias_primed) {
    filtered_measured_w = measured_w;
    bias_primed = true;
  }
  else {
    filtered_measured_w += (dt / (tau + dt)) * (measured_w - filtered_measured_w);
  }
  const float error = filtered_measured_w - limit_w;
  if (error > 0.0f) {
    meter_bias -= control_constants::POWER_BIAS_DOWN_PER_W * error;
  }
  else if (error < -control_constants::POWER_BIAS_RECOVER_BAND_W) {
    meter_bias += control_constants::POWER_BIAS_UP_PER_W *
                  (-error - control_constants::POWER_BIAS_RECOVER_BAND_W);
  }
  meter_bias =
    bounded(meter_bias, control_constants::POWER_BIAS_MIN, control_constants::POWER_BIAS_MAX);
}

// Look-ahead reserve: the faster the demand rises, the more headroom is held back.
float demand_reserve(float demand_w)
{
  constexpr float dt = 0.001f;
  static float previous_demand_w = 0.0f;
  static bool primed = false;
  if (!primed) {
    previous_demand_w = demand_w;
    primed = true;
    return 0.0f;
  }
  const float rate = (demand_w - previous_demand_w) / dt;
  previous_demand_w = demand_w;
  if (rate <= 0.0f) return 0.0f;
  return bounded(
    rate * control_constants::POWER_RESERVE_LEAD_S, 0.0f, control_constants::POWER_RESERVE_MAX_W);
}
}  // namespace

Prediction predict(const MotorState (&motors)[MOTOR_COUNT], bool feedback_valid)
{
  Prediction result = {};
  result.calibrated = control_constants::POWER_MODEL_CALIBRATED;
  const float k1 = control_constants::POWER_K1;
  const float k2 = control_constants::POWER_K2;
  const float k3 = control_constants::POWER_K3;
  const float ratio = control_constants::POWER_MECH_LOSS_RATIO;
  if (
    !std::isfinite(k1) || !std::isfinite(k2) || !std::isfinite(k3) || !std::isfinite(ratio) ||
    k1 < 0.0f || k2 < 0.0f || k3 < 0.0f || ratio < 0.0f)
    return result;

  float torque_squared_sum = 0.0f;
  float speed_squared_sum = 0.0f;
  for (const auto & motor : motors) {
    const float torque = motor.commanded_torque_nm;
    const float speed = motor.measured_speed_rad_s;
    if (!std::isfinite(torque) || !std::isfinite(speed)) return result;
    result.mechanical_w += torque * speed;
    torque_squared_sum += torque * torque;
    speed_squared_sum += speed * speed;
  }
  result.torque_sq_sum = torque_squared_sum;
  result.speed_sq_sum = speed_squared_sum;
  result.torque_loss_w = k1 * torque_squared_sum;
  result.speed_loss_w = k2 * speed_squared_sum;
  result.idle_loss_w = k3;
  // Transmission/ESC efficiency loss: proportional to the power itself, which the
  // K1 term (proportional to torque^2) cannot express.
  result.mech_loss_w = ratio * std::fabs(result.mechanical_w);
  result.total_w = result.mechanical_w + result.mech_loss_w + result.torque_loss_w +
                   result.speed_loss_w + result.idle_loss_w;
  const bool finite = std::isfinite(result.mechanical_w) && std::isfinite(result.mech_loss_w) &&
                      std::isfinite(result.torque_loss_w) && std::isfinite(result.speed_loss_w) &&
                      std::isfinite(result.total_w);
  if (!finite) {
    result = {};
    result.calibrated = control_constants::POWER_MODEL_CALIBRATED;
    return result;
  }
  if (result.total_w < 0.0f) result.total_w = 0.0f;
  result.valid = feedback_valid;
  return result;
}

LimitResult limit(
  const MotorState (&motors)[MOTOR_COUNT], bool feedback_valid, float measured_power_w,
  bool power_measured_valid, uint32_t meter_stamp_ms, uint32_t meter_sequence, uint32_t now_ms)
{
  (void)meter_stamp_ms;
  (void)meter_sequence;
  (void)now_ms;

  LimitResult result = {};
  result.limit_w =
    control_constants::CHASSIS_POWER_LIMIT_W - control_constants::POWER_LIMIT_MARGIN_W;
  result.before = predict(motors, feedback_valid);
  result.meter_used =
    power_measured_valid && std::isfinite(measured_power_w) && measured_power_w >= 0.0f;

  // Bias model: slow multiplicative correction from the filtered measured power.
  if (result.meter_used) update_meter_bias(measured_power_w, result.limit_w);
  // Look-ahead reserve applied on top of the bias-corrected budget.
  const float reserve = demand_reserve(result.before.total_w);
  result.effective_limit_w = result.limit_w * meter_bias - reserve;
  if (result.effective_limit_w < 0.0f) result.effective_limit_w = 0.0f;

  result.meter_bias = meter_bias;
  result.pi_trim_w = result.effective_limit_w - result.limit_w;
  result.pi_integral_w = 0.0f;
  result.measured_filtered_w = bias_primed ? filtered_measured_w : 0.0f;
  result.measured_rise_w_s = 0.0f;
  result.measured_forecast_w = 0.0f;
  result.trend_scale = 1.0f;
  result.meter_interval_ms = 0U;
  result.fast_over_limit =
    result.meter_used && measured_power_w > control_constants::CHASSIS_POWER_LIMIT_W;

  MotorState scaled[MOTOR_COUNT];
  for (std::size_t i = 0; i < MOTOR_COUNT; ++i) scaled[i] = {0.0f, motors[i].measured_speed_rad_s};
  const auto evaluate = [&](float k) {
    for (std::size_t i = 0; i < MOTOR_COUNT; ++i)
      scaled[i].commanded_torque_nm = motors[i].commanded_torque_nm * k;
    return predict(scaled, feedback_valid);
  };
  const auto finish = [&]() {
    result.torque_scale = bounded(result.torque_scale, 0.0f, 1.0f);
    result.after = evaluate(result.torque_scale);
    result.limited = result.torque_scale < 1.0f;
    result.feasible = result.after.valid && result.after.total_w <= result.effective_limit_w;
    g_prediction = result.before;
    g_limit_result = result;
    return result;
  };

  result.limited = true;
  if (!result.before.valid || !std::isfinite(result.effective_limit_w)) return finish();
  if (result.before.total_w <= result.effective_limit_w) {
    result.torque_scale = 1.0f;
    return finish();
  }

  // Model power vs scale is a*k^2 + b*k + c, clipped to nonnegative watts.
  const double a = result.before.torque_loss_w;
  const double b = result.before.mechanical_w;
  float low = 0.0f;
  if (a > 0.0 && b < 0.0) {
    const double minimum = -b / (2.0 * a);
    low = bounded(static_cast<float>(minimum), 0.0f, 1.0f);
  }
  if (a == 0.0 && b < 0.0) low = 1.0f;
  result.torque_scale = low;
  const Prediction minimum = evaluate(low);
  if (!minimum.valid || minimum.total_w > result.effective_limit_w) return finish();
  float high = 1.0f;
  for (unsigned i = 0; i < control_constants::POWER_LIMIT_SEARCH_STEPS; ++i) {
    const float middle = low + (high - low) * 0.5f;
    const Prediction candidate = evaluate(middle);
    if (candidate.valid && candidate.total_w <= result.effective_limit_w)
      low = middle;
    else
      high = middle;
  }
  result.torque_scale = low;
  return finish();
}
}  // namespace chassis_power
