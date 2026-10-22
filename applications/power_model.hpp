#ifndef POWER_MODEL_HPP
#define POWER_MODEL_HPP

#include <cstddef>
#include <cstdint>

namespace chassis_power
{
constexpr std::size_t MOTOR_COUNT = 4;

struct MotorState
{
  float commanded_torque_nm;   // Post-PID, before any power limiting.
  float measured_speed_rad_s;  // Feedback, NOT target speed; same shaft as torque.
};

struct Prediction
{
  float mechanical_w;
  float mech_loss_w;  // Transmission/ESC loss = ratio * |mechanical_w| (bias-era model).
  float torque_loss_w;
  float speed_loss_w;
  float idle_loss_w;
  float torque_sq_sum;  // sum(tau^2), calibration plots only.
  float speed_sq_sum;   // sum(omega^2), calibration plots only.
  float total_w;        // Electrical consumption estimate, clamped to >= 0 W.
  bool valid;           // Finite inputs/parameters/results AND all feedback fresh.
  bool calibrated;      // Independent of valid: initial coefficients are uncalibrated.
};

// Does not modify motor commands or perform power limiting.
// Negative mechanical power represents braking; regeneration needs validation.
Prediction predict(const MotorState (&motors)[MOTOR_COUNT], bool feedback_valid);

struct LimitResult
{
  Prediction before;
  Prediction after;
  float torque_scale;         // K in [0, 1]; calculated each cycle, not a tuning constant.
  float limit_w;              // Nominal budget = hard limit - margin.
  float effective_limit_w;    // limit_w * bias - reserve.
  float pi_trim_w;            // Equivalent bias correction in watts (diagnostics).
  float pi_integral_w;        // Unused by the bias model; kept for plotting code.
  float measured_filtered_w;  // Low-passed measured power (bias regulation input).
  float measured_rise_w_s;
  float measured_forecast_w;
  float trend_scale;  // Always 1.0 in the bias model.
  uint32_t meter_interval_ms;
  bool fast_over_limit;  // Raw reading already above the hard limit.
  float meter_bias;      // Slow multiplicative correction, in [MIN, MAX].
  bool limited;
  bool feasible;    // False on invalid input or no feasible uniform scale.
  bool meter_used;  // True when the measured power was usable this cycle.
};

// Finds the largest feasible uniform torque scale. Does not alter the inputs.
// Invalid feedback/model: returns scale 0. If no scale can meet the limit,
// returns the scale minimizing model power and reports feasible=false.
// Extra arguments are accepted for compatibility with existing callers.
LimitResult limit(
  const MotorState (&motors)[MOTOR_COUNT], bool feedback_valid, float measured_power_w = 0.0f,
  bool power_measured_valid = false, uint32_t meter_stamp_ms = 0, uint32_t meter_sequence = 0,
  uint32_t now_ms = 0);
extern LimitResult g_limit_result;

// Latest prediction BEFORE limiting, updated also for zero-command mode.
extern Prediction g_prediction;
}  // namespace chassis_power

#endif  // POWER_MODEL_HPP
